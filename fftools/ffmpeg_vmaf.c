/*
 * VMAF-based rate control helpers for the ffmpeg CLI.
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "ffmpeg_vmaf.h"

#if CONFIG_LIBVMAF

#include <math.h>
#include <string.h>

#include "ffmpeg.h"

#include "libavcodec/avcodec.h"

#include "libavutil/avassert.h"
#include "libavutil/common.h"
#include "libavutil/fifo.h"
#include "libavutil/frame.h"
#include "libavutil/log.h"
#include "libavutil/pixdesc.h"

#include <libvmaf.h>

#define TARGET_VMAF_DEFAULT_QSCALE 23.0
#define TARGET_VMAF_MIN_QSCALE      1.0
#define TARGET_VMAF_MAX_QSCALE     50.0

typedef struct TargetVMAFState {
    double target;
    double qscale;
    double kp;
    double ki;
    double integral;
    double last_score;
    unsigned frame_count;
    unsigned bpc;
    AVCodecContext *dec_ctx;
    AVFrame        *dec_frame;
    AVFifo         *ref_fifo;
    VmafContext    *vmaf;
    VmafModel      *model;
    enum VmafPoolingMethod pool_method;
    int warned_format;
} TargetVMAFState;

static enum VmafPixelFormat pix_fmt_map(enum AVPixelFormat fmt)
{
    switch (fmt) {
    case AV_PIX_FMT_YUV420P:
    case AV_PIX_FMT_YUV420P10LE:
    case AV_PIX_FMT_YUV420P12LE:
    case AV_PIX_FMT_YUV420P16LE:
        return VMAF_PIX_FMT_YUV420P;
    case AV_PIX_FMT_YUV422P:
    case AV_PIX_FMT_YUV422P10LE:
    case AV_PIX_FMT_YUV422P12LE:
    case AV_PIX_FMT_YUV422P16LE:
        return VMAF_PIX_FMT_YUV422P;
    case AV_PIX_FMT_YUV444P:
    case AV_PIX_FMT_YUV444P10LE:
    case AV_PIX_FMT_YUV444P12LE:
    case AV_PIX_FMT_YUV444P16LE:
        return VMAF_PIX_FMT_YUV444P;
    default:
        return VMAF_PIX_FMT_UNKNOWN;
    }
}

static int copy_picture_data(const AVFrame *src, VmafPicture *dst, unsigned bpc)
{
    const int bytes_per_value = bpc > 8 ? 2 : 1;
    int err = vmaf_picture_alloc(dst, pix_fmt_map(src->format), bpc,
                                 src->width, src->height);
    if (err)
        return AVERROR(ENOMEM);

    for (unsigned i = 0; i < 3; i++) {
        uint8_t *src_data = src->data[i];
        uint8_t *dst_data = dst->data[i];
        for (unsigned j = 0; j < dst->h[i]; j++) {
            memcpy(dst_data, src_data, bytes_per_value * dst->w[i]);
            src_data += src->linesize[i];
            dst_data += dst->stride[i];
        }
    }

    return 0;
}

static void target_vmaf_free_fifo(TargetVMAFState *s)
{
    if (!s->ref_fifo)
        return;

    while (av_fifo_can_read(s->ref_fifo)) {
        AVFrame *ref = NULL;
        av_fifo_read(s->ref_fifo, &ref, 1);
        av_frame_free(&ref);
    }

    av_fifo_freep2(&s->ref_fifo);
}

static void target_vmaf_update_qscale(OutputStream *ost, TargetVMAFState *s,
                                      double score)
{
    const double error = s->target - score;
    double delta;

    s->integral += error;
    s->integral = av_clipd(s->integral, -1000.0, 1000.0);

    delta = s->kp * error + s->ki * s->integral;
    s->qscale -= delta;
    s->qscale = av_clipd(s->qscale, TARGET_VMAF_MIN_QSCALE,
                         TARGET_VMAF_MAX_QSCALE);

    ost->target_vmaf_qscale = s->qscale;
    ost->enc->enc_ctx->global_quality = lrint(FF_QP2LAMBDA * s->qscale);
}

static int target_vmaf_consume(OutputStream *ost, TargetVMAFState *s,
                               AVFrame *dist)
{
    AVFrame *ref = NULL;
    VmafPicture ref_pic = { 0 }, dist_pic = { 0 };
    double pooled_score;
    int err;

    if (!av_fifo_can_read(s->ref_fifo)) {
        av_log(ost, AV_LOG_ERROR,
               "target VMAF controller lost reference frame alignment.\n");
        return AVERROR(EINVAL);
    }

    av_fifo_read(s->ref_fifo, &ref, 1);

    if (ref->width  != dist->width ||
        ref->height != dist->height ||
        ref->format != dist->format) {
        if (!s->warned_format) {
            av_log(ost, AV_LOG_ERROR,
                   "target VMAF controller requires matching dimensions and "
                   "pixel formats between reference and encoded video.\n");
            s->warned_format = 1;
        }
        av_frame_free(&ref);
        return AVERROR(EINVAL);
    }

    if (!s->bpc) {
        const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(dist->format);
        if (!desc) {
            av_frame_free(&ref);
            return AVERROR(EINVAL);
        }
        s->bpc = desc->comp[0].depth;
    }

    err = copy_picture_data(ref, &ref_pic, s->bpc);
    if (!err)
        err = copy_picture_data(dist, &dist_pic, s->bpc);
    av_frame_free(&ref);
    if (err) {
        vmaf_picture_unref(&ref_pic);
        vmaf_picture_unref(&dist_pic);
        return err;
    }

    err = vmaf_read_pictures(s->vmaf, &ref_pic, &dist_pic, s->frame_count++);
    vmaf_picture_unref(&ref_pic);
    vmaf_picture_unref(&dist_pic);
    if (err)
        return AVERROR(EINVAL);

    err = vmaf_score_pooled(s->vmaf, s->model, s->pool_method,
                            &pooled_score, 0, s->frame_count - 1);
    if (err)
        return AVERROR(EINVAL);

    s->last_score = pooled_score;
    ost->target_vmaf_score = pooled_score;

    if (s->frame_count > 1)
        target_vmaf_update_qscale(ost, s, pooled_score);

    return 0;
}

static int target_vmaf_receive(OutputStream *ost, TargetVMAFState *s)
{
    int ret;

    while ((ret = avcodec_receive_frame(s->dec_ctx, s->dec_frame)) >= 0) {
        ret = target_vmaf_consume(ost, s, s->dec_frame);
        av_frame_unref(s->dec_frame);
        if (ret < 0)
            return ret;
    }

    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
        return ret;

    return ret;
}

static int target_vmaf_send(OutputStream *ost, TargetVMAFState *s,
                             const AVPacket *pkt)
{
    int ret;

    ret = avcodec_send_packet(s->dec_ctx, pkt);

    if (ret == AVERROR(EAGAIN)) {
        ret = target_vmaf_receive(ost, s);
        if (ret < 0 && ret != AVERROR_EOF)
            return ret;
        ret = avcodec_send_packet(s->dec_ctx, pkt);
    }

    if (ret == AVERROR_EOF)
        return 0;

    return ret;
}

int ff_target_vmaf_init(OutputStream *ost)
{
    TargetVMAFState *s = NULL;
    const AVCodec *dec;
    VmafConfiguration cfg = {
        .log_level  = VMAF_LOG_LEVEL_WARNING,
        .n_subsample = 1,
        .n_threads  = 0,
    };
    VmafModelConfig model_cfg = {
        .name  = "vmaf",
        .flags = VMAF_MODEL_FLAG_ENABLE_TRANSFORM,
    };
    AVCodecParameters *par = NULL;
    int ret = 0;

    if (ost->target_vmaf < 0.0)
        return 0;

    if (ost->type != AVMEDIA_TYPE_VIDEO)
        return AVERROR(EINVAL);

    s = av_mallocz(sizeof(*s));
    if (!s)
        return AVERROR(ENOMEM);

    s->target      = ost->target_vmaf;
    s->qscale      = TARGET_VMAF_DEFAULT_QSCALE;
    s->kp          = 0.15;
    s->ki          = 0.0025;
    s->pool_method = VMAF_POOL_METHOD_MEAN;

    s->ref_fifo = av_fifo_alloc2(32, sizeof(AVFrame*), AV_FIFO_FLAG_AUTO_GROW);
    if (!s->ref_fifo) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    dec = avcodec_find_decoder(ost->enc->enc_ctx->codec_id);
    if (!dec) {
        av_log(ost, AV_LOG_ERROR,
               "target VMAF controller requires a decoder for %s\n",
               avcodec_get_name(ost->enc->enc_ctx->codec_id));
        ret = AVERROR_DECODER_NOT_FOUND;
        goto fail;
    }

    s->dec_ctx = avcodec_alloc_context3(dec);
    if (!s->dec_ctx) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    par = avcodec_parameters_alloc();
    if (!par) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    ret = avcodec_parameters_from_context(par, ost->enc->enc_ctx);
    if (ret < 0)
        goto fail;

    ret = avcodec_parameters_to_context(s->dec_ctx, par);
    if (ret < 0)
        goto fail;

    avcodec_parameters_free(&par);

    s->dec_ctx->pkt_timebase = ost->enc->enc_ctx->time_base;

    ret = avcodec_open2(s->dec_ctx, dec, NULL);
    if (ret < 0)
        goto fail;

    s->dec_frame = av_frame_alloc();
    if (!s->dec_frame) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    ret = vmaf_init(&s->vmaf, cfg);
    if (ret) {
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = vmaf_model_load(&s->model, &model_cfg, "vmaf_v0.6.1");
    if (ret) {
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = vmaf_use_features_from_model(s->vmaf, s->model);
    if (ret) {
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ost->enc->enc_ctx->flags |= AV_CODEC_FLAG_QSCALE;
    ost->enc->enc_ctx->global_quality = lrint(FF_QP2LAMBDA * s->qscale);
    ost->target_vmaf_qscale = s->qscale;

    ost->target_vmaf_state = s;

    av_log(ost, AV_LOG_INFO,
           "target VMAF controller enabled (goal=%.2f).\n", s->target);

    return 0;

fail:
    avcodec_parameters_free(&par);
    if (s) {
        if (s->model)
            vmaf_model_destroy(s->model);
        if (s->vmaf)
            vmaf_close(s->vmaf);
        av_frame_free(&s->dec_frame);
        avcodec_free_context(&s->dec_ctx);
        target_vmaf_free_fifo(s);
        av_free(s);
    }
    return ret;
}

int ff_target_vmaf_store_frame(OutputStream *ost, const AVFrame *frame)
{
    TargetVMAFState *s = ost->target_vmaf_state;
    AVFrame *clone;

    if (!s || !frame)
        return 0;

    clone = av_frame_clone(frame);
    if (!clone)
        return AVERROR(ENOMEM);

    if (av_fifo_write(s->ref_fifo, &clone, 1) < 0) {
        av_frame_free(&clone);
        return AVERROR(ENOMEM);
    }

    return 0;
}

int ff_target_vmaf_process_packet(OutputStream *ost, const AVPacket *pkt)
{
    TargetVMAFState *s = ost->target_vmaf_state;
    int ret;

    if (!s)
        return 0;

    ret = target_vmaf_receive(ost, s);
    if (ret < 0 && ret != AVERROR_EOF)
        return ret;

    ret = target_vmaf_send(ost, s, pkt);
    if (ret < 0 && ret != AVERROR_EOF)
        return ret;

    ret = target_vmaf_receive(ost, s);
    if (ret == AVERROR_EOF)
        ret = 0;

    return ret;
}

int ff_target_vmaf_finalize(OutputStream *ost)
{
    TargetVMAFState *s = ost->target_vmaf_state;
    int ret;

    if (!s)
        return 0;

    ret = target_vmaf_receive(ost, s);
    if (ret < 0 && ret != AVERROR_EOF)
        return ret;

    ret = target_vmaf_send(ost, s, NULL);
    if (ret < 0 && ret != AVERROR_EOF)
        return ret;

    ret = target_vmaf_receive(ost, s);
    if (ret < 0 && ret != AVERROR_EOF)
        return ret;

    if (s->frame_count) {
        double score;
        int err;

        err = vmaf_read_pictures(s->vmaf, NULL, NULL, 0);
        if (!err)
            err = vmaf_score_pooled(s->vmaf, s->model, s->pool_method,
                                     &score, 0, s->frame_count - 1);
        if (!err)
            ost->target_vmaf_score = score;
    }

    if (ost->target_vmaf_score < 0.0) {
        av_log(ost, AV_LOG_WARNING,
               "target VMAF controller could not evaluate any frames.\n");
    } else if (ost->target_vmaf_score + 1.0 < s->target) {
        av_log(ost, AV_LOG_WARNING,
               "target VMAF %.2f not reached (achieved %.2f).\n",
               s->target, ost->target_vmaf_score);
    } else {
        av_log(ost, AV_LOG_INFO,
               "target VMAF %.2f achieved with %.2f.\n",
               s->target, ost->target_vmaf_score);
    }

    return 0;
}

void ff_target_vmaf_uninit(OutputStream *ost)
{
    TargetVMAFState *s = ost->target_vmaf_state;

    if (!s)
        return;

    target_vmaf_free_fifo(s);
    av_frame_free(&s->dec_frame);
    avcodec_free_context(&s->dec_ctx);
    if (s->model)
        vmaf_model_destroy(s->model);
    if (s->vmaf)
        vmaf_close(s->vmaf);

    av_freep(&ost->target_vmaf_state);
}

#endif /* CONFIG_LIBVMAF */
