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

#ifndef FFTOOLS_FFMPEG_VMAF_H
#define FFTOOLS_FFMPEG_VMAF_H

#include "config.h"

#include <errno.h>

#include "libavutil/error.h"

struct OutputStream;
struct AVFrame;
struct AVPacket;

#if CONFIG_LIBVMAF
int ff_target_vmaf_init(struct OutputStream *ost);
int ff_target_vmaf_store_frame(struct OutputStream *ost, const struct AVFrame *frame);
int ff_target_vmaf_process_packet(struct OutputStream *ost, const struct AVPacket *pkt);
int ff_target_vmaf_finalize(struct OutputStream *ost);
void ff_target_vmaf_uninit(struct OutputStream *ost);
#else
static inline int ff_target_vmaf_init(struct OutputStream *ost)
{
    (void)ost;
    return AVERROR(ENOSYS);
}
static inline int ff_target_vmaf_store_frame(struct OutputStream *ost, const struct AVFrame *frame)
{
    (void)ost;
    (void)frame;
    return 0;
}
static inline int ff_target_vmaf_process_packet(struct OutputStream *ost, const struct AVPacket *pkt)
{
    (void)ost;
    (void)pkt;
    return 0;
}
static inline int ff_target_vmaf_finalize(struct OutputStream *ost)
{
    (void)ost;
    return 0;
}
static inline void ff_target_vmaf_uninit(struct OutputStream *ost)
{
    (void)ost;
}
#endif

#endif /* FFTOOLS_FFMPEG_VMAF_H */
