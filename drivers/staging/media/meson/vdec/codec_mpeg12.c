// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "codec_mpeg12.h"
#include "dos_regs.h"
#include "vdec_helpers.h"

#define SIZE_WORKSPACE		SZ_128K
/* Offset substracted by the firmware from the workspace paddr */
#define WORKSPACE_OFFSET	(5 * SZ_1K)

/* map firmware registers to known MPEG1/2 functions */
#define MREG_SEQ_INFO		AV_SCRATCH_4
	#define MPEG2_SEQ_DAR_MASK	GENMASK(3, 0)
	#define MPEG2_DAR_4_3		2
	#define MPEG2_DAR_16_9		3
	#define MPEG2_DAR_221_100	4
#define MREG_PIC_INFO		AV_SCRATCH_5
#define MREG_PIC_WIDTH		AV_SCRATCH_6
#define MREG_PIC_HEIGHT		AV_SCRATCH_7
#define MREG_BUFFERIN		AV_SCRATCH_8
#define MREG_BUFFEROUT		AV_SCRATCH_9
#define MREG_CMD		AV_SCRATCH_A
#define MREG_CO_MV_START	AV_SCRATCH_B
#define MREG_ERROR_COUNT	AV_SCRATCH_C
#define MREG_FRAME_OFFSET	AV_SCRATCH_D
#define MREG_WAIT_BUFFER	AV_SCRATCH_E
#define MREG_FATAL_ERROR	AV_SCRATCH_F

#define PICINFO_PROG		0x00008000
#define PICINFO_TOP_FIRST	0x00002000
#define PICINFO_TYPE_MASK	GENMASK(2, 0)
#define PICINFO_TYPE_I		1
#define PICINFO_TYPE_P		2
#define PICINFO_TYPE_B		3

struct codec_mpeg12 {
	/* Buffer for the MPEG1/2 Workspace */
	void	  *workspace_vaddr;
	dma_addr_t workspace_paddr;
};

static const u8 eos_sequence[SZ_1K] = { 0x00, 0x00, 0x01, 0xB7 };

static const u8 *codec_mpeg12_eos_sequence(u32 *len)
{
	*len = ARRAY_SIZE(eos_sequence);
	return eos_sequence;
}

static int codec_mpeg12_can_recycle(struct amvdec_core *core)
{
	return !amvdec_read_dos(core, MREG_BUFFERIN);
}

static void codec_mpeg12_recycle(struct amvdec_core *core, u32 buf_idx)
{
	amvdec_write_dos(core, MREG_BUFFERIN, buf_idx + 1);
}

static int codec_mpeg12_start(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg12 *mpeg12;
	int ret;

	mpeg12 = kzalloc_obj(*mpeg12);
	if (!mpeg12)
		return -ENOMEM;

	/* Allocate some memory for the MPEG1/2 decoder's state */
	mpeg12->workspace_vaddr = dma_alloc_coherent(core->dev, SIZE_WORKSPACE,
						     &mpeg12->workspace_paddr,
						     GFP_KERNEL);
	if (!mpeg12->workspace_vaddr) {
		dev_err(core->dev, "Failed to request MPEG 1/2 Workspace\n");
		ret = -ENOMEM;
		goto free_mpeg12;
	}

	/*
	 * Configure canvases with the initial resolution so the firmware
	 * has valid output buffers from the start. Unlike H.264, the
	 * MPEG1/2 firmware begins decoding immediately without pausing
	 * for resolution detection. Canvases are reconfigured in
	 * resume() if the actual stream resolution differs.
	 */
	ret = amvdec_set_canvases(sess, (u32[]){ AV_SCRATCH_0, 0 },
					(u32[]){ 6, 0 });
	if (ret)
		goto free_workspace;

	amvdec_write_dos(core, POWER_CTL_VLD, BIT(4));
	amvdec_write_dos(core, MREG_CO_MV_START,
			 mpeg12->workspace_paddr + WORKSPACE_OFFSET);

	amvdec_write_dos(core, MPEG1_2_REG, 0);
	amvdec_write_dos(core, PSCALE_CTRL, 0);
	amvdec_write_dos(core, PIC_HEAD_INFO, 0x380);
	amvdec_write_dos(core, M4_CONTROL_REG, 0);
	amvdec_write_dos(core, MREG_BUFFERIN, 0);
	amvdec_write_dos(core, MREG_BUFFEROUT, 0);
	amvdec_write_dos(core, MREG_CMD, (sess->width << 16) | sess->height);
	amvdec_write_dos(core, MREG_ERROR_COUNT, 0);
	amvdec_write_dos(core, MREG_FATAL_ERROR, 0);
	amvdec_write_dos(core, MREG_WAIT_BUFFER, 0);

	sess->priv = mpeg12;

	return 0;

free_workspace:
	dma_free_coherent(core->dev, SIZE_WORKSPACE, mpeg12->workspace_vaddr,
			  mpeg12->workspace_paddr);
free_mpeg12:
	kfree(mpeg12);
	return ret;
}

static void codec_mpeg12_resume(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	/*
	 * Canvas entries are written to consecutive AV_SCRATCH registers
	 * starting at AV_SCRATCH_0. Limit to 6 entries so we do not
	 * overwrite AV_SCRATCH_6 (MREG_PIC_WIDTH) and AV_SCRATCH_7
	 * (MREG_PIC_HEIGHT) which the firmware uses to report resolution.
	 */
	amvdec_set_canvases(sess, (u32[]){ AV_SCRATCH_0, 0 },
				  (u32[]){ 6, 0 });
	amvdec_write_dos(core, MREG_CMD, (sess->width << 16) | sess->height);
}

static int codec_mpeg12_stop(struct amvdec_session *sess)
{
	struct codec_mpeg12 *mpeg12 = sess->priv;
	struct amvdec_core *core = sess->core;

	if (mpeg12->workspace_vaddr)
		dma_free_coherent(core->dev, SIZE_WORKSPACE,
				  mpeg12->workspace_vaddr,
				  mpeg12->workspace_paddr);

	return 0;
}

static void codec_mpeg12_update_dar(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	u32 seq = amvdec_read_dos(core, MREG_SEQ_INFO);
	u32 ar = seq & MPEG2_SEQ_DAR_MASK;

	switch (ar) {
	case MPEG2_DAR_4_3:
		amvdec_set_par_from_dar(sess, 4, 3);
		break;
	case MPEG2_DAR_16_9:
		amvdec_set_par_from_dar(sess, 16, 9);
		break;
	case MPEG2_DAR_221_100:
		amvdec_set_par_from_dar(sess, 221, 100);
		break;
	default:
		sess->pixelaspect.numerator = 1;
		sess->pixelaspect.denominator = 1;
		break;
	}
}

static void codec_mpeg12_src_change(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	u32 width = amvdec_read_dos(core, MREG_PIC_WIDTH);
	u32 height = amvdec_read_dos(core, MREG_PIC_HEIGHT);

	if (!width || !height)
		return;

	codec_mpeg12_update_dar(sess);
	amvdec_src_change(sess, width, height, 8);
}

static irqreturn_t codec_mpeg12_threaded_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	u32 reg;
	u32 pic_info;
	u32 is_progressive;
	u32 buffer_index;
	u32 field = V4L2_FIELD_NONE;
	u32 pic_type;
	u32 type = 0;
	u32 offset;
	u32 width;
	u32 height;

	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);
	reg = amvdec_read_dos(core, MREG_FATAL_ERROR);
	if (reg == 1) {
		dev_err(core->dev, "MPEG1/2 fatal error\n");
		amvdec_abort(sess);
		return IRQ_HANDLED;
	}

	/* Acknowledge any buffer-wait request from firmware */
	if (amvdec_read_dos(core, MREG_WAIT_BUFFER))
		amvdec_write_dos(core, MREG_WAIT_BUFFER, 0);

	reg = amvdec_read_dos(core, MREG_BUFFEROUT);
	if (!reg)
		return IRQ_HANDLED;

	/* Unclear what this means */
	if ((reg & GENMASK(23, 17)) == GENMASK(23, 17))
		goto end;

	buffer_index = ((reg & 0xf) - 1) & 7;

	/*
	 * When the decoder has not yet started (STATUS_INIT), read the
	 * firmware-reported resolution to trigger the initial source
	 * change event.  We can only read MREG_PIC_WIDTH/HEIGHT here
	 * because after resume, amvdec_set_canvases() writes canvas
	 * entries into the same AV_SCRATCH register range, overwriting
	 * those firmware registers.
	 *
	 * For broadcast streams the sequence header may not be in the
	 * first packets, so width/height may be zero initially — just
	 * recycle the buffer and let the firmware keep parsing.
	 */
	if (sess->status == STATUS_INIT) {
		width = amvdec_read_dos(core, MREG_PIC_WIDTH);
		height = amvdec_read_dos(core, MREG_PIC_HEIGHT);
		if (width && height)
			codec_mpeg12_src_change(sess);
		goto recycle;
	}

	/*
	 * Frames arriving before the capture queue is running (e.g.
	 * between source change and userspace configuring capture)
	 * cannot be delivered. Recycle the firmware buffer so the
	 * decoder does not stall waiting for output buffers.
	 */
	if (sess->status != STATUS_RUNNING)
		goto recycle;

	pic_info = amvdec_read_dos(core, MREG_PIC_INFO);
	is_progressive = pic_info & PICINFO_PROG;

	if (!is_progressive)
		field = (pic_info & PICINFO_TOP_FIRST) ?
			V4L2_FIELD_INTERLACED_TB :
			V4L2_FIELD_INTERLACED_BT;

	pic_type = pic_info & PICINFO_TYPE_MASK;
	if (pic_type == PICINFO_TYPE_I)
		type = 1;
	else if (pic_type == PICINFO_TYPE_P)
		type = 2;
	else if (pic_type == PICINFO_TYPE_B)
		type = 3;

	codec_mpeg12_update_dar(sess);
	offset = amvdec_read_dos(core, MREG_FRAME_OFFSET);
	amvdec_dst_buf_done_idx(sess, buffer_index, offset, field, type);
	goto end;

recycle:
	/*
	 * Return this buffer to the firmware so it can keep decoding.
	 * MREG_BUFFERIN accepts one index at a time; if it is busy the
	 * firmware will reclaim the buffer via MREG_WAIT_BUFFER later.
	 */
	if (codec_mpeg12_can_recycle(core))
		codec_mpeg12_recycle(core, buffer_index);
end:
	amvdec_write_dos(core, MREG_BUFFEROUT, 0);
	return IRQ_HANDLED;
}

static irqreturn_t codec_mpeg12_isr(struct amvdec_session *sess)
{
	return IRQ_WAKE_THREAD;
}

struct amvdec_codec_ops codec_mpeg12_ops = {
	.start = codec_mpeg12_start,
	.stop = codec_mpeg12_stop,
	.isr = codec_mpeg12_isr,
	.threaded_isr = codec_mpeg12_threaded_isr,
	.can_recycle = codec_mpeg12_can_recycle,
	.recycle = codec_mpeg12_recycle,
	.resume = codec_mpeg12_resume,
	.eos_sequence = codec_mpeg12_eos_sequence,
};
