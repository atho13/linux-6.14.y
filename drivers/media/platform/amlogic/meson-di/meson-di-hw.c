// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Amlogic GX SoC Video Deinterlacer (DI) hardware programming
 *
 * Copyright (C) 2026 Christian Hewitt <christianshewitt@gmail.com>
 *
 * The deinterlacer runs a two stage pipeline. The "pre" stage reads the two
 * interlaced fields, runs noise reduction and per-pixel motion detection and
 * writes a motion-information buffer to memory. The "post" stage reads the
 * fields together with the motion information and blends them into a
 * progressive frame, run once per output field so that two progressive
 * frames are produced for each interlaced input.
 *
 * Only the core motion-adaptive path is implemented: film-mode/pulldown
 * cadence detection, 3D detection, AFBC compression and the tunable noise
 * reduction coefficients of the vendor driver are left at their defaults.
 */

#include <linux/bitops.h>
#include <linux/io.h>

#include <linux/soc/amlogic/meson-canvas.h>

#include <media/videobuf2-dma-contig.h>

#include "meson-di.h"
#include "meson-di-regs.h"

#define DI_HOLD_LINE		10
#define DI_FIFO_SIZE		0xc0

static void meson_di_config_canvas(struct meson_di *di, unsigned int idx,
				   dma_addr_t addr, u32 stride, u32 height)
{
	meson_canvas_config(di->canvas, di->canvas_idx[idx], addr, stride,
			    height, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    MESON_CANVAS_ENDIAN_SWAP64);
}

void meson_di_hw_init(struct meson_di *di)
{
	u32 clkg = DI_CLKG_CTRL_GXL;

	if (di->match_data && di->match_data->hw_version == 2)
		clkg = DI_CLKG_CTRL_GXBB;

	/* Ungate the DI clocks. */
	di_write(di, DI_CLKG_CTRL, clkg);

	/* Enable all DI arbiter ports. */
	di_update_bits(di, DI_ARB_CTRL, 0xf0f << 16, 0xf0f << 16);

	/* Program the read-MIF FIFO sizes. */
	di_write(di, DI_INP_LUMA_FIFO_SIZE, DI_FIFO_SIZE);
	di_write(di, DI_MEM_LUMA_FIFO_SIZE, DI_FIFO_SIZE);
	di_write(di, DI_IF1_LUMA_FIFO_SIZE, DI_FIFO_SIZE);
	di_write(di, VD1_IF0_LUMA_FIFO_SIZE, DI_FIFO_SIZE);
}

void meson_di_hw_disable(struct meson_di *di)
{
	di_write(di, DI_PRE_CTRL, 0);
	di_write(di, DI_POST_CTRL, 0);
	di_write(di, DI_CLKG_CTRL, DI_CLKG_CTRL_OFF);
}

/*
 * Halt the pipeline and mask/clear all interrupt sources. Used to fence the
 * hardware when a job is torn down mid-flight.
 */
void meson_di_hw_stop(struct meson_di *di)
{
	di_write(di, DI_PRE_CTRL, 0);
	di_write(di, DI_POST_CTRL, 0);
	di_write(di, DI_INTR_CTRL,
		 (GENMASK(9, 0) << DI_INTR_MASK_SHIFT) | GENMASK(15, 0));
}

/*
 * Program the frame geometry and interrupt routing. Called once per stream
 * when the format is known.
 */
void meson_di_hw_setup(struct meson_di_ctx *ctx)
{
	struct meson_di *di = ctx->di;
	u32 width = ctx->in.width;
	u32 height = ctx->in.height;
	u32 field_height = height / 2;
	u32 mask;

	/* Pre operates on a single field, post on the full progressive frame. */
	di_write(di, DI_PRE_SIZE, (width - 1) | ((field_height - 1) << 16));
	di_write(di, DI_POST_SIZE, (width - 1) | ((height - 1) << 16));

	/* Blend window covers the whole frame. */
	di_write(di, DI_BLEND_REG0_X, (width - 1));
	di_write(di, DI_BLEND_REG0_Y, (height - 1));
	di_write(di, DI_BLEND_REG1_X, (width - 1));
	di_write(di, DI_BLEND_REG1_Y, (height - 1));
	di_write(di, DI_BLEND_REG2_X, (width - 1));
	di_write(di, DI_BLEND_REG2_Y, (height - 1));
	di_write(di, DI_BLEND_REG3_X, (width - 1));
	di_write(di, DI_BLEND_REG3_Y, (height - 1));

	/*
	 * Unmask the NRWR, MTNWR (pre) and DIWR (post) completion sources and
	 * mask everything else. The low half-word is write-1-to-clear, so
	 * clear any pending status at the same time.
	 */
	mask = GENMASK(9, 0) << DI_INTR_MASK_SHIFT;
	mask &= ~((DI_INTR_NRWR_DONE | DI_INTR_MTNWR_DONE | DI_INTR_DIWR_DONE)
		  << DI_INTR_MASK_SHIFT);
	di_write(di, DI_INTR_CTRL, mask | GENMASK(15, 0));

	/* Route the post blend result into the DI write-back path (not VPP). */
	di_update_bits(di, VIU_MISC_CTRL0, 0x7 << 16, 0x5 << 16);
	di_update_bits(di, VIU_MISC_CTRL0, BIT(28), BIT(28));
}

/*
 * Program a read MIF that consumes one interlaced field of an NV12 buffer.
 * @phase selects the top (0) or bottom (1) field lines.
 */
static void meson_di_set_read_mif(struct meson_di *di, u16 gen, u16 canvas,
				  u16 luma_x, u16 luma_y, u16 chroma_x,
				  u16 chroma_y, u8 canvas_y, u8 canvas_c,
				  u32 width, u32 field_height, u8 phase)
{
	/*
	 * NV12 semi-planar, separate luma/chroma canvases, little-endian,
	 * bursts of 3/1/1 for y/cb/cr, hold-line as configured. Bit0 (enable)
	 * is left clear here and set when the pipeline is kicked.
	 */
	di_write(di, gen, (DI_HOLD_LINE << 19) | (3 << 8) | (1 << 10) |
		 (1 << 12) | (1 << 25) | (phase << 4));
	di_write(di, canvas, canvas_y | (canvas_c << 8));

	di_write(di, luma_x, (width - 1) << 16);
	di_write(di, luma_y, (field_height - 1) << 16);
	di_write(di, chroma_x, (width / 2 - 1) << 16);
	di_write(di, chroma_y, (field_height / 2 - 1) << 16);
}

static void meson_di_set_wr_mif(struct meson_di *di, u16 x, u16 y, u16 ctrl,
				u8 canvas_y, u8 canvas_c, u32 width,
				u32 height)
{
	di_write(di, x, width - 1);
	di_write(di, y, (3u << 30) | BIT(15) | (height - 1));
	di_write(di, ctrl, canvas_y | (canvas_c << 8) | (2 << 26));
}

void meson_di_hw_pre(struct meson_di_ctx *ctx, struct vb2_v4l2_buffer *src)
{
	struct meson_di *di = ctx->di;
	dma_addr_t src_y = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);
	dma_addr_t src_c = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 1);
	u32 width = ctx->in.width;
	u32 height = ctx->in.height;
	u32 field_height = height / 2;
	u32 stride = ctx->in.plane_fmt[0].bytesperline;
	bool bottom_first = ctx->field == V4L2_FIELD_INTERLACED_BT;

	/* Source planes: full-frame canvases, field selection is done by the MIF. */
	meson_di_config_canvas(di, MESON_DI_CANVAS_SRC_Y, src_y, stride,
			       height);
	meson_di_config_canvas(di, MESON_DI_CANVAS_SRC_C, src_c, stride,
			       height / 2);
	/* NR write-back scratch (denoised current field). */
	meson_di_config_canvas(di, MESON_DI_CANVAS_NR_Y, di->nr_dma, stride,
			       field_height);
	meson_di_config_canvas(di, MESON_DI_CANVAS_NR_C,
			       di->nr_dma + stride * field_height, stride,
			       field_height / 2);
	/* Motion information buffer. */
	meson_di_config_canvas(di, MESON_DI_CANVAS_MTN, di->mtn_dma, width,
			       field_height);

	/* Current field read MIF. */
	meson_di_set_read_mif(di, DI_INP_GEN_REG, DI_INP_CANVAS0,
			      DI_INP_LUMA_X0, DI_INP_LUMA_Y0,
			      DI_INP_CHROMA_X0, DI_INP_CHROMA_Y0,
			      di->canvas_idx[MESON_DI_CANVAS_SRC_Y],
			      di->canvas_idx[MESON_DI_CANVAS_SRC_C],
			      width, field_height, 0);
	/* NV12 semi-planar select. */
	di_write(di, DI_INP_GEN_REG2, 1);

	/* Opposite field read MIF (memory). */
	meson_di_set_read_mif(di, DI_MEM_GEN_REG, DI_MEM_CANVAS0,
			      DI_MEM_LUMA_X0, DI_MEM_LUMA_Y0,
			      DI_MEM_CHROMA_X0, DI_MEM_CHROMA_Y0,
			      di->canvas_idx[MESON_DI_CANVAS_SRC_Y],
			      di->canvas_idx[MESON_DI_CANVAS_SRC_C],
			      width, field_height, 1);

	/* NR and motion write-back MIFs. */
	meson_di_set_wr_mif(di, DI_NRWR_X, DI_NRWR_Y, DI_NRWR_CTRL,
			    di->canvas_idx[MESON_DI_CANVAS_NR_Y],
			    di->canvas_idx[MESON_DI_CANVAS_NR_C],
			    width, field_height);
	di_write(di, DI_MTNWR_X, (width - 1) << 16);
	di_write(di, DI_MTNWR_Y, (field_height - 1) << 16);
	di_write(di, DI_MTNWR_CTRL, di->canvas_idx[MESON_DI_CANVAS_MTN]);

	/* Motion detection uses madi mode 5. */
	di_update_bits(di, DI_MTN_1_CTRL1, 0x7 << 29, 0x5 << 29);

	/*
	 * Run noise reduction (default pass-through coefficients) and motion
	 * detection. Bit29 selects which field is read first.
	 */
	di_write(di, DI_PRE_CTRL,
		 DI_PRE_CTRL_NR_EN | DI_PRE_CTRL_MTN_EN |
		 DI_PRE_CTRL_MTN_AFTER_NR |
		 DI_PRE_CTRL_HOLD_LINE(DI_HOLD_LINE) |
		 (bottom_first ? DI_PRE_CTRL_FIELD_NUM : 0));

	/* Enable the read MIFs. */
	di_update_bits(di, DI_INP_GEN_REG, DI_MIF_GEN_REG_EN, DI_MIF_GEN_REG_EN);
	di_update_bits(di, DI_MEM_GEN_REG, DI_MIF_GEN_REG_EN, DI_MIF_GEN_REG_EN);

	/* Kick the pre stage (frame + soft reset). */
	di_update_bits(di, DI_PRE_CTRL,
		       DI_PRE_CTRL_FRAME_RST | DI_PRE_CTRL_SOFT_RST,
		       DI_PRE_CTRL_FRAME_RST | DI_PRE_CTRL_SOFT_RST);
}

void meson_di_hw_post(struct meson_di_ctx *ctx, struct vb2_v4l2_buffer *dst,
		      bool bottom_field)
{
	struct meson_di *di = ctx->di;
	dma_addr_t dst_y = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	dma_addr_t dst_c = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 1);
	u32 width = ctx->in.width;
	u32 height = ctx->in.height;
	u32 field_height = height / 2;
	u32 stride = ctx->out.plane_fmt[0].bytesperline;

	/* Output planes. */
	meson_di_config_canvas(di, MESON_DI_CANVAS_OUT_Y, dst_y, stride,
			       height);
	meson_di_config_canvas(di, MESON_DI_CANVAS_OUT_C, dst_c, stride,
			       height / 2);

	/* Current field (IF0, on GX the shared VD1 video plane MIF). */
	meson_di_set_read_mif(di, VD1_IF0_GEN_REG, VD1_IF0_CANVAS0,
			      VD1_IF0_LUMA_X0, VD1_IF0_LUMA_Y0,
			      VD1_IF0_CHROMA_X0, VD1_IF0_CHROMA_Y0,
			      di->canvas_idx[MESON_DI_CANVAS_SRC_Y],
			      di->canvas_idx[MESON_DI_CANVAS_SRC_C],
			      width, field_height, bottom_field ? 1 : 0);
	/* Opposite field (IF1). */
	meson_di_set_read_mif(di, DI_IF1_GEN_REG, DI_IF1_CANVAS0,
			      DI_IF1_LUMA_X0, DI_IF1_LUMA_Y0,
			      DI_IF1_CHROMA_X0, DI_IF1_CHROMA_Y0,
			      di->canvas_idx[MESON_DI_CANVAS_SRC_Y],
			      di->canvas_idx[MESON_DI_CANVAS_SRC_C],
			      width, field_height, bottom_field ? 0 : 1);

	/* Motion read MIF. */
	di_write(di, DI_MTNPRD_X, (width - 1) << 16);
	di_write(di, DI_MTNPRD_Y, (field_height - 1) << 16);
	di_write(di, DI_MTNRD_CTRL, di->canvas_idx[MESON_DI_CANVAS_MTN] << 8);

	/* Progressive output write-back MIF. */
	meson_di_set_wr_mif(di, DI_DIWR_X, DI_DIWR_Y, DI_DIWR_CTRL,
			    di->canvas_idx[MESON_DI_CANVAS_OUT_Y],
			    di->canvas_idx[MESON_DI_CANVAS_OUT_C],
			    width, height);
	di_update_bits(di, DI_DIWR_CTRL, BIT(30), BIT(30));

	/* Motion-adaptive blend. */
	di_write(di, DI_BLEND_CTRL, DI_BLEND_CTRL_EN | DI_BLEND_CTRL_FIX(7) |
		 DI_BLEND_CTRL_MODE(1));

	/* Enable the read MIFs. */
	di_update_bits(di, VD1_IF0_GEN_REG, DI_MIF_GEN_REG_EN,
		       DI_MIF_GEN_REG_EN);
	di_update_bits(di, DI_IF1_GEN_REG, DI_MIF_GEN_REG_EN,
		       DI_MIF_GEN_REG_EN);

	/*
	 * Program and kick the post stage. Output goes to DDR (bit7), not VPP.
	 * Bit29 selects the output field, bits[31:30] trigger the pass.
	 */
	di_write(di, DI_POST_CTRL,
		 DI_POST_CTRL_LBUF0_EN | DI_POST_CTRL_EI_EN |
		 DI_POST_CTRL_MTN_LBUF_EN | DI_POST_CTRL_MTNP_RD_EN |
		 DI_POST_CTRL_BLEND_EN | DI_POST_CTRL_MUX_EN |
		 DI_POST_CTRL_DDR_EN |
		 DI_POST_CTRL_HOLD_LINE(DI_HOLD_LINE) |
		 (bottom_field ? DI_POST_CTRL_FIELD_NUM : 0) |
		 DI_POST_CTRL_FRAME_RST | DI_POST_CTRL_SOFT_RST);
}

bool meson_di_hw_pre_done(struct meson_di *di)
{
	u32 val = di_read(di, DI_INTR_CTRL);
	u32 pre = val & (DI_INTR_NRWR_DONE | DI_INTR_MTNWR_DONE);

	if (!pre)
		return false;

	/* Acknowledge whichever pre sources fired (write-1-to-clear). */
	di_write(di, DI_INTR_CTRL, val & ~DI_INTR_DIWR_DONE);

	/*
	 * The post stage consumes the motion buffer, so the pre stage is only
	 * complete once motion write-back (MTNWR) has finished. A lone NRWR
	 * completion is acknowledged above and we wait for MTNWR.
	 */
	if (!(pre & DI_INTR_MTNWR_DONE))
		return false;

	/* Disable the pre read MIFs. */
	di_update_bits(di, DI_INP_GEN_REG, DI_MIF_GEN_REG_EN, 0);
	di_update_bits(di, DI_MEM_GEN_REG, DI_MIF_GEN_REG_EN, 0);

	return true;
}

bool meson_di_hw_post_done(struct meson_di *di)
{
	u32 val = di_read(di, DI_INTR_CTRL);

	if (!(val & DI_INTR_DIWR_DONE))
		return false;

	/* Acknowledge the post source and drop the DDR write enable. */
	di_write(di, DI_INTR_CTRL, (val & GENMASK(31, 16)) | DI_INTR_DIWR_DONE);
	di_update_bits(di, DI_POST_CTRL, DI_POST_CTRL_DDR_EN, 0);

	/* Disable the post read MIFs. */
	di_update_bits(di, VD1_IF0_GEN_REG, DI_MIF_GEN_REG_EN, 0);
	di_update_bits(di, DI_IF1_GEN_REG, DI_MIF_GEN_REG_EN, 0);

	return true;
}
