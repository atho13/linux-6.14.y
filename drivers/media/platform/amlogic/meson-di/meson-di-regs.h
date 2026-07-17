/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Amlogic GX SoC Video Deinterlacer (DI) register definitions
 *
 * Copyright (C) 2026 Christian Hewitt <christianshewitt@gmail.com>
 *
 * Register offsets are VCBUS word offsets, matching the vendor register map.
 * The accessors in meson-di.h rebase them onto the mapped register window.
 */

#ifndef __MESON_DI_REGS_H
#define __MESON_DI_REGS_H

/* Top-level control */
#define DI_PRE_CTRL			0x1700
#define DI_POST_CTRL			0x1701
#define DI_POST_SIZE			0x1702
#define DI_PRE_SIZE			0x1703
#define DI_CANVAS_URGENT0		0x170a
#define DI_BLEND_CTRL			0x170d
#define DI_ARB_CTRL			0x170f
#define DI_CLKG_CTRL			0x1718
#define DI_INTR_CTRL			0x1730
#define DI_MTN_1_CTRL1			0x1740

/* Blend window */
#define DI_BLEND_REG0_X			0x1710
#define DI_BLEND_REG0_Y			0x1711
#define DI_BLEND_REG1_X			0x1712
#define DI_BLEND_REG1_Y			0x1713
#define DI_BLEND_REG2_X			0x1714
#define DI_BLEND_REG2_Y			0x1715
#define DI_BLEND_REG3_X			0x1716
#define DI_BLEND_REG3_Y			0x1717

/* Simple write-back MIFs (canvas index carried in the CTRL register) */
#define DI_NRWR_X			0x17c0
#define DI_NRWR_Y			0x17c1
#define DI_NRWR_CTRL			0x17c2
#define DI_MTNWR_X			0x17c3
#define DI_MTNWR_Y			0x17c4
#define DI_MTNWR_CTRL			0x17c5
#define DI_DIWR_X			0x17c6
#define DI_DIWR_Y			0x17c7
#define DI_DIWR_CTRL			0x17c8
#define DI_MTNPRD_X			0x17cb
#define DI_MTNPRD_Y			0x17cc
#define DI_MTNRD_CTRL			0x17cd

/* Input (current field) read MIF */
#define DI_INP_GEN_REG			0x17ce
#define DI_INP_CANVAS0			0x17cf
#define DI_INP_LUMA_X0			0x17d0
#define DI_INP_LUMA_Y0			0x17d1
#define DI_INP_CHROMA_X0		0x17d2
#define DI_INP_CHROMA_Y0		0x17d3
#define DI_INP_RPT_LOOP			0x17d4
#define DI_INP_LUMA0_RPT_PAT		0x17d5
#define DI_INP_CHROMA0_RPT_PAT		0x17d6
#define DI_INP_DUMMY_PIXEL		0x17d7
#define DI_INP_LUMA_FIFO_SIZE		0x17d8
#define DI_INP_FMT_CTRL			0x17d9
#define DI_INP_FMT_W			0x17da
#define DI_INP_GEN_REG2			0x1791
#define DI_INP_GEN_REG3			0x20a8

/* Memory (opposite field) read MIF */
#define DI_MEM_GEN_REG			0x17db
#define DI_MEM_CANVAS0			0x17dc
#define DI_MEM_LUMA_X0			0x17dd
#define DI_MEM_LUMA_Y0			0x17de
#define DI_MEM_CHROMA_X0		0x17df
#define DI_MEM_CHROMA_Y0		0x17e0
#define DI_MEM_LUMA_FIFO_SIZE		0x17e5
#define DI_MEM_FMT_W			0x17e7

/* IF1 (post opposite field) read MIF */
#define DI_IF1_GEN_REG			0x17e8
#define DI_IF1_CANVAS0			0x17e9
#define DI_IF1_LUMA_X0			0x17ea
#define DI_IF1_LUMA_Y0			0x17eb
#define DI_IF1_CHROMA_X0		0x17ec
#define DI_IF1_CHROMA_Y0		0x17ed
#define DI_IF1_RPT_LOOP			0x17ee
#define DI_IF1_LUMA0_RPT_PAT		0x17ef
#define DI_IF1_CHROMA0_RPT_PAT		0x17f0
#define DI_IF1_DUMMY_PIXEL		0x17f1
#define DI_IF1_LUMA_FIFO_SIZE		0x17f2
#define DI_IF1_FMT_CTRL			0x17f3
#define DI_IF1_FMT_W			0x17f4

/*
 * IF0 (post current field) read MIF. On GX this is physically the VD1 video
 * plane input MIF, which is shared with the display pipeline.
 */
#define VD1_IF0_GEN_REG			0x1a50
#define VD1_IF0_CANVAS0			0x1a52
#define VD1_IF0_LUMA_X0			0x1a54
#define VD1_IF0_LUMA_Y0			0x1a55
#define VD1_IF0_CHROMA_X0		0x1a56
#define VD1_IF0_CHROMA_Y0		0x1a57
#define VD1_IF0_RPT_LOOP		0x1a5b
#define VD1_IF0_LUMA0_RPT_PAT		0x1a5c
#define VD1_IF0_CHROMA0_RPT_PAT		0x1a5d
#define VD1_IF0_LUMA_FIFO_SIZE		0x1a63
#define VD1_IF0_GEN_REG2		0x1a6d
#define VD1_IF0_FMT_CTRL		0x1a68
#define VD1_IF0_FMT_W			0x1a69
#define VD1_IF0_LUMA_FIFO_SIZE_DEF	0xc0

/* Global VIU routing (shared with the display pipeline) */
#define VIU_MISC_CTRL0			0x1a06

/* DI_CLKG_CTRL values */
#define DI_CLKG_CTRL_GXBB		0x00000001
#define DI_CLKG_CTRL_GXL		0xffff0001
#define DI_CLKG_CTRL_OFF		0x00000002

/* DI_PRE_CTRL bits */
#define DI_PRE_CTRL_NR_EN		BIT(0)
#define DI_PRE_CTRL_MTN_EN		BIT(1)
#define DI_PRE_CTRL_MTN_AFTER_NR	BIT(22)
#define DI_PRE_CTRL_HOLD_LINE(x)	((x) << 16)
#define DI_PRE_CTRL_FIELD_NUM		BIT(29)
#define DI_PRE_CTRL_FRAME_RST		BIT(30)
#define DI_PRE_CTRL_SOFT_RST		BIT(31)

/* DI_POST_CTRL bits */
#define DI_POST_CTRL_LBUF0_EN		BIT(0)
#define DI_POST_CTRL_EI_EN		BIT(2)
#define DI_POST_CTRL_MTN_LBUF_EN	BIT(3)
#define DI_POST_CTRL_MTNP_RD_EN		BIT(4)
#define DI_POST_CTRL_BLEND_EN		BIT(5)
#define DI_POST_CTRL_MUX_EN		BIT(6)
#define DI_POST_CTRL_DDR_EN		BIT(7)
#define DI_POST_CTRL_VPP_EN		BIT(8)
#define DI_POST_CTRL_HOLD_LINE(x)	((x) << 16)
#define DI_POST_CTRL_FIELD_NUM		BIT(29)
#define DI_POST_CTRL_FRAME_RST		BIT(30)
#define DI_POST_CTRL_SOFT_RST		BIT(31)

/* DI_BLEND_CTRL bits */
#define DI_BLEND_CTRL_MODE(x)		((x) << 20)
#define DI_BLEND_CTRL_FIX(x)		((x) << 22)
#define DI_BLEND_CTRL_EN		BIT(31)

/* DI_INTR_CTRL bits (low half is write-1-to-clear, high half masks) */
#define DI_INTR_NRWR_DONE		BIT(0)
#define DI_INTR_MTNWR_DONE		BIT(1)
#define DI_INTR_DIWR_DONE		BIT(2)
#define DI_INTR_MASK_SHIFT		16

/* MIF GEN_REG common bits */
#define DI_MIF_GEN_REG_EN		BIT(0)

#endif /* __MESON_DI_REGS_H */
