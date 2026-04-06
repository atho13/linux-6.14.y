// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic DVB regmap configuration
 *
 * Copyright (C) 2026 Kağan Kadioğlu <kagankadioglutk@hotmail.com>
 */
#include <linux/regmap.h>
#include "amlogic_dvb.h"
#include "amlogic-dvb-regs.h"

/*
 * aml_dvb_volatile_reg — which registers will bypass cache
 *
 * CRITICAL: FM_WR_ADDR (woff 0x07) MUST be volatile for every demux.
 *   CPU writes 0x8000|addr → hardware clears BIT(15).
 *   With REGCACHE_RBTREE, read returns cached 0x8000 value
 *   → fm_wait_ready() waits forever (FM busy timeout).
 *
 * Same issue applies to STB_INT_STATUS (woff 0x23):
 *   Hardware sets bits, CPU clears them — cache goes stale.
 *
 * Solution: mark these offsets as volatile within the per-demux stride (0x140).
 */
static bool aml_dvb_volatile_reg(struct device *dev, unsigned int reg)
{
	u32 offset;

	/* STB_VERSION_O(0) — read-only */
	if (reg == 0x00)
		return true;

	/* WR_PTR registers — hardware writes */
	if (reg >= DEMUX_WR_PTR_START && reg < DEMUX_WR_PTR_END)
		return true;

	/*
	 * Per-demux volatile registers (stride = 0x140 = 320 bytes):
	 *   woff 0x06 = FM_WR_DATA   (0x18) — no read-after-write needed
	 *   woff 0x07 = FM_WR_ADDR   (0x1C) — BIT(15) cleared by hw
	 *   woff 0x0b = OM_CMD_STATUS (0x2C) — status register
	 *   woff 0x11 = SEC_BUFF_BUSY (0x44) — hw writes
	 *   woff 0x12 = SEC_BUFF_READY(0x48) — hw writes
	 *   woff 0x13 = SEC_BUFF_NUMBER(0x4C) — hw writes
	 *   woff 0x23 = STB_INT_STATUS(0x8C) — hw writes, cpu clears
	 */
	offset = reg % 0x140;
	switch (offset) {
	case 0x18: /* FM_WR_DATA */
	case 0x1C: /* FM_WR_ADDR   ← MOST CRITICAL */
	case 0x2C: /* OM_CMD_STATUS */
	case 0x44: /* SEC_BUFF_BUSY */
	case 0x48: /* SEC_BUFF_READY */
	case 0x4C: /* SEC_BUFF_NUMBER */
	case 0x8C: /* STB_INT_STATUS */
		return true;
	}

	return false;
}

/* Demux region regmap config */
const struct regmap_config aml_demux_regmap_config = {
	.name = "demux",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x1000,
	.writeable_reg = NULL,
	.volatile_reg = aml_dvb_volatile_reg,
	.cache_type = REGCACHE_RBTREE,
	.fast_io = true,
};

/*
 * Async FIFO region regmap config (registers are purely volatile)
 *
 * S905X3 physical layout (verified with dev/mem):
 *   base = 0xFFD09000 (async_fifo2), size = 0x2000
 *   FIFO[0] offset = 0x1000 (+0x00..+0x14) → 0xFFD0A000 (async_fifo)
 *   FIFO[1] offset = 0x0000 (+0x00..+0x14) → 0xFFD09000 (async_fifo2)
 *
 * max_register: last register = FIFO[0]+0x10 = 0x1010
 */
const struct regmap_config aml_async_regmap_config = {
	.name = "asyncfifo",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register =
		0x1FFC, /* 0x2000 region — FIFO[0] 0x1000..0x1014 inclusive */
	.cache_type = REGCACHE_NONE,
	.fast_io = true,
};
