/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Amlogic DVB/TS hardware register definitions
 *
 * Copyright (C) 2026 Kağan Kadioğlu <kagankadioglutk@hotmail.com>
 */
#ifndef __AML_DVB_REGS_H
#define __AML_DVB_REGS_H

#include <linux/bitfield.h>
#include <linux/bits.h>

/*
 * Amlogic DVB/TS Hardware Registers
 *
 * CRITICAL: All offsets are in BYTES!
 * Demux registers: word-offset × 4 = byte address.
 * Demux stride: 0x140 (320 bytes) – reserved region per demux.
 *
 * PHYSICAL ADDRESS MAP (S905X3, verified with dev/mem):
 *   0xffd06000  = TS_IN / S2P / TOP_CONFIG  ("ts" DTS region)
 *   0xff638000  = DEMUX core + ASYNC FIFO   ("demux" DTS region)
 *
 * AML_TS_REG() is used to distinguish TS registers from DEMUX registers.
 * aml_write_reg()/aml_read_reg() check this flag and route to the correct regmap.
 */

/* TS regmap routing marker — bit28 does not exceed regmap max size (0x400) */
#define AML_TS_REG_FLAG   BIT(28)
#define AML_TS_REG(off)   ((u32)(off) | AML_TS_REG_FLAG)
#define AML_IS_TS_REG(r)  ((r) & AML_TS_REG_FLAG)
#define AML_TS_OFF(r)     ((r) & ~AML_TS_REG_FLAG)

/* Word offset → byte offset conversion */
/* Stride = 0x140 byte: datasheet Demux1(0xc1105940)-Demux0(0xc1105800)=0x140 */
#define AML_DEMUX_REG(d, woff)      ((d) * 0x140 + (woff) * 4)

/* ========================================================================
 * DEMUX CORE REGISTERS (per-core)
 * ======================================================================== */

/* Offset 0x00=STB_VERSION_O(RO!), 0x04=DEMUX_CONTROL_O (datasheet) */
#define STB_VERSION_O(d)          AML_DEMUX_REG(d, 0x00)  /* read-only version */
#define DEMUX_CONTROL(d)          AML_DEMUX_REG(d, 0x04)

/*
 * FEC_INPUT_CONTROL — woff 0x02 = byte 0x008 per demux (stride 0x140)
 *
 * devmem verification (S905X3, vendor kernel not loaded):
 *   0xff638008 = 0x00FFFFFF  → reset/default, nothing written
 *
 * Vendor dmx_enable() writes to this register:
 *   bits[14:12] = FEC_SEL  — physical TS port selection
 *     0 = TS0 parallel    1 = TS1 parallel
 *     2 = TS2 parallel    3 = TS3 parallel
 *     4 = S_TS2 serial   5 = S_TS1 serial   6 = S_TS0 serial
 *   bits[11:0]  = fec_ctrl — DTS ts<n>_control value (K5: 0x000)
 *
 * K5 expectation: FEC_SEL=1 (TS1 parallel) → 0x00001000
 */
#define FEC_INPUT_CONTROL(d)      AML_DEMUX_REG(d, 0x02)  /* byte 0x008 */
#define   FEC_SEL_SHIFT           12
#define   FEC_SEL_MASK            GENMASK(14, 12)
#define   FEC_SEL_TS0_PAR         0   /* parallel TS0 */
#define   FEC_SEL_TS1_PAR         1   /* parallel TS1 */
#define   FEC_SEL_TS2_PAR         2   /* parallel TS2 */
#define   FEC_SEL_TS3_PAR         3   /* parallel TS3 */
#define   FEC_SEL_S_TS2           4   /* serial S2P2 */
#define   FEC_SEL_S_TS1           5   /* serial S2P1 */
#define   FEC_SEL_S_TS0           6   /* serial S2P0 */
/* bit31+30: Free-running clock gates — REQUIRED on SM1.
 * devmem verification: DEMUX_CONTROL=0xC3C00750 → bit31+30 SET.
 * Without these bits HW does not respond / register access fails. */
#define   ENABLE_FREE_CLK_FEC_DATA_VALID  BIT(31)
#define   ENABLE_FREE_CLK_STB_REG         BIT(30)
#define   TS_RECORDER_ENABLE              BIT(9)
/*
 * TS_RECORDER_SELECT (bit10):
 *   0 = after PID filter  → only PID_TYPE=RECORDER_STREAM(7) packets go to AsyncFIFO
 *   1 = before PID filter → all raw TS packets go to AsyncFIFO (REQUIRED for swfilter)
 *
 * Vendor c_stb_define.h: "Bit 10 - ts_recorder_select 0:after PID filter 1:before PID filter"
 *
 * Our swfilter architecture: receives all TS packets from AsyncFIFO and applies PID filtering in software.
 * Therefore TS_RECORDER_SELECT=1 MUST BE set — otherwise PAT/section packets
 * (PID_TYPE=SECTION_PACKET=3) go to the section buffer, and AsyncFIFO receives no data.
 */
#define   TS_RECORDER_SELECT             BIT(10)
#define   SECTION_END_WITH_TABLE_ID       BIT(8)
#define   KEEP_DUPLICATE_PACKAGE          BIT(6)
#define   STB_DEMUX_ENABLE                BIT(4)
#define   NOT_USE_OF_SOP_INPUT            BIT(1)
#define   IGNORE_NON_SOP_FEC_ERROR        BIT(0)
/* vendor devmem verified ( channel enabled/disabled difference):
 *   DEMUX_CONTROL disabled=0x009FF8EF, enabled=0xC3C00750
 *   Without Bit31+30+8+6+4 SET, the hardware does not work. */
#define   DMX_RECORDER_EN          (BIT(9) | BIT(4))

/* dev/mem verified: idx 0x03 @ 0xff63800C = 0x000007FF (sync byte 0xFF + 11-bit mask) */
#define FEC_SYNC_BYTE(d)          AML_DEMUX_REG(d, 0x03)
#define FM_WR_DATA(d)             AML_DEMUX_REG(d, 0x06)
#define FM_WR_ADDR(d)             AML_DEMUX_REG(d, 0x07)
#define   FM_WR_DATA_REQUEST       BIT(15)

/*
 * FM memory word format (32-bit):
 *   [31:16] = even index (channel or filter)
 *   [15:0]  = odd  index
 *
 * Channel word: (pkt_type<<13) | pid  — PID_TYPE=13
 *   pkt_type: SECTION_PACKET=3, RECORDER_STREAM=7, VIDEO=0, AUDIO=1 ...
 *
 * Filter byte 0 (table_id):
 *   bit15 = MASKHIGH, bit14 = MASKLOW
 *   bit13 = DISABLE_PID_CHECK, bit12:8 = cid, bit7:0 = value
 *
 * Filter byte 1..14:
 *   bit15 = MASK, bit14 = MASK_EQ (neq)
 *   bit13 = DISABLE_PID_CHECK, bit12:8 = cid, bit7:0 = value|advance
 */
#define   FM_PID_TYPE_SHIFT        13
#define   FM_PKT_VIDEO             0
#define   FM_PKT_AUDIO             1
#define   FM_PKT_SECTION           3
#define   FM_PKT_SCR               5
#define   FM_PKT_OTHER_PES         6
#define   FM_PKT_RECORDER          7
#define   FM_CHAN_UNUSED_PID       0x1FFF
#define   FM_CHAN_UNUSED_TARGET    ((7 << FM_PID_TYPE_SHIFT) | FM_CHAN_UNUSED_PID)

/* Filter byte 0 bits */
#define   FM_FB0_MASKLOW           BIT(14)  /* table_id low nibble masked  */
#define   FM_FB0_MASKHIGH          BIT(15)  /* table_id high nibble masked */
#define   FM_FB0_NO_PID_CHECK      BIT(13)
#define   FM_FB0_CID_SHIFT         8

/* Filter byte 1..14 bits */
#define   FM_FBN_MASK              BIT(15)  /* byte masked (don't check)   */
#define   FM_FBN_MASK_EQ           BIT(14)  /* neq mode                    */
#define   FM_FBN_NO_PID_CHECK      BIT(13)
#define   FM_FBN_CID_SHIFT         8

#define MAX_FM_COMP_ADDR(d)       AML_DEMUX_REG(d, 0x08)

#define SEC_BUFF_01_START(d)      AML_DEMUX_REG(d, 0x0e)
#define SEC_BUFF_23_START(d)      AML_DEMUX_REG(d, 0x0f)
#define SEC_BUFF_SIZE(d)          AML_DEMUX_REG(d, 0x10)
#define SEC_BUFF_BUSY(d)          AML_DEMUX_REG(d, 0x11)
#define SEC_BUFF_READY(d)         AML_DEMUX_REG(d, 0x12)
#define SEC_BUFF_NUMBER(d)        AML_DEMUX_REG(d, 0x13)
#define   SEC_BUFF_INDEX           GENMASK(4, 0)
#define   SEC_BUFF_FILTER_NUM      GENMASK(12, 8)

/* PTS/DTS/PCR Registers (33-bit timestamp support) */
#define PCR_DEMUX(d)              AML_DEMUX_REG(d, 0x1a)
#define VIDEO_PTS_DEMUX(d)        AML_DEMUX_REG(d, 0x1b)
#define VIDEO_DTS_DEMUX(d)        AML_DEMUX_REG(d, 0x1c)
#define AUDIO_PTS_DEMUX(d)        AML_DEMUX_REG(d, 0x1d)
#define SUB_PTS_DEMUX(d)          AML_DEMUX_REG(d, 0x1e)

#define STB_PTS_DTS_STATUS(d)     AML_DEMUX_REG(d, 0x1f)
#define   VIDEO_PTS_READY          BIT(0)
#define   VIDEO_DTS_READY          BIT(1)
#define   AUDIO_PTS_READY          BIT(2)
#define   AUDIO_DTS_READY          BIT(3)
#define   SUB_PTS_READY            BIT(4)
#define   VIDEO_PTS_BIT32          BIT(12)
#define   VIDEO_DTS_BIT32          BIT(13)
#define   AUDIO_PTS_BIT32          BIT(14)
#define   AUDIO_DTS_BIT32          BIT(15)
#define   SUB_PTS_BIT32            BIT(16)
#define   PCR_BIT32                BIT(18)

/* Interrupt Registers */
#define STB_INT_STATUS(d)         AML_DEMUX_REG(d, 0x23)
#define STB_INT_MASK(d)           AML_DEMUX_REG(d, 0x32)

/*
 * AML_DEMUX_INT_MASK — identical to vendor DEMUX_INT_MASK,
 * BUT INT_NEW_PDTS (BIT4) intentionally DISABLED:
 *   NEW_PDTS fires very rapidly → when the kernel sees >1000 IRQs within 100ms
 *   it writes STB_INT_MASK=0, disabling all dmx IRQs.
 *   Result: SEC_BUFF_READY IRQ also stops arriving → HW section filter stalls.
 *   PTS/PCR is already sent separately via PCR_READY (BIT10).
 */
#define AML_DEMUX_INT_MASK   0x0995u
/* vendor devmem verification: STB_INT_MASK = 0x0995 (during scan)
 * bit0=TS_ERR, bit2=SEC_READY, bit4=SEC_BUFF_READY, bit7=SUB_PES,
 * bit8=OTHER_PES, bit11=INPUT_TIMEOUT
 * Old 0x0585 was missing section filter IRQs */
#define   INT_INPUT_TIME_OUT       BIT(12)
#define   INT_AUDIO_SPLICING_POINT BIT(11)
#define   INT_PCR_READY            BIT(10)
#define   INT_VIDEO_SPLICING_POINT BIT(9)
#define   INT_OTHER_PES_READY      BIT(8)
#define   INT_SUB_PES_READY        BIT(7)
#define   INT_DISCONTINUITY        BIT(6)
#define   INT_DUPLICATED_PACK_FOUND BIT(5)
#define   INT_NEW_PDTS             BIT(4)
#define   INT_OM_CMD_BUFFER        BIT(3)
#define   INT_SECTION_READY        BIT(2)
#define   INT_TS_ERROR             BIT(1)
#define   INT_TS_ERROR_PIN         BIT(0)

/* DMA Control */
/* OM (Output Manager) */
#define STB_OM_CTL(d)             AML_DEMUX_REG(d, 0x22)
#define OM_CMD_STATUS(d)          AML_DEMUX_REG(d, 0x0b)

/* Endian configuration
 * Vendor dmx_enable: (1<<SEPARATE_ENDIAN)|(7<<SCR_ENDIAN)|(7<<SUB_ENDIAN)|
 *   (7<<AUDIO_ENDIAN)|(7<<VIDEO_ENDIAN)|(7<<OTHER_ENDIAN)|(7<<BYPASS_ENDIAN)
 */
#define DEMUX_ENDIAN(d)           AML_DEMUX_REG(d, 0x24)
#define   SEPARATE_ENDIAN          31
#define   OTHER_PES_ENDIAN         21
#define   SCR_ENDIAN               18
#define   SUB_ENDIAN               15
#define   AUDIO_ENDIAN             12
#define   VIDEO_ENDIAN             9
#define   OTHER_ENDIAN             6
#define   BYPASS_ENDIAN            3
#define   SECTION_ENDIAN           0
#define AML_DEMUX_ENDIAN_VAL  ((1u << SEPARATE_ENDIAN) | \
                               (7u << SCR_ENDIAN)       | \
                               (7u << SUB_ENDIAN)       | \
                               (7u << AUDIO_ENDIAN)     | \
                               (7u << VIDEO_ENDIAN)     | \
                               (7u << OTHER_ENDIAN)     | \
                               (7u << BYPASS_ENDIAN))
/* = 0x801FFFF8 */

/* TS HIU (High Interface Unit) */
#define TS_HIU_CTL(d)             AML_DEMUX_REG(d, 0x25)
#define   USE_HI_BSF_INTERFACE     7

/* Section buffer base address (physical >> 16) */
#define SEC_BUFF_BASE(d)          AML_DEMUX_REG(d, 0x26)

/* PES strong sync magic */
#define PES_STRONG_SYNC(d)        AML_DEMUX_REG(d, 0x36)

#define DEMUX_MEM_REQ_EN(d)       AML_DEMUX_REG(d, 0x27)
#define   OTHER_PES_AHB_DMA_EN     BIT(11)
#define   SUB_AHB_DMA_EN           BIT(10)
#define   BYPASS_AHB_DMA_EN        BIT(9)
#define   SECTION_AHB_DMA_EN       BIT(8)
#define   VIDEO_PACKET_DMA         BIT(0)
#define   AUDIO_PACKET_DMA         BIT(1)
#define   SUB_PACKET_DMA           BIT(2)
#define   SECTION_PACKET_DMA       BIT(3)
#define   OTHER_PES_PACKET_DMA     BIT(6)

/*
 * AML_MEM_REQ_EN_HW — HW section DMA path (vendor USE_AHB_MODE)
 *   SECTION_AHB_DMA_EN=1: PID match → SEC_BUFF DMA → SEC_BUFF_READY IRQ
 *   Bit 0-6: packet type enable bitmask
 */
#define AML_MEM_REQ_EN_HW  (OTHER_PES_AHB_DMA_EN | \
                            SECTION_AHB_DMA_EN    | \
                            OTHER_PES_PACKET_DMA  | \
                            SECTION_PACKET_DMA    | \
                            SUB_PACKET_DMA        | \
                            AUDIO_PACKET_DMA      | \
                            VIDEO_PACKET_DMA)
/* = 0x096F */

/*
 * AML_MEM_REQ_EN_SW — SW filter fallback path (BYPASS_AHB_DMA_EN)
 *   BYPASS_AHB_DMA_EN=1: raw TS → AsyncFIFO → dvb_dmx_swfilter()
 *   SECTION_AHB_DMA_EN=0: HW section DMA disabled
 *   Used when sf_mode=true.
 */
#define AML_MEM_REQ_EN_SW  (OTHER_PES_AHB_DMA_EN |                             BYPASS_AHB_DMA_EN     |                             OTHER_PES_PACKET_DMA  |                             SECTION_PACKET_DMA    |                             SUB_PACKET_DMA        |                             AUDIO_PACKET_DMA      |                             VIDEO_PACKET_DMA)
/* = 0x0A4F (BYPASS=1, SECTION_AHB=0) */

/*
 * VIDEO_STREAM_ID — video2 recorder stream config
 *   record=1: 0xFFFF0000 (vendor: bit[31:16])
 *   record=0: 0x00000000
 */
#define VIDEO_STREAM_ID(d)        AML_DEMUX_REG(d, 0x1c)
#define   VIDEO2_FOR_RECORDER_STREAM  (BIT(25) | (7 << 22))

/* Write Pointers */
#define VIDEO_PDTS_WR_PTR(d)      AML_DEMUX_REG(d, 0x28)
#define AUDIO_PDTS_WR_PTR(d)      AML_DEMUX_REG(d, 0x29)
#define OTHER_WR_PTR(d)           AML_DEMUX_REG(d, 0x2e)
#define OB_START(d)               AML_DEMUX_REG(d, 0x2f)
#define OB_LAST_ADDR(d)           AML_DEMUX_REG(d, 0x30)

/* Timeout, packet count, small section, channel control */
#define DEMUX_INPUT_TIMEOUT_C(d)  AML_DEMUX_REG(d, 0x46)
#define DEMUX_INPUT_TIMEOUT(d)    AML_DEMUX_REG(d, 0x47)
#define DEMUX_PACKET_COUNT_DISABLE(d) AML_DEMUX_REG(d, 0x48)
#define DEMUX_PACKET_COUNT(d)     AML_DEMUX_REG(d, 0x49)
#define DEMUX_SMALL_SEC_CTL(d)    AML_DEMUX_REG(d, 0x4c)
#define   SMALL_SEC_ENABLE         BIT(0)
#define DEMUX_CHAN_RECORD_EN(d)   AML_DEMUX_REG(d, 0x4a)
#define DEMUX_CHAN_PROCESS_EN(d)  AML_DEMUX_REG(d, 0x4b)

/* ========================================================================
 * TOP-LEVEL / GLOBAL REGISTERS (byte offsets – word offset × 4)
 * ======================================================================== */

#define TS_HIU1_CONFIG            (0x4e * 4)   /* 0x138 */
#define TS_TOP_CONFIG1            (0x4f * 4)   /* 0x13c */
#define STB_RECORDER2_CNTL        (0xee * 4)   /* 0x3b8 */
#define STB_S2P2_CONFIG           (0xef * 4)   /* 0x3bc */
/*
 * STB_TOP_CONFIG — Global TS input → demux routing MUX
 *
 * SM1 (S905X3): byte offset 0x3c0 from base_ts (0xffd06000)
 *   Physical: 0xffd063c0
 *   Global region following the 3 × TS_IN blocks (0x140 bytes each).
 *
 * WITHOUT this register the demux receives no data from any TS input —
 * even if FEC_INPUT_CONTROL is written correctly.
 * The vendor driver writes this register at every init.
 *
 * Bit fields:
 *   [23:22] DEMUX_2_INPUT_SOURCE  00=TS0, 01=TS1, 10=TS2, 11=DMA(HIU)
 *   [21:20] DEMUX_1_INPUT_SOURCE  (same encoding)
 *   [19:18] DEMUX_0_INPUT_SOURCE  (same encoding)
 *   [15]    INVERT_TS_1_FEC_CLK
 *   [14]    INVERT_TS_0_FEC_CLK
 *   [13]    S2P_1_FEC_SERIAL_SELECT  1=serial, 0=parallel
 *   [12]    S2P_0_FEC_SERIAL_SELECT
 *   [11]    TS_1_FEC_SERIAL_SELECT   1=serial, 0=parallel
 *   [10]    TS_0_FEC_SERIAL_SELECT
 *
 * K5 expectation (tsin_b → TS1 → DMX0, parallel):
 *   bits[19:18] = 01 (TS1 → Demux0)
 *   bits[11:10] = 00 (parallel)
 *   Value: 0x00040000
 */
#define STB_TOP_CONFIG            (0xf0 * 4)   /* 0x3c0 */
/* DEMUX_X_INPUT_SOURCE fields */
#define STB_DMX_SRC_SHIFT(d)     (18 + (d) * 2)
#define STB_DMX_SRC_MASK(d)      (0x3u << STB_DMX_SRC_SHIFT(d))
#define STB_DMX_SRC_TS0          0x0   /* TS Input 0 (tsin_a) */
#define STB_DMX_SRC_TS1          0x1   /* TS Input 1 (tsin_b) */
#define STB_DMX_SRC_TS2          0x2   /* TS Input 2          */
#define STB_DMX_SRC_DMA          0x3   /* DMA/HIU             */
/* TS serial/parallel select bits */
#define STB_TS1_SERIAL_SEL       BIT(11)
#define STB_TS0_SERIAL_SEL       BIT(10)
#define TS_TOP_CONFIG             (0xf1 * 4)   /* 0x3c4 */   /* <<-- CORRECTED */
#define TS_FILE_CONFIG            (0xf2 * 4)   /* 0x3c8 */
#define TS_PL_PID_INDEX           (0xf3 * 4)   /* 0x3cc */
#define TS_PL_PID_DATA            (0xf4 * 4)   /* 0x3d0 */
#define COMM_DESC_KEY0            (0xf5 * 4)   /* 0x3d4 */
#define COMM_DESC_KEY1            (0xf6 * 4)   /* 0x3d8 */
#define COMM_DESC_KEY_RW          (0xf7 * 4)   /* 0x3dc */
#define CIPLUS_KEY0               (0xf8 * 4)   /* 0x3e0 */
#define CIPLUS_KEY1               (0xf9 * 4)   /* 0x3e4 */
#define CIPLUS_KEY2               (0xfa * 4)   /* 0x3e8 */
#define CIPLUS_KEY3               (0xfb * 4)   /* 0x3ec */
#define CIPLUS_KEY_WR             (0xfc * 4)   /* 0x3f0 */
#define CIPLUS_CONFIG             (0xfd * 4)   /* 0x3f4 */
#define CIPLUS_ENDIAN             (0xfe * 4)   /* 0x3f8 */
#define COMM_DESC_2_CTL           (0xff * 4)   /* 0x3fc */

/* ========================================================================
 * TS INPUT REGISTERS (base: 0xffd06000, ts regmap)
 *
 * dev/mem VERIFIED (S905X3):
 *   TS_IN blocks at 0xffd06000 base, stride=0x140 byte
 *   TS_IN[0] = 0xffd06000 (active, packet counter changing)
 *   TS_IN[1] = 0xffd06140 (reset/disabled state)
 *   TS_IN[2] = 0xffd06280
 *
 *   Within each block:
 *     +0x000 = TS_IN_CTRL   (idx 0x00, value 0x00030003)
 *     +0x004 = TS_IN_STATUS (idx 0x01, active=0x0000AAA0, disabled=0xFE015AA5)
 *     +0x014 = TS_IN_CLK    (idx 0x05, value 0x0000BB47)
 *     +0x040 = TS_S2P_CTRL  (idx 0x10, value 0x0000CCCC)
 *     +0x11C = TS_PKT_COUNT (idx 0x47, packet counter — variable)
 *
 * AML_TS_REG() marker: ensures aml_write_reg() routes these addresses
 * to regmap_ts (0xffd06000).
 * ======================================================================== */
#define TS_IN_CTRL(n)             AML_TS_REG((n) * 0x140 + 0x000)
#define   TS_IN_ENABLE             BIT(0)
#define   TS_IN_ACTIVE             BIT(1)	/* vendor: always set */
#define   TS_IN_SERIAL             BIT(2)
#define   TS_IN_PARALLEL           BIT(3)
#define   TS_IN_CLK_INVERT         BIT(4)
#define   TS_IN_DATA_INVERT        BIT(5)
#define   TS_IN_SYNC_INVERT        BIT(6)
#define   TS_IN_VALID_INVERT       BIT(7)
#define   TS_IN_ERROR_INVERT       BIT(8)
#define   TS_IN_PARALLEL_8BIT      (BIT(16) | BIT(17))  /* vendor bit[17:16]=0b11 */
/* Vendor verified default value: 0xFFD06000=0xFFD06140=0x00030003 */
#define   TS_IN_CTRL_PARALLEL      0x00030003
#define   TS_IN_RESET              BIT(1)
/* TS_IN_STATUS: active port = 0x0000AAA0, disabled = 0xFE015AA5 */
#define TS_IN_STATUS(n)           AML_TS_REG((n) * 0x140 + 0x004)
#define   TS_IN_ACTIVE_PATTERN     0x0000AAA0

#define TS_IN_CLK_CTRL(n)         AML_TS_REG((n) * 0x140 + 0x014)

/* TS_PKT_COUNT: packet counter, bit31=valid */
#define TS_PKT_COUNT(n)           AML_TS_REG((n) * 0x140 + 0x11C)

/* S2P = Serial-to-Parallel converter, within TS_IN block at +0x040 offset */
#define TS_S2P_CTRL(n)            AML_TS_REG((n) * 0x140 + 0x040)
#define   TS_S2P_ENABLE            BIT(0)
#define   TS_S2P_RESET             BIT(1)
#define   TS_S2P_CLK_INVERT        BIT(2)
#define   TS_S2P_DATA_INVERT       BIT(3)
#define   TS_S2P_SYNC_INVERT       BIT(4)
#define   TS_S2P_VALID_INVERT      BIT(5)
#define   TS_S2P_ERROR_INVERT      BIT(6)
#define   TS_S2P_CLK_DIV           GENMASK(15, 8)
/* vendor verified default value (devmem 0xFFD06040 = 0x0000CCCC):
 * CLK_DIV=0xCC, CLK_INVERT+DATA_INVERT, BIT0 clear (S2P bypass in parallel mode) */
#define   TS_S2P_VENDOR_INIT       0x0000CCCC
#define   TS_S2P_SERIAL_SEL        GENMASK(17, 16)

/* Descrambler per core */
#define DSC_CTRL(n)               (0x300 + (n) * 0x10)
#define   DSC_CTRL_RESET           BIT(0)
#define   DSC_CTRL_ENABLE          BIT(1)

/* TS rate control */
#define TS_RATE_CTRL              (0x340)   /* 0x340 (byte offset) */
#define   TS_RATE_MASK             GENMASK(15, 0)
#define   TS_RATE_ENABLE           BIT(31)

/* Hardware PID filter (per demux, 32 slot) – these are already byte offsets */
#define DEMUX_PID_INDEX           0x400
#define DEMUX_PID_VALUE           0x404
#define AML_DEMUX_PID_BASE        0x400

/* Hardware section filter (per demux) – byte offset */
#define AML_SEC_FILTER_BASE       0x800
#define AML_SEC_FILTER(slot, byte)    (AML_SEC_FILTER_BASE + (slot) * 32 + (byte))
#define AML_SEC_MASK(slot, byte)      (AML_SEC_FILTER_BASE + (slot) * 32 + 16 + (byte))
#define AML_SEC_CTRL(slot)            (0x8c0 + (slot) * 4)
#define   AML_SEC_ENABLE           BIT(0)
#define   AML_SEC_DISABLE_PID_CHECK BIT(1)
#define   AML_SEC_PID_INDEX        GENMASK(7, 4)

/* IRQ threshold */
#define AML_DEMUX_IRQ_TH          0x90

/* ========================================================================
 * ASYNC FIFO REGISTERS (within demux regmap, base: 0xff638000)
 *
 * dev/mem VERIFIED (S905X3):
 *   ASYNC FIFO table start = 0xff638000 + 0x140
 *   Stride = 0x10 byte / channel
 *   Each channel has 2 active registers:
 *     +0x00 = CTRL  (FIFO[0]=0x8FF403CF active, others=0x8FF003C4/0x8FF005C4)
 *     +0x04 = ENDIAN (all 0x18101810)
 *   8+ channels present (continues in Core1 region)
 *
 * NOTE: These registers are accessed via the demux regmap (0xff638000),
 *       not via the ts regmap (0xffd06000)!
 * ======================================================================== */
/* ========================================================================
 * ASYNC FIFO REGISTERS  (separate regmap: regmap_async — 0xFFD0A000 based)
 *
 * S905X3 Memory Map (XLS confirmed):
 *   async_fifo  → 0xFFD0A000..0xFFD0AFFF  (4 KB, FIFO channel 0)
 *   async_fifo2 → 0xFFD09000..0xFFD09FFF  (4 KB, FIFO channel 1)
 *   async_fifo3 → 0xFFD26000..0xFFD26FFF  (4 KB, S905D3 only)
 *
 * IMPORTANT: These registers are NOT in the 0xFF638000 (demux) regmap!
 *   0xFF638180 = STB_INT_MASK (demux interrupt mask) → not async FIFO.
 *   Each async_fifo block uses its own 4KB physical page.
 *
 * Verified with dev/mem at 0xFFD0A000 (active DVR channel):
 *   +0x00 = 0x0A7BAD80  → WR_PTR (dynamic, hardware writes — read-only)
 *   +0x04 = 0x80301000  → CTRL   (bit31=enable, source mux bits)
 *   +0x08 = 0x03B00000  → BASE_ADDR (DMA buffer physical RAM address)
 *   +0x0C = 0x002007FF  → SIZE/FLUSH config
 *   +0x10 = 0xFFFFFFFF  → IRQ_MASK (all masked = no IRQ)
 *
 * Channel → physical block mapping (S905X3):
 *   FIFO[0] → 0xFFD0A000 (async_fifo)
 *   FIFO[1] → 0xFFD09000 (async_fifo2)
 *   regmap_async must be defined as the "async-fifo" region in DTS.
 *
 * Vendor 5.15 kernel scripts BUG: uses 0xFF634400+0x180
 * (CBUS addressing confused with physical MMIO — WRONG).
 * ======================================================================== */

/* =========================================================================
 * ASYNC FIFO REGISTER LAYOUT — HARDWARE VERIFIED
 *
 * Verification: vendor kernel, active DVR channel, devmem read
 * Date: 2026-03-03
 *
 * Physical blocks (S905X3 memory map XLS):
 *   async_fifo  → 0xFFD0A000  (FIFO[0]) — active, DVR recording
 *   async_fifo2 → 0xFFD09000  (FIFO[1]) — inactive
 *
 * regmap_async base = 0xFFD09000 (DTS: async-fifo 0x2000 size)
 *   FIFO[1] offset = 0x0000  → 0xFFD09000 (async_fifo2)
 *   FIFO[0] offset = 0x1000  → 0xFFD0A000 (async_fifo)
 *
 * Active channel hardware values (FIFO[0]):
 *   +0x00 = 0x0ABD7B00   WR_PTR — continuously incrementing (DMA write pointer)
 *   +0x04 = 0x80301000   CTRL   — BIT31=1(EN), BIT29=0, bit[21:20]=3(src)
 *   +0x08 = 0x03B00000   BASE   — CMA buffer @ 62.5MB RAM
 *   +0x0C = 0x002007FF   SIZE   — BIT21=1(cfg), bit[12:0]=0x7FF(flush)
 *   +0x10 = 0xFFFFFFFF   IRQ    — all IRQs masked
 *   +0x14 = 0x00000000   (unused)
 *
 * Inactive channel (FIFO[1]):
 *   +0x04 = 0x20000000   CTRL — BIT29=1(IDLE), BIT31=0(disabled)
 *   +0x08 = 0x00000000   BASE — no DMA buffer
 *   +0x0C = 0x00200000   SIZE — BIT21=1, threshold=0
 * ========================================================================= */

/* =========================================================================
 * ASYNC FIFO REGISTER LAYOUT — VENDOR c_stb_define.h VERIFIED
 *
 * Physical addresses (S905X3, DTS base=0xFFD09000):
 *   FIFO[0] → offset 0x1000 → 0xFFD0A000 (async_fifo,  PRIMARY/ACTIVE)
 *   FIFO[1] → offset 0x0000 → 0xFFD09000 (async_fifo2, SECONDARY/IDLE)
 *
 * Each FIFO has 4 registers (word offset, byte stride=4):
 *   REG0 (+0x00): DMA start address (physical RAM)
 *   REG1 (+0x04): FLUSH control
 *   REG2 (+0x08): FILL/SOURCE control
 *   REG3 (+0x0C): IRQ threshold
 *
 * REG1 bit definitions (vendor c_stb_define.h):
 *   bit31    FLUSH_STATUS (RO — data being flushed)
 *   bit30    ERR          (RO)
 *   bit29    FIFO_EMPTY   (RO)
 *   bit24    TO_HIU       (HIU path selection)
 *   bit23    FLUSH        (immediate flush pulse)
 *   bit22    RESET        (reset pulse — set then clear)
 *   bit21    WRAP_EN      (ring buffer wrap enabled)
 *   bit20    FLUSH_EN     (flush enabled — REQUIRED for IRQ)
 *   bit[14:0] FLUSH_CNT  (128-byte block count = buf_size>>7)
 *
 * REG2 bit definitions:
 *   bit[24:23] SOURCE     (DMX0=3, DMX1=2, DMX2=0)
 *   bit[22:21] ENDIAN     (1 = LE)
 *   bit20      FILL_EN    (fill path enabled)
 *   bit[19:0]  FILL_CNT
 *
 * REG3:
 *   bit[15:0] IRQ_THRESH  (128-byte block count, IRQ period)
 *
 * devmem verification ( active channel):
 *   REG1 = 0x80301000
 *     bit31(RO FLUSH_STATUS)=1, bit21(WRAP_EN)=1,
 *     bit20(FLUSH_EN)=1, FLUSH_CNT=0x1000
 * ========================================================================= */

/* FIFO[n] register addresses — offset from DTS base 0xFFD09000 */
#define ASYNC_FIFO_BASE(n)   ((n) == 0 ? 0x1000 : 0x0000)
#define ASYNC_FIFO_REG0(n)   (ASYNC_FIFO_BASE(n) + 0x00)  /* DMA addr   (RW) */
#define ASYNC_FIFO_REG1(n)   (ASYNC_FIFO_BASE(n) + 0x04)  /* FLUSH ctrl (RW) */
#define ASYNC_FIFO_REG2(n)   (ASYNC_FIFO_BASE(n) + 0x08)  /* FILL/SRC   (RW) */
#define ASYNC_FIFO_REG3(n)   (ASYNC_FIFO_BASE(n) + 0x0C)  /* IRQ thresh (RW) */

/* REG1 bits */
#define AFIFO_RESET          BIT(22)   /* pulse: set then clear */
#define AFIFO_WRAP_EN        BIT(21)   /* ring buffer wrap */
#define AFIFO_FLUSH_EN       BIT(20)   /* flush enable — REQUIRED for IRQ */

/* REG2 bits */
#define AFIFO_SOURCE_SHIFT   23
#define AFIFO_SOURCE_MASK    GENMASK(24, 23)
#define AFIFO_SOURCE(s)      ((s) << AFIFO_SOURCE_SHIFT)
/* source_val: DMX0=3, DMX1=2, DMX2=0 (vendor reset_async_fifos) */
#define AFIFO_SRC_DMX0       3
#define AFIFO_SRC_DMX1       2
#define AFIFO_SRC_DMX2       0
#define AFIFO_ENDIAN_VAL     BIT(21)   /* 1 = little-endian */
#define AFIFO_FILL_EN        BIT(20)   /* fill path enable */

/* REG3 bit definitions */
#define AFIFO_IRQ_EN         BIT(21)   /* IRQ arm — WITHOUT this, IRQ never arrives!
                                        * Device evidence: idle FIFO[1] REG3=0x00200000 (BIT21=1 hardware default)
                                        * Vendor active: REG3=0x002007FF (BIT21=1 + threshold)
                                        * V2.16 bug: only 0x7FF was written → BIT21=0 → no IRQ */

/* Backwards compatibility aliases */
#define ASYNC_FIFO_WR_PTR(n)      ASYNC_FIFO_REG0(n)  /* RO: HW write pointer */
#define ASYNC_FIFO_CTRL(n)        ASYNC_FIFO_REG1(n)
#define ASYNC_FIFO_BASE_ADDR(n)   ASYNC_FIFO_REG0(n)
#define ASYNC_FIFO_SIZE(n)        ASYNC_FIFO_REG3(n)
#define ASYNC_FIFO_IRQ_MASK(n)    ASYNC_FIFO_REG3(n)

/* ASYNC_FIFO_IDLE: value written to REG1 to stop the FIFO.
 * All control bits (FLUSH_EN, WRAP_EN, RESET) are cleared. */
#define ASYNC_FIFO_IDLE           0U
/* +0x14 unused (0x00000000) */

/* ==================== HIU (Clock) Registers ==================== */
/* HIU base address = 0xff63c000
 *
 * Vendor kernel hardware values (devmem read):
 *   HHI_GCLK_MPEG0 (0xFF63C11C) = 0xFFFFFFFF — all clock gates open
 *   HHI_GCLK_MPEG2 (0xFF63C128) = 0x80010136 — BIT16(TS) + BIT5(AIFIFO2) set
 *   HHI_TS_CLK_CNTL (0xFF63C190) = 0x00000001 — BIT0=enable (not BIT8!)
 *
 * Result: clocks are already enabled by the vendor kernel.
 * The mainline driver will do the same via clk_prepare_enable().
 */
#define HHI_GCLK_MPEG0            (0x47 * 4)   /* 0x11C */
#define HHI_GCLK_MPEG2            (0x4a * 4)   /* 0x128 — BIT16=TS, BIT5=AIFIFO2 */
#define HHI_TS_CLK_CNTL           (0x64 * 4)   /* 0x190 */
#define   TS_CLK_ENABLE            BIT(0)       /* vendor: 0x1 = BIT0, not BIT8 */
#define   TS_CLK_DIV_MASK          GENMASK(7, 0)

/* ==================== AO Domain Registers (Power) ==================== */
/* AO base address is typically 0xff800000, offsets are word offsets */
#define AO_RTI_GEN_PWR_SLEEP0     (0x3a * 4)   /* 0xe8 */
#define AO_RTI_GEN_PWR_ACK0       (0x3c * 4)   /* 0xf0 */
#define   PWR_DOS_VDEC             BIT(1)   /* vdec power (demux depends on this) */
#define   PWR_DOS_HCODEC           BIT(0)

/* Register ranges for volatility/caching hints
 *
 * DEMUX_CONTROL(d) = d*0x140 + 0x10 (byte offset)
 * STB_VERSION_O(0) = 0x00 → read-only (vendor verified)
 * WR_PTR registers are dynamic (hardware writes)
 *
 * NOTE: Previously DEMUX_STATUS_START=0x10 — clashed with DEMUX_CONTROL
 * and regmap write returned -EIO. Removed with writeable_reg=NULL.
 */
#define DEMUX_WR_PTR_START        (0x28 * 4)   /* byte 0xA0 — woff 0x28 */
#define DEMUX_WR_PTR_END          (0x40 * 4)   /* byte 0x100 */

#endif /* __AML_DVB_REGS_H */
