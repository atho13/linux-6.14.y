// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic DVB hardware demux — V2.60
 *
 * Architecture (same as vendor aml_dmx.c):
 *   HW section path: DMX_TYPE_SEC → FM memory → SEC_BUFF DMA
 *     → SEC_BUFF_READY IRQ → process_section() → cb.sec()
 *   DVR path: DMX_TYPE_TS/DVR → TS_RECORDER_ENABLE → AFIFO → cb.ts()
 *   Both run concurrently on the same DMX.
 */
#define CREATE_TRACE_POINTS
#include "amlogic-dvb-trace.h"
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/pm_runtime.h>
#include <media/dmxdev.h>
#include <media/dvb_demux.h>
#include <media/dvb_net.h>
#include "amlogic_dvb.h"
#include "amlogic-dvb-regs.h"

/* ==================================================================
 * Module parameters — forced SW filtering
 * ================================================================== */
static int force_sec_sf;
module_param(force_sec_sf, int, 0644);
MODULE_PARM_DESC(force_sec_sf, "Force section feeds to SW filter (debug)");

static int force_pes_sf;
module_param(force_pes_sf, int, 0644);
MODULE_PARM_DESC(force_pes_sf, "Force PES feeds to SW filter (debug)");

static int force_all_sf;
module_param(force_all_sf, int, 0644);
MODULE_PARM_DESC(force_all_sf, "Force all feeds to SW filter (debug)");

#define AML_HW_PID_SLOTS  AML_CHANNEL_COUNT

/* forward */
static void dmx_enable(struct aml_dmx *dmx);
static int dmx_set_chan_regs(struct aml_dmx *dmx, int cid);
static int dmx_set_filter_regs(struct aml_dmx *dmx, int fid);

/* ======================================================================
 * sec_filter_match — vendor verbatim
 * ====================================================================== */
static bool sec_filter_match(struct aml_dmx *dmx, int fid, const u8 *p)
{
	int b;
	u8 neq = 0;
	if (!dmx->filter[fid].used ||
	    !dmx->channel[dmx->filter[fid].chan_id].used)
		return false;
	for (b = 0; b < AML_FILTER_LEN; b++) {
		u8 xor = p[b] ^ dmx->filter[fid].value[b];
		if (xor&dmx->filter[fid].maskandmode[b])
			return false;
		if (xor&dmx->filter[fid].maskandnotmode[b])
			neq = 1;
	}
	if (dmx->filter[fid].neq && !neq)
		return false;
	return true;
}

/* ======================================================================
 * section_notify — Linux DVB core'a section ilet
 * ====================================================================== */
static void section_notify(struct aml_dmx *dmx, int fid, const u8 *p)
{
	int cid = dmx->filter[fid].chan_id;
	int slen = (((p[1] & 0x0F) << 8) | p[2]) + 3;
	struct dvb_demux_feed *feed;
	if (cid < 0 || cid >= AML_CHANNEL_COUNT)
		return;
	feed = dmx->channel[cid].feed;
	if (feed && feed->cb.sec)
		feed->cb.sec((u8 *)p, slen, NULL, 0, dmx->filter[fid].filter,
			     0);
}

/* ======================================================================
 * hardware_match_section + software_match_section
 * ====================================================================== */
static void hardware_match_section(struct aml_dmx *dmx, u16 sec_num,
				   u16 buf_num)
{
	const u8 *p;
	int i, cid, need_crc;

	if (sec_num >= AML_FILTER_COUNT)
		return;
	p = (const u8 *)dmx->sec_buf[buf_num].addr;
	cid = dmx->filter[sec_num].chan_id;
	need_crc = 1;

	for (i = 0; i < AML_FILTER_COUNT; i++) {
		if (dmx->filter[i].chan_id != cid)
			continue;
		if (!sec_filter_match(dmx, i, p))
			continue;
		if (need_crc) {
			struct dvb_demux_feed *feed = dmx->channel[cid].feed;
			if (feed && feed->feed.sec.check_crc) {
				struct dvb_demux *dem = feed->demux;
				int slen = (((p[1] & 0x0F) << 8) | p[2]) + 3;
				feed->feed.sec.seclen = slen;
				feed->feed.sec.crc_val = ~0;
				if (dem->check_crc32(feed, p, slen))
					return;
			}
			need_crc = 0;
		}
		section_notify(dmx, i, p);
	}
}

static void software_match_section(struct aml_dmx *dmx, u16 buf_num)
{
	const u8 *p = (const u8 *)dmx->sec_buf[buf_num].addr;
	int i;
	for (i = 0; i < AML_FILTER_COUNT; i++)
		if (dmx->filter[i].used && sec_filter_match(dmx, i, p))
			section_notify(dmx, i, p);
}

/* ======================================================================
 * aml_dmx_process_section — SEC_BUFF_READY IRQ handler (vendor verbatim)
 * ====================================================================== */
void aml_dmx_process_section(struct aml_dmx *dmx)
{
	struct aml_dvb *dvb = dmx->dvb;
	u32 ready = 0, busy = 0, nr = 0;
	u16 sec_num;
	int i;

	aml_read_reg(dvb, SEC_BUFF_READY(dmx->id), &ready);
	if (!ready)
		return;

	for (i = 0; i < AML_SEC_BUF_COUNT; i++) {
		if (!(ready & BIT(i)))
			continue;

		aml_read_reg(dvb, SEC_BUFF_BUSY(dmx->id), &busy);
		aml_write_reg(dvb, SEC_BUFF_NUMBER(dmx->id), (u32)i);
		aml_read_reg(dvb, SEC_BUFF_NUMBER(dmx->id), &nr);
		sec_num = (u16)((nr >> 8) & 0x1F);

		if (!(busy & BIT(i))) {
			/* busy=0 watchdog */
			dmx->sec_buf_watchdog_count[i] &= 0xFFu;
			if (++dmx->sec_buf_watchdog_count[i] >=
			    AML_SEC_WD_BUSY0_MAX) {
				software_match_section(dmx, (u16)i);
				dmx->sec_buf_watchdog_count[i] = 0;
				aml_write_reg(dvb, SEC_BUFF_READY(dmx->id),
					      BIT(i));
			}
			continue;
		}
		if (sec_num >= AML_FILTER_COUNT) {
			/* invalid filter watchdog */
			dmx->sec_buf_watchdog_count[i] &= 0xFFFFu;
			dmx->sec_buf_watchdog_count[i] += 0x100;
			if (dmx->sec_buf_watchdog_count[i] >=
			    AML_SEC_WD_BUSY1_MAX) {
				software_match_section(dmx, (u16)i);
				dmx->sec_buf_watchdog_count[i] = 0;
				aml_write_reg(dvb, SEC_BUFF_READY(dmx->id),
					      BIT(i));
				aml_write_reg(dvb, SEC_BUFF_BUSY(dmx->id),
					      BIT(i));
			}
			continue;
		}
		/* Normal */
		dmx->sec_buf_watchdog_count[i] = 0;
		trace_aml_dvb_section_ready(dmx->id, (u16)i, sec_num);
		hardware_match_section(dmx, sec_num, (u16)i);
		aml_write_reg(dvb, SEC_BUFF_READY(dmx->id), BIT(i));
		aml_write_reg(dvb, SEC_BUFF_BUSY(dmx->id), BIT(i));
	}
}
EXPORT_SYMBOL_GPL(aml_dmx_process_section);

/* ======================================================================
 * sec_buf_alloc / sec_buf_free (vendor dmx_alloc_sec_buffer birebir)
 * ====================================================================== */
static int sec_buf_alloc(struct aml_dmx *dmx)
{
	struct aml_dvb *dvb = dmx->dvb;
	unsigned long grp_addr[AML_SEC_BUF_GRP_COUNT];
	int grp_len[AML_SEC_BUF_GRP_COUNT];
	unsigned long base;
	void *virt;
	int i;

	if (dmx->sec_pages)
		return 0;

	for (i = 0; i < AML_SEC_BUF_GRP_COUNT; i++)
		grp_len[i] = (1 << AML_SEC_GRP_LEN_SHIFT) * 8;
	dmx->sec_total_len = grp_len[0] * AML_SEC_BUF_GRP_COUNT;

	virt = (void *)__get_free_pages(GFP_KERNEL,
					get_order(dmx->sec_total_len));
	if (!virt) {
		dev_err(dvb->dev, "dmx%d: sec_buf alloc failed\n", dmx->id);
		return -ENOMEM;
	}
	dmx->sec_pages = (unsigned long)virt;
	dmx->sec_pages_map = dma_map_single(dvb->dev, virt, dmx->sec_total_len,
					    DMA_FROM_DEVICE);
	if (dma_mapping_error(dvb->dev, dmx->sec_pages_map)) {
		free_pages(dmx->sec_pages, get_order(dmx->sec_total_len));
		dmx->sec_pages = 0;
		return -ENOMEM;
	}

	grp_addr[0] = dmx->sec_pages_map;
	for (i = 1; i < AML_SEC_BUF_GRP_COUNT; i++)
		grp_addr[i] = grp_addr[i - 1] + grp_len[i - 1];

	dmx->sec_buf[0].addr = dmx->sec_pages;
	dmx->sec_buf[0].len = grp_len[0] / 8;
	for (i = 1; i < AML_SEC_BUF_COUNT; i++) {
		dmx->sec_buf[i].addr =
			dmx->sec_buf[i - 1].addr + dmx->sec_buf[i - 1].len;
		dmx->sec_buf[i].len = grp_len[i / 8] / 8;
	}

	base = grp_addr[0] & 0xFFFF0000UL;
	aml_write_reg(dvb, SEC_BUFF_BASE(dmx->id), (u32)(base >> 16));
	aml_write_reg(dvb, SEC_BUFF_01_START(dmx->id),
		      (u32)((((grp_addr[0] - base) >> 8) << 16) |
			    ((grp_addr[1] - base) >> 8)));
	aml_write_reg(dvb, SEC_BUFF_23_START(dmx->id),
		      (u32)((((grp_addr[2] - base) >> 8) << 16) |
			    ((grp_addr[3] - base) >> 8)));
	aml_write_reg(dvb, SEC_BUFF_SIZE(dmx->id),
		      (u32)(AML_SEC_GRP_LEN_SHIFT |
			    (AML_SEC_GRP_LEN_SHIFT << 4) |
			    (AML_SEC_GRP_LEN_SHIFT << 8) |
			    (AML_SEC_GRP_LEN_SHIFT << 12)));

	dev_info(dvb->dev,
		 "dmx%d: sec_buf OK %d bytes phys=0x%08llx base=0x%04llx\n",
		 dmx->id, dmx->sec_total_len,
		 (unsigned long long)dmx->sec_pages_map,
		 (unsigned long long)(dmx->sec_pages_map >> 16));
	dev_info(
		dvb->dev,
		"dmx%d: sec_buf grp0=0x%08llx grp1=0x%08llx grp2=0x%08llx grp3=0x%08llx\n",
		dmx->id, (unsigned long long)grp_addr[0],
		(unsigned long long)grp_addr[1],
		(unsigned long long)grp_addr[2],
		(unsigned long long)grp_addr[3]);
	return 0;
}

static void sec_buf_free(struct aml_dmx *dmx)
{
	if (!dmx->sec_pages)
		return;
	dma_unmap_single(dmx->dvb->dev, dmx->sec_pages_map, dmx->sec_total_len,
			 DMA_FROM_DEVICE);
	free_pages(dmx->sec_pages, get_order(dmx->sec_total_len));
	dmx->sec_pages = 0;
	dmx->sec_pages_map = 0;
}

/* ==================================================================
 * HW/SW Filter transition mechanism
 *
 * HW path (default): SECTION_AHB_DMA_EN=1, sec_buf DMA
 *   → SEC_BUFF_READY IRQ → process_section() → cb.sec()
 *
 * SW path (fallback): BYPASS_AHB_DMA_EN=1, raw TS → AsyncFIFO
 *   → dvb_dmx_swfilter() → Linux DVB core
 *   Triggers: 32 HW slots full, PID=0x2000, force_*_sf param
 *
 * Important: SF path does not use a separate DMX/AFIFO (we are not
 * repeating the vendor's mistake). Same DMX, only DEMUX_MEM_REQ_EN changes.
 * ================================================================== */

/* sf_feed_sf — 1: HW path kullan, 0: SW path kullan */
static int sf_feed_sf(struct aml_dmx *dmx, struct dvb_demux_feed *feed,
		      int add_not_remove)
{
	/* If already in SW mode, stay in SW */
	if (dmx->sf_mode)
		return 0;

	/* All-pass: HW PID filter 0x2000 yapamaz */
	if (feed->pid == 0x2000)
		return 0;

	/* Debug parameters */
	if (force_all_sf)
		return 0;
	if (feed->type == DMX_TYPE_TS && force_pes_sf)
		return 0;
	if (feed->type == DMX_TYPE_SEC && force_sec_sf)
		return 0;

	/* 32 HW slot dolu → SW fallback */
	if (add_not_remove && dmx->hw_pid_count >= AML_CHANNEL_COUNT)
		return 0;

	return 1; /* HW */
}

/* sf_add_feed — put feed into SW filter mode
 * SF path: BYPASS_AHB_DMA_EN on the same DMX → AsyncFIFO → swfilter
 * NOTE: We do not use DMX2/AFIFO1 like the vendor — unnecessary complexity. */
static void sf_add_feed(struct aml_dmx *dmx, struct dvb_demux_feed *feed)
{
	dmx->sf_mode = true;
	dmx_enable(dmx); /* BYPASS_AHB_DMA_EN=1 set edilir */
	dev_info(dmx->dvb->dev,
		 "dmx%d: PID=0x%04x → SW filter (hw_slots=%d/%d)\n", dmx->id,
		 feed->pid, dmx->hw_pid_count, AML_CHANNEL_COUNT);
}

/* sf_remove_feed — feed has left SW mode; return to HW if possible */
static void sf_remove_feed(struct aml_dmx *dmx, struct dvb_demux_feed *feed)
{
	/* If no active SW feeds remain and HW slots are available, return to HW */
	if (dmx->sf_mode && dmx->hw_pid_count < AML_CHANNEL_COUNT) {
		dmx->sf_mode = false;
		dmx_enable(dmx); /* SECTION_AHB_DMA_EN=1 geri gelir */
		dev_info(dmx->dvb->dev,
			 "dmx%d: PID=0x%04x → returned to HW filter\n", dmx->id,
			 feed->pid);
	}
}

/* ======================================================================
 * FM (Filter Memory) write helpers — vendor dmx_set_chan/filter_regs
 *
 * FM word format (32-bit):
 *   [31:16] = even channel/filter entry
 *   [15:0]  = odd  channel/filter entry
 *   addr = index >> 1  ← hardware architecture, 2 entries = 1 word
 *
 * FM_WR_ADDR[15] = WR_REQUEST strobe (hardware clears)
 * FM_WR_ADDR[31:24] = advance byte (filter count info)
 * ====================================================================== */

static int fm_wait_ready(struct aml_dmx *dmx)
{
	int retry = 5000;
	u32 v = 0;

	while (retry--) {
		aml_read_reg(dmx->dvb, FM_WR_ADDR(dmx->id), &v);
		if (!(v & FM_WR_DATA_REQUEST))
			return 0;
		udelay(1);
	}
	dev_warn(dmx->dvb->dev, "dmx%d: FM busy timeout\n", dmx->id);
	return -ETIMEDOUT;
}

/* chan_target — vendor dmx_get_chan_target birebir */
static u32 chan_target(struct aml_dmx *dmx, int cid)
{
	u32 type;

	if (cid < 0 || cid >= AML_CHANNEL_COUNT || !dmx->channel[cid].used)
		return FM_CHAN_UNUSED_TARGET;

	if (dmx->channel[cid].type == DMX_TYPE_SEC) {
		type = FM_PKT_SECTION;
	} else {
		switch (dmx->channel[cid].pes_type) {
		case DMX_PES_VIDEO:
			type = FM_PKT_VIDEO;
			break;
		case DMX_PES_AUDIO:
			type = FM_PKT_AUDIO;
			break;
		case DMX_PES_SUBTITLE:
		case DMX_PES_TELETEXT:
			type = FM_PKT_SECTION;
			break;
		case DMX_PES_PCR:
			type = FM_PKT_SCR;
			break;
		default:
			type = FM_PKT_RECORDER;
			break;
		}
	}
	dmx->channel[cid].pkt_type = type;
	return ((u32)type << FM_PID_TYPE_SHIFT) |
	       (dmx->channel[cid].pid & 0x1FFF);
}

/*
 * filter_target — vendor dmx_get_filter_target birebir
 *
 * Byte 0 (table_id): MASKHIGH/MASKLOW ile nibble maskeleme
 * Byte 1-2: always zero (section length, reserved)
 * Byte 3-14: value comparison with MASK/MASK_EQ
 */
static u32 filter_target_byte(struct aml_dmx *dmx, int fid, int byte_idx)
{
	struct dmx_section_filter *filter;
	int cid, neq_bytes, i;
	u32 target = 0;

	if (fid < 0 || fid >= AML_FILTER_COUNT || !dmx->filter[fid].used) {
		/* unused: byte0=0x1FFF, others=0x9FFF */
		return (byte_idx == 0) ? 0x1FFF : 0x9FFF;
	}

	cid = dmx->filter[fid].chan_id;
	filter = dmx->filter[fid].filter;

	/* neq_bytes count */
	neq_bytes = 0;
	if (filter->filter_mode[0] != 0xFF) {
		neq_bytes = 2;
	} else {
		for (i = 3; i < AML_FILTER_LEN; i++)
			if (filter->filter_mode[i] != 0xFF)
				neq_bytes++;
	}

	if (byte_idx == 0) {
		/* table_id byte */
		u8 value = filter->filter_value[0];
		u8 mask = filter->filter_mask[0];
		u8 mode = filter->filter_mode[0];
		u8 v = 0, adv = 0;
		u32 mb = 1, mb1 = 1;

		if ((mode == 0xFF) && mask) {
			u8 t = mask & 0xF0;
			if (t) {
				mb1 = 0;
				adv |= t ^ 0xF0;
			}
			v |= (value & 0xF0) | adv;
			t = mask & 0x0F;
			if (t) {
				mb = 0;
				adv |= t ^ 0x0F;
			}
			v |= (value & 0x0F) | adv;
		}
		target = (mb1 << 15) | (mb << 14) | (0 << 13) |
			 ((u32)cid << FM_FB0_CID_SHIFT) | v;

	} else if (byte_idx < 3) {
		/* bytes 1-2: section length/reserved — skip */
		target = FM_FBN_MASK | (0 << 13) |
			 ((u32)cid << FM_FBN_CID_SHIFT);

	} else {
		/* bytes 3-14: payload match */
		u8 value = filter->filter_value[byte_idx];
		u8 mask = filter->filter_mask[byte_idx];
		u8 mode = filter->filter_mode[byte_idx];
		u32 mb = 1, nb = 0;
		u8 adv = 0, v = 0;

		if (mask) {
			if (mode == 0xFF) {
				mb = 0;
				nb = 0;
				adv = mask ^ 0xFF;
				v = value | adv;
			} else if (neq_bytes == 1) {
				mb = 0;
				nb = 1;
				adv = mask ^ 0xFF;
				v = value & ~adv;
			}
		}
		target = (mb << 15) | (nb << 14) | (0 << 13) |
			 ((u32)cid << FM_FBN_CID_SHIFT) | v;
	}
	return target;
}

/*
 * dmx_set_chan_regs — vendor dmx_set_chan_regs birebir
 * Two channels per FM word: addr = cid >> 1
 */
static int dmx_set_chan_regs(struct aml_dmx *dmx, int cid)
{
	u32 data, addr, max, cur;
	int even, odd, i;

	if (cid < 0 || cid >= AML_CHANNEL_COUNT)
		return -EINVAL;
	if (fm_wait_ready(dmx))
		return -ETIMEDOUT;

	/* Dual packing: even and odd channels together */
	if (cid & 1) {
		even = cid - 1;
		odd = cid;
	} else {
		even = cid;
		odd = cid + 1;
	}

	data = (chan_target(dmx, even) << 16) | chan_target(dmx, odd);
	addr = (u32)(cid >> 1);

	aml_write_reg(dmx->dvb, FM_WR_DATA(dmx->id), data);
	aml_write_reg(dmx->dvb, FM_WR_ADDR(dmx->id), 0x8000u | addr);

	/* MAX_FM_COMP_ADDR — update max channel index (bit[3:0] = max>>1) */
	for (max = AML_CHANNEL_COUNT - 1, i = AML_CHANNEL_COUNT - 1; i > 0; i--)
		if (dmx->channel[i].used) {
			max = i;
			break;
		}
	aml_read_reg(dmx->dvb, MAX_FM_COMP_ADDR(dmx->id), &cur);
	aml_write_reg(dmx->dvb, MAX_FM_COMP_ADDR(dmx->id),
		      (cur & 0xF0u) | ((u32)max >> 1));

	dev_dbg(dmx->dvb->dev,
		"dmx%d: FM chan cid=%d addr=0x%02x data=0x%08x\n", dmx->id, cid,
		addr, data);
	return 0;
}

/*
 * dmx_set_filter_regs — vendor dmx_set_filter_regs birebir
 * Two filters per FM row: addr = (fid>>1) | ((byte+1)<<4)
 */
static int dmx_set_filter_regs(struct aml_dmx *dmx, int fid)
{
	u32 data, addr, max, cur;
	int even_fid, odd_fid, b, i;

	if (fid < 0 || fid >= AML_FILTER_COUNT)
		return -EINVAL;

	if (fid & 1) {
		even_fid = fid - 1;
		odd_fid = fid;
	} else {
		even_fid = fid;
		odd_fid = fid + 1;
	}

	for (b = 0; b < AML_FILTER_LEN; b++) {
		if (fm_wait_ready(dmx))
			return -ETIMEDOUT;

		data = (filter_target_byte(dmx, even_fid, b) << 16) |
		       filter_target_byte(dmx, odd_fid, b);
		addr = ((u32)fid >> 1) | (((u32)b + 1) << 4);

		aml_write_reg(dmx->dvb, FM_WR_DATA(dmx->id), data);
		aml_write_reg(dmx->dvb, FM_WR_ADDR(dmx->id), 0x8000u | addr);
	}

	/* MAX_FM_COMP_ADDR — update max filter index (bit[7:4] = max>>1) */
	for (max = AML_FILTER_COUNT - 1, i = AML_FILTER_COUNT - 1; i > 0; i--)
		if (dmx->filter[i].used) {
			max = i;
			break;
		}
	aml_read_reg(dmx->dvb, MAX_FM_COMP_ADDR(dmx->id), &cur);
	aml_write_reg(dmx->dvb, MAX_FM_COMP_ADDR(dmx->id),
		      (cur & 0x0Fu) | (((u32)max >> 1) << 4));

	dev_dbg(dmx->dvb->dev,
		"dmx%d: FM filter fid=%d cid=%d even=%d odd=%d\n", dmx->id, fid,
		dmx->filter[fid].used ? dmx->filter[fid].chan_id : -1, even_fid,
		odd_fid);
	return 0;
}

/* ======================================================================
 * Channel / filter allocation
 * ====================================================================== */
static int dmx_alloc_chan(struct aml_dmx *dmx, int type, int pes_type, u16 pid)
{
	int id = -1, i;
	for (i = 0; i < AML_CHANNEL_COUNT; i++)
		if (!dmx->channel[i].used) {
			id = i;
			break;
		}
	if (id < 0)
		return -ENOSPC;
	dmx->channel[id].used = true;
	dmx->channel[id].pid = pid;
	dmx->channel[id].type = type;
	dmx->channel[id].pkt_type = (type == DMX_TYPE_SEC) ? 3 : 0;
	dmx->channel[id].pes_type = pes_type;
	dmx->channel[id].feed = NULL;
	dmx->channel[id].dvr_feed = NULL;
	dmx->channel[id].filter_count = 0;
	dmx->chan_count++;
	dmx->hw_pid_count++;
	dmx_enable(dmx);
	dmx_set_chan_regs(dmx, id);
	return id;
}

static void dmx_free_chan(struct aml_dmx *dmx, int cid)
{
	if (cid < 0 || cid >= AML_CHANNEL_COUNT || !dmx->channel[cid].used)
		return;
	dmx->channel[cid].used = false;
	dmx->channel[cid].pid = 0x1FFF;
	dmx->channel[cid].feed = NULL;
	dmx->channel[cid].dvr_feed = NULL;
	dmx->chan_count--;
	if (dmx->hw_pid_count > 0)
		dmx->hw_pid_count--;
	dmx->chan_record_bits &= ~BIT(cid);
	aml_write_reg(dmx->dvb, DEMUX_CHAN_RECORD_EN(dmx->id),
		      dmx->chan_record_bits);
	dmx_set_chan_regs(dmx, cid);
	dmx_enable(dmx);
}

static int dmx_alloc_filter(struct aml_dmx *dmx, int cid,
			    struct dvb_demux_filter *df)
{
	struct dmx_section_filter *sf = &df->filter;
	int id = -1, i;

	for (i = 0; i < AML_FILTER_COUNT; i++)
		if (!dmx->filter[i].used) {
			id = i;
			break;
		}
	if (id < 0)
		return -ENOSPC;

	/* filter_target_byte() reads sf->filter_value/mask/mode directly */
	dmx->filter[id].used = true;
	dmx->filter[id].chan_id = cid;
	dmx->filter[id].filter = sf;

	/* Populate value/maskandmode for sec_filter_match */
	for (i = 0; i < AML_FILTER_LEN; i++) {
		u8 m = sf->filter_mask[i];
		u8 mo = sf->filter_mode[i];
		dmx->filter[id].value[i] = sf->filter_value[i] & m;
		dmx->filter[id].maskandmode[i] = m & mo;
		dmx->filter[id].maskandnotmode[i] = m & (~mo);
		if (dmx->filter[id].maskandnotmode[i])
			dmx->filter[id].neq = true;
	}

	dmx->channel[cid].filter_count++;
	df->hw_handle = (u16)id;
	dmx_set_filter_regs(dmx, id);
	return id;
}

static void dmx_free_filter(struct aml_dmx *dmx, int fid)
{
	if (fid < 0 || fid >= AML_FILTER_COUNT || !dmx->filter[fid].used)
		return;
	dmx->channel[dmx->filter[fid].chan_id].filter_count--;
	dmx->filter[fid].used = false;
	dmx->filter[fid].chan_id = -1;
	dmx->filter[fid].filter = NULL;
}

/* ======================================================================
 * dmx_enable — DEMUX_CONTROL + MEM_REQ (vendor dmx_enable logic)
 * ====================================================================== */
static void dmx_enable(struct aml_dmx *dmx)
{
	struct aml_dvb *dvb = dmx->dvb;
	bool record = dmx->record;
	u32 fec_sel = 0, fec_ctrl = 0;

	if (!dmx->chan_count && !record) {
		aml_write_reg(dvb, STB_INT_MASK(dmx->id), 0);
		aml_write_reg(dvb, DEMUX_CONTROL(dmx->id), 0);
		return;
	}

	if (dmx->source >= AML_TS_SRC_FRONTEND_TS0 &&
	    dmx->source <= AML_TS_SRC_FRONTEND_TS3) {
		unsigned int p = dmx->source - AML_TS_SRC_FRONTEND_TS0;
		/*
		 * FEC_SEL[14:12] source selection:
		 *   0=TS0p(parallel)  1=TS1p  4=STS2(serial) 5=STS1 6=STS0
		 * For serial input (tsin_a_ao), STS0=6 must be used.
		 * For parallel input, port index maps directly to FEC_SEL.
		 * Source: vendor c_stb_regs_define.h + devmem verification.
		 */
		if (p < AML_MAX_TS_INPUT && dvb->ts[p].is_serial) {
			/* Serial: STS0=6, STS1=5, STS2=4 */
			fec_sel = 6 - p;
		} else {
			/* Parallel: TS0p=0, TS1p=1 */
			fec_sel = p;
		}
		fec_ctrl = dvb->ts[p].fec_ctrl & 0xFFF;
	}

	aml_write_reg(dvb, STB_INT_MASK(dmx->id), AML_DEMUX_INT_MASK);
	aml_write_reg(dvb, PES_STRONG_SYNC(dmx->id), 0x1234u);
	aml_write_reg(dvb, DEMUX_ENDIAN(dmx->id), AML_DEMUX_ENDIAN_VAL);
	aml_write_reg(dvb, TS_HIU_CTL(dmx->id), BIT(USE_HI_BSF_INTERFACE));
	aml_write_reg(dvb, FEC_INPUT_CONTROL(dmx->id),
		      ((u32)fec_sel << FEC_SEL_SHIFT) | fec_ctrl);
	aml_write_reg(dvb, STB_OM_CTL(dmx->id), (0x40u << 9) | 0x7Fu);
	aml_write_reg(dvb, VIDEO_STREAM_ID(dmx->id), record ? 0xFFFF0000u : 0u);
	/*
	 * TS_RECORDER_ENABLE: enables TS packet flow to the DMX.
	 * Even if SECTION_AHB_DMA_EN is active, if this bit is zero
	 * TS data does not reach the demux core → section filter gets no data.
	 * Therefore it is always set when there are active feeds (chan_count > 0)
	 * or recording is active (vendor hybrid architecture).
	 *
	 * TS_RECORDER_SELECT: only DVR recording (record=true) requires it.
	 *   0 = AsyncFIFO bypass (section-only mode)
	 *   1 = AsyncFIFO → DVR recording
	 */
	bool ts_en = (dmx->chan_count > 0) || record;
	/* Vendor devmem: section-only=0xC0000550, DVR=0xC3C00750
	 * TS_RECORDER_SELECT (bit10): vendor section modda da SET ediyor
	 * TS_RECORDER_ENABLE (bit9): sadece DVR modda SET
	 */
	aml_write_reg(dvb, DEMUX_CONTROL(dmx->id),
		      ENABLE_FREE_CLK_FEC_DATA_VALID | ENABLE_FREE_CLK_STB_REG |
			      TS_RECORDER_SELECT |
			      (record ? TS_RECORDER_ENABLE : 0u) |
			      SECTION_END_WITH_TABLE_ID |
			      KEEP_DUPLICATE_PACKAGE | STB_DEMUX_ENABLE |
			      (record ? VIDEO2_FOR_RECORDER_STREAM : 0u));
	/* HW/SW path selection: sf_mode=true → BYPASS (raw TS → swfilter)
	 *                     sf_mode=false → HW section DMA */
	if (dmx->sf_mode) {
		aml_write_reg(dvb, DEMUX_MEM_REQ_EN(dmx->id),
			      AML_MEM_REQ_EN_SW);
		dev_info(dvb->dev, "dmx%d: MEM_REQ=SW (BYPASS=1)\n", dmx->id);
	} else {
		aml_write_reg(dvb, DEMUX_MEM_REQ_EN(dmx->id),
			      AML_MEM_REQ_EN_HW);
		dev_info(dvb->dev, "dmx%d: MEM_REQ=HW (SECTION_AHB=1)\n",
			 dmx->id);
	}

	dev_info(
		dvb->dev,
		"dmx%d: dmx_enable CTRL=%08x MASK=%08x STB_EN=1 FEC=%08x END=%08x HIU=%08x\n",
		dmx->id,
		ENABLE_FREE_CLK_FEC_DATA_VALID | ENABLE_FREE_CLK_STB_REG |
			(record ? TS_RECORDER_SELECT : 0u) |
			(ts_en ? TS_RECORDER_ENABLE : 0u) |
			SECTION_END_WITH_TABLE_ID | KEEP_DUPLICATE_PACKAGE |
			STB_DEMUX_ENABLE |
			(record ? VIDEO2_FOR_RECORDER_STREAM : 0u),
		AML_DEMUX_INT_MASK, ((u32)fec_sel << FEC_SEL_SHIFT) | fec_ctrl,
		AML_DEMUX_ENDIAN_VAL, BIT(USE_HI_BSF_INTERFACE));
}

/* ======================================================================
 * start_feed / stop_feed
 * ====================================================================== */
int aml_dvb_start_feed(struct dvb_demux_feed *feed)
{
	struct aml_dmx *dmx = feed->demux->priv;
	struct aml_dvb *dvb = dmx->dvb;
	int cid = -1, ret;
	bool is_dvr;

	dev_info(dvb->dev, "dmx%d: start_feed PID=0x%04x type=%d\n", dmx->id,
		 feed->pid, feed->type);

	ret = pm_runtime_get_sync(dvb->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dvb->dev);
		return ret;
	}

	if (feed->type == DMX_TYPE_SEC) {
		/* HW or SW section filter path */
		if (!sf_feed_sf(dmx, feed, 1)) {
			/* SW fallback — 32 slots full or force_sec_sf */
			sf_add_feed(dmx, feed);
			atomic_inc(&dvb->feed_count);
			feed->priv = (void *)-1L; /* SW path marker */
			pm_runtime_mark_last_busy(dvb->dev);
			pm_runtime_put_autosuspend(dvb->dev);
			dev_info(dvb->dev, "dmx%d: PID=0x%04x → SW filter\n",
				 dmx->id, feed->pid);
			return 0;
		}
		/* HW section filter path */
		cid = dmx_alloc_chan(dmx, DMX_TYPE_SEC, DMX_PES_OTHER,
				     feed->pid);
		if (cid < 0) {
			ret = cid;
			goto err;
		}
		dmx->channel[cid].feed = feed;
		feed->priv = (void *)(long)cid;

		if (feed->filter) {
			struct dvb_demux_filter *df = feed->filter;
			while (df) {
				dmx_alloc_filter(dmx, cid, df);
				df = df->next;
			}
		}

		if (!dmx->sec_pages) {
			ret = sec_buf_alloc(dmx);
			if (ret) {
				dmx_free_chan(dmx, cid);
				goto err;
			}
		}

	} else {
		/* DVR / TS path */
		is_dvr = (feed->pid == 0x1FFF || (feed->ts_type & TS_PACKET));
		cid = dmx_alloc_chan(dmx, DMX_TYPE_TS, feed->pes_type,
				     feed->pid);
		if (cid < 0) {
			ret = cid;
			goto err;
		}
		dmx->channel[cid].feed = feed;
		feed->priv = (void *)(long)cid;

		if (is_dvr) {
			dmx->channel[cid].dvr_feed = feed;
			dmx->chan_record_bits |= BIT(cid);
			aml_write_reg(dvb, DEMUX_CHAN_RECORD_EN(dmx->id),
				      dmx->chan_record_bits);
			if (!dmx->record) {
				dmx->record = true;
				dmx_enable(dmx);
				dev_info(dvb->dev, "dmx%d: record=1\n",
					 dmx->id);
			}
		}
	}

	atomic_inc(&dvb->feed_count);
	dev_info(
		dvb->dev,
		"dmx%d: start_feed OK cid=%d CTRL=0x%08x MASK=0x%08x STATUS=0x%x\n",
		dmx->id, cid,
		ENABLE_FREE_CLK_FEC_DATA_VALID | ENABLE_FREE_CLK_STB_REG |
			STB_DEMUX_ENABLE,
		AML_DEMUX_INT_MASK, 0);
	return 0;
err:
	pm_runtime_put_noidle(dvb->dev);
	return ret;
}
EXPORT_SYMBOL_GPL(aml_dvb_start_feed);

int aml_dvb_stop_feed(struct dvb_demux_feed *feed)
{
	struct aml_dmx *dmx = feed->demux->priv;
	struct aml_dvb *dvb = dmx->dvb;
	int cid = (int)(long)feed->priv;

	dev_info(dvb->dev, "dmx%d: stop_feed PID=0x%04x cid=%d\n", dmx->id,
		 feed->pid, cid);

	if (feed->type == DMX_TYPE_SEC) {
		/* SW path marker */
		if ((long)feed->priv == -1L) {
			sf_remove_feed(dmx, feed);
			atomic_dec(&dvb->feed_count);
			feed->priv = NULL;
			pm_runtime_mark_last_busy(dvb->dev);
			pm_runtime_put_autosuspend(dvb->dev);
			return 0;
		}
		if (feed->filter) {
			struct dvb_demux_filter *df = feed->filter;
			while (df) {
				if (df->hw_handle != (u16)-1) {
					dmx_free_filter(dmx,
							(int)df->hw_handle);
					df->hw_handle = (u16)-1;
				}
				df = df->next;
			}
		}
		if (cid >= 0)
			dmx_free_chan(dmx, cid);

	} else {
		bool is_dvr =
			(feed->pid == 0x1FFF || (feed->ts_type & TS_PACKET));
		if (is_dvr && cid >= 0) {
			dmx->chan_record_bits &= ~BIT(cid);
			aml_write_reg(dvb, DEMUX_CHAN_RECORD_EN(dmx->id),
				      dmx->chan_record_bits);
			if (!dmx->chan_record_bits && dmx->record) {
				dmx->record = false;
				dmx_enable(dmx);
				dev_info(dvb->dev, "dmx%d: record=0\n",
					 dmx->id);
			}
		}
		if (cid >= 0)
			dmx_free_chan(dmx, cid);
	}

	atomic_dec(&dvb->feed_count);
	feed->priv = NULL;
	pm_runtime_mark_last_busy(dvb->dev);
	pm_runtime_put_autosuspend(dvb->dev);
	return 0;
}
EXPORT_SYMBOL_GPL(aml_dvb_stop_feed);

/* ======================================================================
 * DVB layer
 * ====================================================================== */
int aml_dmx_dvb_init(struct aml_dmx *dmx, struct dvb_adapter *adap)
{
	int ret;
	dmx->demux.priv = dmx;
	dmx->demux.filternum = AML_FILTER_COUNT;
	dmx->demux.feednum = AML_CHANNEL_COUNT;
	dmx->demux.start_feed = aml_dvb_start_feed;
	dmx->demux.stop_feed = aml_dvb_stop_feed;
	dmx->demux.dmx.capabilities = DMX_TS_FILTERING | DMX_SECTION_FILTERING |
				      DMX_MEMORY_BASED_FILTERING;
	ret = dvb_dmx_init(&dmx->demux);
	if (ret)
		return ret;
	dmx->dmxdev.filternum = dmx->demux.filternum;
	dmx->dmxdev.demux = &dmx->demux.dmx;
	dmx->dmxdev.capabilities = 0;
	ret = dvb_dmxdev_init(&dmx->dmxdev, adap);
	if (ret) {
		dvb_dmx_release(&dmx->demux);
		return ret;
	}
	ret = dvb_net_init(adap, &dmx->dvbnet, &dmx->demux.dmx);
	if (ret) {
		dvb_dmxdev_release(&dmx->dmxdev);
		dvb_dmx_release(&dmx->demux);
		return ret;
	}
	dev_info(dmx->dvb->dev, "dmx%d: dvb_init OK\n", dmx->id);
	return 0;
}
EXPORT_SYMBOL_GPL(aml_dmx_dvb_init);

void aml_dmx_dvb_release(struct aml_dmx *dmx)
{
	dvb_net_release(&dmx->dvbnet);
	dvb_dmxdev_release(&dmx->dmxdev);
	dvb_dmx_release(&dmx->demux);
}
EXPORT_SYMBOL_GPL(aml_dmx_dvb_release);

/* ======================================================================
 * Hardware init / release
 * ====================================================================== */
int aml_timeout_init(struct aml_dmx *dmx)
{
	aml_write_reg(dmx->dvb, DEMUX_INPUT_TIMEOUT(dmx->id),
		      AML_TIMEOUT_DEFAULT);
	return 0;
}
EXPORT_SYMBOL_GPL(aml_timeout_init);
void aml_timeout_release(struct aml_dmx *dmx)
{
	aml_write_reg(dmx->dvb, DEMUX_INPUT_TIMEOUT(dmx->id), 0);
}
EXPORT_SYMBOL_GPL(aml_timeout_release);
int aml_smallsec_init(struct aml_dmx *dmx)
{
	aml_write_reg(dmx->dvb, DEMUX_SMALL_SEC_CTL(dmx->id), 0);
	return 0;
}
EXPORT_SYMBOL_GPL(aml_smallsec_init);
void aml_smallsec_release(struct aml_dmx *dmx)
{
	aml_write_reg(dmx->dvb, DEMUX_SMALL_SEC_CTL(dmx->id), 0);
}
EXPORT_SYMBOL_GPL(aml_smallsec_release);

int aml_dmx_hw_init(struct aml_dmx *dmx)
{
	struct aml_dvb *dvb = dmx->dvb;
	struct device *dev = dvb->dev;
	int i, ret;

	dev_info(dev, "dmx%d: hw_init\n", dmx->id);
	spin_lock_init(&dmx->lock);
	spin_lock_init(&dmx->pid_lock);
	spin_lock_init(&dmx->sec_lock);
	atomic_set(&dmx->dvr_feed_count, 0);
	bitmap_zero(dmx->pid_bitmap, AML_HW_PID_SLOTS);
	u64_stats_init(&dmx->stats.syncp);

	dmx->chan_count = 0;
	dmx->record = false;
	dmx->chan_record_bits = 0;
	dmx->sec_pages = 0;
	dmx->sec_pages_map = 0;
	dmx->source = AML_TS_SRC_NONE;
	dmx->sf_mode = false;
	dmx->hw_pid_count = 0;
	memset(dmx->sec_buf_watchdog_count, 0,
	       sizeof(dmx->sec_buf_watchdog_count));
	for (i = 0; i < AML_CHANNEL_COUNT; i++)
		dmx->channel[i].used = false;
	for (i = 0; i < AML_FILTER_COUNT; i++) {
		dmx->filter[i].used = false;
		dmx->filter[i].chan_id = -1;
	}

	dmx->dvr_buf_size = SZ_4M;
	dmx->dvr_buf = dma_alloc_coherent(dev, dmx->dvr_buf_size, &dmx->dvr_dma,
					  GFP_KERNEL);
	if (!dmx->dvr_buf)
		dev_warn(dev, "dmx%d: DVR buf alloc failed\n", dmx->id);
	else
		dev_info(dev, "dmx%d: DVR buf OK phys=%pad\n", dmx->id,
			 &dmx->dvr_dma);

	ret = aml_smallsec_init(dmx);
	if (ret)
		goto err;
	ret = aml_timeout_init(dmx);
	if (ret)
		goto err_ss;

	dev_info(dev, "dmx%d: hw_init OK\n", dmx->id);
	return 0;
err_ss:
	aml_smallsec_release(dmx);
err:
	if (dmx->dvr_buf)
		dma_free_coherent(dev, dmx->dvr_buf_size, dmx->dvr_buf,
				  dmx->dvr_dma);
	return ret;
}
EXPORT_SYMBOL_GPL(aml_dmx_hw_init);

void aml_dmx_hw_release(struct aml_dmx *dmx)
{
	struct aml_dvb *dvb = dmx->dvb;
	aml_write_reg(dvb, STB_INT_MASK(dmx->id), 0);
	aml_write_reg(dvb, DEMUX_CONTROL(dmx->id), 0);
	sec_buf_free(dmx);
	aml_smallsec_release(dmx);
	aml_timeout_release(dmx);
	if (dmx->dvr_buf) {
		dma_free_coherent(dvb->dev, dmx->dvr_buf_size, dmx->dvr_buf,
				  dmx->dvr_dma);
		dmx->dvr_buf = NULL;
	}
}
EXPORT_SYMBOL_GPL(aml_dmx_hw_release);

/* ======================================================================
 * TS source
 * ====================================================================== */
int aml_dmx_set_source(struct aml_dmx *dmx, unsigned int source)
{
	struct aml_dvb *dvb = dmx->dvb;

	dmx->source = source;

	if (source >= AML_TS_SRC_FRONTEND_TS0 &&
	    source <= AML_TS_SRC_FRONTEND_TS3) {
		unsigned int p = source - AML_TS_SRC_FRONTEND_TS0;
		/* Serial: STS0=6, STS1=5, STS2=4 — Parallel: TS0p=0, TS1p=1 */
		u32 fec_sel_val =
			(p < AML_MAX_TS_INPUT && dvb->ts[p].is_serial) ?
				(6 - p) :
				p;
		u32 fec = (fec_sel_val << FEC_SEL_SHIFT) |
			  (dvb->ts[p].fec_ctrl & 0xFFF);
		u32 stb = 0;

		aml_write_reg(dvb, FEC_INPUT_CONTROL(dmx->id), fec);
		/* STB_TOP_CONFIG: vendor = 0x00000000, not written */
		dev_info(
			dvb->dev,
			"dmx%d: source=%d STB_TOP=0x00000000 FEC_CTRL=0x%08x\n",
			dmx->id, source, fec);
		dev_info(dvb->dev,
			 "Demux %d <- tsin_%c (AML_TS_SRC_FRONTEND_TS%d)\n",
			 dmx->id, 'a' + p, p);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(aml_dmx_set_source);

/* Stubs — backwards compatibility */
void aml_dmx_enable_recording(struct aml_dmx *dmx, bool enable)
{
	dev_dbg(dmx->dvb->dev,
		"dmx%d: enable_recording(%d) managed by stop_feed\n", dmx->id,
		enable);
}
EXPORT_SYMBOL_GPL(aml_dmx_enable_recording);

int aml_dmx_hw_pid_alloc(struct aml_dmx *dmx, u16 pid)
{
	return -ENOSYS;
}
EXPORT_SYMBOL_GPL(aml_dmx_hw_pid_alloc);
void aml_dmx_hw_pid_free(struct aml_dmx *dmx, int slot)
{
}
EXPORT_SYMBOL_GPL(aml_dmx_hw_pid_free);
