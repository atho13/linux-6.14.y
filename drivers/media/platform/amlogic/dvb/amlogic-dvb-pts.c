// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic DVB PTS/PCR Extraction
 *
 * Copyright (C) 2026 Kağan Kadioğlu <kagankadioglutk@hotmail.com>
 */

#include "amlogic_dvb.h"
#include "amlogic-dvb-regs.h"

u64 aml_dmx_get_video_pts(struct aml_dmx *dmx)
{
	struct aml_dvb *dvb = dmx->dvb;
	u32 pts_lo, pts_hi, pts_hi2;
	u64 pts;
	int ret;

	/* Race fix: pts_hi is read twice. pts_lo is read in between.
	 * If pts_hi has changed, a 32-bit overflow has occurred — retry. */
	do {
		ret = aml_read_reg(dvb, STB_PTS_DTS_STATUS(dmx->id), &pts_hi);
		if (ret)
			return 0;
		ret = aml_read_reg(dvb, VIDEO_PTS_DEMUX(dmx->id), &pts_lo);
		if (ret)
			return 0;
		ret = aml_read_reg(dvb, STB_PTS_DTS_STATUS(dmx->id), &pts_hi2);
		if (ret)
			return 0;
	} while (pts_hi != pts_hi2);

	pts = pts_lo;
	if (pts_hi & VIDEO_PTS_BIT32)
		pts |= BIT_ULL(32);

	return pts;
}
EXPORT_SYMBOL_GPL(aml_dmx_get_video_pts);

u64 aml_dmx_get_audio_pts(struct aml_dmx *dmx)
{
	struct aml_dvb *dvb = dmx->dvb;
	u32 pts_lo, pts_hi, pts_hi2;
	u64 pts;
	int ret;

	do {
		ret = aml_read_reg(dvb, STB_PTS_DTS_STATUS(dmx->id), &pts_hi);
		if (ret)
			return 0;
		ret = aml_read_reg(dvb, AUDIO_PTS_DEMUX(dmx->id), &pts_lo);
		if (ret)
			return 0;
		ret = aml_read_reg(dvb, STB_PTS_DTS_STATUS(dmx->id), &pts_hi2);
		if (ret)
			return 0;
	} while (pts_hi != pts_hi2);

	pts = pts_lo;
	if (pts_hi & AUDIO_PTS_BIT32)
		pts |= BIT_ULL(32);

	return pts;
}
EXPORT_SYMBOL_GPL(aml_dmx_get_audio_pts);

u64 aml_dmx_get_pcr(struct aml_dmx *dmx)
{
	struct aml_dvb *dvb = dmx->dvb;
	u32 pcr_lo, status, status2;
	u64 pcr;
	int ret;

	do {
		ret = aml_read_reg(dvb, STB_PTS_DTS_STATUS(dmx->id), &status);
		if (ret)
			return 0;
		ret = aml_read_reg(dvb, PCR_DEMUX(dmx->id), &pcr_lo);
		if (ret)
			return 0;
		ret = aml_read_reg(dvb, STB_PTS_DTS_STATUS(dmx->id), &status2);
		if (ret)
			return 0;
	} while (status != status2);

	pcr = pcr_lo;
	if (status & PCR_BIT32)
		pcr |= BIT_ULL(32);
	return pcr;
}
EXPORT_SYMBOL_GPL(aml_dmx_get_pcr);