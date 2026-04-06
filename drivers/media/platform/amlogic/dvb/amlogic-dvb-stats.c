// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic DVB Enhanced Statistics Module (u64_stats_t based)
 *
 * Copyright (C) 2026 Kağan Kadioğlu <kagankadioglutk@hotmail.com>
 */

#include <linux/u64_stats_sync.h>
#include "amlogic_dvb.h"

void aml_stats_init(struct aml_dmx *dmx)
{
	u64_stats_init(&dmx->stats.syncp);
}
EXPORT_SYMBOL_GPL(aml_stats_init);

void aml_stats_cleanup(struct aml_dmx *dmx)
{
	/* Nothing */
}
EXPORT_SYMBOL_GPL(aml_stats_cleanup);

void aml_stats_update_packet(struct aml_dmx *dmx, int type, u64 count)
{
	u64_stats_update_begin(&dmx->stats.syncp);
	u64_stats_add(&dmx->stats.ts_packets, count);
	if (type == 0)
		u64_stats_add(&dmx->stats.section_count, count);
	else if (type == 1)
		u64_stats_add(&dmx->stats.pes_count, count);
	u64_stats_update_end(&dmx->stats.syncp);
}
EXPORT_SYMBOL_GPL(aml_stats_update_packet);

void aml_stats_update_error(struct aml_dmx *dmx, int error_type)
{
	u64_stats_update_begin(&dmx->stats.syncp);
	switch (error_type) {
	case 0:
		u64_stats_inc(&dmx->stats.ts_errors);
		break;
	case 1:
		u64_stats_inc(&dmx->stats.crc_errors);
		break;
	case 2:
		u64_stats_inc(&dmx->stats.overflow_errors);
		break;
	case 3:
		u64_stats_inc(&dmx->stats.cc_errors);
		break;
	/* BUG-11: missing cases added */
	case 4:
		u64_stats_inc(&dmx->stats.fifo_errors);
		break;
	case 5:
		u64_stats_inc(&dmx->stats.ts_drops);
		break;
	default:
		break;
	}
	u64_stats_update_end(&dmx->stats.syncp);
}
EXPORT_SYMBOL_GPL(aml_stats_update_error);

int aml_stats_get_summary(struct aml_dmx *dmx, char *buf, size_t size)
{
	unsigned int start;
	u64 ts_packets, section_count, pes_count, ts_errors, crc_errors,
		overflow_errors, cc_errors, pcr_count, fifo_errors, ts_drops,
		irq_count, tasklet_count;

	do {
		start = u64_stats_fetch_begin(&dmx->stats.syncp);
		ts_packets = u64_stats_read(&dmx->stats.ts_packets);
		section_count = u64_stats_read(&dmx->stats.section_count);
		pes_count = u64_stats_read(&dmx->stats.pes_count);
		ts_errors = u64_stats_read(&dmx->stats.ts_errors);
		crc_errors = u64_stats_read(&dmx->stats.crc_errors);
		overflow_errors = u64_stats_read(&dmx->stats.overflow_errors);
		cc_errors = u64_stats_read(&dmx->stats.cc_errors);
		pcr_count = u64_stats_read(&dmx->stats.pcr_count);
		fifo_errors = u64_stats_read(&dmx->stats.fifo_errors);
		ts_drops = u64_stats_read(&dmx->stats.ts_drops);
		irq_count = u64_stats_read(&dmx->stats.irq_count);
		tasklet_count = u64_stats_read(&dmx->stats.tasklet_count);
	} while (u64_stats_fetch_retry(&dmx->stats.syncp, start));

	return snprintf(buf, size,
			"Demux %d Statistics:\n"
			"  TS Packets:    %llu\n"
			"  TS Errors:     %llu\n"
			"  CRC Errors:    %llu\n"
			"  Overflows:     %llu\n"
			"  CC Errors:     %llu\n"
			"  Sections:      %llu\n"
			"  PES Packets:   %llu\n"
			"  PCR Count:     %llu\n"
			"  FIFO Errors:   %llu\n"
			"  TS Drops:      %llu\n"
			"  IRQ Count:     %llu\n"
			"  Tasklet Runs:  %llu\n",
			dmx->id, ts_packets, ts_errors, crc_errors,
			overflow_errors, cc_errors, section_count, pes_count,
			pcr_count, fifo_errors, ts_drops, irq_count,
			tasklet_count);
}
EXPORT_SYMBOL_GPL(aml_stats_get_summary);

void aml_stats_reset(struct aml_dmx *dmx)
{
	aml_stats_init(dmx);
}
EXPORT_SYMBOL_GPL(aml_stats_reset);