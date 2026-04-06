// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic DVB debugfs interface
 *
 * Copyright (C) 2026 Kağan Kadioğlu <kagankadioglutk@hotmail.com>
 */

#ifdef CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include "amlogic_dvb.h"
#include "amlogic-dvb-regs.h"

/* BUG-9: aml_dvb_debug is now u32 */
extern u32 aml_dvb_debug;

static const struct debugfs_reg32 aml_dvb_debug_regs[] = {
	{ "DEMUX_CONTROL_0", DEMUX_CONTROL(0) },
	{ "FEC_SYNC_BYTE_0", FEC_SYNC_BYTE(0) },
	{ "STB_INT_STATUS_0", STB_INT_STATUS(0) },
	{ "STB_INT_MASK_0", STB_INT_MASK(0) },
	{ "PCR_DEMUX_0", PCR_DEMUX(0) },
	{ "VIDEO_PTS_DEMUX_0", VIDEO_PTS_DEMUX(0) },
	{ "AUDIO_PTS_DEMUX_0", AUDIO_PTS_DEMUX(0) },
	{ "STB_PTS_DTS_STATUS_0", STB_PTS_DTS_STATUS(0) },
	{ "DEMUX_CONTROL_1", DEMUX_CONTROL(1) },
	{ "STB_INT_STATUS_1", STB_INT_STATUS(1) },
	{ "DEMUX_CONTROL_2", DEMUX_CONTROL(2) },
	{ "STB_INT_STATUS_2", STB_INT_STATUS(2) },
	{ "TS_TOP_CONFIG", TS_TOP_CONFIG },
	{ "TS_PL_PID_INDEX", TS_PL_PID_INDEX },
	{ "TS_PL_PID_DATA", TS_PL_PID_DATA },
};

static int aml_dvb_caps_show(struct seq_file *s, void *unused)
{
	struct aml_dvb *dvb = s->private;
	struct aml_dvb_hw_caps *caps = &dvb->caps;

	seq_printf(s, "SoC: %s\n", caps->soc_name);
	seq_printf(s, "Demux cores: %d\n", caps->num_demux);
	seq_printf(s, "TS inputs: %d\n", caps->num_ts_inputs);
	seq_printf(s, "S2P converters: %d\n", caps->num_s2p);
	seq_printf(s, "PID filters: %d\n", caps->num_pid_filters);
	seq_printf(s, "Section filters: %d\n", caps->num_sec_filters);
	seq_printf(s, "Async FIFOs: %d\n", caps->num_asyncfifo);
	seq_printf(s, "Descrambler: %s\n",
		   caps->has_descrambler ? "Yes" : "No");
	seq_printf(s, "CI+: %s\n", caps->has_ciplus ? "Yes" : "No");
	seq_printf(s, "Hardware filtering: %s\n",
		   caps->has_hwfilter ? "Yes" : "No");
	seq_printf(s, "DMA: %s\n", caps->has_dma ? "Yes" : "No");
	seq_printf(s, "Max bitrate: %d Mbps\n", caps->max_bitrate);

	if (dvb->tuner_reset_gpio)
		seq_puts(s, "Tuner reset GPIO: Available\n");
	if (dvb->tuner_power_gpio)
		seq_puts(s, "Tuner power GPIO: Available\n");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(aml_dvb_caps);

static int aml_dvb_debugfs_stats_show(struct seq_file *s, void *v)
{
	struct aml_dvb *dvb = s->private;
	unsigned int start;
	u64 irq, pkt;

	do {
		start = u64_stats_fetch_begin(&dvb->stats_syncp);
		irq = u64_stats_read(&dvb->stats_irq);
		pkt = u64_stats_read(&dvb->stats_packet);
	} while (u64_stats_fetch_retry(&dvb->stats_syncp, start));

	seq_printf(s, "IRQ count:    %llu\n", irq);
	seq_printf(s, "Packet count: %llu\n", pkt);
	/* BUG-4: feed_count is read atomically */
	seq_printf(s, "Feed count:   %d\n", atomic_read(&dvb->feed_count));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(aml_dvb_debugfs_stats);

static int aml_dvb_debugfs_demux_show(struct seq_file *s, void *v)
{
	struct aml_dmx *dmx = s->private;
	int i, a_chan = 0, a_filt = 0;
	unsigned int start;
	u64 ts_err, crc_err, ovf_err, cc_err, fifo_err, ts_drop;

	seq_printf(s, "Demux %d\n", dmx->id);
	seq_printf(s, "  source: %u\n", dmx->source);
	seq_printf(s, "  suspended: %d\n", dmx->suspended);

	do {
		start = u64_stats_fetch_begin(&dmx->stats.syncp);
		ts_err = u64_stats_read(&dmx->stats.ts_errors);
		crc_err = u64_stats_read(&dmx->stats.crc_errors);
		ovf_err = u64_stats_read(&dmx->stats.overflow_errors);
		cc_err = u64_stats_read(&dmx->stats.cc_errors);
		fifo_err = u64_stats_read(&dmx->stats.fifo_errors);
		ts_drop = u64_stats_read(&dmx->stats.ts_drops);
	} while (u64_stats_fetch_retry(&dmx->stats.syncp, start));

	seq_printf(s, "  TS errors: %llu\n", ts_err);
	seq_printf(s, "  CRC errors: %llu\n", crc_err);
	seq_printf(s, "  overflow: %llu\n", ovf_err);
	seq_printf(s, "  CC errors: %llu\n", cc_err);
	seq_printf(s, "  FIFO errors: %llu\n", fifo_err);
	seq_printf(s, "  TS drops: %llu\n", ts_drop);
	seq_printf(s, "  PCR: %llu\n", dmx->pcr);

	for (i = 0; i < AML_CHANNEL_COUNT; i++)
		if (dmx->channel[i].used)
			a_chan++;
	for (i = 0; i < AML_FILTER_COUNT; i++)
		if (dmx->filter[i].used)
			a_filt++;

	seq_printf(s, "  active channels: %d/%d\n", a_chan, AML_CHANNEL_COUNT);
	seq_printf(s, "  active filters: %d/%d\n", a_filt, AML_FILTER_COUNT);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(aml_dvb_debugfs_demux);

void aml_dvb_debugfs_init(struct aml_dvb *dvb)
{
	struct dentry *root;
	char name[16];
	int i;

	root = debugfs_create_dir(dev_name(dvb->dev), NULL);
	if (!root) {
		dev_err(dvb->dev, "Failed to create debugfs directory\n");
		return;
	}

	dvb->debugfs_root = root;

	dvb->regset.regs = aml_dvb_debug_regs;
	dvb->regset.nregs = ARRAY_SIZE(aml_dvb_debug_regs);
	dvb->regset.base = dvb->base_demux;
	debugfs_create_regset32("registers", 0444, root, &dvb->regset);

	debugfs_create_file("capabilities", 0444, root, dvb,
			    &aml_dvb_caps_fops);

	debugfs_create_file("stats", 0444, root, dvb,
			    &aml_dvb_debugfs_stats_fops);

	/* BUG-9: debug level with u32 cast */
	debugfs_create_u32("aml_debug_level", 0644, root, &aml_dvb_debug);

	for (i = 0; i < dvb->caps.num_demux; i++) {
		snprintf(name, sizeof(name), "dmx%d", i);
		debugfs_create_file(name, 0444, root, &dvb->demux[i],
				    &aml_dvb_debugfs_demux_fops);
	}
}

void aml_dvb_debugfs_exit(struct aml_dvb *dvb)
{
	debugfs_remove_recursive(dvb->debugfs_root);
	dvb->debugfs_root = NULL;
}

#endif /* CONFIG_DEBUG_FS */