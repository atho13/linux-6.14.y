/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Amlogic DVB SoC capability definitions
 *
 * Copyright (C) 2026 Kağan Kadioğlu <kagankadioglutk@hotmail.com>
 */
#ifndef __AML_DVB_SOC_H
#define __AML_DVB_SOC_H

struct aml_dvb_soc_data {
	u8 num_demux; /* hardware demux cores */
	u8 num_ts_input;
	u8 num_s2p;
	u8 num_dsc;
	u8 num_asyncfifo;
	bool has_descrambler;
	bool has_rate_control;
	u32 max_bitrate; /* default target bitrate */
};

#endif