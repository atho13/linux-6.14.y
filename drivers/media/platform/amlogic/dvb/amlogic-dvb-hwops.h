/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Amlogic DVB hardware operations interface
 *
 * Copyright (C) 2026 Kağan Kadioğlu <kagankadioglutk@hotmail.com>
 */
#ifndef __AML_DVB_HWOPS_H
#define __AML_DVB_HWOPS_H

struct aml_dvb;

struct aml_dvb_hw_ops {
	void (*reset_fifo)(struct aml_dvb *dvb, int id);
	void (*enable_ts_input)(struct aml_dvb *dvb, int idx);
	void (*disable_ts_input)(struct aml_dvb *dvb, int idx);
	void (*set_ts_rate)(struct aml_dvb *dvb, u32 rate_kbps);
	int (*get_irq_status)(struct aml_dvb *dvb, int demux_id, u32 *status);
};

#endif