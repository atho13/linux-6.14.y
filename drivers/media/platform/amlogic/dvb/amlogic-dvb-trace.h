/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Amlogic DVB tracepoint definitions
 *
 * Copyright (C) 2026 Kağan Kadioğlu <kagankadioglutk@hotmail.com>
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM aml_dvb

#if !defined(_TRACE_AML_DVB_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_AML_DVB_H

#include <linux/tracepoint.h>

TRACE_EVENT(aml_dvb_fifo_overflow, TP_PROTO(int id), TP_ARGS(id),
	    TP_STRUCT__entry(__field(int, id)),
	    TP_fast_assign(__entry->id = id;),
	    TP_printk("fifo overflow demux=%d", __entry->id));

TRACE_EVENT(aml_dvb_ts_drop, TP_PROTO(u16 pid), TP_ARGS(pid),
	    TP_STRUCT__entry(__field(u16, pid)),
	    TP_fast_assign(__entry->pid = pid;),
	    TP_printk("TS packet dropped pid=0x%x", __entry->pid));

TRACE_EVENT(aml_dvb_dma_starvation, TP_PROTO(int dmx), TP_ARGS(dmx),
	    TP_STRUCT__entry(__field(int, dmx)),
	    TP_fast_assign(__entry->dmx = dmx;),
	    TP_printk("DMA starvation demux=%d", __entry->dmx));

TRACE_EVENT(aml_dvb_section_ready, TP_PROTO(int dmx, u16 buf_num, u16 sec_num),
	    TP_ARGS(dmx, buf_num, sec_num),
	    TP_STRUCT__entry(__field(int, dmx) __field(u16, buf_num)
				     __field(u16, sec_num)),
	    TP_fast_assign(__entry->dmx = dmx; __entry->buf_num = buf_num;
			   __entry->sec_num = sec_num;),
	    TP_printk("section ready demux=%d buf=%u filter=%u", __entry->dmx,
		      __entry->buf_num, __entry->sec_num));

#endif /* _TRACE_AML_DVB_H */

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE amlogic-dvb-trace
#include <trace/define_trace.h>