// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic DVB Async FIFO (DVR) – workqueue‑based poll engine, lockless ring
 *
 * Copyright (C) 2026 Kağan Kadioğlu <kagankadioglutk@hotmail.com>
 */

#include <linux/interrupt.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <media/dvb_demux.h>
#include "amlogic_dvb.h"
#include "amlogic-dvb-regs.h"
#include "amlogic-dvb-trace.h"

#define RING_ENTRIES(afifo)    ((afifo)->buf_size / (afifo)->flush_size)
#define RING_MASK(afifo)        (RING_ENTRIES(afifo) - 1)

static inline u32 ring_avail(struct aml_asyncfifo *afifo)
{
	u32 head = smp_load_acquire(&afifo->head);
	u32 tail = READ_ONCE(afifo->tail);
	return (head - tail) & RING_MASK(afifo);
}

static inline void ring_consume(struct aml_asyncfifo *afifo, u32 chunks)
{
	u32 new_tail = (READ_ONCE(afifo->tail) + chunks) & RING_MASK(afifo);
	/* smp_store_release: tail update becomes visible after DMA reads */
	smp_store_release(&afifo->tail, new_tail);
}

static void aml_asyncfifo_update_irq_threshold(struct aml_asyncfifo *afifo)
{
	struct aml_dvb *dvb = afifo->dvb;
	u32 fill = ring_avail(afifo);
	u32 new_thresh;

	fill = fill * 100 / afifo->ring_entries;
	if (fill > 75)
		new_thresh = afifo->irq_threshold_max;
	else if (fill > 50)
		new_thresh = afifo->irq_threshold_min * 2;
	else if (fill > 25)
		new_thresh = afifo->irq_threshold_min;
	else
		new_thresh = afifo->irq_threshold_min / 2;

	new_thresh = clamp(new_thresh, afifo->irq_threshold_min,
			   afifo->irq_threshold_max);

	if (new_thresh != afifo->irq_threshold) {
		u32 thresh_val;

		afifo->irq_threshold = new_thresh;

		/*
		 * IRQ threshold is in REG3 (+0x0C) bit[15:0] field.
		 * Specified as a 128-byte block count.
		 */
		thresh_val = min_t(u32, new_thresh >> 7, 0xFFFF);
		thresh_val |=
			AFIFO_IRQ_EN; /* BIT21: IRQ arm must be preserved */
		aml_write_async(dvb, ASYNC_FIFO_REG3(afifo->id), thresh_val);

		dev_dbg(dvb->dev, "afifo%d: irq_threshold=%u thresh_val=0x%x\n",
			afifo->id, new_thresh, thresh_val);
	}
}

static void aml_asyncfifo_poll_work(struct work_struct *work)
{
	struct aml_asyncfifo *afifo =
		container_of(work, struct aml_asyncfifo, poll_work);
	struct aml_dvb *dvb = afifo->dvb;
	struct aml_dmx *dmx;
	u32 budget = afifo->poll_budget;
	int work_done = 0;
	u32 avail, tail;
	void *buf;
	size_t len;

	if (unlikely(!afifo->enabled))
		goto out;

	dev_dbg(dvb->dev,
		"afifo%d: poll_work starting head=%u tail=%u avail=%u\n",
		afifo->id, READ_ONCE(afifo->head), afifo->tail,
		ring_avail(afifo));

	while (budget--) {
		avail = ring_avail(afifo);
		if (avail == 0)
			break;

		tail = READ_ONCE(afifo->tail);
		buf = afifo->buf_virt + tail * afifo->flush_size;
		len = afifo->flush_size;

		prefetch(buf);
		prefetch(buf + 64);
		prefetch(buf + 128);
		prefetch(buf + 192);

		dma_rmb();

		if (unlikely(afifo->source >= dvb->caps.num_demux)) {
			dev_err(dvb->dev,
				"afifo%d: invalid source %d, dropping chunk\n",
				afifo->id, afifo->source);
			trace_aml_dvb_dma_starvation(afifo->source);
			ring_consume(afifo, 1);
			continue;
		}
		dmx = &dvb->demux[afifo->source];

		/*
         * V2.60: DVR path — vendor dvr_process_channel logic.
         *
         * HW section filter (SEC_BUFF_READY IRQ) and DVR recorder path
         * (TS_RECORDER_ENABLE) are independent:
         *   - Section PAT/PMT/SDT → SEC_BUFF DMA → dmx IRQ → process_section()
         *   - Raw TS → AFIFO DMA → AFIFO IRQ → this function → cb.ts()
         *
         * If DVR feed exists, deliver directly via cb.ts().
         * If no DVR feed (section feed only): skip the chunk.
         */
		/*
         * V2.60: Dual path support
         *   sf_mode=true  → dvb_dmx_swfilter() (SW section/PES filter)
         *   sf_mode=false → cb.ts() (DVR recorder, raw TS)
         */
		if (dmx->sf_mode) {
			/* SW filter path — send all raw TS to sw filter */
			dvb_dmx_swfilter(&dmx->demux, buf, len);
			dev_dbg(dvb->dev, "afifo%d: SW filter chunk=%zu\n",
				afifo->id, len);
		} else {
			/* DVR path — deliver via cb.ts() */
			int i;
			bool delivered = false;
			for (i = 0; i < AML_CHANNEL_COUNT; i++) {
				if (dmx->channel[i].used &&
				    dmx->channel[i].dvr_feed) {
					struct dvb_demux_feed *dvr =
						dmx->channel[i].dvr_feed;
					if (dvr->cb.ts)
						dvr->cb.ts(buf, len, NULL, 0,
							   &dvr->feed.ts, 0);
					delivered = true;
					break;
				}
			}
			if (!delivered)
				dev_dbg(dvb->dev,
					"afifo%d: no DVR feed, chunk skipped\n",
					afifo->id);
		}

		ring_consume(afifo, 1);
		work_done++;
	}

	dev_dbg(dvb->dev, "afifo%d: poll_work done: %d chunks processed\n",
		afifo->id, work_done);

	if (work_done >= afifo->poll_budget / 2)
		afifo->poll_budget = min(afifo->poll_budget * 2, 1024U);
	else if (work_done == 0)
		afifo->poll_budget = max(afifo->poll_budget / 2, 64U);

	aml_asyncfifo_update_irq_threshold(afifo);

out:
	/*
     * Race condition fix:
     * atomic_set(&pending, 0) MUST be done FIRST, ring_avail check AFTER.
     *
     * Wrong order:
     *   1. ring_avail() == 0 → loop ends
     *   2. IRQ arrives → pending=1, work queued
     *   3. atomic_set(pending, 0) → clears the 1 from IRQ
     *   → Work does not re-enter the queue, new data is not processed! ←
     *
     * Correct order:
     *   1. atomic_set(pending, 0)  ← clear pending
     *   2. ring_avail() > 0 → still data → pending=1 + re-queue
     *   If IRQ arrives between 1 and 2 → pending becomes 1 → condition
     *   is not met but work has already been queued by the IRQ. Safe.
     */
	atomic_set(&afifo->pending, 0);
	smp_mb(); /* memory barrier: pending=0 store becomes visible before ring_avail read */

	if (ring_avail(afifo) > 0 && !atomic_xchg(&afifo->pending, 1))
		queue_work_on(afifo->cpu, system_highpri_wq, &afifo->poll_work);
}

void aml_asyncfifo_check_backpressure(struct aml_dvb *dvb)
{
	bool throttle = false;
	int i;

	for (i = 0; i < dvb->caps.num_asyncfifo; i++) {
		struct aml_asyncfifo *afifo = &dvb->asyncfifo[i];
		if (!afifo->enabled)
			continue;
		if (ring_avail(afifo) >= afifo->ring_entries * 3 / 4) {
			throttle = true;
			dev_dbg(dvb->dev,
				"backpressure: afifo%d fill high, throttling\n",
				i);
			break;
		}
	}

	aml_update_bits(dvb, TS_TOP_CONFIG, BIT(0), throttle ? 0 : BIT(0));
}
EXPORT_SYMBOL_GPL(aml_asyncfifo_check_backpressure);

int aml_asyncfifo_init(struct aml_dvb *dvb)
{
	int i, ret;

	dev_info(dvb->dev, "asyncfifo_init: starting, num=%d\n",
		 dvb->caps.num_asyncfifo);

	for (i = 0; i < dvb->caps.num_asyncfifo; i++) {
		struct aml_asyncfifo *afifo = &dvb->asyncfifo[i];
		u32 ctrl_val;

		dev_info(dvb->dev, "asyncfifo_init: initializing FIFO %d\n", i);

		afifo->id = i;
		afifo->dvb = dvb;
		afifo->buf_size = AML_ASYNC_BUF_SIZE;
		afifo->flush_size =
			ALIGN(AML_ASYNC_FLUSH_SIZE, SMP_CACHE_BYTES);
		afifo->enabled = false;
		afifo->source = i;
		afifo->head = 0;
		afifo->tail = 0;
		afifo->prev_chunk = 0;
		atomic_set(&afifo->pending, 0);
		afifo->poll_budget = 256;
		afifo->ring_entries = RING_ENTRIES(afifo);

		if (!is_power_of_2(afifo->ring_entries)) {
			dev_err(dvb->dev,
				"FIFO %d: ring_entries=%u not power of 2\n", i,
				afifo->ring_entries);
			ret = -EINVAL;
			goto err_free_buf;
		}

		afifo->irq_threshold_min = AML_ASYNC_FLUSH_SIZE;
		/* threshold_max must not exceed half the ring size.
		 * If flush_size * 4 > buf_size/2, IRQ will never fire. */
		afifo->irq_threshold_max = min_t(
			size_t, AML_ASYNC_FLUSH_SIZE * 4, afifo->buf_size / 2);
		afifo->irq_threshold = afifo->irq_threshold_min;
		afifo->cpu = i % num_online_cpus();

		INIT_WORK(&afifo->poll_work, aml_asyncfifo_poll_work);

		/* Allocate DMA ring buffer */
		dev_info(dvb->dev,
			 "afifo%d: allocating coherent buffer (%zu bytes)\n", i,
			 afifo->buf_size);
		afifo->buf_virt = dma_alloc_coherent(dvb->dev, afifo->buf_size,
						     &afifo->buf_addr,
						     GFP_KERNEL | GFP_DMA32);
		if (!afifo->buf_virt) {
			dev_err(dvb->dev,
				"afifo%d: dma_alloc_coherent failed\n", i);
			ret = -ENOMEM;
			goto err_free_buf;
		}
		dev_info(dvb->dev, "afifo%d: dma buffer virt=%p dma=%pad\n", i,
			 afifo->buf_virt, &afifo->buf_addr);

		/*
		 * Vendor async_fifo_set_regs() order (c_stb_define.h bit definitions):
		 *
		 * REG0 (+0x00): DMA start address
		 * REG1 (+0x04): FLUSH control
		 *   bit22 = ASYNC_FIFO_RESET   (pulse — set first, then clear)
		 *   bit21 = ASYNC_FIFO_WRAP_EN (ring buffer wrap)
		 *   bit20 = ASYNC_FIFO_FLUSH_EN (flush active — without this, no IRQ)
		 *   bit[14:0] = ASYNC_FIFO_FLUSH_CNT (128-byte block count = size>>7)
		 * REG2 (+0x08): FILL control
		 *   bit[24:23] = ASYNC_FIFO_SOURCE (DMX0=3, DMX1=2, DMX2=0)
		 *   bit[22:21] = ASYNC_FIFO_ENDIAN (1)
		 *   bit20 = ASYNC_FIFO_FILL_EN
		 * REG3 (+0x0C): IRQ threshold (128-byte block count)
		 *
		 * FIFO[0] → 0xFFD0A000 (primary channel, active)
		 * FIFO[1] → 0xFFD09000 (secondary, idle)
		 *
		 * devmem verification: REG1=0x80301000
		 *   bit31 = FLUSH_STATUS (RO, data is flowing)
		 *   bit21 = WRAP_EN ✓  bit20 = FLUSH_EN ✓  FLUSH_CNT=0x1000
		 */
		/* All FIFOs are initialised equally — afifo1 is needed for the second frontend */

		/* STEP 1: REG0 — DMA ring buffer start address */
		ret = aml_write_async(dvb, ASYNC_FIFO_REG0(i), afifo->buf_addr);
		if (ret) {
			dev_err(dvb->dev, "afifo%d: REG0 write failed: %d\n", i,
				ret);
			goto err_free_dma;
		}
		dev_info(dvb->dev, "afifo%d: REG0 (DMA addr) = 0x%08x\n", i,
			 (u32)afifo->buf_addr);

		/* STEP 2: REG1 — RESET pulse: RESET|WRAP_EN|FLUSH_CNT */
		{
			/*
			 * V2.18 CRITICAL FIX: FLUSH_CNT unit is 128 BYTES, NOT 1KB!
			 *
			 * SM1 device devmem evidence (V2.17 at runtime):
			 *   When flush_cnt = 512 (buf_size >> 10) was written,
			 *   WR_PTR NEVER exceeded 64KB (all 22 measurements within 0-63KB).
			 *   512 × 128B = 64KB  ← EXACT MATCH with observation ✓
			 *   512 × 1KB  = 512KB ← DID NOT MATCH ✗
			 *
			 * Bug impact (V2.17):
			 *   HW ring = 64KB, SW ring_entries = 8 (8×64KB = 512KB).
			 *   Only slot 0 (0-64KB) received real TS data.
			 *   Slot 1-7 (64KB-512KB): zeros/garbage → swfilter found no sections.
			 *   Effective data window: 1 in 8 IRQs → PAT partially working,
			 *   PMT/SDT/NIT were timing out.
			 *
			 * Correct formula: buf_size >> 7
			 *   512KB / 128B = 4096 = 0x1000 → ring = 4096 × 128B = 512KB ✓
			 *
			 * V2.08 "1KB unit" observation was probably for a different SoC (GXL/GXM).
			 * Verified by devmem measurement that the unit is 128 bytes for SM1 (S905X3).
			 */
			u32 flush_cnt = (afifo->buf_size >> 7) & 0x7FFF;

			ctrl_val = AFIFO_RESET | AFIFO_WRAP_EN | flush_cnt;
			aml_write_async(dvb, ASYNC_FIFO_REG1(i), ctrl_val);
			dev_info(
				dvb->dev,
				"afifo%d: REG1 reset pulse = 0x%08x (flush_cnt=0x%x)\n",
				i, ctrl_val, flush_cnt);

			/* Clear the RESET bit */
			ctrl_val &= ~AFIFO_RESET;
			aml_write_async(dvb, ASYNC_FIFO_REG1(i), ctrl_val);

			/* Set FLUSH_EN — without this, IRQ does not fire */
			ctrl_val |= AFIFO_FLUSH_EN;
			ret = aml_write_async(dvb, ASYNC_FIFO_REG1(i),
					      ctrl_val);
			if (ret) {
				dev_err(dvb->dev,
					"afifo%d: REG1 write failed: %d\n", i,
					ret);
				goto err_free_dma;
			}
			dev_info(dvb->dev,
				 "afifo%d: REG1 (WRAP+FLUSH) = 0x%08x\n", i,
				 ctrl_val);
		}

		/* STEP 3: REG2 — no FILL_EN at init, no source either.
		 * FILL_EN is only enabled in the aml_asyncfifo_set_source() call.
		 * If FILL_EN were set here, source=0(dmx0) would be the default →
		 * asyncfifo0 and asyncfifo1 would be fed from the same source. */
		aml_write_async(dvb, ASYNC_FIFO_REG2(i), 0);
		dev_info(dvb->dev,
			 "afifo%d: REG2 (FILL_EN off, no src) = 0x00000000\n",
			 i);

		/* STEP 4: REG3 — IRQ threshold + AFIFO_IRQ_EN (BIT21)
		 *
		 * V2.17 CRITICAL FIX: WITHOUT BIT21 (AFIFO_IRQ_EN), IRQ NEVER FIRES!
		 *
		 * Evidence (device devmem):
		 *   - Idle FIFO[1] REG3 = 0x00200000 (BIT21=1, hardware reset default)
		 *   - Vendor active FIFO  REG3 = 0x002007FF (BIT21=1 + threshold=0x7FF)
		 *   - V2.16: only 0x7FF was written (BIT21=0) → despite WR_PTR moving,
		 *     IRQ never arrived → poll_work did not run → PAT/PMT zero
		 *
		 * REG3 unit is 128 bytes: 64KB / 128 - 1 = 511 = 0x1FF
		 * Combined with BIT21: 0x002001FF
		 */
		{
			u32 irq_thresh = (afifo->flush_size >> 7) - 1;

			irq_thresh = min_t(u32, irq_thresh, 0x7FFF);
			irq_thresh |=
				AFIFO_IRQ_EN; /* BIT21: IRQ arm — without this, no IRQ! */
			aml_write_async(dvb, ASYNC_FIFO_REG3(i), irq_thresh);
			dev_info(
				dvb->dev,
				"afifo%d: REG3 (IRQ thresh) = 0x%06x (every %zuKB)\n",
				i, irq_thresh, afifo->flush_size / 1024);
		}

		afifo->enabled = true;
		dev_info(
			dvb->dev,
			"Async FIFO %d: base=%pad size=%zu flush=%zu REG1=0x%08x CPU%d\n",
			i, &afifo->buf_addr, afifo->buf_size, afifo->flush_size,
			ctrl_val, afifo->cpu);
		continue;

err_free_dma:
		dma_free_coherent(dvb->dev, afifo->buf_size, afifo->buf_virt,
				  afifo->buf_addr);
		afifo->buf_virt = NULL;
err_free_buf:
		while (--i >= 0) {
			struct aml_asyncfifo *prev = &dvb->asyncfifo[i];

			if (!prev->enabled)
				continue;
			cancel_work_sync(&prev->poll_work);
			aml_write_async(dvb, ASYNC_FIFO_REG1(i), 0);
			dma_free_coherent(dvb->dev, prev->buf_size,
					  prev->buf_virt, prev->buf_addr);
			prev->enabled = false;
		}
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(aml_asyncfifo_init);

void aml_asyncfifo_release(struct aml_dvb *dvb)
{
	int i;

	dev_info(dvb->dev, "asyncfifo_release\n");

	for (i = 0; i < dvb->caps.num_asyncfifo; i++) {
		struct aml_asyncfifo *afifo = &dvb->asyncfifo[i];

		if (!afifo->enabled)
			continue;

		dev_dbg(dvb->dev, "afifo%d: releasing\n", i);
		cancel_work_sync(&afifo->poll_work);
		/* Stop the FIFO: clear FLUSH_EN and FILL_EN */
		aml_write_async(dvb, ASYNC_FIFO_REG1(i), 0);
		aml_write_async(dvb, ASYNC_FIFO_REG2(i), 0);
		dma_free_coherent(dvb->dev, afifo->buf_size, afifo->buf_virt,
				  afifo->buf_addr);
		afifo->buf_virt = NULL;
		afifo->enabled = false;
	}
}
EXPORT_SYMBOL_GPL(aml_asyncfifo_release);

int aml_asyncfifo_set_source(struct aml_asyncfifo *afifo, unsigned int source)
{
	struct aml_dvb *dvb = afifo->dvb;
	/* Vendor reset_async_fifos: DMX0→3, DMX1→2, DMX2→0 */
	static const u8 src_map[] = { AFIFO_SRC_DMX0, AFIFO_SRC_DMX1,
				      AFIFO_SRC_DMX2 };
	u32 src_val, reg2;

	dev_dbg(dvb->dev, "afifo%d: set_source %u\n", afifo->id, source);

	if (!afifo->enabled)
		return -EINVAL;
	if (source >= dvb->caps.num_demux)
		return -EINVAL;

	afifo->source = source;
	src_val = (source < ARRAY_SIZE(src_map)) ? src_map[source] : 0;

	/* REG2: SOURCE[24:23] | ENDIAN | FILL_EN
	 * Change source connection, FILL_EN and ENDIAN remain unchanged */
	reg2 = AFIFO_SOURCE(src_val) | AFIFO_ENDIAN_VAL | AFIFO_FILL_EN;
	dev_info(dvb->dev, "afifo%d: REG2 source=%u val=%u reg2=0x%08x\n",
		 afifo->id, source, src_val, reg2);
	return aml_write_async(dvb, ASYNC_FIFO_REG2(afifo->id), reg2);
}
EXPORT_SYMBOL_GPL(aml_asyncfifo_set_source);

irqreturn_t aml_asyncfifo_irq_handler(int irq, void *dev_id)
{
	struct aml_asyncfifo *afifo = dev_id;
	struct aml_dvb *dvb = afifo->dvb;
	u32 wr_ptr, new_head;

	dev_dbg(dvb->dev, "afifo%d: IRQ fired! irq=%d\n", afifo->id, irq);

	/*
	 * Delta approach — safest method.
	 *
	 * abs_chunk = WR_PTR instantaneous position within the ring (0..ring_entries-1).
	 * prev_chunk = abs_chunk from the previous IRQ.
	 * delta = how many new chunks have been filled (wrap-safe).
	 *
	 * head is a monotonically increasing integer (never masked).
	 * ring_avail = (head - tail) & RING_MASK — always correct.
	 */
	if (aml_read_async(dvb, ASYNC_FIFO_WR_PTR(afifo->id), &wr_ptr) == 0) {
		u32 ring_mask = afifo->ring_entries - 1;
		u32 base_addr = (u32)afifo->buf_addr;
		u32 ring_size = (u32)afifo->buf_size;
		u32 offset = (wr_ptr - base_addr) & (ring_size - 1);
		u32 abs_chunk = offset / (u32)afifo->flush_size;
		u32 prev_abs = READ_ONCE(afifo->prev_chunk);
		u32 delta = (abs_chunk - prev_abs) & ring_mask;

		/* Guarantee at least 1 new chunk */
		if (delta == 0)
			delta = 1;

		WRITE_ONCE(afifo->prev_chunk, abs_chunk);
		new_head = READ_ONCE(afifo->head) + delta;
	} else {
		new_head = READ_ONCE(afifo->head) + 1;
	}

	smp_store_release(&afifo->head, new_head);

	aml_asyncfifo_check_backpressure(dvb);

	if (!atomic_xchg(&afifo->pending, 1))
		queue_work_on(afifo->cpu, system_highpri_wq, &afifo->poll_work);

	return IRQ_HANDLED;
}
EXPORT_SYMBOL_GPL(aml_asyncfifo_irq_handler);

bool aml_asyncfifo_has_data(struct aml_dvb *dvb)
{
	int i;
	for (i = 0; i < dvb->caps.num_asyncfifo; i++) {
		struct aml_asyncfifo *afifo = &dvb->asyncfifo[i];
		if (READ_ONCE(afifo->head) != READ_ONCE(afifo->tail))
			return true;
	}
	return false;
}
EXPORT_SYMBOL_GPL(aml_asyncfifo_has_data);

void aml_asyncfifo_process_all(struct aml_dvb *dvb)
{
	int i;
	for (i = 0; i < dvb->caps.num_asyncfifo; i++) {
		struct aml_asyncfifo *afifo = &dvb->asyncfifo[i];
		if (afifo->enabled && !atomic_xchg(&afifo->pending, 1))
			queue_work_on(afifo->cpu, system_highpri_wq,
				      &afifo->poll_work);
	}
}
EXPORT_SYMBOL_GPL(aml_asyncfifo_process_all);

u32 aml_get_ring_fill_percent(struct aml_dvb *dvb)
{
	u32 max_fill = 0;
	int i;

	for (i = 0; i < dvb->caps.num_asyncfifo; i++) {
		struct aml_asyncfifo *afifo = &dvb->asyncfifo[i];
		if (!afifo->enabled)
			continue;
		u32 fill = ring_avail(afifo);
		u32 percent = fill * 100 / afifo->ring_entries;
		if (percent > max_fill)
			max_fill = percent;
	}
	return max_fill;
}
EXPORT_SYMBOL_GPL(aml_get_ring_fill_percent);
