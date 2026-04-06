// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic DVB Frontend Platform Glue
 *
 * ARCHITECTURE: avl6862_probe() registers the frontend itself.
 *
 * When platform probe occurs, it adds itself to the global list.
 * avl6862_probe() → finds the adapter via aml_dvb_find_by_fe_node()
 *                 → calls dvb_register_frontend().
 *
 * Module loading order is independent: if platform loads first, the list is ready;
 * if avl6862 loads first, it returns -EPROBE_DEFER and the kernel retries.
 *
 * Copyright (C) 2026 Kağan Kadioğlu <kagankadioglutk@hotmail.com>
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include "amlogic_dvb.h"
#include "amlogic-dvb-hwops.h" // <-- ADDED: for hwops struct definition

/* ── Global DVB adapter list ───────────────────────────────────────── */

struct aml_dvb_entry {
	struct aml_dvb *dvb;
	struct list_head node;
};

#define MAX_DVB_ADAPTERS 4	/* Max DVB platform adapters in the system */

static LIST_HEAD(aml_dvb_list);
static DEFINE_MUTEX(aml_dvb_list_lock);

/* Called when platform probe completes */
void aml_dvb_list_add(struct aml_dvb *dvb)
{
	struct aml_dvb_entry *entry;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return;

	entry->dvb = dvb;
	mutex_lock(&aml_dvb_list_lock);
	list_add(&entry->node, &aml_dvb_list);
	mutex_unlock(&aml_dvb_list_lock);
	dev_info(dvb->dev, "DVB adapter registered in global list\n");
}
EXPORT_SYMBOL_GPL(aml_dvb_list_add);

/* Called during platform remove */
void aml_dvb_list_del(struct aml_dvb *dvb)
{
	struct aml_dvb_entry *entry, *tmp;

	mutex_lock(&aml_dvb_list_lock);
	list_for_each_entry_safe(entry, tmp, &aml_dvb_list, node) {
		if (entry->dvb == dvb) {
			list_del(&entry->node);
			kfree(entry);
			break;
		}
	}
	mutex_unlock(&aml_dvb_list_lock);
}
EXPORT_SYMBOL_GPL(aml_dvb_list_del);

/**
 * aml_dvb_find_by_fe_node - Returns the adapter and fe_idx that contains
 *                            fe_node in the dvb-frontends list.
 *
 * Called outside mutex as of_parse_phandle() may sleep.
 * List snapshot is taken under mutex; OF operations are done outside it.
 */
static struct aml_dvb *aml_dvb_find_by_fe_node(struct device_node *fe_node,
					       int *fe_idx_out)
{
	struct aml_dvb_entry *entry;
	struct aml_dvb *found = NULL;

	/* Snapshot: copy all dvb pointers from the list */
	struct aml_dvb *adapters[MAX_DVB_ADAPTERS];
	int nadapters = 0, i, j;

	mutex_lock(&aml_dvb_list_lock);
	list_for_each_entry(entry, &aml_dvb_list, node) {
		if (nadapters < MAX_DVB_ADAPTERS)
			adapters[nadapters++] = entry->dvb;
	}
	mutex_unlock(&aml_dvb_list_lock);

	/* OF operations outside mutex — safe to sleep */
	for (i = 0; i < nadapters && !found; i++) {
		struct aml_dvb *dvb = adapters[i];
		struct device_node *np = dvb->dev->of_node;
		int count =
			of_count_phandle_with_args(np, "dvb-frontends", NULL);

		for (j = 0; j < count && j < AML_MAX_FRONTEND; j++) {
			struct device_node *ref =
				of_parse_phandle(np, "dvb-frontends", j);
			bool match = (ref == fe_node);
			of_node_put(ref);
			if (match) {
				*fe_idx_out = j;
				found = dvb;
				break;
			}
		}
	}
	return found;
}

/* ── TS source routing (now separate read + apply) ───────────── */

/**
 * aml_dvb_get_ts_source - Reads ts-source and ts-port values from DTS,
 *                          calculates valid values.
 * @dvb: aml_dvb structure
 * @np: frontend device_node
 * @fe_idx: frontend index (zero-based)
 * @ts_demux_out: which demux to use (output)
 * @ts_port_out: which physical TS port to use (output)
 *
 * Return value: 0 success, negative error
 */
static int aml_dvb_get_ts_source(struct aml_dvb *dvb, struct device_node *np,
				 int fe_idx, u32 *ts_demux_out,
				 u32 *ts_port_out)
{
	u32 ts_demux, ts_port;
	int ret;

	/* ts-source: which demux to use (0,1,2) */
	ret = of_property_read_u32(np, "ts-source", &ts_demux);
	if (ret) {
		ts_demux = fe_idx % dvb->caps.num_demux;
		dev_info(dvb->dev,
			 "Frontend %d: no ts-source, defaulting to demux %d\n",
			 fe_idx, ts_demux);
	}
	if (ts_demux >= dvb->caps.num_demux) {
		dev_err(dvb->dev,
			"Frontend %d: invalid ts-source %d, using 0\n", fe_idx,
			ts_demux);
		ts_demux = 0;
	}

	/* ts-port: physical TS input port (0=tsin_a, 1=tsin_b, ...) */
	ret = of_property_read_u32(np, "ts-port", &ts_port);
	if (ret) {
		/* Legacy DTS compatibility: port = demux index */
		ts_port = ts_demux;
		dev_info(dvb->dev,
			 "Frontend %d: no ts-port, using demux index (%d)\n",
			 fe_idx, ts_port);
	}
	if (ts_port >= dvb->caps.num_demux) {
		dev_err(dvb->dev,
			"Frontend %d: invalid ts-port %d, using demux index (%d)\n",
			fe_idx, ts_port, ts_demux);
		ts_port = ts_demux;
	}

	*ts_demux_out = ts_demux;
	*ts_port_out = ts_port;
	return 0;
}

/**
 * aml_dvb_apply_ts_source - Applies the given demux and port values to hardware.
 * @dvb: aml_dvb structure
 * @ts_demux: which demux
 * @ts_port: which physical TS port
 */
static void aml_dvb_apply_ts_source(struct aml_dvb *dvb, u32 ts_demux,
				    u32 ts_port)
{
	aml_dmx_set_source(&dvb->demux[ts_demux],
			   AML_TS_SRC_FRONTEND_TS0 + ts_port);
	dev_info(dvb->dev, "Demux %d <- tsin_%c (AML_TS_SRC_FRONTEND_TS%d)\n",
		 ts_demux, 'a' + ts_port, ts_port);
}

/* ── Called by avl6862_probe() ──────────────────────────────────────── */

/**
 * aml_dvb_register_frontend - Register frontend from demod probe
 * @fe_node: demod's DTS node
 * @fe:      ready dvb_frontend (tuner attached)
 *
 * Return value:
 *  0          - success
 * -EPROBE_DEFER - platform not yet ready, kernel retries
 *  other      - error
 */
int aml_dvb_register_frontend(struct device_node *fe_node,
			      struct dvb_frontend *fe,
			      struct i2c_client *client)
{
	struct aml_dvb *dvb;
	int fe_idx = -1;
	u32 ts_demux, ts_port;
	int ret;

	if (!fe_node || !fe)
		return -EINVAL;

	dvb = aml_dvb_find_by_fe_node(fe_node, &fe_idx);
	if (!dvb) {
		pr_warn("aml_dvb: no adapter found for %pOF — deferring\n",
			fe_node);
		return -EPROBE_DEFER;
	}

	/* Runtime PM: wake device (needed for register writes) */
	ret = pm_runtime_get_sync(dvb->dev);
	if (ret < 0) {
		dev_err(dvb->dev, "Failed to get runtime PM: %d\n", ret);
		pm_runtime_put_noidle(dvb->dev);
		return ret;
	}

	if (dvb->frontend[fe_idx]) {
		dev_warn(dvb->dev, "Frontend %d already registered\n", fe_idx);
		ret = 0;
		goto out_pm_put;
	}

	ret = dvb_register_frontend(&dvb->adapter, fe);
	if (ret < 0) {
		dev_err(dvb->dev, "Frontend %d: dvb_register_frontend: %d\n",
			fe_idx, ret);
		goto out_pm_put;
	}

	dvb->frontend[fe_idx] = fe;
	if (client) {
		/* Save I2C client reference — for /proc/bus/nim_sockets */
		get_device(&client->dev);
		dvb->demod_client[fe_idx] = client;
	}
	if (fe_idx + 1 > dvb->num_frontend)
		dvb->num_frontend = fe_idx + 1;

	/* TS source configuration: read from DTS and apply */
	ret = aml_dvb_get_ts_source(dvb, fe_node, fe_idx, &ts_demux, &ts_port);
	if (ret) {
		dev_err(dvb->dev, "Failed to get TS source: %d\n", ret);
		goto out_unregister;
	}
	aml_dvb_apply_ts_source(dvb, ts_demux, ts_port);

	/* Async FIFO source configuration (if available) */
	if (ts_demux < dvb->caps.num_asyncfifo) {
		ret = aml_asyncfifo_set_source(&dvb->asyncfifo[ts_demux],
					       ts_demux);
		if (ret)
			dev_warn(dvb->dev,
				 "Failed to set async FIFO %d source: %d\n",
				 ts_demux, ret);
		else
			dev_info(dvb->dev,
				 "Async FIFO %d source set to demux %d\n",
				 ts_demux, ts_demux);
	} else {
		dev_dbg(dvb->dev, "No async FIFO for demux %d\n", ts_demux);
	}

	/* TS clock configuration (for 54 MHz parallel mode) */
	if (dvb->hwops && dvb->hwops->set_ts_rate) {
		dvb->hwops->set_ts_rate(dvb, 54000);
		dev_info(dvb->dev, "TS rate set to 54 Mbps\n");
	}

	dev_info(dvb->dev, "Frontend %d registered (demod-initiated)\n",
		 fe_idx);
	ret = 0;

out_unregister:
	if (ret)
		dvb_unregister_frontend(fe);
out_pm_put:
	pm_runtime_put(dvb->dev);
	return ret;
}
EXPORT_SYMBOL_GPL(aml_dvb_register_frontend);

/**
 * aml_dvb_unregister_frontend - Called by avl6862_remove()
 * @fe_node: demod's DTS node
 *
 * If avl6862 is removed before the platform, removes the frontend from the adapter.
 * If already removed during platform remove, the second call is safely ignored.
 */
void aml_dvb_unregister_frontend(struct device_node *fe_node)
{
	struct aml_dvb *dvb;
	int fe_idx = -1;

	dvb = aml_dvb_find_by_fe_node(fe_node, &fe_idx);
	if (!dvb || fe_idx < 0)
		return;

	if (!dvb->frontend[fe_idx])
		return; /* already removed */

	/* Runtime PM: it is good to stay awake during removal too */
	pm_runtime_get_sync(dvb->dev);
	dvb_unregister_frontend(dvb->frontend[fe_idx]);
	dvb->frontend[fe_idx] = NULL;
	pm_runtime_put(dvb->dev);
}
EXPORT_SYMBOL_GPL(aml_dvb_unregister_frontend);

/* ── Platform probe: add adapter to list ────────────────────────────── */

int aml_dvb_probe_frontends(struct aml_dvb *dvb)
{
	struct device *dev = dvb->dev;
	struct device_node *np = dev->of_node;
	struct device_node *fe_node;
	int count, i;

	/* Adapter is now ready — add to list */
	aml_dvb_list_add(dvb);

	if (!np)
		return 0;

	count = of_count_phandle_with_args(np, "dvb-frontends", NULL);
	if (count <= 0)
		return 0;

	dev_info(dev, "DVB adapter ready, expecting %d frontend(s)\n", count);
	for (i = 0; i < count; i++) {
		fe_node = of_parse_phandle(np, "dvb-frontends", i);
		if (!fe_node)
			continue;
		dev_info(dev, "  [%d] %pOF\n", i, fe_node);
		of_node_put(fe_node);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(aml_dvb_probe_frontends);

/* ── Platform remove: remove from list and release frontends ────────── */

void aml_dvb_release_frontends(struct aml_dvb *dvb)
{
	int i;

	aml_dvb_list_del(dvb);

	for (i = 0; i < dvb->num_frontend; i++) {
		if (dvb->frontend[i]) {
			dvb_unregister_frontend(dvb->frontend[i]);
			dvb->frontend[i] = NULL;
		}
		if (dvb->demod_client[i]) {
			put_device(&dvb->demod_client[i]->dev);
			dvb->demod_client[i] = NULL;
		}
	}
	dvb->num_frontend = 0;
}
EXPORT_SYMBOL_GPL(aml_dvb_release_frontends);
