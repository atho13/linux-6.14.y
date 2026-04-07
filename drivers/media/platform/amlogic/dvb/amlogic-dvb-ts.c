// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic DVB TS input and S2P initialization
 *
 * Copyright (C) 2025 Neil Armstrong <neil.armstrong@linaro.org>
 */

#include <linux/pinctrl/consumer.h>
#include "amlogic_dvb.h"
#include "amlogic-dvb-regs.h"

static int aml_ts_input_init(struct aml_dvb *dvb, int idx)
{
	struct device_node *np = dvb->dev->of_node;
	struct device_node *fe_np = NULL;
	struct pinctrl_state *pctrl;
	u32 ts_serial = 0;
	u32 ts_control = 0; /* DTS ts<n>_control → FEC_INPUT_CONTROL[11:0] */
	u32 ctrl;
	int ret;

	/*
     * Search DTS tsin sub-nodes including disabled ones.
     * for_each_available_child_of_node() SKIPS disabled nodes,
     * so for_each_child_of_node() must be used.
     *
     * tsin@N { status = "disabled"; } — this TS port is not physically
     * connected, do not touch. Return 0 and skip.
     *
     * tsin@N { status = "okay"; } — enabled, initialise.
     *
     * If no tsin@N node exists in DTS, continue for legacy DTS compatibility.
     */
	if (np) {
		struct device_node *child;
		bool found = false;
		for_each_child_of_node(np, child) {
			u32 reg = 0xff;
			of_property_read_u32(child, "reg", &reg);
			if (reg != (u32)idx)
				continue;
			found = true;
			if (!of_device_is_available(child)) {
				dev_info(
					dvb->dev,
					"ts_input_init: TS input %d disabled in DTS, skipping\n",
					idx);
				of_node_put(child);
				return 0;
			}
			of_node_put(child);
			break;
		}
		(void)found; /* continue if tsin node is absent in legacy DTS */
	}

	/*
     * Read ts-serial mode from frontend DTS node.
     * Search for frontend with ts-port=<idx> (new DTS).
     * If not found, try dvb-frontends[idx] (legacy DTS compatibility).
     */
	if (np) {
		int fe_count =
			of_count_phandle_with_args(np, "dvb-frontends", NULL);
		int fi;
		bool found_serial = false;
		for (fi = 0; fi < fe_count && fi < AML_MAX_FRONTEND; fi++) {
			u32 port_val = 0xff;
			struct device_node *cand =
				of_parse_phandle(np, "dvb-frontends", fi);
			if (!cand)
				continue;
			/* ts-port (new DTS) or ts-source (legacy DTS) */
			if (of_property_read_u32(cand, "ts-port", &port_val))
				of_property_read_u32(cand, "ts-source",
						     &port_val);
			if (port_val == (u32)idx) {
				of_property_read_u32(cand, "ts-serial",
						     &ts_serial);
				found_serial = true;
				of_node_put(cand);
				break;
			}
			of_node_put(cand);
		}
		/* If no match, try by idx */
		if (!found_serial) {
			fe_np = of_parse_phandle(np, "dvb-frontends", idx);
			if (fe_np) {
				of_property_read_u32(fe_np, "ts-serial",
						     &ts_serial);
				of_node_put(fe_np);
			}
		}

		/* ts<idx>-control → FEC_INPUT_CONTROL[11:0] (vendor DTS: ts1_control=<0x00>) */
		{
			char prop[32];
			snprintf(prop, sizeof(prop), "ts%d-control", idx);
			of_property_read_u32(np, prop, &ts_control);
			dev_info(dvb->dev,
				 "ts_input_init: TS%d ts_control=0x%03x\n", idx,
				 ts_control);
		}
	}

	if (ts_serial) {
		ctrl = TS_IN_ENABLE | TS_IN_SERIAL;
		dvb->ts[idx].is_serial = true;
		dev_info(dvb->dev, "ts_input_init: TS input %d → SERIAL mode\n",
			 idx);
		pctrl = pinctrl_lookup_state(dvb->pinctrl, "serial");
		if (!IS_ERR(pctrl))
			pinctrl_select_state(dvb->pinctrl, pctrl);
	} else {
		ctrl = TS_IN_CTRL_PARALLEL; /* vendor verified: 0x00030003 */
		dvb->ts[idx].is_serial = false;
		dev_info(dvb->dev,
			 "ts_input_init: TS input %d → PARALLEL mode\n", idx);
		pctrl = pinctrl_lookup_state(dvb->pinctrl, "parallel");
		if (!IS_ERR(pctrl))
			pinctrl_select_state(dvb->pinctrl, pctrl);
	}

	dev_info(dvb->dev, "ts_input_init: initializing TS input %d\n", idx);

	/*
     * TS_IN_CTRL address verified with vendor devmem:
     *   TS_IN_CTRL(0) → regmap_ts offset 0x000 → phys 0xFFD06000 = 0x00030003
     *   TS_IN_CTRL(1) → regmap_ts offset 0x140 → phys 0xFFD06140 = 0x00030003
     * regmap_ts ≠ regmap_demux (0xFF638000) — no stride conflict.
     */
	ret = aml_write_reg(dvb, TS_IN_CTRL(idx), TS_IN_RESET);
	if (ret) {
		dev_err(dvb->dev,
			"ts_input_init: failed to reset TS input %d: %d\n", idx,
			ret);
		return ret;
	}
	udelay(10);
	ret = aml_write_reg(dvb, TS_IN_CTRL(idx), ctrl);
	if (ret) {
		dev_err(dvb->dev,
			"ts_input_init: failed to enable TS input %d: %d\n",
			idx, ret);
		return ret;
	}

	dvb->ts[idx].mode = ts_serial ? TS_IN_SERIAL : TS_IN_PARALLEL;
	dvb->ts[idx].fec_ctrl = ts_control &
				0xFFF; /* DTS ts<n>_control, max 12-bit */
	dvb->ts[idx].enabled = true;
	dev_info(
		dvb->dev,
		"ts_input_init: TS input %d initialized OK (mode=0x%x fec_ctrl=0x%03x)\n",
		idx, ctrl, dvb->ts[idx].fec_ctrl);
	return 0;
}

static int aml_s2p_init(struct aml_dvb *dvb, int idx)
{
	/* vendor verified: 0xFFD06040 = 0x0000CCCC
     * CLK_DIV=0xCC, CLK_INVERT+DATA_INVERT set, BIT0(ENABLE)=0 (bypassed in parallel mode)
     */
	u32 ctrl = TS_S2P_VENDOR_INIT;
	struct device_node *np = dvb->dev->of_node;
	struct device_node *child;
	int ret;

	dev_info(dvb->dev, "s2p_init: starting S2P %d\n", idx);

	/* vendor value fixed at 0x0000CCCC — no DTS override */
	(void)np;
	(void)child;

	/* Reset the hardware */
	ret = aml_write_reg(dvb, TS_S2P_CTRL(idx), TS_S2P_RESET);
	if (ret)
		return ret;
	udelay(10);

	/* Write the final CTRL value (target: 0x401 or 0xC01) */
	dev_info(dvb->dev, "s2p_init: writing S2P %d ctrl=0x%x\n", idx, ctrl);
	ret = aml_write_reg(dvb, TS_S2P_CTRL(idx), ctrl);

	dvb->s2p[idx].enabled = true;
	return ret;
}

int aml_ts_hw_init(struct aml_dvb *dvb)
{
	int i, ret;

	dev_info(dvb->dev, "ts_hw_init: starting\n");

	/*
     * Clear STB_TOP_CONFIG: reset all DEMUX_X_INPUT_SOURCE fields.
     * Hardware reset value is undefined — vendor driver writes on every init.
     * aml_dvb_apply_ts_source() will write the correct value afterwards.
     */
	ret = aml_write_reg(dvb, STB_TOP_CONFIG, 0x00000000);
	if (ret)
		dev_warn(dvb->dev,
			 "ts_hw_init: STB_TOP_CONFIG clear failed: %d\n", ret);

	/* Initialise TS inputs */
	for (i = 0; i < dvb->caps.num_ts_inputs; i++) {
		dev_info(dvb->dev, "ts_hw_init: initializing TS input %d/%d\n",
			 i, dvb->caps.num_ts_inputs);
		ret = aml_ts_input_init(dvb, i);
		if (ret) {
			dev_err(dvb->dev,
				"ts_hw_init: aml_ts_input_init(%d) failed: %d\n",
				i, ret);
			goto err;
		}
	}

	/* Initialise S2P converters */
	for (i = 0; i < dvb->caps.num_s2p; i++) {
		dev_info(dvb->dev, "ts_hw_init: initializing S2P %d/%d\n", i,
			 dvb->caps.num_s2p);
		ret = aml_s2p_init(dvb, i);
		if (ret) {
			dev_err(dvb->dev,
				"ts_hw_init: aml_s2p_init(%d) failed: %d\n", i,
				ret);
			goto err;
		}
	}

	/* Configure the TS_TOP_CONFIG register */
	/* vendor verified (devmem 0xFFD063C4 = 0x7700BB47):
     * bit[31:24]=0x77 framing enable bitleri, bit[15:8]=pkt_len-1, bit[7:0]=sync_byte
     */
	u32 ts_top_val = (0x77u << 24) | ((dvb->ts_packet_len - 1) << 8) |
			 dvb->ts_sync_byte;
	dev_info(
		dvb->dev,
		"ts_hw_init: writing TS_TOP_CONFIG = 0x%x (sync_byte=0x%x, packet_len=%d)\n",
		ts_top_val, dvb->ts_sync_byte, dvb->ts_packet_len);
	ret = aml_write_reg(dvb, TS_TOP_CONFIG, ts_top_val);
	if (ret) {
		dev_err(dvb->dev,
			"ts_hw_init: failed to write TS_TOP_CONFIG: %d\n", ret);
		goto err;
	}

	dev_info(dvb->dev, "ts_hw_init: completed successfully\n");
	return 0;

err:
	aml_ts_hw_release(dvb);
	dev_err(dvb->dev, "ts_hw_init: failed with error %d\n", ret);
	return ret;
}
EXPORT_SYMBOL_GPL(aml_ts_hw_init);

void aml_ts_hw_release(struct aml_dvb *dvb)
{
	int i;

	dev_info(dvb->dev, "ts_hw_release: starting\n");

	aml_write_reg(dvb, TS_TOP_CONFIG, 0);

	for (i = 0; i < dvb->caps.num_s2p; i++) {
		if (dvb->s2p[i].enabled) {
			dev_dbg(dvb->dev, "ts_hw_release: disabling S2P %d\n",
				i);
			aml_write_reg(dvb, TS_S2P_CTRL(i), 0);
			dvb->s2p[i].enabled = false;
		}
	}

	for (i = 0; i < dvb->caps.num_ts_inputs; i++) {
		if (dvb->ts[i].enabled) {
			dev_dbg(dvb->dev,
				"ts_hw_release: disabling TS input %d\n", i);
			aml_write_reg(dvb, TS_IN_CTRL(i), 0);
			dvb->ts[i].enabled = false;
		}
	}
	dev_info(dvb->dev, "ts_hw_release: done\n");
}
EXPORT_SYMBOL_GPL(aml_ts_hw_release);
