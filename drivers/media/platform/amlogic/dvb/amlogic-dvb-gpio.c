// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic DVB — GPIO and Pinctrl Management
 *
 * This module manages GPIOs at two levels:
 *
 * 1. Platform level (platform DTS node):
 *    All GPIOs are acquired with devm during driver probe, consumer names are assigned
 *    and added to the struct aml_dvb_gpio table.
 *    The table is visible in /sys/kernel/debug/<dvb>/gpio and /sys/kernel/debug/gpio.
 *
 * 2. Frontend level (I2C client DTS node):
 *    Power/reset sequence is applied during each frontend probe.
 *
 * ── /sys/kernel/debug/gpio output ──────────────────────────────────────
 *  gpio-45 (demod-reset  | amlogic-dvb-demod-reset ) out lo ACTIVE LOW
 *  gpio-46 (demod-power  | amlogic-dvb-demod-power ) out hi
 *  gpio-47 (ant-power    | amlogic-dvb-ant-power   ) out lo
 *  gpio-51 (ts0-data0    | amlogic-dvb-ts0-data0   ) in   -- [PINCTRL]
 *
 * ── /sys/kernel/debug/<dvb>/gpio output ─────────────────────────────────
 *  # Amlogic DVB GPIO Table
 *  # idx  name                  direction  logical  raw  polarity  consumer
 *    [0]  demod-reset           out        1        0    ACTIVE_LOW amlogic-dvb-demod-reset
 *    [1]  demod-power           out        1        1    ACTIVE_HI  amlogic-dvb-demod-power
 *    [2]  ant-power             out        0        0    ACTIVE_HI  amlogic-dvb-ant-power
 *
 * ── /sys/kernel/debug/<dvb>/pinctrl output ──────────────────────────────
 *  pinctrl device : ff634000.periphs-pinctrl
 *  current state  : default
 *  available states: default parallel s_ts0 s_ts1 p_ts0 p_ts1
 *
 *  TS input mux (STB_TOP_CONFIG):
 *    ts0: parallel (GPIOX_0..7 + CLK + SYNC)
 *    ts1: serial   (GPIOZ_2 data, GPIOZ_3 clk)
 *
 * Copyright (C) 2026 Kağan Kadioğlu <kagankadioglutk@hotmail.com>
 */

#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include "amlogic_dvb.h"

/* ── Timing constants ─────────────────────────────────────────────────── */
#define AML_GPIO_PWR_OFF_MS_DEFAULT	200
#define AML_GPIO_PWR_ON_MS_DEFAULT	200
#define AML_GPIO_RST_ASSERT_MS_DEFAULT	20
#define AML_GPIO_RST_RELEASE_MS_DEFAULT	50

/* ── Maximum GPIO table size ────────────────────────────────────────── */
#define AML_MAX_GPIOS  16

/* ── Consumer name prefix ──────────────────────────────────────────────── */
#define AML_GPIO_PREFIX "amlogic-dvb-"

/* ======================================================================
 * Platform GPIO definition table
 *
 * Each entry: DTS property name, GPIOD direction/initial value, human-readable
 * description and consumer label suffix.
 * ====================================================================== */
static const struct {
	const char *prop; /* DTS property root (xxx-gpios) */
	enum gpiod_flags flags; /* initial direction + value */
	const char *label; /* short human-readable name */
	const char *consumer_suffix; /* /sys/kernel/debug/gpio consumer name */
	bool optional; /* do not error if absent */
} aml_platform_gpios[] = {
	/*
	 * Antenna LNA power switch
	 * DTS: ant-power-gpios = <&gpio GPIOH_6 GPIO_ACTIVE_HIGH>;
	 */
	{ "ant-power", GPIOD_OUT_LOW, "ant-power", "ant-power", true },

	/*
	 * Demodulator power (LDO enable)
	 * DTS: demod-power-gpios = <&gpio GPIOH_3 GPIO_ACTIVE_HIGH>;
	 */
	{ "demod-power", GPIOD_OUT_LOW, "demod-power", "demod-power", true },

	/*
	 * Demodulator hardware reset
	 * DTS: demod-reset-gpios = <&gpio GPIOH_4 GPIO_ACTIVE_LOW>;
	 * ACTIVE_LOW: logic-1 = out of reset (released), logic-0 = held in reset
	 */
	{ "demod-reset", GPIOD_OUT_HIGH, "demod-reset", "demod-reset", true },

	/*
	 * Tuner power
	 * DTS: tuner-power-gpios = <&gpio GPIOZ_6 GPIO_ACTIVE_HIGH>;
	 */
	{ "tuner-power", GPIOD_OUT_LOW, "tuner-power", "tuner-power", true },

	/*
	 * Tuner reset
	 * DTS: tuner-reset-gpios = <&gpio GPIOZ_7 GPIO_ACTIVE_LOW>;
	 */
	{ "tuner-reset", GPIOD_OUT_HIGH, "tuner-reset", "tuner-reset", true },

	/*
	 * TS output enable (mux/buffer enable on some boards)
	 * DTS: ts-out-en-gpios = <&gpio GPIOH_7 GPIO_ACTIVE_HIGH>;
	 */
	{ "ts-out-en", GPIOD_OUT_LOW, "ts-out-en", "ts-out-en", true },

	/*
	 * LNB power enable (for DiSEqC implementations)
	 * DTS: lnb-power-gpios = <&gpio GPIOH_8 GPIO_ACTIVE_HIGH>;
	 */
	{ "lnb-power", GPIOD_OUT_LOW, "lnb-power", "lnb-power", true },

	/*
	 * LNB 13V/18V selection pin
	 * DTS: lnb-voltage-gpios = <&gpio GPIOH_9 GPIO_ACTIVE_HIGH>;
	 */
	{ "lnb-voltage", GPIOD_OUT_LOW, "lnb-voltage", "lnb-voltage", true },
};

/* ======================================================================
 * aml_gpio_register_one() — Register a single GPIO and add it to the table
 *
 * 1. devm_gpiod_get_optional() ile GPIO'yu al
 * 2. gpiod_set_consumer_name() ile /sys/kernel/debug/gpio consumer ata
 * 3. Add to aml_dvb_gpio table (for debugfs)
 * ====================================================================== */
static int aml_gpio_register_one(struct aml_dvb *dvb, const char *prop,
				 enum gpiod_flags flags, const char *label,
				 const char *consumer_suffix, bool optional)
{
	struct device *dev = dvb->dev;
	struct gpio_desc *gd;
	char consumer[64];
	int idx;

	gd = optional ? devm_gpiod_get_optional(dev, prop, flags) :
			devm_gpiod_get(dev, prop, flags);

	if (IS_ERR(gd)) {
		if (PTR_ERR(gd) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		if (!optional)
			return dev_err_probe(dev, PTR_ERR(gd),
					     "Failed to acquire GPIO '%s'\n",
					     prop);
		/* optional and error: warn, continue */
		dev_dbg(dev, "GPIO '%s' not found (optional, skipped)\n", prop);
		return 0;
	}

	if (!gd)
		return 0; /* optional, not in DTS */

	/*
	 * Consumer name: the "consumer" column visible in /sys/kernel/debug/gpio.
	 * Format: "amlogic-dvb-<suffix>"
	 * Example: "amlogic-dvb-demod-reset"
	 *
	 * kernel/gpio.c debugfs format:
	 *   gpio-N (PIN-NAME | CONSUMER-NAME) DIR VALUE [FLAG]
	 */
	snprintf(consumer, sizeof(consumer), "%s%s", AML_GPIO_PREFIX,
		 consumer_suffix);
	gpiod_set_consumer_name(gd, consumer);

	/* Add to table */
	idx = dvb->gpio_count;
	if (idx < AML_MAX_GPIOS) {
		dvb->gpios[idx].gd = gd;
		dvb->gpios[idx].label = label;
		dvb->gpios[idx].consumer = consumer_suffix;
		dvb->gpio_count++;
	}

	dev_info(dev, "GPIO %-16s → %-35s dir=%s val=%d\n", label, consumer,
		 (flags & GPIOD_OUT_LOW || flags & GPIOD_OUT_HIGH) ? "out" :
								     "in",
		 gpiod_get_value_cansleep(gd));

	return 0;
}

/* ======================================================================
 * aml_gpio_init() — Register all platform GPIOs during probe
 *
 * Probe order:
 *   1. Register platform GPIOs (this function)
 *   2. Apply pinctrl state (aml_pinctrl_init)
 *   3. Frontend probe (aml_dvb_probe_frontends)
 *
 * This order is critical: pinctrl configures TS pins, GPIOs handle
 * demod/tuner power control, frontend probe requires both.
 * ====================================================================== */
int aml_gpio_init(struct aml_dvb *dvb)
{
	int i, ret;

	dvb->gpio_count = 0;
	memset(dvb->gpios, 0, sizeof(dvb->gpios));

	dev_dbg(dvb->dev, "GPIO: registering platform GPIOs...\n");

	for (i = 0; i < ARRAY_SIZE(aml_platform_gpios); i++) {
		ret = aml_gpio_register_one(
			dvb, aml_platform_gpios[i].prop,
			aml_platform_gpios[i].flags,
			aml_platform_gpios[i].label,
			aml_platform_gpios[i].consumer_suffix,
			aml_platform_gpios[i].optional);
		if (ret)
			return ret;
	}

	/* Backwards compatibility: populate legacy struct fields */
	for (i = 0; i < dvb->gpio_count; i++) {
		if (!strcmp(dvb->gpios[i].label, "demod-power"))
			dvb->tuner_power_gpio = dvb->gpios[i].gd;
		if (!strcmp(dvb->gpios[i].label, "demod-reset"))
			dvb->tuner_reset_gpio = dvb->gpios[i].gd;
	}

	dev_info(
		dvb->dev,
		"GPIO: %d GPIOs registered, visible in /sys/kernel/debug/gpio\n",
		dvb->gpio_count);
	return 0;
}
EXPORT_SYMBOL_GPL(aml_gpio_init);

/* ======================================================================
 * aml_gpio_release() — Release pinctrl (devm GPIOs are freed automatically)
 * ====================================================================== */
void aml_gpio_release(struct aml_dvb *dvb)
{
	if (!IS_ERR_OR_NULL(dvb->pinctrl)) {
		devm_pinctrl_put(dvb->pinctrl);
		dvb->pinctrl = NULL;
	}
}
EXPORT_SYMBOL_GPL(aml_gpio_release);

/* ======================================================================
 * aml_pinctrl_init() — Configure TS pin multiplexer state
 *
 * Pinctrl state order (first found is used):
 *   1. "default"  — general purpose, defined in DTS with pinctrl-0
 *   2. "parallel" — parallel TS mode (8-bit data + CLK + SYNC)
 *
 * The /sys/kernel/debug/pinctrl/<device>/pingroups file shows which pins
 * are assigned to which group. After pinctrl_select_state(), pins are
 * marked with the "PINCTRL" label in /sys/kernel/debug/gpio.
 *
 * DTS example:
 *   pinctrl-names = "default";
 *   pinctrl-0     = <&ts_in_a_pins &ts_in_b_pins>;
 * ====================================================================== */
/* aml_pinctrl_init/release: defined in amlogic-dvb-core.c */

/* ======================================================================
 * aml_demod_power_reset() — Frontend I2C-level power+reset sequence
 *
 * Called during frontend probe. GPIOs are obtained from the I2C client
 * DTS node. Consumer name is assigned in "amlogic-dvb-feN-{power,reset}"
 * format, visible in /sys/kernel/debug/gpio.
 *
 * DTS example (demod I2C node):
 *   power-gpios          = <&gpio GPIOH_3 GPIO_ACTIVE_HIGH>;
 *   reset-gpios          = <&gpio GPIOH_4 GPIO_ACTIVE_LOW>;
 *   tuner-pwr-gpios      = <&gpio GPIOZ_6 GPIO_ACTIVE_HIGH>;
 *   power-off-delay-ms   = <200>;
 *   power-on-delay-ms    = <200>;
 *   reset-assert-ms      = <20>;
 *   reset-release-ms     = <50>;
 * ====================================================================== */
int aml_demod_power_reset(struct device *dev)
{
	struct gpio_desc *pwr, *rst, *tpwr;
	u32 pwr_off_ms = AML_GPIO_PWR_OFF_MS_DEFAULT;
	u32 pwr_on_ms = AML_GPIO_PWR_ON_MS_DEFAULT;
	u32 rst_assert = AML_GPIO_RST_ASSERT_MS_DEFAULT;
	u32 rst_release = AML_GPIO_RST_RELEASE_MS_DEFAULT;
	char consumer[64];
	static atomic_t fe_idx = ATOMIC_INIT(0);
	int idx = atomic_fetch_inc(&fe_idx);

	of_property_read_u32(dev->of_node, "power-off-delay-ms", &pwr_off_ms);
	of_property_read_u32(dev->of_node, "power-on-delay-ms", &pwr_on_ms);
	of_property_read_u32(dev->of_node, "reset-assert-ms", &rst_assert);
	of_property_read_u32(dev->of_node, "reset-release-ms", &rst_release);

	/* ── power GPIO ── */
	pwr = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(pwr))
		return dev_err_probe(dev, PTR_ERR(pwr),
				     "fe%d: failed to acquire power GPIO\n",
				     idx);
	if (pwr) {
		snprintf(consumer, sizeof(consumer),
			 AML_GPIO_PREFIX "fe%d-power", idx);
		gpiod_set_consumer_name(pwr, consumer);
		dev_info(dev, "GPIO: %s → /sys/kernel/debug/gpio\n", consumer);
	}

	/* ── tuner-pwr GPIO ── */
	tpwr = devm_gpiod_get_optional(dev, "tuner-pwr", GPIOD_OUT_LOW);
	if (!IS_ERR_OR_NULL(tpwr)) {
		snprintf(consumer, sizeof(consumer),
			 AML_GPIO_PREFIX "fe%d-tuner-pwr", idx);
		gpiod_set_consumer_name(tpwr, consumer);
		dev_info(dev, "GPIO: %s → /sys/kernel/debug/gpio\n", consumer);
	}

	/* ── reset GPIO ── */
	rst = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst),
				     "fe%d: failed to acquire reset GPIO\n",
				     idx);
	if (rst) {
		snprintf(consumer, sizeof(consumer),
			 AML_GPIO_PREFIX "fe%d-reset", idx);
		gpiod_set_consumer_name(rst, consumer);
		dev_info(dev, "GPIO: %s → /sys/kernel/debug/gpio\n", consumer);
	}

	if (!pwr && !rst && !tpwr) {
		dev_dbg(dev, "fe%d: no GPIOs defined, sequence skipped\n", idx);
		return 0;
	}

	/*
	 * Power and reset sequence:
	 *   1. Power off → wait (let capacitors discharge)
	 *   2. Turn on tuner power (if present)
	 *   3. Turn on demod power → wait (regulator stabilisation)
	 *   4. Reset assert → wait
	 *   5. Reset release → wait (IC boot)
	 */
	if (pwr) {
		dev_dbg(dev, "[GPIO-fe%d] power OFF (wait %u ms)\n", idx,
			pwr_off_ms);
		gpiod_set_value_cansleep(pwr, 0);
		msleep(pwr_off_ms);
	}
	if (tpwr)
		gpiod_set_value_cansleep(tpwr, 1);
	if (pwr) {
		dev_dbg(dev, "[GPIO-fe%d] power ON (wait %u ms)\n", idx,
			pwr_on_ms);
		gpiod_set_value_cansleep(pwr, 1);
		msleep(pwr_on_ms);
	}
	if (rst) {
		dev_dbg(dev, "[GPIO-fe%d] reset assert (%u ms)\n", idx,
			rst_assert);
		gpiod_set_value_cansleep(rst, 1);
		msleep(rst_assert);
		dev_dbg(dev, "[GPIO-fe%d] reset release (boot wait %u ms)\n",
			idx, rst_release);
		gpiod_set_value_cansleep(rst, 0);
		msleep(rst_release);
	}

	dev_info(
		dev,
		"fe%d: GPIO sequence complete (pwr_off=%u pwr_on=%u rst=%u+%u ms)\n",
		idx, pwr_off_ms, pwr_on_ms, rst_assert, rst_release);
	return 0;
}
EXPORT_SYMBOL_GPL(aml_demod_power_reset);

/* ======================================================================
 * debugfs: /sys/kernel/debug/<dvb>/gpio
 *
 * Shows the current state of all platform GPIOs.
 * Complementary to /sys/kernel/debug/gpio — that shows physical pin
 * location; this shows logical state from the driver perspective.
 *
 * Example output:
 *   # Amlogic DVB GPIO Table
 *   # Detailed physical info: /sys/kernel/debug/gpio
 *   # Consumer names:         /sys/kernel/debug/gpio (| consumer column)
 *   # Pinctrl durumu:         /sys/kernel/debug/pinctrl/<dev>/pingroups
 *   #
 *   # idx  name             dir  logical  raw  polarity       consumer
 *     [0]  demod-reset      out       1    0   ACTIVE_LOW     amlogic-dvb-demod-reset
 *     [1]  demod-power      out       1    1   ACTIVE_HIGH    amlogic-dvb-demod-power
 *     [2]  ant-power        out       0    0   ACTIVE_HIGH    amlogic-dvb-ant-power
 *
 *   # Pinctrl
 *     state: default
 *     hint:  cat /sys/kernel/debug/pinctrl/<dev>/pingroups
 * ====================================================================== */
static int aml_gpio_debugfs_show(struct seq_file *s, void *v)
{
	struct aml_dvb *dvb = s->private;
	int i;

	seq_puts(s,
		"# Amlogic DVB GPIO Table\n"
		"# /sys/kernel/debug/gpio         — full GPIO list (consumer column)\n"
		"# /sys/kernel/debug/pinctrl/...  — pinctrl mux details\n"
		"#\n");

	if (dvb->gpio_count == 0) {
		seq_puts(
			s,
			"# No GPIOs defined (gpios property missing in DTS)\n");
		goto pinctrl_section;
	}

	seq_printf(s, "# %-4s  %-16s  %-4s  %-7s  %-4s  %-14s  %s\n", "idx",
		   "name", "dir", "logical", "raw", "polarity",
		   "consumer (/sys/kernel/debug/gpio)");
	seq_puts(s, "#----  ----------------  ----  -------  ----"
		    "  --------------  ---------------------------------\n");

	for (i = 0; i < dvb->gpio_count; i++) {
		struct gpio_desc *gd = dvb->gpios[i].gd;
		int logical, raw;
		bool active_low;
		const char *dir;

		if (!gd)
			continue;

		logical = gpiod_get_value_cansleep(gd);
		raw = gpiod_get_raw_value_cansleep(gd);
		active_low = (logical != raw);
		dir = gpiod_get_direction(gd) ? "in" : "out";

		seq_printf(s, "  [%d]  %-16s  %-4s  %-7d  %-4d  %-14s  %s%s\n",
			   i, dvb->gpios[i].label, dir, logical, raw,
			   active_low ? "ACTIVE_LOW" : "ACTIVE_HIGH",
			   AML_GPIO_PREFIX, dvb->gpios[i].consumer);
	}

pinctrl_section:
	seq_puts(s, "\n# Pinctrl\n");

	if (!IS_ERR_OR_NULL(dvb->pinctrl)) {
		const char *state_name =
			dvb->pins_default ? "default (active)" : "undefined";
		struct device *pindev = &dvb->pdev->dev;
		(void)pindev;

		seq_printf(s, "  state    : %s\n", state_name);
		seq_puts(s, "  detail   : cat /sys/kernel/debug/pinctrl/"
			    "<periphs-pinctrl>/pingroups\n"
			    "  ts-pins  : cat /sys/kernel/debug/pinctrl/"
			    "<periphs-pinctrl>/pins\n");
	} else {
		seq_puts(s, "  pinctrl  : not configured\n");
	}

	seq_puts(
		s,
		"\n# How to read?\n"
		"#   logical=1 + raw=0 + ACTIVE_LOW  → pin physically LOW, logic HIGH (active)\n"
		"#   logical=1 + raw=1 + ACTIVE_HIGH → pin physically HIGH, logic HIGH (active)\n"
		"#   direction=out → driver is controlling\n"
		"#   direction=in  → pin state is being read (input)\n");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(aml_gpio_debugfs);

/* ======================================================================
 * debugfs: /sys/kernel/debug/<dvb>/pinctrl_state
 *
 * Lists the current and available pinctrl states.
 * Reads TS pin group names from the kernel pinctrl subsystem.
 * ====================================================================== */
static int aml_pinctrl_debugfs_show(struct seq_file *s, void *v)
{
	struct aml_dvb *dvb = s->private;

	seq_puts(s, "# Amlogic DVB Pinctrl State\n#\n");

	if (IS_ERR_OR_NULL(dvb->pinctrl)) {
		seq_puts(s, "pinctrl not configured\n\n");
		seq_puts(s, "Add to DTS:\n"
			    "  pinctrl-names = \"default\";\n"
			    "  pinctrl-0     = <&ts_in_a_pins>;\n");
		return 0;
	}

	seq_printf(s, "pinctrl device  : %s\n", dev_name(dvb->dev));
	seq_printf(s, "current state   : %s\n",
		   dvb->pins_default ? "default" : "(none)");

	seq_puts(s, "\n# TS pin mux reference (SM1/S905X3 example):\n"
		    "#\n"
		    "#  Paralel mod (DTS: ts_in_a_pins):\n"
		    "#    GPIOX_0..7 → ts0_d0..d7\n"
		    "#    GPIOX_8    → ts0_clk\n"
		    "#    GPIOX_9    → ts0_sync\n"
		    "#    GPIOX_10   → ts0_valid (optional)\n"
		    "#\n"
		    "#  Serial mode S_TS0 (DTS: s_ts0_pins):\n"
		    "#    GPIOZ_2    → ts0_serial_data\n"
		    "#    GPIOZ_3    → ts0_serial_clk\n"
		    "#\n"
		    "#  List pinctrl pin groups:\n"
		    "#    cat /sys/kernel/debug/pinctrl/<periphs>/pingroups\n"
		    "#\n"
		    "#  Which pin is in which group:\n"
		    "#    cat /sys/kernel/debug/pinctrl/<periphs>/pins\n");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(aml_pinctrl_debugfs);

/* ======================================================================
 * aml_gpio_debugfs_init() — Create debugfs files
 * ====================================================================== */
void aml_gpio_debugfs_init(struct aml_dvb *dvb, struct dentry *root)
{
	if (!root)
		return;

	/* /sys/kernel/debug/<dvb>/gpio — full platform GPIO table */
	debugfs_create_file("gpio", 0444, root, dvb, &aml_gpio_debugfs_fops);

	/* /sys/kernel/debug/<dvb>/pinctrl_state — pinctrl state */
	debugfs_create_file("pinctrl_state", 0444, root, dvb,
			    &aml_pinctrl_debugfs_fops);
}
EXPORT_SYMBOL_GPL(aml_gpio_debugfs_init);
