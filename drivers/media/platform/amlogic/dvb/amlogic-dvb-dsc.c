// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic DVB descrambler (DVB-CSA/AES/DES) support
 * CA extension: CA_SET_DESCR_EX, CA_SC2_SET_DESCR_EX, CA_SET_PID
 *
 * Copyright (C) 2026 Kağan Kadioğlu <kagankadioglutk@hotmail.com>
 */

#include <linux/dvb/ca.h>
#include "amlogic_dvb.h"
#include "amlogic-dvb-regs.h"
#include "amlogic-dvb-ca-ext.h"

static int aml_dsc_set_pid(struct aml_dsc *dsc, int chan, u16 pid)
{
	struct aml_dvb *dvb = dsc->dvb;
	int idx = chan * 2;
	int ret;

	if (chan >= 8)
		return -EINVAL;

	dev_dbg(dvb->dev, "dsc%d: set_pid chan=%d pid=0x%x\n", dsc->id, chan,
		pid);

	ret = aml_write_reg(dvb, TS_PL_PID_INDEX, idx);
	if (ret) {
		dev_err(dvb->dev, "dsc%d: failed to write PID_INDEX: %d\n",
			dsc->id, ret);
		return ret;
	}

	ret = aml_write_reg(dvb, TS_PL_PID_DATA,
			    (pid & 0x1FFF) | ((pid & 0x1FFF) << 16));
	if (ret)
		dev_err(dvb->dev, "dsc%d: failed to write PID_DATA: %d\n",
			dsc->id, ret);
	return ret;
}

static int aml_dsc_set_key(struct aml_dsc *dsc, int chan, const u8 *key,
			   bool odd)
{
	struct aml_dvb *dvb = dsc->dvb;
	u32 key_lo, key_hi;
	u32 reg;
	int ret;

	if (chan >= 8)
		return -EINVAL;

	key_lo = (key[0] << 24) | (key[1] << 16) | (key[2] << 8) | key[3];
	key_hi = (key[4] << 24) | (key[5] << 16) | (key[6] << 8) | key[7];

	dev_dbg(dvb->dev, "dsc%d: set_key chan=%d odd=%d\n", dsc->id, chan,
		odd);

	ret = aml_write_reg(dvb, COMM_DESC_KEY0, key_hi);
	if (ret)
		return ret;
	ret = aml_write_reg(dvb, COMM_DESC_KEY1, key_lo);
	if (ret)
		return ret;

	reg = chan;
	if (odd)
		reg |= DESC_WRITE_CW_HI;
	else
		reg |= DESC_WRITE_CW_LO;

	return aml_write_reg(dvb, COMM_DESC_KEY_RW, reg);
}

/*
 * CA_SET_DESCR_EX — CSA/AES/DES/SM4 extended key set
 * Currently CSA (8-byte CW) is supported; AES/DES/SM4 are stubs.
 */
static int aml_dsc_set_descr_ex(struct aml_dsc *dsc,
				const struct ca_descr_ex *dx)
{
	struct aml_dvb *dvb = dsc->dvb;
	int chan = dx->index;
	bool odd;

	if (chan >= 8)
		return -EINVAL;

	switch (dx->type) {
	case CA_CW_DVB_CSA_EVEN:
		odd = false;
		break;
	case CA_CW_DVB_CSA_ODD:
		odd = true;
		break;
	default:
		/* AES/DES/SM4 — hardware does not directly support these, ignore */
		dev_dbg(dvb->dev,
			"dsc%d: CA_SET_DESCR_EX type=%d (not HW-supported)\n",
			dsc->id, dx->type);
		return 0;
	}

	dev_dbg(dvb->dev, "dsc%d: CA_SET_DESCR_EX chan=%d odd=%d\n", dsc->id,
		chan, odd);

	spin_lock(&dsc->lock);
	if (!dsc->channel[chan].used) {
		dsc->channel[chan].used = true;
		aml_dsc_set_pid(dsc, chan, 0x1FFF);
	}
	memcpy(odd ? dsc->channel[chan].cw_odd : dsc->channel[chan].cw_even,
	       dx->cw, 8);
	aml_dsc_set_key(dsc, chan, dx->cw, odd);
	spin_unlock(&dsc->lock);
	return 0;
}

static int aml_dsc_ioctl(struct file *file, unsigned int cmd, void *parg)
{
	struct dvb_device *dvbdev = file->private_data;
	struct aml_dsc *dsc = dvbdev->priv;
	int ret = 0;

	dev_dbg(dsc->dvb->dev, "dsc%d: ioctl cmd=0x%x\n", dsc->id, cmd);

	switch (cmd) {
	case CA_SET_DESCR: {
		struct ca_descr *d = parg;
		int chan = d->index;
		bool odd = d->parity;

		if (chan >= 8)
			return -EINVAL;

		dev_dbg(dsc->dvb->dev, "dsc%d: CA_SET_DESCR chan=%d odd=%d\n",
			dsc->id, chan, odd);

		spin_lock(&dsc->lock);
		if (!dsc->channel[chan].used) {
			dsc->channel[chan].used = true;
			aml_dsc_set_pid(dsc, chan, 0x1FFF);
		}
		memcpy(odd ? dsc->channel[chan].cw_odd :
			     dsc->channel[chan].cw_even,
		       d->cw, 8);
		ret = aml_dsc_set_key(dsc, chan, d->cw, odd);
		spin_unlock(&dsc->lock);
		break;
	}
	case CA_SET_DESCR_EX: {
		ret = aml_dsc_set_descr_ex(dsc, (struct ca_descr_ex *)parg);
		break;
	}
	case CA_SC2_SET_DESCR_EX: {
		/*
         * SC2 (S905D3+) extended CA — hardware not present on this SoC.
         * Return -ENODEV to userspace so it can fall back.
         */
		dev_dbg(dsc->dvb->dev,
			"dsc%d: CA_SC2_SET_DESCR_EX (not supported on SM1)\n",
			dsc->id);
		ret = -ENODEV;
		break;
	}
	case CA_SET_PID: {
		struct ca_pid *p = parg;
		int chan = p->index;

		if (chan >= 8)
			return -EINVAL;

		dev_dbg(dsc->dvb->dev, "dsc%d: CA_SET_PID chan=%d pid=0x%x\n",
			dsc->id, chan, p->pid);

		spin_lock(&dsc->lock);
		if (p->pid == 0x1FFF || p->pid > 0x1FFF) {
			if (dsc->channel[chan].used) {
				aml_dsc_set_pid(dsc, chan, 0x1FFF);
				dsc->channel[chan].used = false;
			}
		} else {
			dsc->channel[chan].used = true;
			dsc->channel[chan].pid = p->pid;
			aml_dsc_set_pid(dsc, chan, p->pid);
		}
		spin_unlock(&dsc->lock);
		break;
	}
	default:
		ret = -ENOTTY;
		break;
	}
	return ret;
}

static const struct file_operations aml_dsc_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = dvb_generic_ioctl,
	.open = dvb_generic_open,
	.release = dvb_generic_release,
};

static struct dvb_device aml_dsc_template = {
	.priv = NULL,
	.users = 1,
	.readers = 1,
	.writers = 1,
	.fops = &aml_dsc_fops,
	.kernel_ioctl = aml_dsc_ioctl,
};

int aml_dsc_init_all(struct aml_dvb *dvb)
{
	int i, ret;

	dev_info(dvb->dev, "dsc_init_all: starting, num=%d\n",
		 dvb->caps.num_dsc);

	for (i = 0; i < dvb->caps.num_dsc; i++) {
		struct aml_dsc *dsc = &dvb->dsc[i];

		dev_info(dvb->dev, "dsc_init_all: initializing DSC %d\n", i);

		dsc->dvb = dvb;
		dsc->id = i;
		dsc->enabled = false;
		spin_lock_init(&dsc->lock);

		aml_write_reg(dvb, DSC_CTRL(i), DSC_CTRL_RESET);
		udelay(10);
		aml_write_reg(dvb, DSC_CTRL(i), DSC_CTRL_ENABLE);
		dsc->enabled = true;

		dev_info(dvb->dev, "dsc%d: registering CA device\n", i);
		ret = dvb_register_device(&dvb->adapter, &dsc->dev,
					  &aml_dsc_template, dsc, DVB_DEVICE_CA,
					  0);
		if (ret) {
			dev_err(dvb->dev,
				"dsc%d: dvb_register_device failed: %d\n", i,
				ret);
			goto err;
		}
		dev_info(dvb->dev, "dsc%d: initialized OK\n", i);
	}
	return 0;
err:
	aml_dsc_release_all(dvb);
	return ret;
}
EXPORT_SYMBOL_GPL(aml_dsc_init_all);

void aml_dsc_release_all(struct aml_dvb *dvb)
{
	int i;

	dev_info(dvb->dev, "dsc_release_all\n");

	for (i = 0; i < dvb->caps.num_dsc; i++) {
		if (dvb->dsc[i].dev) {
			dvb_unregister_device(dvb->dsc[i].dev);
			dvb->dsc[i].dev = NULL;
		}
		if (dvb->dsc[i].enabled) {
			aml_write_reg(dvb, DSC_CTRL(i), 0);
			dvb->dsc[i].enabled = false;
		}
	}
}
EXPORT_SYMBOL_GPL(aml_dsc_release_all);
