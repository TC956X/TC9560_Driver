/*
 * TC9562 ethernet driver.
 *
 * tc9562mac_ptp.c
 *
 * Copyright (C) 2013  Vayavya Labs Pvt Ltd
 * Copyright (C) 2019 Toshiba Electronic Devices & Storage Corporation
 *
 * This file has been derived from the STMicro Linux driver,
 * and developed or modified for TC9562.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*! History:
 *  30 Sep 2019 : Base lined
 *  VERSION     : 01-00
 */

#include "tc9562mac.h"
#include "tc9562mac_ptp.h"

/**
 * tc9562mac_adjust_freq
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ppb: desired period change in parts ber billion
 *
 * Description: this function will adjust the frequency of hardware clock.
 */
static int tc9562mac_adjust_freq(struct ptp_clock_info *ptp, s32 ppb)
{
	struct tc9562mac_priv *priv =
	    container_of(ptp, struct tc9562mac_priv, ptp_clock_ops);
	unsigned long flags;
	u32 diff, addend;
	int neg_adj = 0;
	u64 adj;

	DBGPR_FUNC_PTP("-->tc9562mac_adjust_freq\n");

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}

	addend = priv->default_addend;
	adj = addend;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);
	addend = neg_adj ? (addend - diff) : (addend + diff);

	spin_lock_irqsave(&priv->ptp_lock, flags);

	priv->hw->ptp->config_addend(priv->ptpaddr, addend);

	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	DBGPR_FUNC_PTP("<--tc9562mac_adjust_freq\n");

	return 0;
}

/**
 * tc9562mac_adjust_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @delta: desired change in nanoseconds
 *
 * Description: this function will shift/adjust the hardware clock time.
 */
static int tc9562mac_adjust_time(struct ptp_clock_info *ptp, s64 delta)
{
	struct tc9562mac_priv *priv =
	    container_of(ptp, struct tc9562mac_priv, ptp_clock_ops);
	unsigned long flags;
	u32 sec, nsec;
	u32 quotient, reminder;
	int neg_adj = 0;

	DBGPR_FUNC_PTP("-->tc9562mac_adjust_time\n");
	
	if (delta < 0) {
		neg_adj = 1;
		delta = -delta;
	}

	quotient = div_u64_rem(delta, 1000000000ULL, &reminder);
	sec = quotient;
	nsec = reminder;

	spin_lock_irqsave(&priv->ptp_lock, flags);

	priv->hw->ptp->adjust_systime(priv->ptpaddr, sec, nsec, neg_adj,
				      priv->plat->has_gmac4);

	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	DBGPR_FUNC_PTP("<--tc9562mac_adjust_time\n");
	
	return 0;
}

/**
 * tc9562mac_get_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ts: pointer to hold time/result
 *
 * Description: this function will read the current time from the
 * hardware clock and store it in @ts.
 */
static int tc9562mac_get_time(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct tc9562mac_priv *priv =
	    container_of(ptp, struct tc9562mac_priv, ptp_clock_ops);
	unsigned long flags;
	u64 ns;

	DBGPR_FUNC_PTP("-->tc9562mac_get_time\n");
		
	spin_lock_irqsave(&priv->ptp_lock, flags);

	ns = priv->hw->ptp->get_systime(priv->ptpaddr);

	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	*ts = ns_to_timespec64(ns);

	DBGPR_FUNC_PTP("<--tc9562mac_get_time\n");

	return 0;
}

/**
 * tc9562mac_set_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ts: time value to set
 *
 * Description: this function will set the current time on the
 * hardware clock.
 */
static int tc9562mac_set_time(struct ptp_clock_info *ptp,
			   const struct timespec64 *ts)
{
	struct tc9562mac_priv *priv =
	    container_of(ptp, struct tc9562mac_priv, ptp_clock_ops);
	unsigned long flags;

	DBGPR_FUNC_PTP("-->tc9562mac_set_time\n");

	spin_lock_irqsave(&priv->ptp_lock, flags);

	priv->hw->ptp->init_systime(priv->ptpaddr, ts->tv_sec, ts->tv_nsec);

	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	DBGPR_FUNC_PTP("<--tc9562mac_set_time\n");

	return 0;
}

static int tc9562mac_enable(struct ptp_clock_info *ptp,
			 struct ptp_clock_request *rq, int on)
{
	DBGPR_FUNC_PTP("-->tc9562mac_enable\n");
	return -EOPNOTSUPP;
}

/* structure describing a PTP hardware clock */
static const struct ptp_clock_info tc9562mac_ptp_clock_ops = {
	.owner = THIS_MODULE,
	.name = "tc9562_ptp_clock",
	.max_adj = 62500000,
	.n_alarm = 0,
	.n_ext_ts = 0,
	.n_per_out = 0,
	.n_pins = 0,
	.pps = 0,
	.adjfreq = tc9562mac_adjust_freq,
	.adjtime = tc9562mac_adjust_time,
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(4, 0, 9))
    .gettime = tc9562mac_get_time,
    .settime = tc9562mac_set_time,
#else	
	.gettime64 = tc9562mac_get_time,
	.settime64 = tc9562mac_set_time,
#endif	
	.enable = tc9562mac_enable,
};

/**
 * tc9562mac_ptp_register
 * @priv: driver private structure
 * Description: this function will register the ptp clock driver
 * to kernel. It also does some house keeping work.
 */
void tc9562mac_ptp_register(struct tc9562mac_priv *priv)
{
	DBGPR_FUNC_PTP("-->tc9562mac_ptp_register\n");
	
	spin_lock_init(&priv->ptp_lock);
	priv->ptp_clock_ops = tc9562mac_ptp_clock_ops;

	priv->ptp_clock = ptp_clock_register(&priv->ptp_clock_ops,
					     priv->device);
	if (IS_ERR(priv->ptp_clock)) {
		netdev_err(priv->dev, "ptp_clock_register failed\n");
		priv->ptp_clock = NULL;
	} else if (priv->ptp_clock)
		netdev_info(priv->dev, "registered PTP clock\n");

	DBGPR_FUNC_PTP("<--tc9562mac_ptp_register\n");
}

/**
 * tc9562mac_ptp_unregister
 * @priv: driver private structure
 * Description: this function will remove/unregister the ptp clock driver
 * from the kernel.
 */
void tc9562mac_ptp_unregister(struct tc9562mac_priv *priv)
{
	DBGPR_FUNC_PTP("-->tc9562mac_ptp_unregister\n");
	if (priv->ptp_clock) {
		ptp_clock_unregister(priv->ptp_clock);
		priv->ptp_clock = NULL;
		pr_debug("Removed PTP HW clock successfully on %s\n",
			 priv->dev->name);
	}
	DBGPR_FUNC_PTP("<--tc9562mac_ptp_unregister\n");
}
