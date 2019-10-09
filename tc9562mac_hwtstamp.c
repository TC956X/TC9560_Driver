/*
 * TC9562 ethernet driver.
 *
 * tc9562mac_hwtstamp.c
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

#include <linux/io.h>
#include <linux/delay.h>
#include "common.h"
#include "tc9562mac_ptp.h"

static void tc9562mac_config_hw_tstamping(void __iomem *ioaddr, u32 data)
{
	DBGPR_FUNC_PTP("-->tc9562mac_config_hw_tstamping\n");
	writel(data, ioaddr + PTP_TCR);
	DBGPR_FUNC_PTP("<--tc9562mac_config_hw_tstamping\n");
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static u32 tc9562mac_get_hw_tstamping(void __iomem *ioaddr)
{
	DBGPR_FUNC_PTP("-->tc9562mac_get_hw_tstamping\n");
	return readl(ioaddr + PTP_TCR);
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

static u32 tc9562mac_get_ptp_period(void __iomem *ioaddr, u32 ptp_clock)
{
	u32 value = readl(ioaddr + PTP_TCR);
	u64 data;

	DBGPR_FUNC_PTP("-->tc9562mac_get_ptp_period\n");

	/* For GMAC3.x, 4.x versions, convert the ptp_clock to nano second
	 *	formula = (1/ptp_clock) * 1000000000
	 * where ptp_clock is 50MHz if fine method is used to update system
	 */
	if (value & PTP_TCR_TSCFUPDT)
		data = (1000000000ULL / ptp_clock);		
	else
		data = (1000000000ULL / 250000000);		

	DBGPR_FUNC_PTP("<--tc9562mac_get_ptp_period\n");

	return (u32)data;
}

static u32 tc9562mac_get_ptp_subperiod(void __iomem *ioaddr, u32 ptp_clock)
{
	u32 value = readl(ioaddr + PTP_TCR);
	u64 data;

	DBGPR_FUNC_PTP("-->tc9562mac_get_ptp_subperiod\n");
	/* For GMAC3.x, 4.x versions, convert the ptp_clock to nano second
	 *	formula = (1/ptp_clock) * 1000000000
	 * where ptp_clock is 50MHz if fine method is used to update system
	 */
	if (value & PTP_TCR_TSCFUPDT)

	data = (1000000000ULL * 1000ULL / ptp_clock);
	else
		data = (1000000000ULL * 1000ULL/ 250000000);		

	DBGPR_FUNC_PTP("<--tc9562mac_get_ptp_subperiod\n");
	return  data - tc9562mac_get_ptp_period(ioaddr, ptp_clock) * 1000;
}

static u32 tc9562mac_config_sub_second_increment(void __iomem *ioaddr,
					      u32 ptp_clock, int gmac4)
{
	u32 value = readl(ioaddr + PTP_TCR);
	u32 subns, ns;
	u64 tmp;

	DBGPR_FUNC_PTP("-->tc9562mac_config_sub_second_increment\n");

	ns = tc9562mac_get_ptp_period(ioaddr, ptp_clock);
	subns = tc9562mac_get_ptp_subperiod(ioaddr, ptp_clock);

	/* 0.465ns accuracy */
	if (!(value & PTP_TCR_TSCTRLSSR)) {
		tmp = ns * 1000;
		ns = DIV_ROUND_CLOSEST(tmp - (tmp % 465), 465);
		subns = DIV_ROUND_CLOSEST((tmp * 256) - (465 * ns * 256), 465);
	} else {
		subns = DIV_ROUND_CLOSEST(subns * 256, 1000);
	}

	ns &= PTP_SSIR_SSINC_MASK;
	subns &= PTP_SSIR_SNSINC_MASK;

	value = ns;
	if (gmac4) {
		value = value << GMAC4_PTP_SSIR_SSINC_SHIFT;
		value |= subns << GMAC4_PTP_SSIR_SNSINC_SHIFT;
	}

	writel(value, ioaddr + PTP_SSIR);

	DBGPR_FUNC_PTP("<--tc9562mac_config_sub_second_increment\n");
	
	return ns;
}

static int tc9562mac_init_systime(void __iomem *ioaddr, u32 sec, u32 nsec)
{
	int limit;
	u32 value;

	DBGPR_FUNC_PTP("-->tc9562mac_init_systime\n");

	writel(sec, ioaddr + PTP_STSUR);
	writel(nsec, ioaddr + PTP_STNSUR);
	/* issue command to initialize the system time value */
	value = readl(ioaddr + PTP_TCR);
	value |= PTP_TCR_TSINIT;
	writel(value, ioaddr + PTP_TCR);

	/* wait for present system time initialize to complete */
	limit = 10;
	while (limit--) {
		if (!(readl(ioaddr + PTP_TCR) & PTP_TCR_TSINIT))
			break;
		mdelay(10);
	}
	if (limit < 0)
		return -EBUSY;

	DBGPR_FUNC_PTP("<--tc9562mac_init_systime\n");

	return 0;
}

static int tc9562mac_config_addend(void __iomem *ioaddr, u32 addend)
{
	u32 value;
	int limit;

	DBGPR_FUNC_PTP("-->tc9562mac_config_addend\n");

	writel(addend, ioaddr + PTP_TAR);
	/* issue command to update the addend value */
	value = readl(ioaddr + PTP_TCR);
	value |= PTP_TCR_TSADDREG;
	writel(value, ioaddr + PTP_TCR);

	/* wait for present addend update to complete */
	limit = 10;
	while (limit--) {
		if (!(readl(ioaddr + PTP_TCR) & PTP_TCR_TSADDREG))
			break;
		mdelay(10);
	}
	if (limit < 0)
		return -EBUSY;

	DBGPR_FUNC_PTP("<--tc9562mac_config_addend\n");

	return 0;
}
static int tc9562mac_adjust_systime(void __iomem *ioaddr, u32 sec, u32 nsec,
				 int add_sub, int gmac4)
{
	u32 value;
	int limit;

	DBGPR_FUNC_PTP("-->tc9562mac_adjust_systime\n");

	/* wait for present system time adjust/update to complete */
	limit = 100000;
	while (limit--) {
		if (!(readl(ioaddr + PTP_TCR) & PTP_TCR_TSUPDT))
			break;
		mdelay(10);
	}
	if (limit < 0)
		return -EBUSY;

	if (add_sub) {
		/* If the new sec value needs to be subtracted with
		 * the system time, then MAC_STSUR reg should be
		 * programmed with (2^32 – <new_sec_value>)
		 */
		if (gmac4)
		{
			sec = (0x100000000ULL - sec);
		}

		value = readl(ioaddr + PTP_TCR);
		if (value & PTP_TCR_TSCTRLSSR)
		{
			nsec = (PTP_DIGITAL_ROLLOVER_MODE - nsec);
		}
		else
		{
			nsec = (PTP_BINARY_ROLLOVER_MODE - nsec);
		}
	}

	writel(sec, ioaddr + PTP_STSUR);
	value = (add_sub << PTP_STNSUR_ADDSUB_SHIFT) | nsec;
	writel(value, ioaddr + PTP_STNSUR);

	/* issue command to initialize the system time value */
	value = readl(ioaddr + PTP_TCR);
	value |= PTP_TCR_TSUPDT;
	writel(value, ioaddr + PTP_TCR);

	/* wait for present system time adjust/update to complete */
	limit = 10;
	while (limit--) {
		if (!(readl(ioaddr + PTP_TCR) & PTP_TCR_TSUPDT))
			break;
		mdelay(10);
	}
	if (limit < 0)
		return -EBUSY;

	DBGPR_FUNC_PTP("<--tc9562mac_adjust_systime\n");
	
	return 0;
}

static u64 tc9562mac_get_systime(void __iomem *ioaddr)
{
	u64 ns;
    u32 sec, sec_rollover_read;
    u32 nsec, nsec_rollover_read;
        
	DBGPR_FUNC_PTP("-->tc9562mac_get_systime\n");

    sec = readl(ioaddr + PTP_STSR);
    nsec = readl(ioaddr + PTP_STNSR);

    sec_rollover_read = readl(ioaddr + PTP_STSR);
    nsec_rollover_read = readl(ioaddr + PTP_STNSR);
    
    if(sec != sec_rollover_read)
    {
        sec = sec_rollover_read;
        nsec = nsec_rollover_read;
    }   

	/* Get the TSSS value */
	ns = nsec;
	/* Get the TSS and convert sec time value to nanosecond */
	ns += sec * 1000000000ULL;

	DBGPR_FUNC_PTP("<--tc9562mac_get_systime\n");

	return ns;
}

const struct tc9562mac_hwtimestamp tc9562mac_ptp = {
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE	
	.get_hw_tstamping = tc9562mac_get_hw_tstamping,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
	.get_ptp_period = tc9562mac_get_ptp_period,
	.config_hw_tstamping = tc9562mac_config_hw_tstamping,
	.init_systime = tc9562mac_init_systime,
	.config_sub_second_increment = tc9562mac_config_sub_second_increment,
	.config_addend = tc9562mac_config_addend,
	.adjust_systime = tc9562mac_adjust_systime,
	.get_systime = tc9562mac_get_systime,
};
