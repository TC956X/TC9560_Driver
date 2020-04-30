/*
 * TC9562 ethernet driver.
 *
 * dwmac5.c
 *
 * Copyright (C) 2017 Synopsys, Inc. and/or its affiliates.
 * Copyright (C) 2020 Toshiba Electronic Devices & Storage Corporation
 *
 * This file has been derived from the STMicro and Synopsys Linux driver,
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
 *  26 Feb 2020 : 1. Added TC - FRP feature support.
                  2. Modified FRP feature to support continues table entries.                
 *  VERSION     : 01-01
 *  30 Sep 2019 : Base lined
 *  VERSION     : 01-00
 */

#include <linux/bitops.h>
#include "common.h"
#include "dwmac4.h"
#include "dwmac5.h"
#include "tc9562mac.h"
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
#include <linux/iopoll.h>
#endif
#include "tc9562mac_ioctl.h"

//#define GCL_PRINT
int dwmac5_est_read(struct mac_device_info *hw, u32 reg, u32 *val,
		bool is_gcla)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 control = 0x0;
	int timeout = 500;
#ifdef GCL_PRINT
	static unsigned int count=0;
#endif 

#ifdef GCL_PRINT
	if (count % 8000 == 0)
	{
    if (readl(ioaddr + MTL_EST_STATUS) & MTL_EST_SWOL)
    {
    		printk("GCL0:hw\n");
				//control &= (~(1 << 5));    // Note: forcing to read bank 0
    }
		else 
		{
    		printk("GCL1:hw\n");
				//control |= (1 << 5);    // Note: forcing to read bank 1 
		}	
	}
count++;
#endif

    if (readl(ioaddr + MTL_EST_STATUS) & MTL_EST_SWOL)
    {
    		//printk("G0:hw\n");
				control &= (~(1 << 5));    // Note: forcing to read bank 0
    }
	else 
	{
		//printk("G1:hw\n");
			control |= (1 << 5);    // Note: forcing to read bank 1 
	}

	control |= (1 << 4);    // Note: debug mode enable  
	control |= reg;
	control |= is_gcla ? 0x0 : MTL_EST_GCRR;
	control |= MTL_EST_R1W0;
//	writel(control, ioaddr + MTL_EST_GCL_CONTROL);

	control |= MTL_EST_SRWO;
	writel(control, ioaddr + MTL_EST_GCL_CONTROL);

	while (--timeout) {
		udelay(10);
		if (readl(ioaddr + MTL_EST_GCL_CONTROL) & MTL_EST_SRWO)
			continue;
		break;
	}

	if (!timeout)
		return -ETIMEDOUT;

	*val = readl(ioaddr + MTL_EST_GCL_DATA);
	return 0;
}

static int dwmac5_est_write(struct mac_device_info *hw, u32 reg, u32 val,
		bool is_gcla)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 control = 0x0;
	int timeout = 5;

	writel(val, ioaddr + MTL_EST_GCL_DATA);

	control |= reg;
	control |= is_gcla ? 0x0 : MTL_EST_GCRR;
	control |= 0x0;
	writel(control, ioaddr + MTL_EST_GCL_CONTROL);

	control |= MTL_EST_SRWO;
	writel(control, ioaddr + MTL_EST_GCL_CONTROL);

	while (--timeout) {
		udelay(1000);
		if (readl(ioaddr + MTL_EST_GCL_CONTROL) & MTL_EST_SRWO)
			continue;
		break;
	}

	if (!timeout)
		return -ETIMEDOUT;
	return 0;
}
#if defined(TX_LOGGING_TRACE)
u32 switch_cnt = 0;
#endif
int dwmac5_est_init(struct net_device *ndev, struct mac_device_info *hw,
		struct tc9562mac_est_cfg *cfg, unsigned int estsel,
		unsigned int estdep, unsigned int estwid)
{
	void __iomem *ioaddr = hw->pcsr;
    	struct tc9562mac_priv *priv = netdev_priv(ndev);
	u32 control, real_btr[2];
	int i;

#if defined(TX_LOGGING_TRACE)
    u32 read_val = 0;
	int j;
	u64 read_btr = 0, read_ctr = 0;
    u32 read_btrl = 0, read_btrh = 0;
    u32 read_ctrl = 0, read_ctrh = 0;
    u32 read_llr = 0, read_ter = 0;

#endif	
	u64 system_time;
	u32 system_time_ns;
	u32 system_time_s;

#if defined(TX_LOGGING_TRACE)
	char *qptr,*pptr;

	pptr = (char *)kzalloc(100*100, GFP_KERNEL);
	if(!pptr)
	{
		printk("Malloc pptr Error\n");
		return -ENOMEM;
	}
	qptr = (char *)kzalloc(100*100, GFP_KERNEL);
	if(!qptr)
	{
		printk("Malloc qptr Error\n");
    	kfree(pptr);        
		return -ENOMEM;
	}
#endif
	DBGPR_FUNC("dwmac5_est_init cfg->gcl_size : %d\n", cfg->gcl_size);

	if (!estsel || !estdep || !estwid || !cfg)
		return -EINVAL;
	if (cfg->gcl_size > estdep) {
		netdev_err(ndev, "Invalid EST configuration supplied\n");
		return -EINVAL;
	}

	if (!cfg->enable) {
		/* Disable EST */
		control = readl(ioaddr + MTL_EST_CONTROL);
		control &= ~MTL_EST_EEST;
		writel(control, ioaddr + MTL_EST_CONTROL);
		return 0;
	}


	/* BTR Offset */
    system_time = priv->hw->ptp->get_systime(priv->ptpaddr);
    system_time_s = system_time / 1000000000;
    system_time_ns = system_time % 1000000000;
    DBGPR_FUNC("syst: sec = 0x%x, ns = 0x%x\n", system_time_s, system_time_ns);
    //system_time_s = readl(priv->ptpaddr + 0x0c) ; 
    //system_time_ns = readl(priv->ptpaddr + 0x08) ; 
     
    real_btr[0] = cfg->btr_offset[0] + (u32)system_time_ns;
    DBGPR_FUNC("syst: %s now.tv_nsec: %x\n", __func__, (u32)system_time_ns);
    real_btr[1] = cfg->btr_offset[1] + (u32)system_time_s;
    DBGPR_FUNC("syst: %s now.tv_sec: %x\n", __func__, (u32)system_time_s);


#define EST_WRITE(__a, __b, __c) do { \
	if (dwmac5_est_write(hw, __a, __b, __c)) \
		goto write_fail; \
	} while (0);

#define EST_READ(__a, __b, __c) do { \
	if (dwmac5_est_read(hw, __a, __b, __c)) \
		goto write_fail; \
	} while (0);

	/* Configure EST */
	printk("*** EST write ***\n");
	EST_WRITE(MTL_EST_BTR_LOW, real_btr[0], false);
	printk(" %s MTL_EST_BTR_LOW: %x\n", __func__,real_btr[0]);    
	EST_WRITE(MTL_EST_BTR_HIGH, real_btr[1], false);
	printk(" %s MTL_EST_BTR_HIGH: %x\n", __func__, real_btr[1]);    
	EST_WRITE(MTL_EST_CTR_LOW, cfg->ctr[0], false);
	printk(" %s MTL_EST_CTR_LOW %x\n", __func__, cfg->ctr[0]);    
	EST_WRITE(MTL_EST_CTR_HIGH, cfg->ctr[1], false);
	printk(" %s MTL_EST_CTR_HIGH %x\n", __func__, cfg->ctr[1]);    
	EST_WRITE(MTL_EST_TER, cfg->ter, false);
	printk(" %s MTL_EST_TER %x\n", __func__, cfg->ter);    
	EST_WRITE(MTL_EST_LLR, cfg->gcl_size, false);
	printk(" %s MTL_EST_LLR,: %x\n", __func__, cfg->gcl_size);    
	for (i = 0; i < cfg->gcl_size; i++) {
		u32 reg = (i << MTL_EST_ADDR_OFFSET) & MTL_EST_ADDR;
		EST_WRITE(reg, cfg->gcl[i], true);
		printk(" %s gcl row:%d: %x\n", __func__, i, cfg->gcl[i]);    
	}


#if defined(TX_LOGGING_TRACE) //Log the actual time of BTR to be used for comparision
	{
		read_btr = ((u64)real_btr[1] * 1000000000ULL) + real_btr[0];
		read_ctr = ((u64)((cfg->ctr[1] & 0xff) * 1000000000ULL)) + cfg->ctr[0];

		if(read_btr != 0)
		{
			//[EST]TS,<BTR Value>,<CTR Value>,<IPG of class A>,<IPG of class B>
			trace_printk("[EST[%d]]TS,%llu,%llu,%d,%d\n",switch_cnt,read_btr,read_ctr,PACKET_IPG,PACKET_CDT_IPG);
			for (i = 0; i < cfg->gcl_size; i++) {
				sprintf((pptr+(i*11)),",%010d",(cfg->gcl[i]&0xffffff));
				for(j=0;j<6;j++)
				{
					sprintf((qptr+(i*6*2)+(j*2)),",%d",((cfg->gcl[i]>>(24+j))&0x1));
				}
			}
			trace_printk("[GCL_TI[%d]]TS%s\n",switch_cnt,pptr);
			trace_printk("[GCL_ROW[%d]]TS%s\n",switch_cnt,qptr);
		}
		switch_cnt++;
	}
#endif


	if (readl(ioaddr + MTL_EST_STATUS) & MTL_EST_SWOL)
	{
		printk("GCL 1 is used by sw\n");
	}
	else 
	{
		printk("GCL 0 is used by sw\n");
	}

	/* Enable EST */
	control = MTL_EST_EEST;
	//control = MTL_EST_EEST + 0x400;
	writel(control, ioaddr + MTL_EST_CONTROL);

	/* Store table */
	control |= MTL_EST_SSWL;
	writel(control, ioaddr + MTL_EST_CONTROL);
    
    /* After storing the table it is not required wait and 
     * check the gate switch as when BTR value configured is more, 
     * it may take more time
     */

	netdev_info(ndev, "Enabling EST\n");

#if defined(TX_LOGGING_TRACE)
	kfree(qptr);
	kfree(pptr);
#endif
	return 0;

write_fail:
	netdev_err(ndev, "Failed to write EST configuration\n");
	return -ETIMEDOUT;
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static void dwmac5_log_error(struct net_device *ndev, u32 value, bool corr,
		const char *module_name, const char **errors_str)
{
	unsigned long loc, mask;

	mask = value;
	for_each_set_bit(loc, &mask, 32) {
		netdev_err(ndev, "Found %s error in %s: '%s'\n", corr ?
				"correctable" : "uncorrectable", module_name,
				errors_str[loc]);
	}
}

static const char *dwmac5_mac_errors[32] = {
	"ATPES: Application Transmit Interface Parity Check Error",
	"TPES: TSO Data Path Parity Check Error",
	"RDPES: Read Descriptor Parity Check Error",
	"MPES: MTL Data Path Parity Check Error",
	"MTSPES: MTL TX Status Data Path Parity Check Error",
	"ARPES: Application Receive Interface Data Path Parity Check Error",
	"CWPES: CSR Write Data Path Parity Check Error",
	"ASRPES: AXI Slave Read Data Path Parity Check Error",
	"TTES: TX FSM Timeout Error",
	"RTES: RX FSM Timeout Error",
	"CTES: CSR FSM Timeout Error",
	"ATES: APP FSM Timeout Error",
	"PTES: PTP FSM Timeout Error",
	"T125ES: TX125 FSM Timeout Error",
	"R125ES: RX125 FSM Timeout Error",
	"RVCTES: REV MDC FSM Timeout Error",
	"MSTTES: Master Read/Write Timeout Error",
	"SLVTES: Slave Read/Write Timeout Error",
	"ATITES: Application Timeout on ATI Interface Error",
	"ARITES: Application Timeout on ARI Interface Error",
	"Unknown Error", /* 20 */
	"Unknown Error", /* 21 */
	"Unknown Error", /* 22 */
	"Unknown Error", /* 23 */
	"FSMPES: FSM State Parity Error",
	"Unknown Error", /* 25 */
	"Unknown Error", /* 26 */
	"Unknown Error", /* 27 */
	"Unknown Error", /* 28 */
	"Unknown Error", /* 29 */
	"Unknown Error", /* 30 */
	"Unknown Error", /* 31 */
};

static bool dwmac5_handle_mac_err(struct net_device *ndev,
		struct mac_device_info *hw, bool correctable)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	value = readl(ioaddr + MAC_DPP_FSM_INT_STATUS);
	writel(value, ioaddr + MAC_DPP_FSM_INT_STATUS);

	dwmac5_log_error(ndev, value, correctable, "MAC", dwmac5_mac_errors);
	return !correctable;
}

static const char *dwmac5_mtl_errors[32] = {
	"TXCES: MTL TX Memory Error",
	"TXAMS: MTL TX Memory Address Mismatch Error",
	"TXUES: MTL TX Memory Error",
	"Unknown Error", /* 3 */
	"RXCES: MTL RX Memory Error",
	"RXAMS: MTL RX Memory Address Mismatch Error",
	"RXUES: MTL RX Memory Error",
	"Unknown Error", /* 7 */
	"ECES: MTL EST Memory Error",
	"EAMS: MTL EST Memory Address Mismatch Error",
	"EUES: MTL EST Memory Error",
	"Unknown Error", /* 11 */
	"RPCES: MTL RX Parser Memory Error",
	"RPAMS: MTL RX Parser Memory Address Mismatch Error",
	"RPUES: MTL RX Parser Memory Error",
	"Unknown Error", /* 15 */
	"Unknown Error", /* 16 */
	"Unknown Error", /* 17 */
	"Unknown Error", /* 18 */
	"Unknown Error", /* 19 */
	"Unknown Error", /* 20 */
	"Unknown Error", /* 21 */
	"Unknown Error", /* 22 */
	"Unknown Error", /* 23 */
	"Unknown Error", /* 24 */
	"Unknown Error", /* 25 */
	"Unknown Error", /* 26 */
	"Unknown Error", /* 27 */
	"Unknown Error", /* 28 */
	"Unknown Error", /* 29 */
	"Unknown Error", /* 30 */
	"Unknown Error", /* 31 */
};

static bool dwmac5_handle_mtl_err(struct net_device *ndev,
		struct mac_device_info *hw, bool correctable)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	value = readl(ioaddr + MTL_ECC_INT_STATUS);
	writel(value, ioaddr + MTL_ECC_INT_STATUS);

	dwmac5_log_error(ndev, value, correctable, "MTL", dwmac5_mtl_errors);
	return !correctable;
}

static const char *dwmac5_dma_errors[32] = {
	"TCES: DMA TSO Memory Error",
	"TAMS: DMA TSO Memory Address Mismatch Error",
	"TUES: DMA TSO Memory Error",
	"Unknown Error", /* 3 */
	"Unknown Error", /* 4 */
	"Unknown Error", /* 5 */
	"Unknown Error", /* 6 */
	"Unknown Error", /* 7 */
	"Unknown Error", /* 8 */
	"Unknown Error", /* 9 */
	"Unknown Error", /* 10 */
	"Unknown Error", /* 11 */
	"Unknown Error", /* 12 */
	"Unknown Error", /* 13 */
	"Unknown Error", /* 14 */
	"Unknown Error", /* 15 */
	"Unknown Error", /* 16 */
	"Unknown Error", /* 17 */
	"Unknown Error", /* 18 */
	"Unknown Error", /* 19 */
	"Unknown Error", /* 20 */
	"Unknown Error", /* 21 */
	"Unknown Error", /* 22 */
	"Unknown Error", /* 23 */
	"Unknown Error", /* 24 */
	"Unknown Error", /* 25 */
	"Unknown Error", /* 26 */
	"Unknown Error", /* 27 */
	"Unknown Error", /* 28 */
	"Unknown Error", /* 29 */
	"Unknown Error", /* 30 */
	"Unknown Error", /* 31 */
};

static bool dwmac5_handle_dma_err(struct net_device *ndev,
		struct mac_device_info *hw, bool correctable)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	value = readl(ioaddr + DMA_ECC_INT_STATUS);
	writel(value, ioaddr + DMA_ECC_INT_STATUS);

	dwmac5_log_error(ndev, value, correctable, "DMA", dwmac5_dma_errors);
	return !correctable;
}

static void dwmac5_safety_feat_disable(struct net_device *ndev,
		struct mac_device_info *hw)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	netdev_info(ndev, "Disabling safety features\n");

	/* 1. Disable Safety Features */
	value = readl(ioaddr + MTL_ECC_CONTROL);
	value &= ~TSOEE; /* TSO ECC */
	value &= ~MRXPEE; /* MTL RX Parser ECC */
	value &= ~MESTEE; /* MTL EST ECC */
	value &= ~MRXEE; /* MTL RX FIFO ECC */
	value &= ~MTXEE; /* MTL TX FIFO ECC */
	writel(value, ioaddr + MTL_ECC_CONTROL);

	/* 2. Disable MTL Safety Interrupts */
	value = readl(ioaddr + MTL_ECC_INT_ENABLE);
	value &= ~RPCEIE; /* RX Parser Memory Correctable Error */
	value &= ~ECEIE; /* EST Memory Correctable Error */
	value &= ~RXCEIE; /* RX Memory Correctable Error */
	value &= ~TXCEIE; /* TX Memory Correctable Error */
	writel(value, ioaddr + MTL_ECC_INT_ENABLE);

	/* 3. Disable DMA Safety Interrupts */
	value = readl(ioaddr + DMA_ECC_INT_ENABLE);
	value &= ~TCEIE; /* TSO Memory Correctable Error */
	writel(value, ioaddr + DMA_ECC_INT_ENABLE);

	/* 4. Disable Data Parity Protection */
	value = readl(ioaddr + MTL_DPP_CONTROL);
	value &= ~EDPP;
	value &= ~EPSI;
	writel(value, ioaddr + MTL_DPP_CONTROL);
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

int dwmac5_safety_feat_init(struct net_device *ndev, struct mac_device_info *hw,
		unsigned int asp)
{
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	void __iomem *ioaddr = hw->pcsr;
	u32 value;
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

	if (!asp)
		return -EINVAL;
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE

	netdev_info(ndev, "Enabling safety features: type=0x%x\n", asp);

	/* 1. Enable Safety Features */
	value = readl(ioaddr + MTL_ECC_CONTROL);
	value |= TSOEE; /* TSO ECC */
	value |= MRXPEE; /* MTL RX Parser ECC */
	value |= MESTEE; /* MTL EST ECC */
	value |= MRXEE; /* MTL RX FIFO ECC */
	value |= MTXEE; /* MTL TX FIFO ECC */
	writel(value, ioaddr + MTL_ECC_CONTROL);

	/* 2. Enable MTL Safety Interrupts */
	value = readl(ioaddr + MTL_ECC_INT_ENABLE);
	value |= RPCEIE; /* RX Parser Memory Correctable Error */
	value |= ECEIE; /* EST Memory Correctable Error */
	value |= RXCEIE; /* RX Memory Correctable Error */
	value |= TXCEIE; /* TX Memory Correctable Error */
	writel(value, ioaddr + MTL_ECC_INT_ENABLE);

	/* 3. Enable DMA Safety Interrupts */
	value = readl(ioaddr + DMA_ECC_INT_ENABLE);
	value |= TCEIE; /* TSO Memory Correctable Error */
	writel(value, ioaddr + DMA_ECC_INT_ENABLE);

	/* Only ECC Protection for External Memory feature is selected */
	if (asp <= 0x1)
		return 0;

	/* 5. Enable Parity and Timeout for FSM */
	value = readl(ioaddr + MAC_FSM_CONTROL);
	value |= PRTYEN; /* FSM Parity Feature */
	value |= TMOUTEN; /* FSM Timeout Feature */
	writel(value, ioaddr + MAC_FSM_CONTROL);

	/* 4. Enable Data Parity Protection */
	value = readl(ioaddr + MTL_DPP_CONTROL);
	value |= EDPP;
	writel(value, ioaddr + MTL_DPP_CONTROL);

	/*
	 * All the Automotive Safety features are selected without the "Parity
	 * Port Enable for external interface" feature.
	 */
	if (asp <= 0x2)
		return 0;

	value |= EPSI;
	writel(value, ioaddr + MTL_DPP_CONTROL);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */	
	return 0;
}

bool dwmac5_safety_feat_irq_status(struct net_device *ndev,
		struct mac_device_info *hw, unsigned int asp)
{
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	void __iomem *ioaddr = hw->pcsr;
	u32 mtl, dma;
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
	bool ret = false;

	if (!asp)
		return false;

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	mtl = readl(ioaddr + MTL_SAFETY_INT_STATUS);
	dma = readl(ioaddr + DMA_SAFETY_INT_STATUS);

	if ((mtl & MCSIS) || (dma & MCSIS))
		ret |= dwmac5_handle_mac_err(ndev, hw, false);
	if ((mtl & (MEUIS | MECIS)) || (dma & (MSUIS | MSCIS)))
		ret |= dwmac5_handle_mtl_err(ndev, hw, (mtl & MECIS) || (dma & MSCIS));
	if (dma & (DEUIS | DECIS))
		ret |= dwmac5_handle_dma_err(ndev, hw, dma & DECIS);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */	
	return ret;
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
void dwmac5_safety_feat_set(struct net_device *ndev, struct mac_device_info *hw,
		unsigned int asp, bool enabled, bool inject, u32 where,
		bool correctable)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	if (enabled) {
		dwmac5_safety_feat_init(ndev, hw, asp);
	} else {
		dwmac5_safety_feat_disable(ndev, hw);
		return;
	}

	switch (where) {
	case TC9562MAC_IOCTL_ECC_ERR_TSO:
		value = readl(ioaddr + MTL_DBG_CTL);
		if (inject) {
			value |= EIEE;
			if (!correctable)
				value |= (0x1 << 17) & EIEC;
		} else {
			value &= ~EIEE;
		}

		writel(value, ioaddr + MTL_DBG_CTL);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_EST:
		value = readl(ioaddr + MTL_EST_GCL_CONTROL);
		if (inject) {
			value |= MTL_EST_ESTEIEE;
			if (!correctable)
				value |= (0x1 << 22) & MTL_EST_ESTEIEC;
		} else {
			value &= ~MTL_EST_ESTEIEE;
		}

		writel(value, ioaddr + MTL_EST_GCL_CONTROL);
		break;
#define FSM_CONTROL_UPDT(__b) do { \
	value = readl(ioaddr + MAC_FSM_CONTROL); \
	if (inject) \
		value |= (__b); \
	else \
		value &= ~(__b); \
	writel(value, ioaddr + MAC_FSM_CONTROL); \
} while(0)
	case TC9562MAC_IOCTL_ECC_ERR_FSM_REVMII:
		FSM_CONTROL_UPDT(RVCPEIN);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_FSM_RX125:
		FSM_CONTROL_UPDT(R125PEIN);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_FSM_TX125:
		FSM_CONTROL_UPDT(T125PEIN);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_FSM_PTP:
		FSM_CONTROL_UPDT(PPEIN);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_FSM_APP:
		FSM_CONTROL_UPDT(APEIN);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_FSM_CSR:
		FSM_CONTROL_UPDT(CPEIN);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_FSM_RX:
		FSM_CONTROL_UPDT(RPEIN);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_FSM_TX:
		FSM_CONTROL_UPDT(TPEIN);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_FSM_TREVMII:
		FSM_CONTROL_UPDT(RVCTEIN);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_FSM_TRX125:
		FSM_CONTROL_UPDT(R125TEIN);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_FSM_TTX125:
		FSM_CONTROL_UPDT(T125TEIN);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_FSM_TPTP:
		FSM_CONTROL_UPDT(PTEIN);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_FSM_TAPP:
		FSM_CONTROL_UPDT(ATEIN);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_FSM_TCSR:
		FSM_CONTROL_UPDT(CTEIN);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_FSM_TRX:
		FSM_CONTROL_UPDT(RTEIN);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_FSM_TTX:
		FSM_CONTROL_UPDT(TTEIN);
		break;
#define DPP_CONTROL_UPDT(__b) do { \
	value = readl(ioaddr + MTL_DPP_CONTROL); \
	if (inject) \
		value |= (__b); \
	else \
		value &= ~(__b); \
	writel(value, ioaddr + MTL_DPP_CONTROL); \
} while(0)
	case TC9562MAC_IOCTL_ECC_ERR_DPP_CSR:
		DPP_CONTROL_UPDT(IPECW);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_DPP_AXI:
		DPP_CONTROL_UPDT(IPEASW);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_DPP_RX:
		DPP_CONTROL_UPDT(IPERD);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_DPP_TX:
		DPP_CONTROL_UPDT(IPETD);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_DPP_DMATSO:
		DPP_CONTROL_UPDT(IPETSO);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_DPP_DMADTX:
		DPP_CONTROL_UPDT(IPEDDC);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_DPP_MTLRX:
		DPP_CONTROL_UPDT(IPEMRF);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_DPP_MTLTX:
		DPP_CONTROL_UPDT(IPEMTS);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_DPP_MTL:
		DPP_CONTROL_UPDT(IPEMC);
		break;
	case TC9562MAC_IOCTL_ECC_ERR_DPP_INTERFACE:
		DPP_CONTROL_UPDT(IPEID);
		break;
	default:
		return;
	}
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,15))

static int dwmac5_rxp_disable(void __iomem *ioaddr)
{
	u32 val;
	int ret;

	val = readl(ioaddr + MTL_OPERATION_MODE);
	val &= ~MTL_FRPE;
	writel(val, ioaddr + MTL_OPERATION_MODE);

	ret = readl_poll_timeout(ioaddr + MTL_RXP_CONTROL_STATUS, val,
			val & RXPI, 1, 10000);
	if (ret)
		return ret;
	return 0;
}

static void dwmac5_rxp_enable(void __iomem *ioaddr)
{
	u32 val;

	val = readl(ioaddr + MTL_OPERATION_MODE);
	val |= MTL_FRPE;
	writel(val, ioaddr + MTL_OPERATION_MODE);
}

static int dwmac5_rxp_update_single_entry(void __iomem *ioaddr,
					  struct tc9562mac_tc_entry *entry,
					  int pos)
{
	int ret, i;

	for (i = 0; i < (sizeof(entry->val) / sizeof(u32)); i++) {
		int real_pos = pos * (sizeof(entry->val) / sizeof(u32)) + i;
		u32 val;

		/* Wait for ready */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
		ret = readl_poll_timeout(ioaddr + MTL_RXP_IACC_CTRL_STATUS,
				val, !(val & STARTBUSY), 1, 10000);
		if (ret)
			return ret;
#else
        {
            int limit = 10000;
            while (limit--) {
                if (!(readl(ioaddr + MTL_RXP_IACC_CTRL_STATUS) & STARTBUSY))
                    break;
                udelay(1);
            }

            if (limit < 0)
                return -EBUSY;
        }
#endif

		/* Write data */
		val = *((u32 *)&entry->val + i);
		//printk("FRP data: %x", val);
		writel(val, ioaddr + MTL_RXP_IACC_DATA);
		/* Write pos */
		val = real_pos & ADDR;
		//printk("Addr FRP: %x \n", val);
		writel(val, ioaddr + MTL_RXP_IACC_CTRL_STATUS);

		/* Write OP */
		val |= WRRDN;
		writel(val, ioaddr + MTL_RXP_IACC_CTRL_STATUS);

		/* Start Write */
		val |= STARTBUSY;
		writel(val, ioaddr + MTL_RXP_IACC_CTRL_STATUS);

			
		/* Wait for ready */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
		ret = readl_poll_timeout(ioaddr + MTL_RXP_IACC_CTRL_STATUS,
				val, !(val & STARTBUSY), 1, 10000);
		if (ret)
			return ret;
#else
        {
            int limit = 10000;
            while (limit--) {
                if (!(readl(ioaddr + MTL_RXP_IACC_CTRL_STATUS) & STARTBUSY))
                    break;
                udelay(1);
            }

            if (limit < 0)
                return -EBUSY;
        }
#endif
	}

	return 0;
}

static struct tc9562mac_tc_entry *
dwmac5_rxp_get_next_entry(struct tc9562mac_tc_entry *entries, unsigned int count,
			  u32 curr_prio)
{
	struct tc9562mac_tc_entry *entry;
	u32 min_prio = ~0x0;
	int i, min_prio_idx;
	bool found = false;

	for (i = count - 1; i >= 0; i--) {
		entry = &entries[i];

		/* Do not update unused entries */
		if (!entry->in_use)
			continue;
		/* Do not update already updated entries (i.e. fragments) */
		if (entry->in_hw)
			continue;
		/* Let last entry be updated last */
		if (entry->is_last)
			continue;
		/* Do not return fragments */
		if (entry->is_frag)
			continue;
		/* Check if we already checked this prio */
		if (entry->prio < curr_prio)
			continue;
		/* Check if this is the minimum prio */
		if (entry->prio < min_prio) {
			min_prio = entry->prio;
			min_prio_idx = i;
			found = true;
		}
	}

	if (found)
		return &entries[min_prio_idx];
	return NULL;
}

int dwmac5_rxp_config(void *p, void *ent,
		      unsigned int count)
{

	struct tc9562mac_priv *priv = (struct tc9562mac_priv *)p;
	struct tc9562mac_tc_entry *entries = (struct tc9562mac_tc_entry *)ent;
	struct tc9562mac_tc_entry *entry, *frag;
	int i, ret, nve = 0;
	u32 curr_prio = 0;
	u32 old_val, val;
	void __iomem *ioaddr = priv->hw->pcsr;

	/* Force disable RX */
	old_val = readl(ioaddr + GMAC_CONFIG);
	val = old_val & ~GMAC_CONFIG_RE;
	writel(val, ioaddr + GMAC_CONFIG);

	/* Disable RX Parser */
	ret = dwmac5_rxp_disable(ioaddr);
	if (ret)
		goto re_enable;

	/* Set all entries as NOT in HW */
	for (i = 0; i < count; i++) {
		entry = &entries[i];
		entry->in_hw = false;
	}

	/* Update entries by reverse order */
	while (1) {
		entry = dwmac5_rxp_get_next_entry(entries, count, curr_prio);
		if (!entry)
			break;

		curr_prio = entry->prio;
		frag = entry->frag_ptr;

		/* Set special fragment requirements */
		if (frag) {
			entry->val.af = 0;
			entry->val.rf = 0;
			entry->val.nc = 1;
			entry->val.ok_index = nve + 2;
		}

		if((!entry->val.rf) && (!entry->val.af)) //To handle continue
		{
			entry->val.nc = 1;
			entry->val.ok_index = nve + 1;//continue to next
		}	
	
		ret = dwmac5_rxp_update_single_entry(ioaddr, entry, nve);
		if (ret)
			goto re_enable;

		entry->table_pos = nve++;
		entry->in_hw = true;

		if (frag && !frag->in_hw) {
			ret = dwmac5_rxp_update_single_entry(ioaddr, frag, nve);
			if (ret)
				goto re_enable;
			frag->table_pos = nve++;
			frag->in_hw = true;
		}
	}

	if (!nve)
		goto re_enable;

	/* Update all pass entry */
	for (i = 0; i < count; i++) {
		entry = &entries[i];
		if (!entry->is_last)
			continue;

		ret = dwmac5_rxp_update_single_entry(ioaddr, entry, nve);
		if (ret)
			goto re_enable;

		entry->table_pos = nve++;
	}

	/* Assume n. of parsable entries == n. of valid entries */
	val = (nve << 16) & NPE;
	val |= nve & NVE;
	writel(val, ioaddr + MTL_RXP_CONTROL_STATUS);

	/* Enable RX Parser */
	dwmac5_rxp_enable(ioaddr);

re_enable:
	/* Re-enable RX */
	writel(old_val, ioaddr + GMAC_CONFIG);
	return ret;
}
#endif
static int dwmac5_rx_parser_write_entry(struct mac_device_info *hw,
		struct tc9562mac_rx_parser_entry *entry, int entry_pos)
{
	void __iomem *ioaddr = hw->pcsr;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
	int ret;
#endif
	int i;

	for (i = 0; i < (sizeof(*entry) / sizeof(u32)); i++) {
		int real_pos = entry_pos * (sizeof(*entry) / sizeof(u32)) + i;
		u32 value = *((u32 *)entry + i);

		writel(value, ioaddr + MTL_RXP_IACC_DATA);

		value = real_pos & ADDR;
		writel(value, ioaddr + MTL_RXP_IACC_CTRL_STATUS);
	
		/* Wait for ready */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
		ret = readl_poll_timeout(ioaddr + MTL_RXP_IACC_CTRL_STATUS,
				value, !(value & STARTBUSY), 1, 10000);
		if (ret)
			return ret;
#else
        {
            int limit = 10000;
            while (limit--) {
                if (!(readl(ioaddr + MTL_RXP_IACC_CTRL_STATUS) & STARTBUSY))
                    break;
                udelay(1);
            }

            if (limit < 0)
                return -EBUSY;
        }
#endif
		/* Write op */
		value |= WRRDN;
		writel(value, ioaddr + MTL_RXP_IACC_CTRL_STATUS);

		/* Start write */
		value |= STARTBUSY;
		writel(value, ioaddr + MTL_RXP_IACC_CTRL_STATUS);

		/* Wait for done */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
		ret = readl_poll_timeout(ioaddr + MTL_RXP_IACC_CTRL_STATUS,
				value, !(value & STARTBUSY), 1, 10000);
		if (ret)
			return ret;

#else
        {
            int limit = 10000;
            while (limit--) {
                if (!(readl(ioaddr + MTL_RXP_IACC_CTRL_STATUS) & STARTBUSY))
                    break;
                udelay(1);
            }

            if (limit < 0)
                return -EBUSY;
        }
#endif			
	}

	return 0;
}

static void dwmac5_rx_parser_enable(struct mac_device_info *hw)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	value = readl(ioaddr + MTL_OPERATION_MODE);
	value |= MTL_FRPE;
	writel(value, ioaddr + MTL_OPERATION_MODE);
}

static int dwmac5_rx_parser_disable(struct mac_device_info *hw)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
	int ret;
#endif

	value = readl(ioaddr + MTL_OPERATION_MODE);
	value &= ~MTL_FRPE;
	writel(value, ioaddr + MTL_OPERATION_MODE);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
	ret = readl_poll_timeout(ioaddr + MTL_RXP_CONTROL_STATUS, value,
			value & RXPI, 1, 10000);
	if (ret)
		return ret;

#else
    {
        int limit = 10000;
        while (limit--) {
            if ((readl(ioaddr + MTL_RXP_CONTROL_STATUS) & RXPI))
                break;
            udelay(1);
        }

        if (limit < 0)
            return -EBUSY;
    }
#endif


	return 0;
}

static int dwmac5_rx_parser_config(struct mac_device_info *hw,
		struct tc9562mac_rx_parser_cfg *cfg)
{
	void __iomem *ioaddr = hw->pcsr;
	int i, ret;
	u32 value;

	if (cfg->npe <= 0 || cfg->nve <= 0)
		return -EINVAL;

	value = (cfg->npe - 1) << 16;
	value |= (cfg->nve - 1) << 0;
	writel(value, ioaddr + MTL_RXP_CONTROL_STATUS);

	for (i = 0; i < cfg->nve; i++) {
		ret = dwmac5_rx_parser_write_entry(hw, &cfg->entries[i], i);
		if (ret)
			return ret;
	}

	return 0;
}

int dwmac5_rx_parser_init(struct net_device *ndev, struct mac_device_info *hw,
		unsigned int spram, unsigned int frpsel, unsigned int frpes,
		struct tc9562mac_rx_parser_cfg *cfg)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value, old_value = readl(ioaddr + GMAC_CONFIG);
	int ret, max_entries = frpes;

	if (!spram || !cfg || !frpsel)
		return -EINVAL;
	if (cfg->nve > max_entries || cfg->npe > max_entries) {
		netdev_err(ndev, "Invalid RX Parser configuration supplied\n");
		return -EINVAL;
	}

	/* Force disable RX */
	value = old_value & ~GMAC_CONFIG_RE;
	writel(value, ioaddr + GMAC_CONFIG);

	/* Disable RX Parser */
	ret = dwmac5_rx_parser_disable(hw);
	if (ret) {
		netdev_err(ndev, "Failed to disable RX Parser\n");
		return ret;
	}

	/* Nothing to do if we don't want to enable RX Parser */
	if (cfg->nve <= 0 || cfg->npe <= 0) {
		/* Restore RX to previous state */
		writel(old_value, ioaddr + GMAC_CONFIG);
		return -EINVAL;
	}

	/* Store table */
	ret = dwmac5_rx_parser_config(hw, cfg);
	if (ret) {
		netdev_err(ndev, "Failed to configure RX Parser\n");
		return ret;
	}

	/* Enable RX Parser */
	dwmac5_rx_parser_enable(hw);

	/* Restore RX to previous state */
	writel(old_value, ioaddr + GMAC_CONFIG);

	netdev_info(ndev, "Enabling RX Parser\n");
	return 0;
}
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
int dwmac5_pps_init(struct net_device *ndev, struct mac_device_info *hw,
		int index, struct tc9562mac_pps_cfg *cfg)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	/* Sanity Checks */
	if ((cfg->ctrl_cmd & ~PPSCMDx(0)) != 0)
		return -EINVAL;
	if ((cfg->trgtmodsel & ~(TRGTMODSELx(0) >> 5)) != 0)
		return -EINVAL;
	if ((cfg->target_time[0] & ~(TTSL0)) != 0)
		return -EINVAL;
	if (!cfg->enable && index) /* Fixed PPS only on output 0 */
		return -EINVAL;

	value = readl(ioaddr + MAC_PPS_CONTROL);

	/* Cleanup old values for this index */
	value &= ~GENMASK(((index + 1) * 8) - 1, index * 8);

	/* Set new values */
	value |= (cfg->trgtmodsel << ((index * 8) + 5)) & TRGTMODSELx(index);
	if (index == 0)
		value |= cfg->enable ? PPSEN0 : 0;

	writel(value, ioaddr + MAC_PPS_CONTROL);

	/* Flexible PPS Settings */
	writel(cfg->target_time[1], ioaddr + MAC_PPSx_TARGET_TIME_SEC(index));
	if (readl(ioaddr + MAC_PPSx_TARGET_TIME_NSEC(index)) & TRGTBUSY0)
		return -EBUSY;
	writel(cfg->target_time[0], ioaddr + MAC_PPSx_TARGET_TIME_NSEC(index));
	writel(cfg->interval, ioaddr + MAC_PPSx_INTERVAL(index));
	writel(cfg->width, ioaddr + MAC_PPSx_WIDTH(index));

	/* Write CMD */
	value |= (cfg->ctrl_cmd << (index * 8)) & PPSCMDx(index);
	writel(value, ioaddr + MAC_PPS_CONTROL);

	netdev_info(ndev, "Enabling %s PPS for output %d\n",
			cfg->enable ? "Flexible" : "Fixed", index);
	return 0;
}

int dwmac5_mcgr_init(struct net_device *ndev, struct mac_device_info *hw,
		struct tc9562mac_mcgr_cfg *cfg, int count)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;
	int i;

	value = readl(ioaddr + MAC_PPS_CONTROL);

	/* First, disable all */
	writel(0x0, ioaddr + MAC_PPS_CONTROL);
	writel(0x0, ioaddr + MAC_GPIO_CONTROL);

	/* Then, set new values */
	for (i = 0; i < count; i++) {
		/* Sanity Checks */
		if ((cfg[i].ctrl & ~PPSCMDx(0)) != 0)
			return -EINVAL;

		/* Cleanup old values for this index */
		value &= ~GENMASK(((i + 1) * 8) - 1, i * 8);

		/* Set new values */
		value |= (cfg[i].ctrl << (i * 8)) & PPSCMDx(i);
		value |= cfg[i].enable ? MCGRENx(i) : 0;

		hw->prev_ts[i][0] = 0;
		hw->prev_ts[i][1] = 0;
		hw->prev_ts_count[i] = 0;
	}

	/* Finally, enable */
	writel(0xf, ioaddr + MAC_GPIO_CONTROL);
	writel(value, ioaddr + MAC_PPS_CONTROL);

	/* Initial check */
	dwmac5_mcgr_intr(hw, cfg, count);
	return 0;
}

void dwmac5_mcgr_intr(struct mac_device_info *hw,
		struct tc9562mac_mcgr_cfg *cfg, int count)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 gpio, ts, next_ts;
	bool found;
	int i, j;

	gpio = readl(ioaddr + MAC_GPIO_STATUS);
	if (!gpio)
		return;

	for (i = 0; i < count; i++) {
		if (!cfg[i].enable)
			continue;

		found = false;
		switch (cfg[i].ctrl) {
		case TC9562MAC_IOCTL_MCGR_CMD_CAPTURE_RISE:
		case TC9562MAC_IOCTL_MCGR_CMD_CAPTURE_FALL:
		case TC9562MAC_IOCTL_MCGR_CMD_CAPTURE_BOTH:
			found = true;
			break;
		default:
			break;
		}

		if (found && (gpio & BIT(i))) {
			ts = readl(ioaddr + MAC_PPSx_TARGET_TIME_SEC(i));
			writel(BIT(i + 16), ioaddr + MAC_GPIO_STATUS);
			writel(0x0, ioaddr + MAC_GPIO_STATUS);
			hw->prev_ts[i][0] = hw->prev_ts[i][1];
			hw->prev_ts[i][1] = ts;
			hw->prev_ts_count[i]++;
			break;
		}
	}

	for (j = 0; j < count; j++) {
		u32 in_idx = cfg[j].debug_route;

		if (!cfg[j].enable)
			continue;

		found = false;
		switch (cfg[j].ctrl) {
		case TC9562MAC_IOCTL_MCGR_CMD_TOGGLE:
		case TC9562MAC_IOCTL_MCGR_CMD_PULSE_LOW:
		case TC9562MAC_IOCTL_MCGR_CMD_PULSE_HIGH:
			found = true;
			break;
		default:
			break;
		}

		if (found && (gpio & BIT(j)) && (hw->prev_ts_count[in_idx] > 1)) {
			next_ts = hw->prev_ts[in_idx][1] - hw->prev_ts[in_idx][0];
			next_ts += readl(ioaddr + MAC_PRESN_TIME_NS);
			hw->prev_ts_count[in_idx]--;

			writel(next_ts, ioaddr + MAC_PPSx_TARGET_TIME_SEC(j));
			writel(BIT(j + 16), ioaddr + MAC_GPIO_STATUS);
			writel(0x0, ioaddr + MAC_GPIO_STATUS);
		}
	}
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
