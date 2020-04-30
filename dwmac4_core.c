/*
 * TC9562 ethernet driver.
 *
 * dwmac4_core.c
 *
 * Copyright (C) 2015 STMicroelectronics Ltd
 * Copyright (C) 2020 Toshiba Electronic Devices & Storage Corporation
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
 *  26 Feb 2020 : 1. Added 4.19 kernel support.
                  2. Added TC - FRP feature support. 
 *  VERSION     : 01-01
 *  30 Sep 2019 : Base lined
 *  VERSION     : 01-00
 */

#include <linux/crc32.h>
#include <linux/slab.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include "tc9562mac.h"
#include "tc9562mac_pcs.h"
#include "dwmac4.h"
#include "dwmac5.h"
#include "tc9562mac_ptp.h"

static void dwmac4_core_init(struct net_device *ndev, struct mac_device_info *hw, int mtu)
{
    struct tc9562mac_priv *priv = netdev_priv(ndev);
	void __iomem *ioaddr = hw->pcsr;
	u32 value = readl(ioaddr + GMAC_CONFIG);

	DBGPR_FUNC("-->dwmac4_core_init\n");

	value |= GMAC_CORE_INIT;

	if (mtu > 1500)
		value |= GMAC_CONFIG_2K;
	if (mtu > 2000)
		value |= GMAC_CONFIG_JE;

	if (hw->ps) {
		value |= GMAC_CONFIG_TE;

		value &= hw->link.speed_mask;
		switch (hw->ps) {
		case SPEED_1000:
			value |= hw->link.speed1000;
			break;
		case SPEED_100:
			value |= hw->link.speed100;
			break;
		case SPEED_10:
			value |= hw->link.speed10;
			break;
		}
	}

	/* phy changes */
	value &=  ~hw->link.speed_mask;//masking 14,15 bit of Mac_configuration register
	value |= hw->link.duplex;
    
    if(PHY_INTERFACE_MODE_RMII == priv->plat->interface) {
    	value |= hw->link.speed100;//value for 14, 15 should be zero for 100 mbps speed selection
	}
	else {
	   /* Let default speed for RGMII, SGMII be 1000Mbps */
	   value |= hw->link.speed1000;//value for 14, 15 should be zero for 1000 mbps speed selection
	}
	
	writel(value, ioaddr + GMAC_CONFIG);
	NDBGPR_L1("GMAC_CONFIG = %x\n", readl(ioaddr + GMAC_CONFIG));

	/* phy changes */
	value = readl(ioaddr + NEMACCTL_OFFSET);
	value &= 0xFFFFFFFC;//masking bit 0:1
	
	if(PHY_INTERFACE_MODE_RMII == priv->plat->interface) {
	    value |= 0x2; // Set speed to 100mbps
	}
	else {
	    value |= 0x0; //Set speed to 1000mbps
	}
	writel(value, ioaddr + NEMACCTL_OFFSET);//EMAC DIV (NEMACCTL)
	NDBGPR_L1("NEMACCTL_OFFSET = %x\n", readl(ioaddr + NEMACCTL_OFFSET));

	/* Mask GMAC interrupts */
	value = GMAC_INT_DEFAULT_MASK;
	if (hw->pmt)
		value |= GMAC_INT_PMT_EN;
	if (hw->pcs)
		value |= GMAC_PCS_IRQ_DEFAULT;

	writel(value, ioaddr + GMAC_INT_EN);
	NDBGPR_L1("GMAC_INT_EN = %x\n", readl(ioaddr + GMAC_INT_EN));
	DBGPR_FUNC("<--dwmac4_core_init\n");
}

static void dwmac4_rx_queue_enable(struct mac_device_info *hw,
				   u8 mode, u32 queue)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value = readl(ioaddr + GMAC_RXQ_CTRL0);

	DBGPR_FUNC("-->dwmac4_rx_queue_enable\n");
	
	value &= GMAC_RX_QUEUE_CLEAR(queue);
	if (mode == MTL_QUEUE_AVB)
		value |= GMAC_RX_AV_QUEUE_ENABLE(queue);
	else if (mode == MTL_QUEUE_DCB)
		value |= GMAC_RX_DCB_QUEUE_ENABLE(queue);

	writel(value, ioaddr + GMAC_RXQ_CTRL0);

	DBGPR_FUNC("<--dwmac4_rx_queue_enable\n");
	
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static void dwmac4_rx_queue_disable(struct mac_device_info *hw, u32 queue)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value = readl(ioaddr + GMAC_RXQ_CTRL0);

	DBGPR_FUNC("-->dwmac4_rx_queue_disable\n");

	value &= GMAC_RX_QUEUE_CLEAR(queue);
	writel(value, ioaddr + GMAC_RXQ_CTRL0);

	DBGPR_FUNC("<--dwmac4_rx_queue_disable\n");
	
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

static void dwmac4_rx_queue_priority(struct mac_device_info *hw,
				     u32 prio, u32 queue)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 base_register;
	u32 value;

	DBGPR_FUNC("-->dwmac4_rx_queue_priority\n");
	
	base_register = (queue < 4) ? GMAC_RXQ_CTRL2 : GMAC_RXQ_CTRL3;

	value = readl(ioaddr + base_register);

	value &= ~GMAC_RXQCTRL_PSRQX_MASK(queue);
	value |= (prio << GMAC_RXQCTRL_PSRQX_SHIFT(queue)) &
						GMAC_RXQCTRL_PSRQX_MASK(queue);
	writel(value, ioaddr + base_register);

	DBGPR_FUNC("<--dwmac4_rx_queue_priority\n");
		
}

static void dwmac4_tx_queue_priority(struct mac_device_info *hw,
				     u32 prio, u32 queue)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 base_register;
	u32 value;

	DBGPR_FUNC("-->dwmac4_tx_queue_priority\n");

	base_register = (queue < 4) ? GMAC_TXQ_PRTY_MAP0 : GMAC_TXQ_PRTY_MAP1;

	value = readl(ioaddr + base_register);

	value &= ~GMAC_TXQCTRL_PSTQX_MASK(queue);
	value |= (prio << GMAC_TXQCTRL_PSTQX_SHIFT(queue)) &
						GMAC_TXQCTRL_PSTQX_MASK(queue);

	writel(value, ioaddr + base_register);

	DBGPR_FUNC("<--dwmac4_tx_queue_priority\n");
	
}

static void dwmac4_rx_queue_routing(struct mac_device_info *hw,
				    u8 packet, u32 queue)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	static const struct tc9562mac_rx_routing route_possibilities[] = {
		{ GMAC_RXQCTRL_AVCPQ_MASK, GMAC_RXQCTRL_AVCPQ_SHIFT },
		{ GMAC_RXQCTRL_PTPQ_MASK, GMAC_RXQCTRL_PTPQ_SHIFT },
		{ GMAC_RXQCTRL_DCBCPQ_MASK, GMAC_RXQCTRL_DCBCPQ_SHIFT },
		{ GMAC_RXQCTRL_UPQ_MASK, GMAC_RXQCTRL_UPQ_SHIFT },
		{ GMAC_RXQCTRL_MCBCQ_MASK, GMAC_RXQCTRL_MCBCQ_SHIFT },
		{ GMAC_RXQCTRL_FPRQ_MASK, GMAC_RXQCTRL_FPRQ_SHIFT },		
	};

	DBGPR_FUNC("-->dwmac4_rx_queue_routing\n");
	
	value = readl(ioaddr + GMAC_RXQ_CTRL1);

	/* routing configuration */
	value &= ~route_possibilities[packet - 1].reg_mask;
	value |= (queue << route_possibilities[packet-1].reg_shift) &
		 route_possibilities[packet - 1].reg_mask;

	/* some packets require extra ops */
	if (packet == PACKET_AVCPQ) {
		value &= ~GMAC_RXQCTRL_TACPQE;
		/* value |= 0x1 << GMAC_RXQCTRL_TACPQE_SHIFT; */
	} else if (packet == PACKET_MCBCQ) {
		value &= ~GMAC_RXQCTRL_MCBCQEN;
		value |= 0x1 << GMAC_RXQCTRL_MCBCQEN_SHIFT;
	}

	writel(value, ioaddr + GMAC_RXQ_CTRL1);

	DBGPR_FUNC("<--dwmac4_rx_queue_routing\n");
	
}

static void dwmac4_prog_mtl_rx_algorithms(struct mac_device_info *hw,
					  u32 rx_alg)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value = readl(ioaddr + MTL_OPERATION_MODE);

	DBGPR_FUNC("-->dwmac4_prog_mtl_rx_algorithms\n");

	value &= ~MTL_OPERATION_RAA;
	switch (rx_alg) {
	case MTL_RX_ALGORITHM_SP:
		value |= MTL_OPERATION_RAA_SP;
		break;
	case MTL_RX_ALGORITHM_WSP:
		value |= MTL_OPERATION_RAA_WSP;
		break;
	default:
		break;
	}

	writel(value, ioaddr + MTL_OPERATION_MODE);
	NDBGPR_L1("RX MTL_OPERATION_MODE = %x\n", readl(ioaddr + MTL_OPERATION_MODE));
	DBGPR_FUNC("<--dwmac4_prog_mtl_rx_algorithms\n");
}

static void dwmac4_prog_mtl_tx_algorithms(struct mac_device_info *hw,
					  u32 tx_alg)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value = readl(ioaddr + MTL_OPERATION_MODE);

	DBGPR_FUNC("-->dwmac4_prog_mtl_tx_algorithms\n");
	
	value &= ~MTL_OPERATION_SCHALG_MASK;
	switch (tx_alg) {
	case MTL_TX_ALGORITHM_WRR:
		value |= MTL_OPERATION_SCHALG_WRR;
		break;
	case MTL_TX_ALGORITHM_WFQ:
		value |= MTL_OPERATION_SCHALG_WFQ;
		break;
	case MTL_TX_ALGORITHM_DWRR:
		value |= MTL_OPERATION_SCHALG_DWRR;
		break;
	case MTL_TX_ALGORITHM_SP:
		value |= MTL_OPERATION_SCHALG_SP;
		break;
	default:
		break;
	}

	writel(value, ioaddr + MTL_OPERATION_MODE);
	NDBGPR_L1("TX MTL_OPERATION_MODE = %x\n", readl(ioaddr + MTL_OPERATION_MODE));
	DBGPR_FUNC("<--dwmac4_prog_mtl_tx_algorithms\n");
}

static void dwmac4_set_mtl_tx_queue_weight(struct mac_device_info *hw,
					   u32 weight, u32 queue)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value = readl(ioaddr + MTL_TXQX_WEIGHT_BASE_ADDR(queue));

	DBGPR_FUNC("-->dwmac4_set_mtl_tx_queue_weight\n");

	value &= ~MTL_TXQ_WEIGHT_ISCQW_MASK;
	value |= weight & MTL_TXQ_WEIGHT_ISCQW_MASK;
	writel(value, ioaddr + MTL_TXQX_WEIGHT_BASE_ADDR(queue));
	NDBGPR_L1("MTL_TXQX_WEIGHT_BASE_ADDR[%d] = %x\n",queue, readl(ioaddr + MTL_TXQX_WEIGHT_BASE_ADDR(queue)));
	DBGPR_FUNC("<--dwmac4_set_mtl_tx_queue_weight\n");
}

static void dwmac4_map_mtl_dma(struct mac_device_info *hw, u32 queue, u32 chan)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value,frp_value;
	DBGPR_FUNC("-->dwmac4_map_mtl_dma\n");

	frp_value = readl(ioaddr + MTL_OPERATION_MODE);

	if (queue < 4)
		value = readl(ioaddr + MTL_RXQ_DMA_MAP0);
	else
		value = readl(ioaddr + MTL_RXQ_DMA_MAP1);

	if (queue == 0) { 
		/* Static Mapping*/ 
		value &= ~MTL_RXQ_DMA_QXDDMACH(4);
		value &= ~MTL_RXQ_DMA_QXMDMACH_MASK(queue);
		value |= MTL_RXQ_DMA_QXMDMACH(chan, queue);
	}else if (queue == 1) {  
		/* Static Mapping*/ 
		value &= ~MTL_RXQ_DMA_QXDDMACH(12);
		value &= ~MTL_RXQ_DMA_QXMDMACH_MASK(queue);
		value |= MTL_RXQ_DMA_QXMDMACH(chan, queue);
	}
	else if (queue == 2){
		if(frp_value & MTL_FRPE){
			/*Dynamic Mapping */
			value |= MTL_RXQ_DMA_QXDDMACH(20);	
		}
		else{ 
			/*Static Mapping */
			value &= ~MTL_RXQ_DMA_QXDDMACH(20);
			value &= ~MTL_RXQ_DMA_QXMDMACH_MASK(queue);
			value |= MTL_RXQ_DMA_QXMDMACH(chan, queue);
		}			
	} else if (queue == 3){
		if(frp_value & MTL_FRPE){
			/* Dynamic Mapping */
			value |= MTL_RXQ_DMA_QXDDMACH(28);
		}
		else{ 
			/* Static Mapping */
			value &= ~MTL_RXQ_DMA_QXDDMACH(28);
			value &= ~MTL_RXQ_DMA_QXMDMACH_MASK(queue);
			value |= MTL_RXQ_DMA_QXMDMACH(chan, queue);
		}
#ifdef TC9562_TEST_RXCH1_FRP_DISABLED
			value &= ~MTL_RXQ_DMA_QXDDMACH(28);
			value &= ~MTL_RXQ_DMA_QXMDMACH_MASK(queue);
			value |= MTL_RXQ_DMA_QXDDMACH(28);
#endif
	}

	if (queue < 4)
		writel(value, ioaddr + MTL_RXQ_DMA_MAP0);
	else
		writel(value, ioaddr + MTL_RXQ_DMA_MAP1);
	NDBGPR_L1("MTL_RXQ_DMA_MAP0 = %x\n",readl(ioaddr + MTL_RXQ_DMA_MAP0));
	DBGPR_FUNC("<--dwmac4_map_mtl_dma\n");
}

static void dwmac4_config_cbs(struct mac_device_info *hw,
			      int send_slope, u32 idle_slope,
			      u32 high_credit, int low_credit, u32 queue)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	DBGPR_FUNC("-->dwmac4_config_cbs\n");
	//printk("dwmac4_config_cbs : idle_slope : %x , send_slope : %x, high_credit : %x, low_credit : %x\n", idle_slope, send_slope, high_credit, low_credit);
	/* enable AV algorithm */
	value = readl(ioaddr + MTL_ETSX_CTRL_BASE_ADDR(queue));
	value |= MTL_ETS_CTRL_AVALG;
	//value |= MTL_ETS_CTRL_CC; // Disable CC
	writel(value, ioaddr + MTL_ETSX_CTRL_BASE_ADDR(queue));

	/* configure send slope */
	value = readl(ioaddr + MTL_SEND_SLP_CREDX_BASE_ADDR(queue));
	value &= ~MTL_SEND_SLP_CRED_SSC_MASK;
	value |= send_slope & MTL_SEND_SLP_CRED_SSC_MASK;
	writel(value, ioaddr + MTL_SEND_SLP_CREDX_BASE_ADDR(queue));

	/* configure idle slope (same register as tx weight) */
	dwmac4_set_mtl_tx_queue_weight(hw, idle_slope, queue);

	/* configure high credit */
	value = readl(ioaddr + MTL_HIGH_CREDX_BASE_ADDR(queue));
	value &= ~MTL_HIGH_CRED_HC_MASK;
	value |= high_credit & MTL_HIGH_CRED_HC_MASK;
	writel(value, ioaddr + MTL_HIGH_CREDX_BASE_ADDR(queue));

	/* configure low credit */
	value = readl(ioaddr + MTL_LOW_CREDX_BASE_ADDR(queue));
	value &= ~MTL_HIGH_CRED_LC_MASK;
	value |= low_credit & MTL_HIGH_CRED_LC_MASK;
	writel(value, ioaddr + MTL_LOW_CREDX_BASE_ADDR(queue));

	DBGPR_FUNC("<--dwmac4_config_cbs\n");
	
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static void dwmac4_dump_regs(struct mac_device_info *hw, u32 *reg_space)
{
	void __iomem *ioaddr = hw->pcsr;
	int i;

	DBGPR_FUNC("-->dwmac4_dump_regs\n");

	for (i = 0; i < GMAC_REG_NUM; i++)
		reg_space[i] = readl(ioaddr + i * 4);

	DBGPR_FUNC("<--dwmac4_dump_regs\n");
	
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

static int dwmac4_rx_ipc_enable(struct mac_device_info *hw)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value = readl(ioaddr + GMAC_CONFIG);

	DBGPR_FUNC("-->dwmac4_rx_ipc_enable\n");

	if (hw->rx_csum)
		value |= GMAC_CONFIG_IPC;
	else
		value &= ~GMAC_CONFIG_IPC;

	writel(value, ioaddr + GMAC_CONFIG);

	value = readl(ioaddr + GMAC_CONFIG);

	DBGPR_FUNC("<--dwmac4_rx_ipc_enable\n");
	return !!(value & GMAC_CONFIG_IPC);
}

static void dwmac4_pmt(struct mac_device_info *hw, unsigned long mode)
{
	void __iomem *ioaddr = hw->pcsr;
	unsigned int pmt = 0;
	u32 config;

	DBGPR_FUNC("-->dwmac4_pmt\n");

	if (mode & WAKE_MAGIC) {
		pr_debug("GMAC: WOL Magic frame\n");
		pmt |= power_down | magic_pkt_en;
	}
	if (mode & WAKE_UCAST) {
		pr_debug("GMAC: WOL on global unicast\n");
		pmt |= power_down | global_unicast | wake_up_frame_en;
	}

	if (pmt) {
		/* The receiver must be enabled for WOL before powering down */
		config = readl(ioaddr + GMAC_CONFIG);
		config |= GMAC_CONFIG_RE;
		writel(config, ioaddr + GMAC_CONFIG);
	}
	writel(pmt, ioaddr + GMAC_PMT);

	DBGPR_FUNC("<--dwmac4_pmt\n");
	
}

static void dwmac4_set_umac_addr(struct mac_device_info *hw,
				 unsigned char *addr, unsigned int reg_n, unsigned int dcs)
{
	void __iomem *ioaddr = hw->pcsr;

	DBGPR_FUNC("-->dwmac4_set_umac_addr\n");

	tc9562mac_dwmac4_set_mac_addr(ioaddr, addr, GMAC_ADDR_HIGH(reg_n),
				   GMAC_ADDR_LOW(reg_n),reg_n,dcs);
	DBGPR_FUNC("<--dwmac4_set_umac_addr\n");
}

static void dwmac4_get_umac_addr(struct mac_device_info *hw,
				 unsigned char *addr, unsigned int reg_n)
{
	void __iomem *ioaddr = hw->pcsr;

	DBGPR_FUNC("-->dwmac4_get_umac_addr\n");

	tc9562mac_dwmac4_get_mac_addr(ioaddr, addr, GMAC_ADDR_HIGH(reg_n),
				   GMAC_ADDR_LOW(reg_n));

	DBGPR_FUNC("<--dwmac4_get_umac_addr\n");
	
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static void dwmac4_set_eee_mode(struct mac_device_info *hw,
				bool en_tx_lpi_clockgating)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	DBGPR_FUNC("-->dwmac4_set_eee_mode\n");

	/* Enable the link status receive on RGMII, SGMII ore SMII
	 * receive path and instruct the transmit to enter in LPI
	 * state.
	 */
	value = readl(ioaddr + GMAC4_LPI_CTRL_STATUS);
	value |= GMAC4_LPI_CTRL_STATUS_LPIEN | GMAC4_LPI_CTRL_STATUS_LPITXA;

	if (en_tx_lpi_clockgating)
		value |= GMAC4_LPI_CTRL_STATUS_LPITCSE;

	writel(value, ioaddr + GMAC4_LPI_CTRL_STATUS);

	DBGPR_FUNC("<--dwmac4_set_eee_mode\n");
	
}

static void dwmac4_reset_eee_mode(struct mac_device_info *hw)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	DBGPR_FUNC("-->dwmac4_reset_eee_mode\n");

	value = readl(ioaddr + GMAC4_LPI_CTRL_STATUS);
	value &= ~(GMAC4_LPI_CTRL_STATUS_LPIEN | GMAC4_LPI_CTRL_STATUS_LPITXA);
	writel(value, ioaddr + GMAC4_LPI_CTRL_STATUS);

	DBGPR_FUNC("<--dwmac4_reset_eee_mode\n");
	
}

static void dwmac4_set_eee_pls(struct mac_device_info *hw, int link)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	DBGPR_FUNC("-->dwmac4_set_eee_pls\n");

	value = readl(ioaddr + GMAC4_LPI_CTRL_STATUS);

	if (link)
		value |= GMAC4_LPI_CTRL_STATUS_PLS;
	else
		value &= ~GMAC4_LPI_CTRL_STATUS_PLS;

	writel(value, ioaddr + GMAC4_LPI_CTRL_STATUS);

	DBGPR_FUNC("<--dwmac4_set_eee_pls\n");
	
}

static void dwmac4_set_eee_timer(struct mac_device_info *hw, int ls, int tw)
{
	void __iomem *ioaddr = hw->pcsr;
	int value = ((tw & 0xffff)) | ((ls & 0x3ff) << 16);

	DBGPR_FUNC("-->dwmac4_set_eee_timer\n");

	/* Program the timers in the LPI timer control register:
	 * LS: minimum time (ms) for which the link
	 *  status from PHY should be ok before transmitting
	 *  the LPI pattern.
	 * TW: minimum time (us) for which the core waits
	 *  after it has stopped transmitting the LPI pattern.
	 */
	writel(value, ioaddr + GMAC4_LPI_TIMER_CTRL);

	DBGPR_FUNC("<--dwmac4_set_eee_timer\n");
	
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

static void dwmac4_set_filter(struct mac_device_info *hw,
			      struct net_device *dev ,unsigned int l2_address_filtering) 
{
	void __iomem *ioaddr = (void __iomem *)dev->base_addr;
	unsigned int value = 0, reg_value;
	int i = 0, reg = 1;
    int bit_nr = 0;

	DBGPR_FUNC("-->dwmac4_set_filter\n");
	
	if (dev->flags & IFF_PROMISC) {
		value = GMAC_PACKET_FILTER_RA;
		DBGPR_TEST("Promiscious mode selected\n");
	} else if ((dev->flags & IFF_ALLMULTI) ||
			(netdev_mc_count(dev) > HASH_TABLE_SIZE)) {
		/* Pass all multi */
		value = GMAC_PACKET_FILTER_PM;
		DBGPR_TEST("Pass all Multicast packet mode selected\n");
		/* Set the 64 bits of the HASH tab. To be updated if taller
		 * hash table is used
		 */
		writel(0xffffffff, ioaddr + GMAC_HASH_TAB_0_31);
		writel(0xffffffff, ioaddr + GMAC_HASH_TAB_32_63);
	} else if (!netdev_mc_empty(dev)) {
		u32 mc_filter[2];
		struct netdev_hw_addr *ha;
		DBGPR_TEST("Pass List of Multicast mode selected\n");
		if (l2_address_filtering == 1)
		{

    		/* Hash filter for multicast */
    		value = GMAC_PACKET_FILTER_HMC;
    		DBGPR_TEST("Hash Filter mode selected\n");

    		memset(mc_filter, 0, sizeof(mc_filter));
    		netdev_for_each_mc_addr(ha, dev) {
    			/* The upper 6 bits of the calculated CRC are used to
    			        * index the content of the Hash Table Reg 0 and 1.
    			        */
        		DBGPR_TEST("mc addr = %#x:%#x:%#x:%#x:%#x:%#x\n",
        		ha->addr[0], ha->addr[1], ha->addr[2],
        		ha->addr[3], ha->addr[4], ha->addr[5]);
    			bit_nr =
    				(bitrev32(~crc32_le(~0, ha->addr, 6)) >> 26);
    			/* The most significant bit determines the register
    			        * to use while the other 5 bits determines the bit
    			        * within the selected register
    			        */
    			mc_filter[bit_nr >> 5] |= (1 << (bit_nr & 0x1F));
    		}
    		writel(mc_filter[0], ioaddr + GMAC_HASH_TAB_0_31);
    		writel(mc_filter[1], ioaddr + GMAC_HASH_TAB_32_63);
            DBGPR_TEST("mc_filter[0] = %lx\n",mc_filter[0]);
            DBGPR_TEST("mc_filter[1] = %lx\n",mc_filter[1]);
		}		else {
    		value = ((value & GMAC_MPFR_HPF_Wr_Mask) | ((0x1 & GMAC_MPFR_HPF_Mask)<<10));
            //reg = 1;
			DBGPR_TEST("PERFECT FILTERING Selected");
				//netdev_mc_count(dev));

		    netdev_for_each_mc_addr(ha, dev) {
			    DBGPR_TEST("mc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n", i,
					ha->addr[0], ha->addr[1], ha->addr[2],
					ha->addr[3], ha->addr[4], ha->addr[5]);
                /* MAC Address indices 0,1,2,3 are reserved so we have to update from indices 4 to 15 address i.e 12 address we can configure*/        
                /*add mc/uc address from index 4, i.e from register 0x300 + (4 * 8)*/        
                if ((i < /*30*/(GMAC_MAX_PERFECT_ADDRESSES - (MAC_ADDR_ADD_SKIP_OFST + 1)))) {
                	dwmac4_set_umac_addr(hw, ha->addr, (reg + MAC_ADDR_ADD_SKIP_OFST), 0);
			        reg++;
			    }
    			i++;
			}
		}
    }
    
	/* Handle multiple unicast addresses */
	if (netdev_uc_count(dev) > GMAC_MAX_PERFECT_ADDRESSES) {
		/* Switch to promiscuous mode if more than 128 addrs
		 * are required
		 */
		value |= GMAC_PACKET_FILTER_RA;
	} else if (!netdev_uc_empty(dev)) {
		//int reg = 1; //donot update the index as multicast and unicast address gets added in the same register
		struct netdev_hw_addr *ha;

		netdev_for_each_uc_addr(ha, dev) {
    		DBGPR_TEST("uc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n", i,
    		ha->addr[0], ha->addr[1], ha->addr[2],
    		ha->addr[3], ha->addr[4], ha->addr[5]);
            /* MAC Address indices 0,1,2,3 are reserved so we have to update from indices 4 to 15 address i.e 12 address we can configure*/        
            /*add mc/uc address from index 4, i.e from register 0x300 + (4 * 8)*/        
            if ((i < (GMAC_MAX_PERFECT_ADDRESSES - (MAC_ADDR_ADD_SKIP_OFST + 1)))) {
                dwmac4_set_umac_addr(hw, ha->addr, (reg + MAC_ADDR_ADD_SKIP_OFST), 0);
                reg++;
            }
            i++;
            
	    }
	}

    reg_value = readl(ioaddr + GMAC_PACKET_FILTER);
    reg_value &=GMAC_PACKET_FILTER_MASK;
    reg_value |=value;
    writel(reg_value, ioaddr + GMAC_PACKET_FILTER);
	value = readl(ioaddr + GMAC_PACKET_FILTER);

	DBGPR_TEST("PKT Filter Value : %lx\n", value);
	DBGPR_FUNC("<----dwmac4_set_filter\n");
	
}

static void dwmac4_flow_ctrl(struct mac_device_info *hw, unsigned int duplex,
			     unsigned int fc, unsigned int pause_time,
			     u32 tx_cnt)
{
	void __iomem *ioaddr = hw->pcsr;
	unsigned int flow = 0;
	u32 queue = 0;

	DBGPR_FUNC("-->dwmac4_flow_ctrl\n");
	pr_debug("GMAC Flow-Control:\n");
	flow = readl(ioaddr + GMAC_RX_FLOW_CTRL);
	flow &= ~GMAC_RX_FLOW_CTRL_RFE;
	if (fc & FLOW_RX) {
		pr_debug("\tReceive Flow-Control ON\n");
		flow |= GMAC_RX_FLOW_CTRL_RFE;
	}
	writel(flow, ioaddr + GMAC_RX_FLOW_CTRL);
	if (fc & FLOW_TX) {
		pr_debug("\tTransmit Flow-Control ON\n");

		if (duplex)
			pr_debug("\tduplex mode: PAUSE %d\n", pause_time);

		for (queue = 0; queue < tx_cnt; queue++) {
    		flow = readl(ioaddr + GMAC_QX_TX_FLOW_CTRL(queue));
      		flow &=~GMAC_TX_FLOW_CTRL_PT_MASK;
			flow |= GMAC_TX_FLOW_CTRL_TFE;

			if (duplex)
				flow |=
				(pause_time << GMAC_TX_FLOW_CTRL_PT_SHIFT);

			writel(flow, ioaddr + GMAC_QX_TX_FLOW_CTRL(queue));
		}
	} else {
	
    	    for (queue = 0; queue < tx_cnt; queue++) {
        		flow = readl(ioaddr + GMAC_QX_TX_FLOW_CTRL(queue));
			    flow &= ~GMAC_TX_FLOW_CTRL_TFE;
				flow &=~GMAC_TX_FLOW_CTRL_PT_MASK;
			    writel(flow, ioaddr + GMAC_QX_TX_FLOW_CTRL(queue));
		    }
	}

	DBGPR_FUNC("<--dwmac4_flow_ctrl\n");
	
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static void dwmac4_ctrl_ane(void __iomem *ioaddr, bool ane, bool srgmi_ral,
			    bool loopback)
{
	DBGPR_FUNC("-->dwmac4_ctrl_ane\n");
	dwmac_ctrl_ane(ioaddr, GMAC_PCS_BASE, ane, srgmi_ral, loopback);
	DBGPR_FUNC("<--dwmac4_ctrl_ane\n");
}

static void dwmac4_rane(void __iomem *ioaddr, bool restart)
{
	dwmac_rane(ioaddr, GMAC_PCS_BASE, restart);
}

static void dwmac4_get_adv_lp(void __iomem *ioaddr, struct rgmii_adv *adv)
{
	DBGPR_FUNC("-->dwmac4_get_adv_lp\n");
	dwmac_get_adv_lp(ioaddr, GMAC_PCS_BASE, adv);
	DBGPR_FUNC("<--dwmac4_get_adv_lp\n");
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

/* RGMII or SMII interface */
static void dwmac4_phystatus(void __iomem *ioaddr, struct tc9562mac_extra_stats *x)
{
	u32 status;

	DBGPR_FUNC("-->dwmac4_phystatus\n");

	status = readl(ioaddr + GMAC_PHYIF_CONTROL_STATUS);
	x->irq_rgmii_n++;

	/* Check the link status */
	if (status & GMAC_PHYIF_CTRLSTATUS_LNKSTS) {
		int speed_value;

		x->pcs_link = 1;

		speed_value = ((status & GMAC_PHYIF_CTRLSTATUS_SPEED) >>
			       GMAC_PHYIF_CTRLSTATUS_SPEED_SHIFT);
		if (speed_value == GMAC_PHYIF_CTRLSTATUS_SPEED_125)
			x->pcs_speed = SPEED_1000;
		else if (speed_value == GMAC_PHYIF_CTRLSTATUS_SPEED_25)
			x->pcs_speed = SPEED_100;
		else
			x->pcs_speed = SPEED_10;

		x->pcs_duplex = (status & GMAC_PHYIF_CTRLSTATUS_LNKMOD);

		pr_info("Link is Up - %d/%s\n", (int)x->pcs_speed,
			x->pcs_duplex ? "Full" : "Half");
	} else {
		x->pcs_link = 0;
		pr_info("Link is Down\n");
	}

	DBGPR_FUNC("<--dwmac4_phystatus\n");
	
}

static int dwmac4_irq_mtl_status(struct mac_device_info *hw, u32 chan)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 mtl_int_qx_status;
	int ret = 0;

	DBGPR_FUNC("-->dwmac4_irq_mtl_status\n");

	mtl_int_qx_status = readl(ioaddr + MTL_INT_STATUS);

	/* Check MTL Interrupt */
	if (mtl_int_qx_status & MTL_INT_QX(chan)) {
		/* read Queue x Interrupt status */
		u32 status = readl(ioaddr + MTL_CHAN_INT_CTRL(chan));

		if (status & MTL_RX_OVERFLOW_INT) {
			/*  clear Interrupt */
			writel(status | MTL_RX_OVERFLOW_INT,
			       ioaddr + MTL_CHAN_INT_CTRL(chan));
			ret = CORE_IRQ_MTL_RX_OVERFLOW;
		}
	}

	DBGPR_FUNC("<--dwmac4_irq_mtl_status\n");

	return ret;
}

static int dwmac4_irq_status(struct mac_device_info *hw,
			     struct tc9562mac_extra_stats *x)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 intr_status;
	u32 intr_enable = readl(ioaddr + GMAC_INT_EN); 
	int ret = 0;
#ifdef VERIFY_PPO_USING_AUX
	int val = 0;
#endif

	DBGPR_FUNC("-->dwmac4_irq_status\n");

	intr_status = readl(ioaddr + GMAC_INT_STATUS);
	intr_status &= intr_enable; 
#ifdef VERIFY_PPO_USING_AUX
		{
			#define TS_STATUS_AUXTSTRIG_BIT			2

			if ( intr_status & ( 0x1 << 12) ) // bit 12 is set
			{
				val = readl(ioaddr + PTP_GMAC4_OFFSET + PTP_TS_STATUS);
				if ( val & ( 1 << TS_STATUS_AUXTSTRIG_BIT) ) { // Aux
					printk("\n");
					printk("second: %x\n",  readl(ioaddr + PTP_GMAC4_OFFSET + 0x4C));  
					printk("subsec: %x\n", readl(ioaddr + PTP_GMAC4_OFFSET + 0x48));  
				}
			}
		}
#endif

	/* Not used events (e.g. MMC interrupts) are not handled. */
	if ((intr_status & mmc_tx_irq))
		x->mmc_tx_irq_n++;
	if (unlikely(intr_status & mmc_rx_irq))
		x->mmc_rx_irq_n++;
	if (unlikely(intr_status & mmc_rx_csum_offload_irq))
		x->mmc_rx_csum_offload_irq_n++;
	/* Clear the PMT bits 5 and 6 by reading the PMT status reg */
	if (unlikely(intr_status & pmt_irq)) {
		readl(ioaddr + GMAC_PMT);
		x->irq_receive_pmt_irq_n++;
	}

	/* MAC tx/rx EEE LPI entry/exit interrupts */
	if (intr_status & lpi_irq) {
		/* Clear LPI interrupt by reading MAC_LPI_Control_Status */
		u32 status = readl(ioaddr + GMAC4_LPI_CTRL_STATUS);

		if (status & GMAC4_LPI_CTRL_STATUS_TLPIEN) {
			ret |= CORE_IRQ_TX_PATH_IN_LPI_MODE;
			x->irq_tx_path_in_lpi_mode_n++;
		}
		if (status & GMAC4_LPI_CTRL_STATUS_TLPIEX) {
			ret |= CORE_IRQ_TX_PATH_EXIT_LPI_MODE;
			x->irq_tx_path_exit_lpi_mode_n++;
		}
		if (status & GMAC4_LPI_CTRL_STATUS_RLPIEN)
			x->irq_rx_path_in_lpi_mode_n++;
		if (status & GMAC4_LPI_CTRL_STATUS_RLPIEX)
			x->irq_rx_path_exit_lpi_mode_n++;
	}
	
	dwmac_pcs_isr(ioaddr, GMAC_PCS_BASE, intr_status, x);
	if (intr_status & PCS_RGSMIIIS_IRQ)
	{
		DBGPR_TEST("\n RGMII/SMII Interrupt");
		dwmac4_phystatus(ioaddr, x);
	}
	DBGPR_FUNC("<--dwmac4_irq_status\n");
	return ret;
}

static void dwmac4_debug(void __iomem *ioaddr, struct tc9562mac_extra_stats *x,
			 u32 rx_queues, u32 tx_queues)
{
	u32 value;
	u32 queue;

	DBGPR_FUNC("-->dwmac4_debug\n");

	for (queue = 0; queue < tx_queues; queue++) {
		value = readl(ioaddr + MTL_CHAN_TX_DEBUG(queue));

		if (value & MTL_DEBUG_TXSTSFSTS)
			x->mtl_tx_status_fifo_full++;
		if (value & MTL_DEBUG_TXFSTS)
			x->mtl_tx_fifo_not_empty++;
		if (value & MTL_DEBUG_TWCSTS)
			x->mmtl_fifo_ctrl++;
		if (value & MTL_DEBUG_TRCSTS_MASK) {
			u32 trcsts = (value & MTL_DEBUG_TRCSTS_MASK)
				     >> MTL_DEBUG_TRCSTS_SHIFT;
			if (trcsts == MTL_DEBUG_TRCSTS_WRITE)
				x->mtl_tx_fifo_read_ctrl_write++;
			else if (trcsts == MTL_DEBUG_TRCSTS_TXW)
				x->mtl_tx_fifo_read_ctrl_wait++;
			else if (trcsts == MTL_DEBUG_TRCSTS_READ)
				x->mtl_tx_fifo_read_ctrl_read++;
			else
				x->mtl_tx_fifo_read_ctrl_idle++;
		}
		if (value & MTL_DEBUG_TXPAUSED)
			x->mac_tx_in_pause++;
	}

	for (queue = 0; queue < rx_queues; queue++) {
		value = readl(ioaddr + MTL_CHAN_RX_DEBUG(queue));

		if (value & MTL_DEBUG_RXFSTS_MASK) {
			u32 rxfsts = (value & MTL_DEBUG_RXFSTS_MASK)
				     >> MTL_DEBUG_RRCSTS_SHIFT;

			if (rxfsts == MTL_DEBUG_RXFSTS_FULL)
				x->mtl_rx_fifo_fill_level_full++;
			else if (rxfsts == MTL_DEBUG_RXFSTS_AT)
				x->mtl_rx_fifo_fill_above_thresh++;
			else if (rxfsts == MTL_DEBUG_RXFSTS_BT)
				x->mtl_rx_fifo_fill_below_thresh++;
			else
				x->mtl_rx_fifo_fill_level_empty++;
		}
		if (value & MTL_DEBUG_RRCSTS_MASK) {
			u32 rrcsts = (value & MTL_DEBUG_RRCSTS_MASK) >>
				     MTL_DEBUG_RRCSTS_SHIFT;

			if (rrcsts == MTL_DEBUG_RRCSTS_FLUSH)
				x->mtl_rx_fifo_read_ctrl_flush++;
			else if (rrcsts == MTL_DEBUG_RRCSTS_RSTAT)
				x->mtl_rx_fifo_read_ctrl_read_data++;
			else if (rrcsts == MTL_DEBUG_RRCSTS_RDATA)
				x->mtl_rx_fifo_read_ctrl_status++;
			else
				x->mtl_rx_fifo_read_ctrl_idle++;
		}
		if (value & MTL_DEBUG_RWCSTS)
			x->mtl_rx_fifo_ctrl_active++;
	}

	/* GMAC debug */
	value = readl(ioaddr + GMAC_DEBUG);

	if (value & GMAC_DEBUG_TFCSTS_MASK) {
		u32 tfcsts = (value & GMAC_DEBUG_TFCSTS_MASK)
			      >> GMAC_DEBUG_TFCSTS_SHIFT;

		if (tfcsts == GMAC_DEBUG_TFCSTS_XFER)
			x->mac_tx_frame_ctrl_xfer++;
		else if (tfcsts == GMAC_DEBUG_TFCSTS_GEN_PAUSE)
			x->mac_tx_frame_ctrl_pause++;
		else if (tfcsts == GMAC_DEBUG_TFCSTS_WAIT)
			x->mac_tx_frame_ctrl_wait++;
		else
			x->mac_tx_frame_ctrl_idle++;
	}
	if (value & GMAC_DEBUG_TPESTS)
		x->mac_gmii_tx_proto_engine++;
	if (value & GMAC_DEBUG_RFCFCSTS_MASK)
		x->mac_rx_frame_ctrl_fifo = (value & GMAC_DEBUG_RFCFCSTS_MASK)
					    >> GMAC_DEBUG_RFCFCSTS_SHIFT;
	if (value & GMAC_DEBUG_RPESTS)
		x->mac_gmii_rx_proto_engine++;

	DBGPR_FUNC("<--dwmac4_debug\n");
	
}

static const struct tc9562mac_ops dwmac4_ops = {
	.core_init = dwmac4_core_init,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE				
	.set_mac = tc9562mac_set_mac,
#else	
	.set_mac = tc9562mac_dwmac4_set_mac,
#endif
	.rx_ipc = dwmac4_rx_ipc_enable,
	.rx_queue_enable = dwmac4_rx_queue_enable,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE		
	.rx_queue_disable = dwmac4_rx_queue_disable,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
	.rx_queue_prio = dwmac4_rx_queue_priority,
	.tx_queue_prio = dwmac4_tx_queue_priority,
	.rx_queue_routing = dwmac4_rx_queue_routing,
	.prog_mtl_rx_algorithms = dwmac4_prog_mtl_rx_algorithms,
	.prog_mtl_tx_algorithms = dwmac4_prog_mtl_tx_algorithms,
	.set_mtl_tx_queue_weight = dwmac4_set_mtl_tx_queue_weight,
	.map_mtl_to_dma = dwmac4_map_mtl_dma,
	.config_cbs = dwmac4_config_cbs,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE		
	.dump_regs = dwmac4_dump_regs,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */		
	.host_irq_status = dwmac4_irq_status,
	.host_mtl_irq_status = dwmac4_irq_mtl_status,
	.flow_ctrl = dwmac4_flow_ctrl,
	.pmt = dwmac4_pmt,
	.set_umac_addr = dwmac4_set_umac_addr,
	.get_umac_addr = dwmac4_get_umac_addr,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE		
	.set_eee_mode = dwmac4_set_eee_mode,
	.reset_eee_mode = dwmac4_reset_eee_mode,
	.set_eee_timer = dwmac4_set_eee_timer,
	.set_eee_pls = dwmac4_set_eee_pls,
	.pcs_ctrl_ane = dwmac4_ctrl_ane,
	.pcs_rane = dwmac4_rane,
	.pcs_get_adv_lp = dwmac4_get_adv_lp,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */	
	.debug = dwmac4_debug,
	.set_filter = dwmac4_set_filter,
};

static const struct tc9562mac_ops dwmac410_ops = {
	.core_init = dwmac4_core_init,
	.set_mac = tc9562mac_dwmac4_set_mac,
	.rx_ipc = dwmac4_rx_ipc_enable,
	.rx_queue_enable = dwmac4_rx_queue_enable,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE		
	.rx_queue_disable = dwmac4_rx_queue_disable,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
	.rx_queue_prio = dwmac4_rx_queue_priority,
	.tx_queue_prio = dwmac4_tx_queue_priority,
	.rx_queue_routing = dwmac4_rx_queue_routing,
	.prog_mtl_rx_algorithms = dwmac4_prog_mtl_rx_algorithms,
	.prog_mtl_tx_algorithms = dwmac4_prog_mtl_tx_algorithms,
	.set_mtl_tx_queue_weight = dwmac4_set_mtl_tx_queue_weight,
	.map_mtl_to_dma = dwmac4_map_mtl_dma,
	.config_cbs = dwmac4_config_cbs,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE	
	.dump_regs = dwmac4_dump_regs,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */	
	.host_irq_status = dwmac4_irq_status,
	.host_mtl_irq_status = dwmac4_irq_mtl_status,
	.flow_ctrl = dwmac4_flow_ctrl,
	.pmt = dwmac4_pmt,
	.set_umac_addr = dwmac4_set_umac_addr,
	.get_umac_addr = dwmac4_get_umac_addr,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	.set_eee_mode = dwmac4_set_eee_mode,
	.reset_eee_mode = dwmac4_reset_eee_mode,
	.set_eee_timer = dwmac4_set_eee_timer,
	.set_eee_pls = dwmac4_set_eee_pls,
	.pcs_ctrl_ane = dwmac4_ctrl_ane,
	.pcs_rane = dwmac4_rane,
	.pcs_get_adv_lp = dwmac4_get_adv_lp,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
	.debug = dwmac4_debug,
	.set_filter = dwmac4_set_filter,
};

static const struct tc9562mac_ops dwmac500_ops = {
	.core_init = dwmac4_core_init,
	.set_mac = tc9562mac_dwmac4_set_mac,
	.rx_ipc = dwmac4_rx_ipc_enable,
	.rx_queue_enable = dwmac4_rx_queue_enable,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE		
	.rx_queue_disable = dwmac4_rx_queue_disable,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
	.rx_queue_prio = dwmac4_rx_queue_priority,
	.tx_queue_prio = dwmac4_tx_queue_priority,
	.rx_queue_routing = dwmac4_rx_queue_routing,
	.prog_mtl_rx_algorithms = dwmac4_prog_mtl_rx_algorithms,
	.prog_mtl_tx_algorithms = dwmac4_prog_mtl_tx_algorithms,
	.set_mtl_tx_queue_weight = dwmac4_set_mtl_tx_queue_weight,
	.map_mtl_to_dma = dwmac4_map_mtl_dma,
	.config_cbs = dwmac4_config_cbs,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE	
	.dump_regs = dwmac4_dump_regs,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */	
	.host_irq_status = dwmac4_irq_status,
	.host_mtl_irq_status = dwmac4_irq_mtl_status,
	.flow_ctrl = dwmac4_flow_ctrl,
	.pmt = dwmac4_pmt,
	.set_umac_addr = dwmac4_set_umac_addr,
	.get_umac_addr = dwmac4_get_umac_addr,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	.set_eee_mode = dwmac4_set_eee_mode,
	.reset_eee_mode = dwmac4_reset_eee_mode,
	.set_eee_timer = dwmac4_set_eee_timer,
	.set_eee_pls = dwmac4_set_eee_pls,
	.pcs_ctrl_ane = dwmac4_ctrl_ane,
	.pcs_rane = dwmac4_rane,
	.pcs_get_adv_lp = dwmac4_get_adv_lp,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */		
	.debug = dwmac4_debug,
	.set_filter = dwmac4_set_filter,
	.est_init = dwmac5_est_init,
};

static const struct tc9562mac_ops dwmac510_ops = {
	.core_init = dwmac4_core_init,
	.set_mac = tc9562mac_dwmac4_set_mac,
	.rx_ipc = dwmac4_rx_ipc_enable,
	.rx_queue_enable = dwmac4_rx_queue_enable,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	.rx_queue_disable = dwmac4_rx_queue_disable,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
	.rx_queue_prio = dwmac4_rx_queue_priority,
	.tx_queue_prio = dwmac4_tx_queue_priority,
	.rx_queue_routing = dwmac4_rx_queue_routing,
	.prog_mtl_rx_algorithms = dwmac4_prog_mtl_rx_algorithms,
	.prog_mtl_tx_algorithms = dwmac4_prog_mtl_tx_algorithms,
	.set_mtl_tx_queue_weight = dwmac4_set_mtl_tx_queue_weight,
	.map_mtl_to_dma = dwmac4_map_mtl_dma,
	.config_cbs = dwmac4_config_cbs,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE	
	.dump_regs = dwmac4_dump_regs,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */	
	.host_irq_status = dwmac4_irq_status,
	.host_mtl_irq_status = dwmac4_irq_mtl_status,
	.flow_ctrl = dwmac4_flow_ctrl,
	.pmt = dwmac4_pmt,
	.set_umac_addr = dwmac4_set_umac_addr,
	.get_umac_addr = dwmac4_get_umac_addr,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE			
	.set_eee_mode = dwmac4_set_eee_mode,
	.reset_eee_mode = dwmac4_reset_eee_mode,
	.set_eee_timer = dwmac4_set_eee_timer,
	.set_eee_pls = dwmac4_set_eee_pls,
	.pcs_ctrl_ane = dwmac4_ctrl_ane,
	.pcs_rane = dwmac4_rane,
	.pcs_get_adv_lp = dwmac4_get_adv_lp,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
	.debug = dwmac4_debug,
	.set_filter = dwmac4_set_filter,
	.est_init = dwmac5_est_init,
	.safety_feat_init = dwmac5_safety_feat_init,
	.safety_feat_irq_status = dwmac5_safety_feat_irq_status,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE	
	.safety_feat_set = dwmac5_safety_feat_set,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
	.rx_parser_init = dwmac5_rx_parser_init,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,15))
	.rx_config = dwmac5_rxp_config,
#endif
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE		
	.pps_init = dwmac5_pps_init,
	.mcgr_init = dwmac5_mcgr_init,
	.mcgr_intr = dwmac5_mcgr_intr,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */		
};

struct mac_device_info *dwmac4_setup(void __iomem *ioaddr, int mcbins,
				     int perfect_uc_entries, int *synopsys_id)
{
	struct mac_device_info *mac;
	u32 hwid = readl(ioaddr + GMAC_VERSION);

	DBGPR_FUNC("-->dwmac4_setup\n");

	mac = kzalloc(sizeof(const struct mac_device_info), GFP_KERNEL);
	if (!mac)
		return NULL;

	mac->pcsr = ioaddr;
	mac->multicast_filter_bins = mcbins;
	mac->unicast_filter_entries = perfect_uc_entries;
	mac->mcast_bits_log2 = 0;

	if (mac->multicast_filter_bins)
		mac->mcast_bits_log2 = ilog2(mac->multicast_filter_bins);

	mac->link.duplex = GMAC_CONFIG_DM;
	mac->link.speed10 = GMAC_CONFIG_PS;
	mac->link.speed100 = GMAC_CONFIG_FES | GMAC_CONFIG_PS;
	mac->link.speed1000 = 0;
	mac->link.speed_mask = GMAC_CONFIG_FES | GMAC_CONFIG_PS;
	mac->mii.addr = GMAC_MDIO_ADDR;
	mac->mii.data = GMAC_MDIO_DATA;
	mac->mii.addr_shift = 21;
	mac->mii.addr_mask = GENMASK(25, 21);
	mac->mii.reg_shift = 16;
	mac->mii.reg_mask = GENMASK(20, 16);
	mac->mii.clk_csr_shift = 8;
	mac->mii.clk_csr_mask = GENMASK(11, 8);

	/* Get and dump the chip ID */
	*synopsys_id = tc9562mac_get_synopsys_id(hwid);

	if (*synopsys_id > DWMAC_CORE_4_00)
		mac->dma = &dwmac410_dma_ops;
	else
		mac->dma = &dwmac4_dma_ops;

	if (*synopsys_id >= DWMAC_CORE_5_10)
		mac->mac = &dwmac510_ops;
	else if (*synopsys_id >= DWMAC_CORE_5_00)
		mac->mac = &dwmac500_ops;
	else if (*synopsys_id >= DWMAC_CORE_4_00)
		mac->mac = &dwmac410_ops;
	else
		mac->mac = &dwmac4_ops;

	DBGPR_FUNC("<--dwmac4_setup\n");

	return mac;
}
