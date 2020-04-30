/*
 * TC9562 ethernet driver.
 *
 * tc9562_pci.c
 *
 * Copyright (C) 2020 Toshiba Electronic Devices & Storage Corporation
 *
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
 *  26 Feb 2020 : 1. Added Unified Driver feature.
                  2. Added SGMII Interface support.
 *  VERSION     : 01-01
 *  30 Sep 2019 : Base lined
 *  VERSION     : 01-00
 */

#include <linux/firmware.h>
#include <linux/pci.h>
#include <linux/dmi.h>

#include "tc9562mac.h"


#ifdef TC9562_LOAD_FW_HEADER
#include "fw.h"
#endif 

extern int tc9562mac_init(void);
extern void tc9562mac_exit(void);

/*
 * This struct is used to associate PCI Function of MAC controller on a board,
 * discovered via DMI, with the address of PHY connected to the MAC. The
 * negative value of the address means that MAC controller is not connected
 * with PHY.
 */
struct tc9562mac_pci_func_data {
	unsigned int func;
	int phy_addr;
};

struct tc9562mac_pci_dmi_data {
	const struct tc9562mac_pci_func_data *func;
	size_t nfuncs;
};

struct tc9562mac_pci_info {
	int (*setup)(struct pci_dev *pdev, struct plat_tc9562macenet_data *plat);
};

static const u8 snps_dev_addr_talker[6] = {0x68, 0x05, 0xca, 0x51, 0x33, 0x81};
static const u8 snps_dev_addr_listener[6] = {0x68, 0x05, 0xca, 0x51, 0x32, 0xcd};
bool is_talker = true;
module_param(is_talker, bool, 0);
MODULE_PARM_DESC(is_talker, "MAC address shall be for a talker (default=true)");

#if 0
static u32 snps_est_ctr[] = {1200000, 0}; /* ns */
static u32 snps_est_gcl_entries[] = {
	0x101d4c0, /* 120000ns = 10% BW - Q0 */
	0xe107ac0, /* 1080000ns = 90% BW - Q1, Q2, Q3 */
};
#endif 
#if 0
static u32 snps_est_ctr[] = {0x7A120, 0}; /* ns */  //500 usec
static u32 snps_est_gcl_entries[] = {
	0x0103d090, /* 120000ns = 10% BW - Q0 */     //250 usec
	0xfe03d090, /* 1080000ns = 90% BW - Q1, Q2, Q3 */  // 250 usec
};
#endif

#if 0 
static u32 snps_est_ctr[] = {0x1312D0, 0}; /* ns */  //250 usec
static u32 snps_est_gcl_entries[] = {
	0xef01e848, /* 120000ns = 10% BW - Q0 */     //125 usec
	0xff01e848, /* 1080000ns = 90% BW - Q1, Q2, Q3 */  // 125 usec
	0xef01e848, /* 120000ns = 10% BW - Q0 */     //125 usec
	0xff01e848, /* 1080000ns = 90% BW - Q1, Q2, Q3 */  // 125 usec
	0xef01e848, /* 120000ns = 10% BW - Q0 */     //125 usec
	0xff01e848, /* 1080000ns = 90% BW - Q1, Q2, Q3 */  // 125 usec
	0xef01e848, /* 120000ns = 10% BW - Q0 */     //125 usec
	0xff01e848, /* 1080000ns = 90% BW - Q1, Q2, Q3 */  // 125 usec
	0xef01e848, /* 120000ns = 10% BW - Q0 */     //125 usec
	0xff01e848, /* 1080000ns = 90% BW - Q1, Q2, Q3 */  // 125 usec
};
#endif 

#if 1 
static u32 snps_est_ctr[] = {0x1E848*4 /* 0x03d090 * 1*/, 0}; /* ns */  //250 usec //200ms 200000000 //125us 0x1E848
static u32 snps_est_gcl_entries[] = {
	0x0f0061a8, //25usec
	0xff0186a0, //100usec
	0x0f0061a8,
	0xff0186a0,
	0x0f0061a8,
	0xff0186a0,
	0x0f0061a8,
	0xff0186a0,
	
};
#endif 
#if 0 
static u32 snps_est_ctr[] = {0x07a120, 0}; /* ns */  //500 usec
static u32 snps_est_gcl_entries[] = {
	0x20006EA8, /*  25000ns -  */     
	0xDF0186A0, /* 120000ns -  */     
	0x0f006EA8, /*  25000ns -  */     
	0xDf0186A0, /* 120000ns -  */     
	0x03006EA8, /*  25000ns -  */     
	0xDf0186A0, /* 120000ns -  */     
	0x07006EA8, /*  25000ns -  */     
	0xDf0186A0, /* 120000ns -  */     
};
#endif 

const tc9562_version_t tc9562_version = {'R', 1, 0, 1};

/*
FRP Instruction table done based on following :

1) The VLAN ID of Audio Packets and the Video Packets are considered as same.
2) VLAN ID of class A & class B considered different.
3) The Values of NVE and NPE are considered as same.
4) The Audio Packets with Stream ID matched are routed to CH-4.
5) The Audio packet which doesnt match stream id & Video Packets are routed to CH-3.
6) Class B packets routed to CH-3.
7) The Legacy Packets with MAC ID of Host PC are routed to CH-0.
8) The Legacy Packets with MAC ID of CM3 are routed to CH-1.
9) Untagged AVB packets and gptp packets are routed to CH-2
10) The Broadcast Packets & multicast packets are routed to CH-0
11) The OK Index Field will contain JUMP Instruction Number.
12) In match_data Field, the Dont care bits are considered as 'F'.

Note: if CM3 traffic enabled change channel number of Broadcast and multicast packet to 0x3 (CH0 and CH1)

*/

static struct tc9562mac_rx_parser_entry snps_rxp_entries[] = {
	{
		.match_data = 0xFFFFF022,.match_en = 0x0000FFFF,.af = 0x0,.rf = 0x0,.im	= 0x0,.nc = 0x1,.res1 = 0x0,.frame_offset = 0x4,.ok_index = 0x7,.dma_ch_no = 0x0,.res2 = 0x0,
	},/* Ether Type Comparision for Audio and Video Data [0]*/
	{
		.match_data = 0xFF6FFFFF,.match_en = 0x00E00000,.af = 0x0,.rf = 0x0,.im	= 0x0,.nc = 0x1,.res1 = 0x0,.frame_offset = 0x3,.ok_index = 0x6,.dma_ch_no = 0x0,.res2 = 0x0,
	},/* VLAN ID Comparision for Audio and Video Data (Class A)[1] */
	{
		.match_data = 0xE0E6FFFF,.match_en = 0xFFFF0000,.af = 0x0,.rf = 0x0,.im	= 0x0,.nc = 0x1,.res1 = 0x0,.frame_offset = 0x5,.ok_index = 0x5,.dma_ch_no = 0x0,.res2 = 0x0,
	},/* Stream ID-1 Comparision for TDM Audio Packet (AF) [2] */
	{
		.match_data = 0xF07DB5B7,.match_en = 0xFFFFFFFF,.af = 0x0,.rf = 0x0,.im	= 0x0,.nc = 0x1,.res1 = 0x0,.frame_offset = 0x6,.ok_index = 0x5,.dma_ch_no = 0x0,.res2 = 0x0,
	},/* Stream ID-2 Comparision for TDM Audio Packet (AF) [3] */
	{
		.match_data = 0xFFFF0000,.match_en = 0x0000FFFF,.af = 0x1,.rf = 0x0,.im	= 0x0,.nc = 0x0,.res1 = 0x0,.frame_offset = 0x7,.ok_index = 0x5,.dma_ch_no = 0x10,.res2 = 0x0,
	},/* Stream ID-3 Comparision for TDM Audio Packet (AF) [4]*/
	{
		.match_data = 0xFFFFFFFF,.match_en = 0x00000000,.af = 0x1,.rf = 0x0,.im = 0x0,.nc = 0x0,.res1 = 0x0,.frame_offset = 0x0,.ok_index = 0x6,.dma_ch_no = 0x8,.res2 = 0x0,
	},/* VLAN ID Comparision for (Class A) Video and Audio Packet, but not TDM   [5] */
	{
		.match_data = 0xFF4FFFFF,.match_en = 0x00E00000,.af = 0x1,.rf = 0x0,.im	= 0x0,.nc = 0x0,.res1 = 0x0,.frame_offset = 0x3,.ok_index = 0x7,.dma_ch_no = 0x8,.res2 = 0x0,
	},/* VLAN ID Comparision for (Class B) Video and Audio Packet [6]  */
	{
		.match_data = 0xb5b7e0e6,.match_en = 0xFFFFFFFF,.af = 0x0,.rf = 0x0,.im	= 0x0,.nc = 0x1,.res1 = 0x0,.frame_offset = 0x0,.ok_index = 0x9,.dma_ch_no = 0x0,.res2 = 0x0,
	},/* MAC ID{ 0xE6, 0xE0, 0xB7, 0xB5, 0x7D, 0xF0} of Host PC Comparision for Legacy Packet [ 7]  */
	{
		.match_data = 0xFFFFf07d,.match_en = 0x0000FFFF,.af = 0x1,.rf = 0x0,.im	= 0x0,.nc = 0x0,.res1 = 0x0,.frame_offset = 0x1,.ok_index = 0x9,.dma_ch_no = 0x1,.res2 = 0x0,
	},/* MAC ID of Host PC Comparision for Legacy Packet [8] */
	{
		.match_data = 0xb5b7e0e8,.match_en = 0xFFFFFFFF,.af = 0x0,.rf = 0x0,.im	= 0x0,.nc = 0x1,.res1 = 0x0,.frame_offset = 0x0,.ok_index = 0xB,.dma_ch_no = 0x0,.res2 = 0x0,
	},/* MAC ID{ 0xE8, 0xE0, 0xB7, 0xB5, 0x7D, 0xF8} of CM3 Comparision for Legacy Packet [9]   */
	{
		.match_data = 0xFFFFf87d,.match_en = 0x0000FFFF,.af = 0x1,.rf = 0x0,.im	= 0x0,.nc = 0x0,.res1 = 0x0,.frame_offset = 0x1,.ok_index = 0xB,.dma_ch_no = 0x2,.res2 = 0x0,
	},/* MAC ID of CM3 Comparision for Legacy Packet [A]  */
	{
		.match_data = 0xFFFFF022,.match_en = 0x0000FFFF,.af = 0x1,.rf = 0x0,.im = 0x0,.nc = 0x0,.res1 = 0x0,.frame_offset = 0x3,.ok_index = 0xC,.dma_ch_no = 0x4,.res2 = 0x0,
	},/* Ether Type Comparision for AV untagged [B]*/			
	{
		.match_data = 0xFFFFF788,.match_en = 0x0000FFFF,.af = 0x1,.rf = 0x0,.im = 0x0,.nc = 0x0,.res1 = 0x0,.frame_offset = 0x3,.ok_index = 0xD,.dma_ch_no = 0x4,.res2 = 0x0,
	},/* Ether Type Comparision for GPTP [C]*/		
	{
		.match_data = 0xFFFFFFFF,.match_en = 0xFFFFFFFF,.af = 0x0,.rf = 0x0,.im	= 0x0,.nc = 0x1,.res1 = 0x0,.frame_offset = 0x0,.ok_index = 0xF,.dma_ch_no = 0x0,.res2 = 0x0,
	},/* DA Comparision for Broadcast Packets [D] */
#ifdef TC9562_PKT_DUP
	{
		.match_data = 0xFFFFFFFF,.match_en = 0x0000FFFF,.af = 0x1,.rf = 0x0,.im	= 0x0,.nc = 0x0,.res1 = 0x0,.frame_offset = 0x1,.ok_index = 0xF,.dma_ch_no = 0x3,.res2 = 0x0,
	},/* DA Comparision for Broadcast Packets [E]*/
	{
		.match_data = 0xFFFFFFFF,.match_en = 0x00000001,.af = 0x1,.rf = 0x0,.im	= 0x0,.nc = 0x0,.res1 = 0x0,.frame_offset = 0x0,.ok_index = 0x10,.dma_ch_no = 0x3,.res2 = 0x0,
	},/* DA Comparision for multicast Packets{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x01} [F]  */
#else
	{
		.match_data = 0xFFFFFFFF,.match_en = 0x0000FFFF,.af = 0x1,.rf = 0x0,.im	= 0x0,.nc = 0x0,.res1 = 0x0,.frame_offset = 0x1,.ok_index = 0xF,.dma_ch_no = 0x1,.res2 = 0x0,
	},/* DA Comparision for Broadcast Packets [E]*/
	{
		.match_data = 0xFFFFFFFF,.match_en = 0x00000001,.af = 0x1,.rf = 0x0,.im	= 0x0,.nc = 0x0,.res1 = 0x0,.frame_offset = 0x0,.ok_index = 0x10,.dma_ch_no = 0x1,.res2 = 0x0,
	},/* DA Comparision for multicast Packets{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x01} [F]  */
#endif
	{
		.match_data = 0xFFFFFFFF,.match_en = 0x00000000,.af = 0x1,.rf = 0x0,.im	= 0x0,.nc = 0x0,.res1 = 0x0,.frame_offset = 0x0,.ok_index = 0x11,.dma_ch_no = 0x1,.res2 = 0x0,
	},/* Pass all packets to avoid dropping [0x10] */
};


int tc9562_load_firmware(struct device* dev, struct tc9562mac_resources* res)
{
	unsigned long adrs = 0, val = 0;
	
#ifdef TC9562_LOAD_FW_HEADER
	unsigned int fw_size = sizeof(fw_data);

	if(fw_size > 320*1024){
	    NMSGPR_ALERT("Error : FW size exceeds the memory size\n");	
	    return -EINVAL; 
	}

	NMSGPR_INFO("FW Loading Start...\n");
	NMSGPR_INFO("FW Size = %d\n", fw_size );

  /* Assert M3 reset */
    adrs = 0x1008;
    val = ioread32( (void*)(res->addr + adrs));
    printk("Reset Register value = %lx\n", val);

    val |= 0x1;
 	iowrite32(val, (void*)(res->addr + adrs));
 	
    adrs = 0;//SRAM Start Address
    do
    {
	    val =  fw_data[adrs+0] << 0;
                val |= fw_data[adrs+1] << 8;
                val |= fw_data[adrs+2] << 16;
                val |= fw_data[adrs+3] << 24;

 		iowrite32(val, (void*)(res->tc9562_SRAM_pci_base_addr + adrs));
                adrs += 4;
    }while(adrs < fw_size);

#else
    const struct firmware *pfw = NULL;
    /* Get TC9562 FW binary through kernel firmware interface request */
     if (request_firmware(&pfw, FIRMWARE_NAME, dev) != 0) {
       NMSGPR_ALERT("TC9562: Error in calling request_firmware ");
       return -EINVAL;
     }

     if (pfw == NULL) {
       NMSGPR_ALERT("TC9562: request_firmware: pfw == NULL");
       return -EINVAL;
     }
    
    if(pfw->size > 320*1024) {
        NMSGPR_ALERT("Error : FW size exceeds the memory size\n");	
	    return -EINVAL; 
    }
    
    NMSGPR_INFO("FW Loading Start...\n");
	NMSGPR_INFO("FW Size = %ld\n", pfw->size );
	
    /* Assert M3 reset */
    adrs = 0x1008;
    val = ioread32( (void*)(res->addr + adrs));
    printk("Reset Register value = %lx\n", val);

    val |= 0x1;
 	iowrite32(val, (void*)(res->addr + adrs));
 	
    /* Copy TC9560 FW to SRAM */
     memcpy((char *)res->tc9562_SRAM_pci_base_addr , pfw->data, pfw->size);
     
     /* Release kernel firmware interface */
     release_firmware(pfw);
#endif

	printk("FW Loading Finish.\n");

    /* De-assert M3 reset */
    adrs = 0x1008;
    val = ioread32( (void*)(res->addr + adrs));
    val &= ~0x3;
 	iowrite32(val, (void*)(res->addr + adrs));

    printk("Neutrino M3 started.\n");
    
    return 0; 
}

EXPORT_SYMBOL_GPL(tc9562_load_firmware);

static void common_default_data(struct plat_tc9562macenet_data *plat)
{
	DBGPR_FUNC("-->common_default_data\n");

	plat->clk_csr = 0x2/* 0x5 */;	/* clk_csr_i = 20-35MHz & MDC = clk_csr_i/16 */
	plat->has_gmac = 0;
	plat->has_gmac4 = 1;
	plat->force_sf_dma_mode = 0;
	plat->force_thresh_dma_mode  = 0;
	plat->force_no_rx_coe = 0;
	plat->force_no_tx_coe = 0;
	plat->mdio_bus_data->phy_reset = NULL;
	plat->mdio_bus_data->phy_mask = 0;
	plat->mac_port_sel_speed = 1000;

	/* Set default value for multicast hash bins */
	plat->multicast_filter_bins = HASH_TABLE_SIZE;

	/* Set default value for unicast filter entries */
	plat->unicast_filter_entries = 1;

	/* Set the maxmtu to a default of JUMBO_LEN */
	plat->maxmtu = (ETH_FRAME_LEN + ETH_FCS_LEN + VLAN_HLEN);

	DBGPR_FUNC("<--common_default_data\n");
}

static int tc9562_default_data(struct pci_dev *pdev,
			       struct plat_tc9562macenet_data *plat)
{
	DBGPR_FUNC("-->tc9562_default_data\n");

	/* Set common default data first */
	common_default_data(plat);

	plat->bus_id = 1;
	plat->phy_addr = -1; 
    plat->pdev = pdev;
    
    if(ENABLE_RMII_INTERFACE == INTERFACE_SELECTED) {
        plat->interface = PHY_INTERFACE_MODE_RMII;        
    	plat->max_speed = 100;
    }
    
    if(ENABLE_RGMII_INTERFACE == INTERFACE_SELECTED) {
    	plat->interface = PHY_INTERFACE_MODE_RGMII;
		plat->max_speed = 1000;
	}
	
	if(ENABLE_SGMII_INTERFACE == INTERFACE_SELECTED) {
	    plat->interface = PHY_INTERFACE_MODE_SGMII;
    	plat->max_speed = 1000;
	}
        
    plat->tso_en = true;//tso feature for channel
#ifdef PTP_CHANGE
	plat->clk_ptp_rate = TC9562_TARGET_PTP_CLK;
#else
	plat->clk_ptp_rate = 58125000; /*Confirm value */
#endif

	plat->dma_cfg->txpbl = 16;
	plat->dma_cfg->rxpbl = 16;
	plat->dma_cfg->pblx8 = true;
	
	plat->axi->axi_lpi_en = 0;
	plat->axi->axi_xit_frm = 0;
	plat->axi->axi_blen[0] = 4;
	plat->axi->axi_blen[1] = 8;
	plat->axi->axi_blen[2] = 16;
	plat->axi->axi_rd_osr_lmt = 0x1F;
	plat->axi->axi_wr_osr_lmt = 0x1;

	/* EST Configuration */
	plat->est_cfg.enable = true;
	plat->est_cfg.btr_offset[0] = 0x0;
	plat->est_cfg.btr_offset[1] = 0x1; /* 1 second offset */
	plat->est_cfg.ctr[0] = snps_est_ctr[0];
	plat->est_cfg.ctr[1] = snps_est_ctr[1];
	plat->est_cfg.ter = 0x0;
	memcpy(plat->est_cfg.gcl, snps_est_gcl_entries,
			ARRAY_SIZE(snps_est_gcl_entries) * sizeof(u32));
	plat->est_cfg.gcl_size = ARRAY_SIZE(snps_est_gcl_entries);

	/* Frame Preemption Configuration */
#ifndef FPE
	plat->fp_en = false;//true based on phy/switch requirement
#else 
	plat->fp_en = true;//true based on phy/switch requirement
#endif 


	/* RX Parser Configuration */
#ifdef TC9562_FRP_ENABLE
	plat->rxp_cfg.enable = true;
#else
	plat->rxp_cfg.enable = false;
#endif
	plat->rxp_cfg.nve = ARRAY_SIZE(snps_rxp_entries);
	plat->rxp_cfg.npe = ARRAY_SIZE(snps_rxp_entries);
	memcpy(plat->rxp_cfg.entries, snps_rxp_entries,
			ARRAY_SIZE(snps_rxp_entries) *
			sizeof(struct tc9562mac_rx_parser_entry));

	/* MTL Configuration */
	/* Static Mapping */
	plat->rx_queues_cfg[0].chan = 2; /*static mapping */
#ifdef UNIFIED_DRIVER
	plat->rx_queues_cfg[1].chan = 4; /*static mapping */
#else
	plat->rx_queues_cfg[1].chan = 3; /*static mapping */
#endif
	plat->rx_queues_cfg[2].chan = 4; /* 4/3 : NA if dynamic(FRP enabled)*/
	plat->rx_queues_cfg[3].chan = 0; /* 0/1 : NA if dynamic*/

	/* MTL Scheduler for RX and TX */
	plat->rx_sched_algorithm = MTL_RX_ALGORITHM_SP;
	plat->tx_sched_algorithm = MTL_TX_ALGORITHM_WRR;

	plat->tx_queues_cfg[0].weight = 0x10;
	plat->tx_queues_cfg[1].weight = 0x11;
	plat->tx_queues_cfg[2].weight = 0x12;
	plat->tx_queues_cfg[3].weight = 0x13;
	plat->tx_queues_cfg[4].weight = 0x14;
	plat->tx_queues_cfg[5].weight = 0x15;

	/* RX Queue 0: RXCH2 - Untagged AVB Control packets + GPTP Packets */
	plat->rx_queues_cfg[0].mode_to_use = MTL_QUEUE_DCB;
	/* RX Queue 1: RXCH3 - AVB data packets */
	plat->rx_queues_cfg[1].mode_to_use = MTL_QUEUE_AVB;
	/* RX Queue 2: RXCH4 - AVB data traffic filtered on stream id */
	plat->rx_queues_cfg[2].mode_to_use = MTL_QUEUE_AVB;
	/* RX Queue 3: RXCH1 - Legacy traffic filtered on ethernet destination address CM3 + Broadcast/Multicast Packet */
	/* RX Queue 3: RXCH0 - Legacy traffic filtered on ethernet destination address PCIe + Broadcast/Multicast Packet +  packets mentioned other than above*/
	plat->rx_queues_cfg[3].mode_to_use = MTL_QUEUE_DCB;

	plat->tx_queues_cfg[0].mode_to_use = MTL_QUEUE_DCB;
	plat->tx_queues_cfg[1].mode_to_use = MTL_QUEUE_DCB;
	plat->tx_queues_cfg[2].mode_to_use = MTL_QUEUE_DCB;
	plat->tx_queues_cfg[3].mode_to_use = MTL_QUEUE_AVB;
	plat->tx_queues_cfg[4].mode_to_use = MTL_QUEUE_AVB;
	plat->tx_queues_cfg[5].mode_to_use = MTL_QUEUE_AVB;

	plat->tx_queues_cfg[0].percentage = 0;
	plat->tx_queues_cfg[1].percentage = 0;
	plat->tx_queues_cfg[2].percentage = 0;
	plat->tx_queues_cfg[3].percentage = 0;

	
	/* CSB: queue 3 -> Class B traffic (25% BW) */
	/* plat->tx_queues_cfg[3].idle_slope = plat->est_cfg.enable ? 0x8e4 : 0x800; */
	plat->tx_queues_cfg[3].idle_slope = 0x800;
	plat->tx_queues_cfg[3].send_slope = 0x1800;
	plat->tx_queues_cfg[3].high_credit = 0x320000;
	plat->tx_queues_cfg[3].low_credit = 0xff6a0000;
	plat->tx_queues_cfg[3].percentage = 25;

	/* CSB: queue 5 -> Class CDT traffic (25% BW) */
	/* plat->tx_queues_cfg[5].idle_slope = plat->est_cfg.enable ? 0x8e4 : 0x800; */
	plat->tx_queues_cfg[5].idle_slope = 0x800;
	plat->tx_queues_cfg[5].send_slope = 0x1800;
	plat->tx_queues_cfg[5].high_credit = 0x320000;
	plat->tx_queues_cfg[5].low_credit = 0xff6a0000;
	plat->tx_queues_cfg[5].percentage = 25;

	/* CSB: queue 4 -> Class A traffic (40%) BW */
	/* plat->tx_queues_cfg[4].idle_slope = plat->est_cfg.enable ? 0xe38 : 0xccc; */
	plat->tx_queues_cfg[4].idle_slope = 0xccc;
	plat->tx_queues_cfg[4].send_slope = 0x1333;
	plat->tx_queues_cfg[4].high_credit = 0x500000;
	plat->tx_queues_cfg[4].low_credit = 0xff880000;
	plat->tx_queues_cfg[4].percentage = 40;
	
	/* Set default number of RX and TX queues to use */
	plat->tx_queues_to_use = 6;
	plat->rx_queues_to_use = 4;

	/* Disable Priority config by default */
	plat->tx_queues_cfg[0].use_prio = false;
	plat->tx_queues_cfg[1].use_prio = false;
	plat->tx_queues_cfg[2].use_prio = false;
	plat->tx_queues_cfg[3].use_prio = false;
	plat->tx_queues_cfg[4].use_prio = false;
	plat->tx_queues_cfg[5].use_prio = false;

	plat->rx_queues_cfg[0].use_prio = false;
  
#if defined(RX_LOGGING_TRACE) || defined(ENABLE_TSN)
	plat->rx_queues_cfg[1].use_prio = true;
	plat->rx_queues_cfg[1].prio = (1 << TC9562_AVB_PRIORITY_CLASS_A) | (1 << TC9562_AVB_PRIORITY_CLASS_B); /* VLAN prio 3 */
	plat->rx_queues_cfg[2].use_prio = true;
	plat->rx_queues_cfg[2].prio = (1 << TC9562_PRIORITY_CLASS_CDT); /* VLAN prio 7 */
#else
	plat->rx_queues_cfg[1].use_prio = true;
	plat->rx_queues_cfg[1].prio = (1 << TC9562_AVB_PRIORITY_CLASS_B); /* VLAN prio 2 */
	plat->rx_queues_cfg[2].use_prio = true;
	plat->rx_queues_cfg[2].prio = (1 << TC9562_AVB_PRIORITY_CLASS_A); /* VLAN prio 3 */
#endif
    plat->rx_queues_cfg[3].use_prio = true;
	plat->rx_queues_cfg[3].prio = 0xFF ; /* All tagged legacy packets must be routed to RxQ3 */

#ifdef UNIFIED_DRIVER
	plat->rx_dma_ch_for_host[0] = 1;
	plat->rx_dma_ch_for_host[1] = 0;
	plat->rx_dma_ch_for_host[2] = 0;
	plat->rx_dma_ch_for_host[3] = 0;
	plat->rx_dma_ch_for_host[4] = 1;
	plat->rx_dma_ch_for_host[5] = 0;

	plat->tx_dma_ch_for_host[0] = 1;
	plat->tx_dma_ch_for_host[1] = 0;
	plat->tx_dma_ch_for_host[2] = 0;
	plat->tx_dma_ch_for_host[3] = 1;
	plat->tx_dma_ch_for_host[4] = 1;
	plat->tx_dma_ch_for_host[5] = 0;
#endif
	DBGPR_FUNC("<--tc9562_default_data\n");
	return 0;
}

static void tc9562_pcie_shutdown(struct pci_dev *pdev)
{
	DBGPR_FUNC( "-->tc9562_pcie_shutdown\n");
	NMSGPR_ALERT( "Handle the shutdown\n");
	DBGPR_FUNC( "<--tc9562_pcie_shutdown\n");

	return;
}

static int tc9562_pcie_suspend_late(struct pci_dev *pdev, pm_message_t state)
{
	DBGPR_FUNC( "-->tc9562_PCIe_suspend_late\n");
	NMSGPR_ALERT( "Handle the suspend_late\n");
	DBGPR_FUNC( "<--tc9562_pcie_suspend_late\n");

	return 0;
}

static int tc9562_pcie_resume_early(struct pci_dev *pdev)
{
	DBGPR_FUNC( "-->tc9562_pcie_resume_early\n");
	NMSGPR_ALERT( "Handle the resume_early\n");
	DBGPR_FUNC( "<--tc9562_pcie_resume_early\n");

	return 0;
}

#ifdef CONFIG_PM
/*!
 * \brief Routine to put the device in suspend mode
 *
 * \details This function gets called by PCI core when the device is being
 * suspended. The suspended state is passed as input argument to it.
 * Following operations are performed in this function,
 * - stop the phy.
 * - detach the device from stack.
 * - stop the queue.
 * - Disable napi.
 * - Stop DMA TX and RX process.
 * - Enable power down mode using PMT module or disable MAC TX and RX process.
 * - Save the pci state.
 *
 * \param[in] pdev \96 pointer to pci device structure.
 * \param[in] state \96 suspend state of device.
 *
 * \return int
 *
 * \retval 0
 */

static int tc9562_pcie_suspend(struct pci_dev *pdev, pm_message_t state)
{
	DBGPR_FUNC("-->tc9562_pcie_suspend\n");

	tc9562mac_suspend(&pdev->dev);
	pci_save_state(pdev);
	pci_set_power_state(pdev, pci_choose_state(pdev, state));

	DBGPR_FUNC("<--tc9562_pcie_suspend\n");

	return 0;
}

void tc9562_config_tamap(void __iomem *reg_pci_base_addr)
{
	u64 adrs, replacement_adrs;
	unsigned int tmap_no, varOFFSET_ADR_UP, varOFFSET_ADR_DW, varOFFSET_EN;
	unsigned int varOFFSET_REPLACE_UP, varOFFSET_REPLACE_DW, varOFFSET_OW;

	DBGPR_FUNC("-->tc9562_config_tamap \n");
	/* Configure TMAP 0 to access full range of host memory */
	tmap_no = 0;
	adrs = ( (unsigned long long)0x00000010 << 32);
	replacement_adrs = ( (unsigned long long)0x00000000 << 32);
	
	varOFFSET_ADR_UP = adrs>>32;
	varOFFSET_ADR_DW = adrs&0xFFFFF000;
	varOFFSET_EN = 1;
	varOFFSET_REPLACE_UP = replacement_adrs>>32;
	varOFFSET_REPLACE_DW = replacement_adrs&0xFFFFF000;
	varOFFSET_OW = 28;
	
	if(varOFFSET_OW >= 53) //53 is the max limit defined in spec
		NMSGPR_ALERT("No of mask bit error, HW cannot support %d mask bits\n", varOFFSET_OW);

	writel(varOFFSET_ADR_UP, reg_pci_base_addr + PCIE_RANGE_UP_OFFSET_RgOffAddr(tmap_no));
	writel(varOFFSET_ADR_DW | varOFFSET_EN, reg_pci_base_addr + PCIE_RANGE_EN_RgOffAddr(tmap_no));
	writel(varOFFSET_REPLACE_UP, reg_pci_base_addr + PCIE_RANGE_UP_RPLC_RgOffAddr(tmap_no));
	writel(varOFFSET_REPLACE_DW | varOFFSET_OW, reg_pci_base_addr + PCIE_RANGE_WIDTH_RgOffAddr(tmap_no));\

	NDBGPR_L1("RANGE_UP_OFFSET[%d] = 0x%08x\n", tmap_no, readl(reg_pci_base_addr + PCIE_RANGE_UP_OFFSET_RgOffAddr(tmap_no)));
	NDBGPR_L1("RANGE_EN[%d] = 0x%08x\n", tmap_no, readl(reg_pci_base_addr + PCIE_RANGE_EN_RgOffAddr(tmap_no)));
	NDBGPR_L1("RANGE_UP_RPLC[%d] = 0x%08x\n", tmap_no, readl(reg_pci_base_addr + PCIE_RANGE_UP_RPLC_RgOffAddr(tmap_no)));
	NDBGPR_L1("RANGE_WIDTH[%d] = 0x%08x\n", tmap_no, readl(reg_pci_base_addr + PCIE_RANGE_WIDTH_RgOffAddr(tmap_no)));
	
	DBGPR_FUNC("<--tc9562_config_tamap \n");
}


void tc9562_configure(void __iomem *reg_pci_base_addr)
{
	unsigned int rd_val = 0;
	DBGPR_FUNC("--> tc9562_configure \n");
		
	rd_val = readl(reg_pci_base_addr + NCLKCTRL_OFFSET);
	if (ENABLE_SGMII_INTERFACE == INTERFACE_SELECTED)
	{
		writel((rd_val | 0x0300ffff), reg_pci_base_addr + NCLKCTRL_OFFSET);//For Bring up  // SGMII
	}
	else
	{
		/*Change according to firmware */
		//writel((rd_val | 0x6290), reg_pci_base_addr + NCLKCTRL_OFFSET); /* SRAMCEN | MACTXCEN | PCIECEN | INTCEN | MACRXCEN*/BIT 24 ?
		writel((rd_val | 0xffff), reg_pci_base_addr + NCLKCTRL_OFFSET);//For Bring up
	}
	
	rd_val = readl(reg_pci_base_addr + NRSTCTRL_OFFSET);
	if (ENABLE_SGMII_INTERFACE == INTERFACE_SELECTED)
	{
		writel((rd_val & ~0x02000090), reg_pci_base_addr + NRSTCTRL_OFFSET); /*  SGMRST | MACRST | INTRST */   //for  SGMII
	}
	else
	{
		writel((rd_val & ~0x0090), reg_pci_base_addr + NRSTCTRL_OFFSET); /*  MACRST | INTRST */
	}

  //SGMII
	if (ENABLE_SGMII_INTERFACE == INTERFACE_SELECTED)
	{
        // for SGMII config   	
	rd_val = readl(reg_pci_base_addr + 0xc088);
	writel((rd_val & ~0x00000071), reg_pci_base_addr + 0xc088);  

	rd_val = readl(reg_pci_base_addr + 0xc0d0);
	writel((rd_val & ~0x00000001), reg_pci_base_addr + 0xc0d0);  

	rd_val = readl(reg_pci_base_addr + 0xc094);
	writel((rd_val & ~0x00000001), reg_pci_base_addr + 0xc094);  

	rd_val = readl(reg_pci_base_addr + 0xc0e4);
	writel(((rd_val & ~0x00000007) | 0x1) , reg_pci_base_addr + 0xc0e4);  

	rd_val = readl(reg_pci_base_addr + 0xc008);
	writel(((rd_val & ~0x0000001f) | 0x8), reg_pci_base_addr + 0xc008);  


	rd_val = readl(reg_pci_base_addr + 0xc020);
	writel(((rd_val & ~0x0000000f) | 0x2), reg_pci_base_addr + 0xc020);  
	
	rd_val = readl(reg_pci_base_addr + 0xc0d4);
	writel(((rd_val & ~0x000000ff) | 0x12), reg_pci_base_addr + 0xc0d4); 

	rd_val = readl(reg_pci_base_addr + 0xc02c);
	writel(((rd_val & ~0x0000007f) | 0x32), reg_pci_base_addr + 0xc02c);  

	rd_val = readl(reg_pci_base_addr + 0xc0ec);
	writel(((rd_val & ~0x0000ffff) | 0x800), reg_pci_base_addr + 0xc0ec);  

	rd_val = readl(reg_pci_base_addr + 0xc000);
	writel(((rd_val & ~0x00000001) | 0x1), reg_pci_base_addr + 0xc000);  

	rd_val = readl(reg_pci_base_addr + 0xc000);
	writel(((rd_val & ~0x00000010) | 0x10), reg_pci_base_addr + 0xc000);  

	rd_val = readl(reg_pci_base_addr + 0xc004);
	writel(((rd_val & ~0x00000001) | 0x1), reg_pci_base_addr + 0xc004);  

	udelay(15);
	rd_val = readl(reg_pci_base_addr + 0xc014);
	writel(((rd_val & ~0x00000001) | 0x1), reg_pci_base_addr + 0xc014);  

	rd_val = readl(reg_pci_base_addr + 0xc0c0);
	writel(((rd_val & ~0x00000001) | 0x1), reg_pci_base_addr + 0xc0c0);  

 	udelay(1);
	rd_val = readl(reg_pci_base_addr + 0xc0c8);
	writel(((rd_val & ~0x00000001) | 0x1), reg_pci_base_addr + 0xc0c8);  

	udelay(19);
	rd_val = readl(reg_pci_base_addr + 0xc014);
	writel(((rd_val & ~0x00000010) | 0x10), reg_pci_base_addr + 0xc014);  

	ndelay(80);
	rd_val = readl(reg_pci_base_addr + 0xc080);
	writel(((rd_val & ~0x00000001) | 0x1), reg_pci_base_addr + 0xc080);  

	
	ndelay(80);
	rd_val = readl(reg_pci_base_addr + 0xc084);
	writel((rd_val & ~0x00000001) , reg_pci_base_addr + 0xc084);  

	rd_val = readl(reg_pci_base_addr + 0xc0cc);
	writel((rd_val & ~0x00000001) , reg_pci_base_addr + 0xc0cc);  
	}


	tc9562_config_tamap(reg_pci_base_addr);
	DBGPR_FUNC("<-- tc9562_configure \n");
}

/*!
 * \brief Routine to resume device operation
 *
 * \details This function gets called by PCI core when the device is being
 * resumed. It is always called after suspend has been called. These function
 * reverse operations performed at suspend time. Following operations are
 * performed in this function,
 * - restores the saved pci power state.
 * - Wakeup the device using PMT module if supported.
 * - Starts the phy.
 * - Enable MAC and DMA TX and RX process.
 * - Attach the device to stack.
 * - Enable napi.
 * - Starts the queue.
 *
 * \param[in] pdev – pointer to pci device structure.
 *
 * \return int
 *
 * \retval 0
 */

static int tc9562_pcie_resume(struct pci_dev *pdev)
{
	int ret = 0;
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct tc9562mac_priv *priv = netdev_priv(ndev);
	
	DBGPR_FUNC("-->tc9562_pcie_resume\n");

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	tc9562_configure(priv->ioaddr);
	tc9562mac_resume(&pdev->dev);
	
	DBGPR_FUNC("<--tc9562_pcie_resume\n");

	return ret;
}

#endif	/* CONFIG_PM */

static const struct tc9562mac_pci_info tc9562_pci_info = {
	.setup = tc9562_default_data,
};

/*!
* \brief API to initialize the device.
*
* \details This probing function gets called (during execution of
* pci_register_driver() for already existing devices or later if a
* new device gets inserted) for all PCI devices which match the ID table
* and are not "owned" by the other drivers yet. This function gets passed
* a "struct pci_dev *" for each device whose entry in the ID table matches
* the device. The probe function returns zero when the driver chooses to take
* "ownership" of the device or an error code (negative number) otherwise.
* The probe function always gets called from process context, so it can sleep.
*
* \param[in] pdev - pointer to pci_dev structure.
* \param[in] id   - pointer to table of device ID/ID's the driver is inerested.
*
* \return integer
*
* \retval 0 on success & -ve number on failure.
*/
static int tc9562_pci_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	
	struct tc9562mac_pci_info *info = (struct tc9562mac_pci_info *)id->driver_data;
	struct plat_tc9562macenet_data *plat = NULL;
	struct tc9562mac_resources res;
	int ret = 0;
	char version_str[32];
	tc9562_version_t *fw_version;
	int reg;
	uint8_t SgmSigPol = 1; /* To handle SGM_SIG_DET */
	
	DBGPR_FUNC("-->tc9562_pci_probe\n");
	
	sprintf(version_str, "Host Driver Version %s_%d.%d-%d", (tc9562_version.rel_dbg == 'D')?"DBG":"REL", tc9562_version.major, tc9562_version.minor, tc9562_version.sub_minor);
	NMSGPR_INFO("%s\n", version_str);	

	memset(&res, 0, sizeof(res));		
	plat = devm_kzalloc(&pdev->dev, sizeof(*plat), GFP_KERNEL);
	if (!plat)
		return -ENOMEM;

	plat->mdio_bus_data = devm_kzalloc(&pdev->dev,
					   sizeof(*plat->mdio_bus_data),
					   GFP_KERNEL);
	if (!plat->mdio_bus_data)
		return -ENOMEM;

	plat->dma_cfg = devm_kzalloc(&pdev->dev, sizeof(*plat->dma_cfg),
					 GFP_KERNEL);
	if (!plat->dma_cfg)
		return -ENOMEM;

	if (!plat->axi) {
		plat->axi = kzalloc(sizeof(struct tc9562mac_axi), GFP_KERNEL);

		if (!plat->axi)
			return -ENOMEM;
	}

	/* Enable pci device */
	ret = pci_enable_device(pdev);
	if (ret) {
		NMSGPR_ALERT("%s:Unable to enable device\n", TC9562_RESOURCE_NAME);
		goto err_out_enb_failed;	
	}
	
	/* Query and set the appropriate masks for DMA operations. */
    if ((pci_set_dma_mask(pdev, DMA_BIT_MASK(64))) &&
	    (pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64)))) {
        	NMSGPR_ALERT("%s: 64 bits DMA Configuration not supported, aborting\n",
			             pci_name(pdev));
			ret = -ENODEV;
			goto err_out_dma_mask_failed;
    }

	if (pci_request_regions(pdev, TC9562_RESOURCE_NAME)) {
			NMSGPR_ALERT("%s:Failed to get PCI regions\n", TC9562_RESOURCE_NAME);
			ret = -ENODEV;
			goto err_out_req_reg_failed;
	}
		
	pci_set_master(pdev);
		
	/* Read BAR0 and map the TC9562 register base address
	Read BAR1 and map the TC9562 SRAM memory address
	Read BAR2 and map the TC9562 Flash memory address */
	NDBGPR_L1("BAR0 length = %lld kb\n", pci_resource_len(pdev, 0));
	NDBGPR_L1("BAR2 length = %lld kb\n", pci_resource_len(pdev, 2));
	NDBGPR_L1("BAR4 length = %lld kb\n", pci_resource_len(pdev, 4));
	NDBGPR_L1("BAR0 iommu address = 0x%llx\n", pci_resource_start(pdev, 0));
	NDBGPR_L1("BAR2 iommu address = 0x%llx\n", pci_resource_start(pdev, 2));
	NDBGPR_L1("BAR4 iommu address = 0x%llx\n", pci_resource_start(pdev, 4));
	
	res.addr = ioremap_nocache(pci_resource_start(pdev, 0), pci_resource_len(pdev, 0));
	
	if (((void __iomem *)res.addr == NULL)) {
        NMSGPR_ALERT( "%s: cannot map TC9562 BAR0, aborting", pci_name(pdev));
        ret = -EIO;
        goto err_out_map_failed;
    }

	res.tc9562_SRAM_pci_base_addr = ioremap_nocache(pci_resource_start(pdev, 2), pci_resource_len(pdev, 2));
	
	if (((void __iomem *)res.tc9562_SRAM_pci_base_addr == NULL)) {
		pci_iounmap(pdev, (void __iomem *)res.addr);
        NMSGPR_ALERT( "%s: cannot map TC9562 BAR2, aborting", pci_name(pdev));
        ret = -EIO;
        goto err_out_map_failed;
    }
	
	res.tc9562_FLASH_pci_base_addr = ioremap_nocache(pci_resource_start(pdev, 4), pci_resource_len(pdev, 4));
	
	if (((void __iomem *)res.tc9562_FLASH_pci_base_addr == NULL)) {
		pci_iounmap(pdev, (void __iomem *)res.addr);
		pci_iounmap(pdev, (void __iomem *)res.tc9562_SRAM_pci_base_addr);
		NMSGPR_ALERT( "%s: cannot map TC9562 BAR4, aborting", pci_name(pdev));
        ret = -EIO;
        goto err_out_map_failed;
    }
	
	NDBGPR_L1("BAR0 virtual address = %p\n", res.addr);
	NDBGPR_L1("BAR2 virtual address = %p\n", res.tc9562_SRAM_pci_base_addr);
	NDBGPR_L1("BAR4 virtual address = %p\n", res.tc9562_FLASH_pci_base_addr);

	ret = info->setup(pdev, plat);
	if (ret)
		return ret;
	/* pci_set_drvdata(pdev, ); */
	
	tc9562_configure(res.addr);
	/* Assertion of EMAC software Reset*/
	ret = readl(res.addr + 0x1008);
	ret |= (0x1 << 7);
	writel(ret, res.addr + 0x1008);

	if (ENABLE_SGMII_INTERFACE == INTERFACE_SELECTED)
	{
    	if(1 == ENABLE_SGM_SIG_DET){
			ret = readl(res.addr + 0x1524);
			ret = (ret & ~0x00f00000) | 0x00200000;
			writel(ret, res.addr + 0x1524);
			
			SgmSigPol = 0; /* Active High */
		}else if(2 == ENABLE_SGM_SIG_DET){
		    ret = readl(res.addr + 0x1524);
		    ret = (ret & ~0x00f00000) | 0x00200000;
		    writel(ret, res.addr + 0x1524);	
			SgmSigPol = 1; /* Active low */
		}
	}

	/* Interface configuration */
	ret = readl(res.addr + 0x0010);
	ret &= 0xffffffc7;
	if(ENABLE_RMII_INTERFACE == INTERFACE_SELECTED)
		ret |= 0x20;
	else if(ENABLE_RGMII_INTERFACE == INTERFACE_SELECTED)
		ret |= 0x8;		
 	else if(ENABLE_SGMII_INTERFACE == INTERFACE_SELECTED)
		ret |= 0x10;
	ret &= ~(0x00000800); /* Mask Polarity */
	if(1 == SgmSigPol){
		ret |= 0x00000800; /* Set Active low */
	}

    writel(ret, res.addr + 0x0010);

	/* De-assertion of EMAC software Reset*/
	ret = readl(res.addr + 0x1008);
	ret &= 0xFFFFFF7F;
	writel(ret, res.addr + 0x1008);	
	
	ret = pci_enable_msi(pdev);
	if (ret) {
		NMSGPR_ALERT("%s:Enable MSI error\n", TC9562_RESOURCE_NAME);
		goto err_out_msi_failed;
	}
	pci_write_config_dword(pdev, pdev->msi_cap + PCI_MSI_MASK_64, 0);

	res.wol_irq = pdev->irq;
	res.irq = pdev->irq;
	NDBGPR_L1("Allocated IRQ Number = %d\n", res.irq);

	ret = tc9562mac_dvr_probe(&pdev->dev, plat, &res);
	if(ret) {
    	DBGPR_FUNC("<--tc9562_pci_probe : ret: %d\n", ret);
    	goto err_dvr_probe;
    }
    
	reg = readl(res.tc9562_SRAM_pci_base_addr + 0x4f900);
	fw_version = (struct tc9562_version_s *)(&reg);
	sprintf(version_str, "Firmware Version %s_%d.%d-%d", (fw_version->rel_dbg == 'D')?"DBG":"REL", fw_version->major, fw_version->minor, fw_version->sub_minor);
	NMSGPR_INFO("%s\n", version_str);
	
	return ret;
	
err_out_msi_failed:
err_dvr_probe:
    pci_disable_msi(pdev);
	if (((void __iomem *)res.addr != NULL))
		pci_iounmap(pdev, (void __iomem *)res.addr);
	if (((void __iomem *)res.tc9562_SRAM_pci_base_addr != NULL))
		pci_iounmap(pdev, (void __iomem *)res.tc9562_SRAM_pci_base_addr);
	if (((void __iomem *)res.tc9562_FLASH_pci_base_addr != NULL))
		pci_iounmap(pdev, (void __iomem *)res.tc9562_FLASH_pci_base_addr);
err_out_map_failed:
	pci_release_regions(pdev);
err_out_dma_mask_failed:
err_out_req_reg_failed:
	pci_disable_device(pdev);
err_out_enb_failed:
	return ret;
}

/**
 * tc9562_pci_remove
 *
 * \brief API to release all the resources from the driver.
*
* \details The remove function gets called whenever a device being handled
* by this driver is removed (either during deregistration of the driver or
* when it is manually pulled out of a hot-pluggable slot). This function
* should reverse operations performed at probe time. The remove function
* always gets called from process context, so it can sleep.
*
* \param[in] pdev - pointer to pci_dev structure.
*
* \return void
 */
static void tc9562_pci_remove(struct pci_dev *pdev)
{
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct tc9562mac_priv *priv = netdev_priv(ndev);

	DBGPR_FUNC("-->tc9562_pci_remove\n");
	tc9562mac_dvr_remove(&pdev->dev);

	pdev->irq = 0;
	pci_disable_msi(pdev);
	if ((void __iomem *)priv->ioaddr != NULL)
        pci_iounmap(pdev, (void __iomem *)priv->ioaddr);
	if ((void __iomem *)priv->tc9562_SRAM_pci_base_addr != NULL)
		pci_iounmap(pdev, (void __iomem *)priv->tc9562_SRAM_pci_base_addr);
	if ((void __iomem *)priv->tc9562_FLASH_pci_base_addr != NULL)
		pci_iounmap(pdev, (void __iomem *)priv->tc9562_FLASH_pci_base_addr);
	
	/* pci_set_drvdata(pdev, NULL); */
	pci_release_regions(pdev);
	pci_disable_device(pdev);
		
	DBGPR_FUNC("<--tc9562_pci_remove\n");

	return;
}

static struct pci_device_id tc9562_id_table = {
    PCI_DEVICE_CLASS(PCI_ETHC_CLASS_CODE, 0xffffff),
    PCI_DEVICE(VENDOR_ID, DEVICE_ID),
    .driver_data = (kernel_ulong_t)&tc9562_pci_info
};

static struct pci_driver tc9562_pci_driver = {
	.name			= TC9562_RESOURCE_NAME,
	.id_table		= &tc9562_id_table,
	.probe			= tc9562_pci_probe,
	.remove			= tc9562_pci_remove,
	.shutdown		= tc9562_pcie_shutdown,
	.suspend_late	= tc9562_pcie_suspend_late,
	.resume_early	= tc9562_pcie_resume_early,
#ifdef CONFIG_PM
	.suspend		= tc9562_pcie_suspend,
	.resume			= tc9562_pcie_resume,
#endif

	.driver			= {
		.name		= TC9562_RESOURCE_NAME,
		.owner		= THIS_MODULE,
	},
};

/*!
* \brief API to register the driver.
*
* \details This is the first function called when the driver is loaded.
* It register the driver with PCI sub-system
*
* \return void.
*/

static int __init tc9562_init_module(void)
{
	int ret = 0;

	DBGPR_FUNC("-->tc9562_init_module\n");

	ret = pci_register_driver(&tc9562_pci_driver);
	if (ret < 0) {
		NMSGPR_ERR("TC9562 : Driver registration failed");
		return ret;
	}

	tc9562mac_init();

	DBGPR_FUNC("<--tc9562_init_module\n");

	return ret;
}

/*!
* \brief API to unregister the driver.
*
* \details This is the first function called when the driver is removed.
* It unregister the driver from PCI sub-system
*
* \return void.
*/

static void __exit tc9562_exit_module(void)
{
	DBGPR_FUNC("-->tc9562_exit_module\n");

	pci_unregister_driver(&tc9562_pci_driver);
	tc9562mac_exit();

	DBGPR_FUNC("<--tc9562_exit_module\n");
}

/*!
* \brief Macro to register the driver registration function.
*
* \details A module always begin with either the init_module or the function
* you specify with module_init call. This is the entry function for modules;
* it tells the kernel what functionality the module provides and sets up the
* kernel to run the module's functions when they're needed. Once it does this,
* entry function returns and the module does nothing until the kernel wants
* to do something with the code that the module provides.
*/
module_init(tc9562_init_module);

/*!
* \brief Macro to register the driver un-registration function.
*
* \details All modules end by calling either cleanup_module or the function
* you specify with the module_exit call. This is the exit function for modules;
* it undoes whatever entry function did. It unregisters the functionality
* that the entry function registered.
*/
module_exit(tc9562_exit_module);

MODULE_DESCRIPTION("TC9562 PCI Ethernet driver");
MODULE_AUTHOR("TAEC/TDSC");
MODULE_LICENSE("GPL");
