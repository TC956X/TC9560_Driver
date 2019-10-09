/*
 * TC9562 ethernet driver.
 *
 * common.h 
 *
 * Copyright (C) 2007-2009 STMicroelectronics Ltd
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

#ifndef __COMMON_H__
#define __COMMON_H__

#include <linux/etherdevice.h>
#include <linux/netdevice.h>
//#include <linux/tc9562mac.h>
#include "tc9562mac_inc.h"
#include <linux/phy.h>
#include <linux/module.h>
#include <linux/version.h>
#if IS_ENABLED(CONFIG_VLAN_8021Q)
#define TC9562MAC_VLAN_TAG_USED
#include <linux/if_vlan.h>
#endif

#include "descs.h"
#include "mmc.h"


/* UnSupported and UnTested TC9562 Feature macro */

#define TC9562_UNSUPPORTED_UNTESTED_FEATURE

/* Configuration macros of TC9562 */

#define VENDOR_ID 0x1179	
#define DEVICE_ID 0x021f

#define PTP_CHANGE

/* skip the addresses added during hw setup*/
/*To do: If source address (SA) replacement or SA insertion feature is supported, MAC_ADDR_ADD_SKIP_OFST should be increased accordingly */
#define MAC_ADDR_ADD_SKIP_OFST 3
//#define VERIFY_PPO_USING_AUX


/* PCI new class code  */
#define PCI_ETHC_CLASS_CODE	0x020000
#define TC9562_DEFINED
#define HOST_PHYSICAL_ADRS_MASK	(0x10) // bit no 37: (1<<36)
//#define CL45_PHY //Enable for CL45 PHY
//#define TC9562_POLLING_METHOD
#ifdef TC9562_POLLING_METHOD
#define TC9562_POLL_DELAY_US	    (1000) //Any value less then 1000 will be considered as 1000us
#endif

//#define TC9562_DUMP_MAC_REG /* Dump MAC registers */

//#define TC9562_LOAD_FW_HEADER
#define FIRMWARE_NAME "TC9562_Firmware_PCIeBridge.bin"

/* C45 registers */
#define PHY_CL45_CTRL_REG_MMD_BANK      (7) /* CL45_CTRL_REG is the equivalent of CL22 BMCR */
#define PHY_CL45_CTRL_REG_ADDR          (0x200)

#define PHY_CL45_STATUS_REG_MMD_BANK    (7) /* CL45_STATUS_REG is the equivalent of CL22 BMSR */
#define PHY_CL45_STATUS_REG_ADDR        (0x201)

#define PHY_CL45_PHYID1_MMD_BANK        (1) /* CL45_PHYID1 is the equivalent of CL22 PHYIDR1 */
#define PHY_CL45_PHYID1_ADDR            (2)

#define PHY_CL45_PHYID2_MMD_BANK        (1) /* CL45_PHYID1 is the equivalent of CL22 PHYIDR2 */
#define PHY_CL45_PHYID2_ADDR            (3)

#define PHY_CL45_PHYID1_REG ((PHY_CL45_PHYID1_MMD_BANK << 16) | (PHY_CL45_PHYID1_ADDR))
#define PHY_CL45_PHYID2_REG ((PHY_CL45_PHYID2_MMD_BANK << 16) | (PHY_CL45_PHYID2_ADDR))

//#define ENABLE_TSN
//#define TX_LOGGING_TRACE	1
//#define RX_LOGGING_TRACE	1

#if defined(TX_LOGGING_TRACE)
#define PACKET_IPG      125000
#define PACKET_CDT_IPG  500000
#endif

//#define TC9562_TX_TIMESTAMP_ALL

/* Debug prints */
#define NMSGPR_INFO(x...)  printk(KERN_INFO x)
#define NMSGPR_ALERT(x...) printk(KERN_ALERT x)
#define NMSGPR_ERR(x...)   printk(KERN_ERR x)

//#define TC9562_DBG_FUNC
//#define TC9562_DBG_PTP
//#define TC9562_DBG_TSN
//#define TC9562_DBG_MDIO
//#define TC9562_DBG_ETHTOOL
//#define TC9562_DBG_L1
//#define TC9562_DBG_L2
//#define TC9562_TEST
//#define TC9562_MSI_GEN_SW_AGENT /*Macro to enable and handle SW MSI interrupt*/
//#define TC9562_TEST_RXCH1_FRP_DISABLED
//#define TC9562_PKT_DUP
//#define TC9562_FRP_ENABLE
#ifdef TC9562_DBG_FUNC
#define DBGPR_FUNC(x...) printk(KERN_ALERT x)
#else
#define DBGPR_FUNC(x...) do { } while (0)
#endif
#ifdef TC9562_TEST
#define DBGPR_TEST(x...) printk(KERN_ALERT x)
#else
#define DBGPR_TEST(x...) do { } while (0)
#endif

#ifdef TC9562_DBG_PTP
#define DBGPR_FUNC_PTP(x...) printk(KERN_ALERT x)
#else
#define DBGPR_FUNC_PTP(x...) do { } while (0)
#endif

#ifdef TC9562_DBG_TSN
#define DBGPR_FUNC_TSN(x...) printk(KERN_ALERT x)
#else
#define DBGPR_FUNC_TSN(x...) do { } while (0)
#endif

#ifdef TC9562_DBG_MDIO
#define DBGPR_FUNC_MDIO(x...) printk(KERN_ALERT x)
#else
#define DBGPR_FUNC_MDIO(x...) do { } while (0)
#endif

#ifdef TC9562_DBG_ETHTOOL
#define DBGPR_FUNC_ETHTOOL(x...) printk(KERN_ALERT x)
#else
#define DBGPR_FUNC_ETHTOOL(x...) do { } while (0)
#endif

#ifdef TC9562_DBG_L1
#define NDBGPR_L1(x...) printk(KERN_DEBUG x)
#else
#define NDBGPR_L1(x...) do { } while (0)
#endif

#ifdef TC9562_DBG_L2
#define NDBGPR_L2(x...) printk(KERN_DEBUG x)
#else
#define NDBGPR_L2(x...) do { } while (0)
#endif

/*	MAC_OFFSET
	Synopsys IP MAC Block	: 0x0_4000_A000	- 0x0_4000_ABFF
	Synopsys IP MTL Block	: 0x0_4000_AC00	- 0x0_4000_AFFF
	Synopsys IP DMA Block	: 0x0_4000_B000	- 0x0_4000_B4FF
*/
#define  MAC_OFFSET						0xA000	

#define NCID_OFFSET       				(0x0000)      /* TC9562 Chip and revision ID       */
#define NMODESTS_OFFSET   				(0x0004)      /* TC9562 current operation mode     */
#define NFUNCEN0_OFFSET   				(0x0008)      /* TC9562 pin mux control            */
#define NQSPIDIV_OFFSET   				(0x000C)      /* TC9562 QSPI divider setting for serial clock  */
#define NEMACCTL_OFFSET   				(0x0010)      /* TC9562 eMAC div and control       */ 
#define NSLEEPCTR_OFFSET   				(0x0014)
#define NPCIEBOOT_OFFSET   				(0x0018)
#define NSECCTRL_OFFSET   				(0x001C)
#define NPCIECLKREQDLY_OFFSET   		(0x0020)

#define NCTLSTS_OFFSET    				(0x1000)      /* TC9562 control and status         */
#define	NCLKCTRL_OFFSET					(0x1004)      /* TC9562 clock control              */
#define	NRSTCTRL_OFFSET					(0x1008)      /* TC9562 reset control              */
#define NBUSCTRL_OFFSET   				(0x0014)

#define NSPLLPARAM_OFFSET 				(0x1020)      /* TC9562 System PLL parameters      */
#define NSPLLUPDT_OFFSET  				(0x1024)      /* TC9562 System PLL update          */

#define NEMACTXCDLY_OFFSET   			(0x1050)
#define NEMACINTO0EN_OFFSET   			(0x1054)
#define NEMACINTO1EN_OFFSET   			(0x1058)

#define GPIOI0_OFFSET   				(0x1200)
#define GPIOI1_OFFSET   				(0x1204)
#define GPIOE0_OFFSET   				(0x1208)
#define GPIOE1_OFFSET   				(0x120C)
#define GPIOO0_OFFSET   				(0x1210)
#define GPIOO1_OFFSET   				(0x1214)
#define NPCIEPWR_OFFSET   				(0x1300)
#define I2CERRADD_OFFSET   				(0x1400)
#define SPIERRADD_OFFSET   				(0x1404)
#define NFUNCEN1_OFFSET   				(0x1514)
#define NFUNCEN2_OFFSET   				(0x151C)
#define NFUNCEN3_OFFSET   				(0x1524)
#define NFUNCEN4_OFFSET   				(0x1528)
#define NFUNCEN5_OFFSET   				(0x152C)
#define NFUNCEN6_OFFSET   				(0x1530)
#define NFUNCEN7_OFFSET   				(0x153C)

#define NIOCFG1_OFFSET   				(0x1614)
#define NIOCFG2_OFFSET   				(0x161C)
#define NIOCFG3_OFFSET   				(0x1624)
#define NIOCFG4_OFFSET   				(0x1628)
#define NIOCFG7_OFFSET   				(0x163C)

#define NIOEN1_OFFSET   				(0x1714)
#define NIOEN2_OFFSET   				(0x171C)
#define NIOEN3_OFFSET   				(0x1724)
#define NIOEN4_OFFSET   				(0x1728)
#define NIOEN7_OFFSET   				(0x173C)

/* PCIe registers */
#define PCIE_OFFSET	        			(0x20000)           
#define PCIE_RANGE_UP_OFFSET_RgOffAddr(no)  (PCIE_OFFSET + 0x6200 + (no*0x10) )    
#define PCIE_RANGE_EN_RgOffAddr(no)         (PCIE_OFFSET + 0x6204 + (no*0x10) )    
#define PCIE_RANGE_UP_RPLC_RgOffAddr(no)    (PCIE_OFFSET + 0x6208 + (no*0x10) )    
#define PCIE_RANGE_WIDTH_RgOffAddr(no)      (PCIE_OFFSET + 0x620C + (no*0x10) )    

#define INTC_OFFSET						(0x8000)
#define INTC_INTSTATUS                  (INTC_OFFSET)
#define INTMCUMASK0						(INTC_OFFSET + 0x20)
#define INTMCUMASK1						(INTC_OFFSET + 0x24)
#define INTMCUMASK2  					(INTC_OFFSET + 0x28)

#define INTC_INTSTS_MAC_EVENT           BIT(11)

#define MII_AUX_CTRL					0x12	/* Auxillary control register */

#define TC9562_NMODESTS_HOST_BOOT_MASK (0x200)
/* Synopsys Core versions */
#define	DWMAC_CORE_3_40	0x34
#define	DWMAC_CORE_3_50	0x35
#define	DWMAC_CORE_4_00	0x40
#define	DWMAC_CORE_5_00	0x50
#define DWMAC_CORE_5_10 0x51
#define TC9562MAC_CHAN0	0	/* Always supported and default for all chips */

/* Minimum dedicated AVB queues to perform AVB/TSN */
#define MIN_AVB_QUEUES	3 /* Q3, Q4, Q5 */

/* Default Queues indexes */
#define CLASS_CDT	        5
#define AVB_CLASSA_CH	    4
#define AVB_CLASSB_CH	    3
#define AVB_PTPCP_CH		2
#define CM3_BEST_EFF_CH		1
#define HOST_BEST_EFF_CH	0

#define TC9562_GPTP_ETH_TYPE		(0x88F7)
#define TC9562_ETH_TYPE_AVB		(0x22F0)
#define TC9562_VLAN_TAG			(0x8100)
#define TC9562_AVB_PRIORITY_CLASS_A	(3)
#define TC9562_AVB_PRIORITY_CLASS_B	(2)
#define TC9562_PRIORITY_CLASS_CDT	(7)

/* Class Bandwidth (%) */
#define AVB_CLASSA_BW	40
#define AVB_CLASSB_BW	20

/* Scale factor for the CBS calculus */
#define AVB_CBS_SCALE	1024

/* Port Rates */
#define PORT_RATE_SGMII	8
#define PORT_RATE_GMII	4

/* These need to be power of two, and >= 4 */
#define DMA_TX_SIZE  512   
#define DMA_RX_SIZE 128
#define TC9562MAC_GET_ENTRY(x, size)	((x + 1) & (size - 1))

#ifdef PTP_CHANGE

/*PTP def */
#define TC9562_PTP_SYSCLOCK    250000000 /* System clock is 250MHz */
#define TC9562_TARGET_PTP_CLK  50000000 
#endif

#define PROTOCOL_TYPE_UDP       0x11
#define SIZE_FOR_SEGMENTATION   2048//1400

#undef FRAME_FILTER_DEBUG
/* #define FRAME_FILTER_DEBUG */

#define MAX_AVB_SUPPORTED_TXQ 3
/* Extra statistic and debug information exposed by ethtool */
struct tc9562mac_extra_stats {
	/* Transmit errors */
	unsigned long tx_underflow ____cacheline_aligned;
	unsigned long tx_carrier;
	unsigned long tx_losscarrier;
	unsigned long vlan_tag;
	unsigned long tx_deferred;
	unsigned long tx_vlan;
	unsigned long tx_jabber;
	unsigned long tx_frame_flushed;
	unsigned long tx_payload_error;
	unsigned long tx_ip_header_error;
	/* Receive errors */
	unsigned long rx_desc;
	unsigned long sa_filter_fail;
	unsigned long overflow_error;
	unsigned long ipc_csum_error;
	unsigned long rx_collision;
	unsigned long rx_crc_errors;
	unsigned long dribbling_bit;
	unsigned long rx_length;
	unsigned long rx_mii;
	unsigned long rx_multicast;
	unsigned long rx_gmac_overflow;
	unsigned long rx_watchdog;
	unsigned long da_rx_filter_fail;
	unsigned long sa_rx_filter_fail;
	unsigned long rx_missed_cntr;
	unsigned long rx_overflow_cntr;
	unsigned long rx_vlan;
	/* Tx/Rx IRQ error info */
	unsigned long tx_undeflow_irq;
	unsigned long tx_process_stopped_irq;
	unsigned long tx_jabber_irq;
	unsigned long rx_overflow_irq;
	unsigned long rx_buf_unav_irq;
	unsigned long rx_process_stopped_irq;
	unsigned long rx_watchdog_irq;
	unsigned long tx_early_irq;
	unsigned long fatal_bus_error_irq;
	/* Tx/Rx IRQ Events */
	unsigned long rx_early_irq;
	unsigned long threshold;
	unsigned long tx_pkt_n;
	unsigned long rx_pkt_n;
	unsigned long normal_irq_n;
	unsigned long rx_normal_irq_n;
	unsigned long napi_poll;
	unsigned long tx_normal_irq_n;
	unsigned long tx_clean;
	unsigned long tx_set_ic_bit;
	unsigned long irq_receive_pmt_irq_n;
	/* MMC info */
	unsigned long mmc_tx_irq_n;
	unsigned long mmc_rx_irq_n;
	unsigned long mmc_rx_csum_offload_irq_n;
	/* EEE */
	unsigned long irq_tx_path_in_lpi_mode_n;
	unsigned long irq_tx_path_exit_lpi_mode_n;
	unsigned long irq_rx_path_in_lpi_mode_n;
	unsigned long irq_rx_path_exit_lpi_mode_n;
	unsigned long phy_eee_wakeup_error_n;
	/* Extended RDES status */
	unsigned long ip_hdr_err;
	unsigned long ip_payload_err;
	unsigned long ip_csum_bypassed;
	unsigned long ipv4_pkt_rcvd;
	unsigned long ipv6_pkt_rcvd;
	unsigned long no_ptp_rx_msg_type_ext;
	unsigned long ptp_rx_msg_type_sync;
	unsigned long ptp_rx_msg_type_follow_up;
	unsigned long ptp_rx_msg_type_delay_req;
	unsigned long ptp_rx_msg_type_delay_resp;
	unsigned long ptp_rx_msg_type_pdelay_req;
	unsigned long ptp_rx_msg_type_pdelay_resp;
	unsigned long ptp_rx_msg_type_pdelay_follow_up;
	unsigned long ptp_rx_msg_type_announce;
	unsigned long ptp_rx_msg_type_management;
	unsigned long ptp_rx_msg_pkt_reserved_type;
	unsigned long ptp_frame_type;
	unsigned long ptp_ver;
	unsigned long timestamp_dropped;
	unsigned long av_pkt_rcvd;
	unsigned long av_tagged_pkt_rcvd;
	unsigned long vlan_tag_priority_val;
	unsigned long l3_filter_match;
	unsigned long l4_filter_match;
	unsigned long l3_l4_filter_no_match;
	/* PCS */
	unsigned long irq_pcs_ane_n;
	unsigned long irq_pcs_link_n;
	unsigned long irq_rgmii_n;
	unsigned long pcs_link;
	unsigned long pcs_duplex;
	unsigned long pcs_speed;
	/* debug register */
	unsigned long mtl_tx_status_fifo_full;
	unsigned long mtl_tx_fifo_not_empty;
	unsigned long mmtl_fifo_ctrl;
	unsigned long mtl_tx_fifo_read_ctrl_write;
	unsigned long mtl_tx_fifo_read_ctrl_wait;
	unsigned long mtl_tx_fifo_read_ctrl_read;
	unsigned long mtl_tx_fifo_read_ctrl_idle;
	unsigned long mac_tx_in_pause;
	unsigned long mac_tx_frame_ctrl_xfer;
	unsigned long mac_tx_frame_ctrl_idle;
	unsigned long mac_tx_frame_ctrl_wait;
	unsigned long mac_tx_frame_ctrl_pause;
	unsigned long mac_gmii_tx_proto_engine;
	unsigned long mtl_rx_fifo_fill_level_full;
	unsigned long mtl_rx_fifo_fill_above_thresh;
	unsigned long mtl_rx_fifo_fill_below_thresh;
	unsigned long mtl_rx_fifo_fill_level_empty;
	unsigned long mtl_rx_fifo_read_ctrl_flush;
	unsigned long mtl_rx_fifo_read_ctrl_read_data;
	unsigned long mtl_rx_fifo_read_ctrl_status;
	unsigned long mtl_rx_fifo_read_ctrl_idle;
	unsigned long mtl_rx_fifo_ctrl_active;
	unsigned long mac_rx_frame_ctrl_fifo;
	unsigned long mac_gmii_rx_proto_engine;
	/* TSO */
	unsigned long tx_tso_frames;
	unsigned long tx_tso_nfrags;
};

/* CSR Frequency Access Defines*/
#define CSR_F_35M	35000000
#define CSR_F_60M	60000000
#define CSR_F_100M	100000000
#define CSR_F_150M	150000000
#define CSR_F_250M	250000000
#define CSR_F_300M	300000000

#define	MAC_CSR_H_FRQ_MASK	0x20

#define HASH_TABLE_SIZE 64
#define PAUSE_TIME 0xffff

/* Flow Control defines */
#define FLOW_OFF	0
#define FLOW_RX		1
#define FLOW_TX		2
#define FLOW_AUTO	(FLOW_TX | FLOW_RX)

/* PCS defines */
#define TC9562MAC_PCS_RGMII	(1 << 0)
#define TC9562MAC_PCS_SGMII	(1 << 1)
#define TC9562MAC_PCS_TBI		(1 << 2)
#define TC9562MAC_PCS_RTBI		(1 << 3)

#define SF_DMA_MODE 1		/* DMA STORE-AND-FORWARD Operation Mode */

/* DWC Supported Interfaces */
#define PHY_INTF_GMII_MII	0x0
#define PHY_INTF_RGMII	    0x1
#define PHY_INTF_SGMII	    0x2
#define PHY_INTF_TBI	    0x3
#define PHY_INTF_RMII	    0x4
#define PHY_INTF_RTBI	    0x5
#define PHY_INTF_SMII	    0x6
#define PHY_INTF_RevMII	    0x7

/* DAM HW feature register fields */
#define DMA_HW_FEAT_MIISEL	0x00000001	/* 10/100 Mbps Support */
#define DMA_HW_FEAT_GMIISEL	0x00000002	/* 1000 Mbps Support */
#define DMA_HW_FEAT_HDSEL	0x00000004	/* Half-Duplex Support */
#define DMA_HW_FEAT_EXTHASHEN	0x00000008	/* Expanded DA Hash Filter */
#define DMA_HW_FEAT_HASHSEL	0x00000010	/* HASH Filter */
#define DMA_HW_FEAT_ADDMAC	0x00000020	/* Multiple MAC Addr Reg */
#define DMA_HW_FEAT_PCSSEL	0x00000040	/* PCS registers */
#define DMA_HW_FEAT_L3L4FLTREN	0x00000080	/* Layer 3 & Layer 4 Feature */
#define DMA_HW_FEAT_SMASEL	0x00000100	/* SMA(MDIO) Interface */
#define DMA_HW_FEAT_RWKSEL	0x00000200	/* PMT Remote Wakeup */
#define DMA_HW_FEAT_MGKSEL	0x00000400	/* PMT Magic Packet */
#define DMA_HW_FEAT_MMCSEL	0x00000800	/* RMON Module */
#define DMA_HW_FEAT_TSVER1SEL	0x00001000	/* Only IEEE 1588-2002 */
#define DMA_HW_FEAT_TSVER2SEL	0x00002000	/* IEEE 1588-2008 PTPv2 */
#define DMA_HW_FEAT_EEESEL	0x00004000	/* Energy Efficient Ethernet */
#define DMA_HW_FEAT_AVSEL	0x00008000	/* AV Feature */
#define DMA_HW_FEAT_TXCOESEL	0x00010000	/* Checksum Offload in Tx */
#define DMA_HW_FEAT_RXTYP1COE	0x00020000	/* IP COE (Type 1) in Rx */
#define DMA_HW_FEAT_RXTYP2COE	0x00040000	/* IP COE (Type 2) in Rx */
#define DMA_HW_FEAT_RXFIFOSIZE	0x00080000	/* Rx FIFO > 2048 Bytes */
#define DMA_HW_FEAT_RXCHCNT	0x00300000	/* No. additional Rx Channels */
#define DMA_HW_FEAT_TXCHCNT	0x00c00000	/* No. additional Tx Channels */
#define DMA_HW_FEAT_ENHDESSEL	0x01000000	/* Alternate Descriptor */
/* Timestamping with Internal System Time */
#define DMA_HW_FEAT_INTTSEN	0x02000000
#define DMA_HW_FEAT_FLEXIPPSEN	0x04000000	/* Flexible PPS Output */
#define DMA_HW_FEAT_SAVLANINS	0x08000000	/* Source Addr or VLAN */
#define DMA_HW_FEAT_ACTPHYIF	0x70000000	/* Active/selected PHY iface */
#define DEFAULT_DMA_PBL		8

/* PCS status and mask defines */
#define	PCS_ANE_IRQ		BIT(2)	/* PCS Auto-Negotiation */
#define	PCS_LINK_IRQ		BIT(1)	/* PCS Link */
#define	PCS_RGSMIIIS_IRQ	BIT(0)	/* RGMII or SMII Interrupt */

/* Max/Min RI Watchdog Timer count value */
#define MAX_DMA_RIWT		0xff
#define MIN_DMA_RIWT		0x20
/* Tx coalesce parameters */
#define TC9562MAC_COAL_TX_TIMER	10000
#define TC9562MAC_MAX_COAL_TX_TICK	100000
#define TC9562MAC_TX_MAX_FRAMES	256
#define TC9562MAC_TX_FRAMES	16

/* Packets types */
enum packets_types {
	PACKET_AVCPQ = 0x1, /* AV Untagged Control packets */
	PACKET_PTPQ = 0x2, /* PTP Packets */
	PACKET_DCBCPQ = 0x3, /* DCB Control Packets */
	PACKET_UPQ = 0x4, /* Untagged Packets */
	PACKET_MCBCQ = 0x5, /* Multicast & Broadcast Packets */
	PACKET_FPE_RESIDUE = 0x6, /* Frame Pre-emption residue packets */	
};

/* Rx IPC status */
enum rx_frame_status {
	good_frame = 0x0,
	discard_frame = 0x1,
	csum_none = 0x2,
	llc_snap = 0x4,
	dma_own = 0x8,
	rx_not_ls = 0x10,
};

/* Tx status */
enum tx_frame_status {
	tx_done = 0x0,
	tx_not_ls = 0x1,
	tx_err = 0x2,
	tx_dma_own = 0x4,
};

enum dma_irq_status {
	tx_hard_error = 0x1,
	tx_hard_error_bump_tc = 0x2,
	handle_rx = 0x4,
	handle_tx = 0x8,
};

/* EEE and LPI defines */
#define	CORE_IRQ_TX_PATH_IN_LPI_MODE	(1 << 0)
#define	CORE_IRQ_TX_PATH_EXIT_LPI_MODE	(1 << 1)
#define	CORE_IRQ_RX_PATH_IN_LPI_MODE	(1 << 2)
#define	CORE_IRQ_RX_PATH_EXIT_LPI_MODE	(1 << 3)

#define CORE_IRQ_MTL_RX_OVERFLOW	BIT(8)

/* Physical Coding Sublayer */
struct rgmii_adv {
	unsigned int pause;
	unsigned int duplex;
	unsigned int lp_pause;
	unsigned int lp_duplex;
};

#define TC9562MAC_PCS_PAUSE	1
#define TC9562MAC_PCS_ASYM_PAUSE	2

/* DMA HW capabilities */
struct dma_features {
	unsigned int mbps_10_100;
	unsigned int mbps_1000;
	unsigned int half_duplex;
	unsigned int hash_filter;
	unsigned int multi_addr;
	unsigned int pcs;
	unsigned int sma_mdio;
	unsigned int pmt_remote_wake_up;
	unsigned int pmt_magic_frame;
	unsigned int rmon;
	unsigned int sa_vlan_ins;
	/* IEEE 1588-2002 */
	unsigned int time_stamp;
	/* IEEE 1588-2008 */
	unsigned int atime_stamp;
	/* 802.3az - Energy-Efficient Ethernet (EEE) */
	unsigned int eee;
	unsigned int av;
	unsigned int tsoen;
	/* TX and RX csum */
	unsigned int tx_coe;
	unsigned int rx_coe;
	unsigned int rx_coe_type1;
	unsigned int rx_coe_type2;
	unsigned int rxfifo_over_2048;
	/* TX and RX number of channels */
	unsigned int number_rx_channel;
	unsigned int number_tx_channel;
	/* TX and RX number of queues */
	unsigned int number_rx_queues;
	unsigned int number_tx_queues;
	/* Alternate (enhanced) DESC mode */
	unsigned int enh_desc;
	/* TX and RX FIFO sizes */
	unsigned int tx_fifo_size;
	unsigned int rx_fifo_size;
	/* TSN feature */
	unsigned int tsn;
	/* Frame Preemption */
	unsigned int fpesel;
	/* Enhancements to Scheduling Trafic */
	unsigned int estsel;
	unsigned int estdep;
	unsigned int estwid;
	/* Automotive Safety Package */
	unsigned int asp;
	/* RX Parser */
	unsigned int spram;
	unsigned int frpsel;
	unsigned int frpes;
	/* PPS */
	unsigned int ppsoutnum;
};

/* GMAC TX FIFO is 8K, Rx FIFO is 16K */
#define BUF_SIZE_16KiB 16384
#define BUF_SIZE_8KiB 8192
#define BUF_SIZE_4KiB 4096
#define BUF_SIZE_2KiB 2048

/* Power Down and WOL */
#define PMT_NOT_SUPPORTED 0
#define PMT_SUPPORTED 1

/* Common MAC defines */
#define MAC_CTRL_REG		0x00000000	/* MAC Control */
#define MAC_ENABLE_TX		0x00000008	/* Transmitter Enable */
#define MAC_ENABLE_RX		0x00000004	/* Receiver Enable */

/* Default LPI timers */
#define TC9562MAC_DEFAULT_LIT_LS	0x3E8
#define TC9562MAC_DEFAULT_TWT_LS	0x1E

#define TC9562MAC_CHAIN_MODE	0x1
#define TC9562MAC_RING_MODE	0x2

#define JUMBO_LEN		9000
#define ETHNORMAL_LEN		1500
#define MAX_SUPPORTED_MTU (ETH_FRAME_LEN + ETH_FCS_LEN + VLAN_HLEN)
#define MIN_SUPPORTED_MTU (ETH_ZLEN + ETH_FCS_LEN + VLAN_HLEN)

/* Descriptors helpers */
struct tc9562mac_desc_ops {
	/* DMA RX descriptor ring initialization */
	void (*init_rx_desc) (struct dma_desc *p, int disable_rx_ic, int mode,
			      int end);
	/* DMA TX descriptor ring initialization */
	void (*init_tx_desc) (struct dma_desc *p, int mode, int end);

	/* Invoked by the xmit function to prepare the tx descriptor */
	void (*prepare_tx_desc) (struct dma_desc *p, int is_fs, int len,
				 bool csum_flag, int mode, bool tx_own,
				 bool ls, unsigned int tot_pkt_len);
	void (*prepare_tso_tx_desc)(struct dma_desc *p, int is_fs, int len1,
				    int len2, bool tx_own, bool ls,
				    unsigned int tcphdrlen,
				    unsigned int tcppayloadlen);
	/* Set/get the owner of the descriptor */
	void (*set_tx_owner) (struct dma_desc *p);
	int (*get_tx_owner) (struct dma_desc *p);
	/* Clean the tx descriptor as soon as the tx irq is received */
	void (*release_tx_desc) (struct dma_desc *p, int mode);
	/* Clear interrupt on tx frame completion. When this bit is
	 * set an interrupt happens as soon as the frame is transmitted */
	void (*set_tx_ic)(struct dma_desc *p);
	/* Last tx segment reports the transmit status */
	int (*get_tx_ls) (struct dma_desc *p);
	/* Return the transmit status looking at the TDES1 */
	int (*tx_status) (void *data, struct tc9562mac_extra_stats *x,
			  struct dma_desc *p, void __iomem *ioaddr);
	/* Get the buffer size from the descriptor */
	int (*get_tx_len) (struct dma_desc *p);
	/* Handle extra events on specific interrupts hw dependent */
	void (*set_rx_owner) (struct dma_desc *p);
	/* Get the receive frame size */
	int (*get_rx_frame_len) (struct dma_desc *p, int rx_coe_type);
	/* Return the reception status looking at the RDES1 */
	int (*rx_status) (void *data, struct tc9562mac_extra_stats *x,
			  struct dma_desc *p);
	void (*rx_extended_status) (void *data, struct tc9562mac_extra_stats *x,
				    struct dma_extended_desc *p);
	/* Set tx timestamp enable bit */
	void (*enable_tx_timestamp) (struct dma_desc *p);
	/* get tx timestamp status */
	int (*get_tx_timestamp_status) (struct dma_desc *p);
	/* get timestamp value */
	 u64(*get_timestamp) (void *desc, u32 ats);
	/* get rx timestamp status */
    int (*get_rx_timestamp_status)(void *desc, void *next_desc, u32 ats);
	/* Display ring */
	void (*display_ring)(void *head, unsigned int size, bool rx);
	/* set MSS via context descriptor */
	void (*set_mss)(struct dma_desc *p, unsigned int mss);
/* Callbacks for Enhanced Descriptors */
	/* DMA TX descriptor ring initialization */
	void (*init_etx_desc) (struct dma_enhanced_desc *p, int mode, int end);

	/* Invoked by the xmit function to prepare the tx descriptor */
	void (*prepare_etx_desc) (struct dma_enhanced_desc *p, int is_fs, int len,
				 bool csum_flag, int mode, bool tx_own,
				 bool ls, unsigned int tot_pkt_len, int GSN, unsigned int launch_time);
	void (*prepare_tso_etx_desc)(struct dma_enhanced_desc *p, int is_fs, int len1,
				    int len2, bool tx_own, bool ls,
				    unsigned int tcphdrlen,
				    unsigned int tcppayloadlen);
	/* Set/get the owner of the descriptor */
	void (*set_etx_owner) (struct dma_enhanced_desc *p);
	int (*get_etx_owner) (struct dma_enhanced_desc *p);
	/* Clean the tx descriptor as soon as the tx irq is received */
	void (*release_etx_desc) (struct dma_enhanced_desc *p, int mode);
	/* Clear interrupt on tx frame completion. When this bit is
	 * set an interrupt happens as soon as the frame is transmitted */
	void (*set_etx_ic)(struct dma_enhanced_desc *p);
	/* Last tx segment reports the transmit status */
	int (*get_etx_ls) (struct dma_enhanced_desc *p);
	/* Return the transmit status looking at the TDES1 */
	int (*etx_status) (void *data, struct tc9562mac_extra_stats *x,
			  struct dma_enhanced_desc *p, void __iomem *ioaddr);
	/* Get the buffer size from the descriptor */
	int (*get_etx_len) (struct dma_enhanced_desc *p);
	/* Set tx timestamp enable bit */
	void (*enable_etx_timestamp) (struct dma_enhanced_desc *p);
	/* get tx timestamp status */
	int (*get_etx_timestamp_status) (struct dma_enhanced_desc *p);
	/* get timestamp value */
	 u64(*get_en_timestamp) (void *desc, u32 ats);
	/* Display ring */
	void (*display_en_ring)(void *head, unsigned int size, bool rx);
	/* set MSS via context descriptor */
	void (*set_en_mss)(struct dma_enhanced_desc *p, unsigned int mss);
};

extern const struct tc9562mac_desc_ops enh_desc_ops;
extern const struct tc9562mac_desc_ops ndesc_ops;

/* Specific DMA helpers */
struct tc9562mac_dma_ops {
	/* DMA core initialization */
	int (*reset)(void __iomem *ioaddr);
	void (*init)(void __iomem *ioaddr, struct tc9562mac_dma_cfg *dma_cfg,
		     u32 dma_tx, u32 dma_rx, int atds);
	void (*init_chan)(void __iomem *ioaddr,
			  struct tc9562mac_dma_cfg *dma_cfg, u32 chan);
#ifdef TC9562_DEFINED
	void (*init_rx_chan)(void __iomem *ioaddr,
			     struct tc9562mac_dma_cfg *dma_cfg,
			     u64 dma_rx_phy, u32 dma_buf_sz, u32 chan);
	void (*init_tx_chan)(void __iomem *ioaddr,
			     struct tc9562mac_dma_cfg *dma_cfg,
			     u64 dma_tx_phy, u32 chan);
#else
	void (*init_rx_chan)(void __iomem *ioaddr,
			 struct tc9562mac_dma_cfg *dma_cfg,
			 u32 dma_rx_phy, u32 dma_buf_sz, u32 chan);
	void (*init_tx_chan)(void __iomem *ioaddr,
			 struct tc9562mac_dma_cfg *dma_cfg,
			 u32 dma_tx_phy, u32 chan);
#endif
	/* Configure the AXI Bus Mode Register */
	void (*axi)(void __iomem *ioaddr, struct tc9562mac_axi *axi);
	/* Dump DMA registers */
	void (*dump_regs)(void __iomem *ioaddr, u32 *reg_space);
	/* Set tx/rx threshold in the csr6 register
	 * An invalid value enables the store-and-forward mode */
	void (*dma_mode)(void __iomem *ioaddr, int txmode, int rxmode,
			 int rxfifosz);
	void (*dma_rx_mode)(void __iomem *ioaddr, int mode, u32 channel,
			    int fifosz, u8 qmode);
	void (*dma_tx_mode)(void __iomem *ioaddr, int mode, u32 channel,
			    int fifosz, u8 qmode);
	/* To track extra statistic (if supported) */
	void (*dma_diagnostic_fr) (void *data, struct tc9562mac_extra_stats *x,
				   void __iomem *ioaddr);
	void (*enable_dma_transmission) (void __iomem *ioaddr);
	void (*enable_dma_irq)(void __iomem *ioaddr, u32 chan);
	void (*disable_dma_irq)(void __iomem *ioaddr, u32 chan);
	void (*start_tx)(void __iomem *ioaddr, u32 chan);
	void (*stop_tx)(void __iomem *ioaddr, u32 chan);
	void (*start_rx)(void __iomem *ioaddr, u32 chan);
	void (*stop_rx)(void __iomem *ioaddr, u32 chan);
	int (*dma_interrupt) (void __iomem *ioaddr,
			      struct tc9562mac_extra_stats *x, u32 chan);
	/* If supported then get the optional core features */
	void (*get_hw_feature)(void __iomem *ioaddr,
			       struct dma_features *dma_cap, int ip_version);
	/* Program the HW RX Watchdog */
	void (*rx_watchdog)(void __iomem *ioaddr, u32 riwt, u32 number_chan);
	void (*set_tx_ring_len)(void __iomem *ioaddr, u32 len, u32 chan);
	void (*set_rx_ring_len)(void __iomem *ioaddr, u32 len, u32 chan);
	void (*set_rx_tail_ptr)(void __iomem *ioaddr, u32 tail_ptr, u32 chan);
	void (*set_tx_tail_ptr)(void __iomem *ioaddr, u32 tail_ptr, u32 chan);
	void (*enable_tso)(void __iomem *ioaddr, bool en, u32 chan);
	void (*enable_edse)(void __iomem *ioaddr, bool en, u32 chan);
};

struct mac_device_info;

/* Helpers to program the MAC core */
struct tc9562mac_ops {
	/* MAC core initialization */
	void (*core_init)(struct net_device *ndev, struct mac_device_info *hw, int mtu);
	/* Enable the MAC RX/TX */
	void (*set_mac)(void __iomem *ioaddr, bool enable);
	/* Enable and verify that the IPC module is supported */
	int (*rx_ipc)(struct mac_device_info *hw);
	/* Enable RX Queues */
	void (*rx_queue_enable)(struct mac_device_info *hw, u8 mode, u32 queue);
	/* Disable RX Queues */
	void (*rx_queue_disable)(struct mac_device_info *hw, u32 queue);
	/* RX Queues Priority */
	void (*rx_queue_prio)(struct mac_device_info *hw, u32 prio, u32 queue);
	/* TX Queues Priority */
	void (*tx_queue_prio)(struct mac_device_info *hw, u32 prio, u32 queue);
	/* RX Queues Routing */
	void (*rx_queue_routing)(struct mac_device_info *hw, u8 packet,
				 u32 queue);
	/* Program RX Algorithms */
	void (*prog_mtl_rx_algorithms)(struct mac_device_info *hw, u32 rx_alg);
	/* Program TX Algorithms */
	void (*prog_mtl_tx_algorithms)(struct mac_device_info *hw, u32 tx_alg);
	/* Set MTL TX queues weight */
	void (*set_mtl_tx_queue_weight)(struct mac_device_info *hw,
					u32 weight, u32 queue);
	/* RX MTL queue to RX dma mapping */
	void (*map_mtl_to_dma)(struct mac_device_info *hw, u32 queue, u32 chan);
	/* Configure AV Algorithm */
	void (*config_cbs)(struct mac_device_info *hw, int send_slope,
			   u32 idle_slope, u32 high_credit, int low_credit,
			   u32 queue);
	/* Dump MAC registers */
	void (*dump_regs)(struct mac_device_info *hw, u32 *reg_space);
	/* Handle extra events on specific interrupts hw dependent */
	int (*host_irq_status)(struct mac_device_info *hw,
			       struct tc9562mac_extra_stats *x);
	/* Handle MTL interrupts */
	int (*host_mtl_irq_status)(struct mac_device_info *hw, u32 chan);
	/* Multicast filter setting */
	void (*set_filter)(struct mac_device_info *hw, struct net_device *dev, unsigned int l2_address_filtering);
	/* Flow control setting */
	void (*flow_ctrl)(struct mac_device_info *hw, unsigned int duplex,
			  unsigned int fc, unsigned int pause_time, u32 tx_cnt);
	/* Set power management mode (e.g. magic frame) */
	void (*pmt)(struct mac_device_info *hw, unsigned long mode);
	/* Set/Get Unicast MAC addresses */
	void (*set_umac_addr)(struct mac_device_info *hw, unsigned char *addr,
			      unsigned int reg_n, unsigned int dcs);
	void (*get_umac_addr)(struct mac_device_info *hw, unsigned char *addr,
			      unsigned int reg_n);
	void (*set_eee_mode)(struct mac_device_info *hw,
			     bool en_tx_lpi_clockgating);
	void (*reset_eee_mode)(struct mac_device_info *hw);
	void (*set_eee_timer)(struct mac_device_info *hw, int ls, int tw);
	void (*set_eee_pls)(struct mac_device_info *hw, int link);
	void (*debug)(void __iomem *ioaddr, struct tc9562mac_extra_stats *x,
		      u32 rx_queues, u32 tx_queues);
	/* PCS calls */
	void (*pcs_ctrl_ane)(void __iomem *ioaddr, bool ane, bool srgmi_ral,
			     bool loopback);
	void (*pcs_rane)(void __iomem *ioaddr, bool restart);
	void (*pcs_get_adv_lp)(void __iomem *ioaddr, struct rgmii_adv *adv);
	/* Enhancements to Scheduling Traffic */
	int (*est_init)(struct net_device *ndev, struct mac_device_info *hw,
		struct tc9562mac_est_cfg *cfg, unsigned int estsel,
		unsigned int estdep, unsigned int estwid);
	/* Safety Features */
	int (*safety_feat_init)(struct net_device *ndev,
			struct mac_device_info *hw, unsigned int asp);
	bool (*safety_feat_irq_status)(struct net_device *ndev,
			struct mac_device_info *hw, unsigned int asp);
	void (*safety_feat_set)(struct net_device *ndev,
			struct mac_device_info *hw, unsigned int asp,
			bool enabled, bool inject, u32 where, bool correctable);
	/* RX Parser */
	int (*rx_parser_init)(struct net_device *ndev,
			struct mac_device_info *hw, unsigned int spram,
			unsigned int frpsel, unsigned int frpes,
			struct tc9562mac_rx_parser_cfg *cfg);
	/* PPS */
	int (*pps_init)(struct net_device *ndev, struct mac_device_info *hw,
			int index, struct tc9562mac_pps_cfg *cfg);
	/* MCGR */
	int (*mcgr_init)(struct net_device *ndev, struct mac_device_info *hw,
			struct tc9562mac_mcgr_cfg *cfg, int count);
	void (*mcgr_intr)(struct mac_device_info *hw,
			struct tc9562mac_mcgr_cfg *cfg, int count);
};

/* PTP and HW Timer helpers */
struct tc9562mac_hwtimestamp {
	u32 (*get_hw_tstamping) (void __iomem *addr);
	u32 (*get_ptp_period)(void __iomem *ioaddr, u32 ptp_clock);
	void (*config_hw_tstamping) (void __iomem *ioaddr, u32 data);
	u32 (*config_sub_second_increment)(void __iomem *ioaddr, u32 ptp_clock,
					   int gmac4);
	int (*init_systime) (void __iomem *ioaddr, u32 sec, u32 nsec);
	int (*config_addend) (void __iomem *ioaddr, u32 addend);
	int (*adjust_systime) (void __iomem *ioaddr, u32 sec, u32 nsec,
			       int add_sub, int gmac4);
	 u64(*get_systime) (void __iomem *ioaddr);
};

extern const struct tc9562mac_hwtimestamp tc9562mac_ptp;
extern const struct tc9562mac_mode_ops dwmac4_ring_mode_ops;

struct mac_link {
	u32 speed_mask;
	u32 speed10;
	u32 speed100;
	u32 speed1000;
	u32 duplex;
};

struct mii_regs {
	unsigned int addr;	/* MII Address */
	unsigned int data;	/* MII Data */
	unsigned int addr_shift;	/* MII address shift */
	unsigned int reg_shift;		/* MII reg shift */
	unsigned int addr_mask;		/* MII address mask */
	unsigned int reg_mask;		/* MII reg mask */
	unsigned int clk_csr_shift;
	unsigned int clk_csr_mask;
};

/* Helpers to manage the descriptors for chain and ring modes */
struct tc9562mac_mode_ops {
	void (*init) (void *des, dma_addr_t phy_addr, unsigned int size,
		      unsigned int extend_desc);
	unsigned int (*is_jumbo_frm) (int len, int ehn_desc);
	int (*jumbo_frm)(void *priv, struct sk_buff *skb, int csum);
	int (*set_16kib_bfsize)(int mtu);
	void (*init_desc3)(struct dma_desc *p);
	void (*refill_desc3) (void *priv, struct dma_desc *p);
	void (*clean_desc3) (void *priv, struct dma_desc *p);
};

struct mac_device_info {
	const struct tc9562mac_ops *mac;
	const struct tc9562mac_desc_ops *desc;
	const struct tc9562mac_dma_ops *dma;
	const struct tc9562mac_mode_ops *mode;
	const struct tc9562mac_hwtimestamp *ptp;
	struct mii_regs mii;	/* MII register Addresses */
	struct mac_link link;
	void __iomem *pcsr;     /* vpointer to device CSRs */
	int multicast_filter_bins;
	int unicast_filter_entries;
	int mcast_bits_log2;
	unsigned int rx_csum;
	unsigned int pcs;
	unsigned int pmt;
	unsigned int ps;
	u32 prev_ts[TC9562MAC_PPS_MAX_NUM][2];
	u32 prev_ts_count[TC9562MAC_PPS_MAX_NUM];
};

struct tc9562mac_rx_routing {
	u32 reg_mask;
	u32 reg_shift;
};

struct mac_device_info *dwmac1000_setup(void __iomem *ioaddr, int mcbins,
					int perfect_uc_entries,
					int *synopsys_id);
struct mac_device_info *dwmac100_setup(void __iomem *ioaddr, int *synopsys_id);
struct mac_device_info *dwmac4_setup(void __iomem *ioaddr, int mcbins,
				     int perfect_uc_entries, int *synopsys_id);

void tc9562mac_set_mac_addr(void __iomem *ioaddr, u8 addr[6],
			 unsigned int high, unsigned int low);
void tc9562mac_get_mac_addr(void __iomem *ioaddr, unsigned char *addr,
			 unsigned int high, unsigned int low);
void tc9562mac_set_mac(void __iomem *ioaddr, bool enable);

void tc9562mac_dwmac4_set_mac_addr(void __iomem *ioaddr, u8 addr[6],
				unsigned int high, unsigned int low,unsigned int reg_n, unsigned int dcs);
void tc9562mac_dwmac4_get_mac_addr(void __iomem *ioaddr, unsigned char *addr,
				unsigned int high, unsigned int low);
void tc9562mac_dwmac4_set_mac(void __iomem *ioaddr, bool enable);

void dwmac_dma_flush_tx_fifo(void __iomem *ioaddr);

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
extern const struct tc9562mac_mode_ops ring_mode_ops;
extern const struct tc9562mac_mode_ops chain_mode_ops;
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

extern const struct tc9562mac_desc_ops dwmac4_desc_ops;
extern const struct tc9562mac_desc_ops dwmac4_edesc_ops;

/**
 * tc9562mac_get_synopsys_id - return the SYINID.
 * @priv: driver private structure
 * Description: this simple function is to decode and return the SYINID
 * starting from the HW core register.
 */
static inline u32 tc9562mac_get_synopsys_id(u32 hwid)
{
	/* Check Synopsys Id (not available on old chips) */
	if (likely(hwid)) {
		u32 uid = ((hwid & 0x0000ff00) >> 8);
		u32 synid = (hwid & 0x000000ff);

		pr_info("tc9562mac - user ID: 0x%x, Synopsys ID: 0x%x\n",
			uid, synid);

		return synid;
	}
	return 0;
}
#endif /* __COMMON_H__ */
