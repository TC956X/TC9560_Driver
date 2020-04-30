/*
 * TC9562 ethernet driver.
 *
 * tc9562mac_ioctl.h
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
 *  26 Feb 2020 : 1. Added Unified Driver feature.
                  2. Added TDM Start/Stop Support for Unified Design
 *  VERSION     : 01-01
 *  30 Sep 2019 : Base lined
 *  VERSION     : 01-00
 */

#ifndef __TC9562MAC_IOCTL_H__
#define __TC9562MAC_IOCTL_H__

enum {
	TC9562MAC_GET_QMODE = 0x1,
	TC9562MAC_SET_QMODE = 0x2,
	TC9562MAC_GET_CBS = 0x3,
	TC9562MAC_SET_CBS = 0x4,
	TC9562MAC_GET_EST = 0x5,
	TC9562MAC_SET_EST = 0x6,
	TC9562MAC_GET_FPE = 0x7,
	TC9562MAC_SET_FPE = 0x8,
	TC9562MAC_GET_ECC = 0x9,
	TC9562MAC_SET_ECC = 0xa,
	TC9562MAC_GET_RXP = 0xb,
	TC9562MAC_SET_RXP = 0xc,
	TC9562MAC_GET_MCGR = 0xd,
	TC9562MAC_SET_MCGR = 0xe,
	TC9562MAC_GET_PPS = 0xf,
	TC9562MAC_SET_PPS = 0x10,
	tc9562_GET_SPEED = 0x11,
	tc9562_GET_TX_FREE_DESC = 0x12,    
	tc9562_REG_RD = 0x13,
	tc9562_REG_WR = 0x14,
	tc9562_SET_MAC_LOOPBACK = 0x15,
	tc9562_SET_PHY_LOOPBACK = 0x16,
	tc9562_L2_DA_FILTERING_CMD = 0x17,
	tc9562_SET_PPS_OUT = 0x18,
	tc9562_PTPCLK_CONFIG = 0x19,
	tc9562_SA0_VLAN_INS_REP_DESC = 0x1a,
	tc9562_SA1_VLAN_INS_REP_DESC = 0x1b,
	tc9562_SA0_VLAN_INS_REP_REG = 0x1c,
	tc9562_SA1_VLAN_INS_REP_REG = 0x1d,
    tc9562_GET_TX_QCNT = 0x1e,
    tc9562_GET_RX_QCNT = 0x1f,
    tc9562_PCIE_CONFIG_REG_RD = 0x20,
    tc9562_PCIE_CONFIG_REG_WR = 0x21,
    tc9562_VLAN_FILTERING = 0x22,
    tc9562_PTPOFFLOADING = 0x23,
    TC9562_TDM_INIT = 0x24,
    TC9562_TDM_UNINIT = 0x25,
};

#define TC9562MAC_IOCTL_QMODE_DCB		0x0
#define TC9562MAC_IOCTL_QMODE_AVB		0x1

struct tc9562mac_ioctl_qmode_cfg {
	__u32 cmd;
	__u32 queue_idx;
	__u32 queue_mode;
};

struct tc9562_ioctl_cbs_params {
	__u32 send_slope;
	__u32 idle_slope;
	__u32 high_credit;
	__u32 low_credit;
	__u32 percentage;
};

struct tc9562mac_ioctl_cbs_cfg {
	__u32 cmd;
	__u32 queue_idx;
    struct tc9562_ioctl_cbs_params speed100cfg;
    struct tc9562_ioctl_cbs_params speed1000cfg;
};

struct tc9562_ioctl_speed {
	__u32 cmd;
	__u32 queue_idx;
 	__u32 connected_speed;
};

struct tc9562_ioctl_l2_da_filter {
	unsigned int cmd;
	unsigned int chInx;
	int command_error;
	/* 0 - perfect and 1 - hash filtering */
	int perfect_hash;
	/* 0 - perfect and 1 - inverse matching */
	int perfect_inverse_match;
};

struct tc9562_ioctl_free_desc {
	__u32 cmd;
	__u32 queue_idx;
 	__u32 *ptr;
};

struct tc9562_ioctl_reg_rd_wr {
	__u32 cmd;
	__u32 queue_idx;
	__u32 bar_num;
	__u32 addr;
	__u32 *ptr;
};
struct tc9562_ioctl_loopback {
	__u32 cmd;
 	__u32 flags;
};
struct tc9562mac_ioctl_fpe_cfg {
	__u32 cmd;
	__u32 enabled;
	__u32 pec;
	__u32 afsz;
	__u32 RA_time;
        __u32 HA_time;
};

/* Memory Errors */
#define TC9562MAC_IOCTL_ECC_ERR_TSO		0
#define TC9562MAC_IOCTL_ECC_ERR_EST		1
/* Parity Errors in FSM */
#define TC9562MAC_IOCTL_ECC_ERR_FSM_REVMII		2
#define TC9562MAC_IOCTL_ECC_ERR_FSM_RX125		3
#define TC9562MAC_IOCTL_ECC_ERR_FSM_TX125		4
#define TC9562MAC_IOCTL_ECC_ERR_FSM_PTP		5
#define TC9562MAC_IOCTL_ECC_ERR_FSM_APP		6
#define TC9562MAC_IOCTL_ECC_ERR_FSM_CSR		7
#define TC9562MAC_IOCTL_ECC_ERR_FSM_RX		8
#define TC9562MAC_IOCTL_ECC_ERR_FSM_TX		9
/* Timeout Errors in FSM */
#define TC9562MAC_IOCTL_ECC_ERR_FSM_TREVMII	10
#define TC9562MAC_IOCTL_ECC_ERR_FSM_TRX125		11
#define TC9562MAC_IOCTL_ECC_ERR_FSM_TTX125		12
#define TC9562MAC_IOCTL_ECC_ERR_FSM_TPTP		13
#define TC9562MAC_IOCTL_ECC_ERR_FSM_TAPP		14
#define TC9562MAC_IOCTL_ECC_ERR_FSM_TCSR		15
#define TC9562MAC_IOCTL_ECC_ERR_FSM_TRX		16
#define TC9562MAC_IOCTL_ECC_ERR_FSM_TTX		17
/* Parity Errors in DPP */
#define TC9562MAC_IOCTL_ECC_ERR_DPP_CSR		18
#define TC9562MAC_IOCTL_ECC_ERR_DPP_AXI		19
#define TC9562MAC_IOCTL_ECC_ERR_DPP_RX		20
#define TC9562MAC_IOCTL_ECC_ERR_DPP_TX		21
#define TC9562MAC_IOCTL_ECC_ERR_DPP_DMATSO		22
#define TC9562MAC_IOCTL_ECC_ERR_DPP_DMADTX		23
#define TC9562MAC_IOCTL_ECC_ERR_DPP_MTLRX		24
#define TC9562MAC_IOCTL_ECC_ERR_DPP_MTLTX		25
#define TC9562MAC_IOCTL_ECC_ERR_DPP_MTL		26
#define TC9562MAC_IOCTL_ECC_ERR_DPP_INTERFACE	27

struct tc9562mac_ioctl_ecc_cfg {
	__u32 cmd;
	__u32 supported;
	__u32 enabled;
	__u32 err_inject_supported;
	__u32 err_inject;
	__u32 err_where;
	__u32 err_correctable;
};

#define TC9562MAC_IOCTL_EST_GCL_MAX_ENTRIES		1024

struct tc9562mac_ioctl_est_cfg {
	__u32 cmd;
	__u32 enabled;
	__u32 estwid;
	__u32 estdep;
	__u32 btr_offset[2];
	__u32 ctr[2];
	__u32 ter;
	__u32 gcl[TC9562MAC_IOCTL_EST_GCL_MAX_ENTRIES];
	__u32 gcl_size;
};

struct tc9562mac_ioctl_rxp_entry {
	__u32 match_data;
	__u32 match_en;
	__u8 af:1;
	__u8 rf:1;
	__u8 im:1;
	__u8 nc:1;
	__u8 res1:4;
	__u8 frame_offset;
	__u8 ok_index;
	__u8 dma_ch_no;
	__u32 res2;
} __attribute__((packed));

#define TC9562MAC_IOCTL_RXP_MAX_ENTRIES		256

struct tc9562mac_ioctl_rxp_cfg {
	__u32 cmd;
	__u32 frpes;
	__u32 enabled;
	__u32 nve;
	__u32 npe;
	struct tc9562mac_ioctl_rxp_entry entries[TC9562MAC_IOCTL_RXP_MAX_ENTRIES];
};

#define TC9562MAC_IOCTL_MCGR_CMD_NO_OP			0x0
#define TC9562MAC_IOCTL_MCGR_CMD_CAPTURE_RISE		0x1
#define TC9562MAC_IOCTL_MCGR_CMD_CAPTURE_FALL		0x2
#define TC9562MAC_IOCTL_MCGR_CMD_CAPTURE_BOTH		0x3
#define TC9562MAC_IOCTL_MCGR_CMD_TOGGLE			0x9
#define TC9562MAC_IOCTL_MCGR_CMD_PULSE_LOW			0xA
#define TC9562MAC_IOCTL_MCGR_CMD_PULSE_HIGH		0xB

struct tc9562mac_ioctl_mcgr_cfg {
	__u32 cmd;
	__u32 index;
	__u32 enabled;
	__u32 ctrl;
	__u32 debug_route;
};

#define TC9562MAC_IOCTL_PPS_TRGTMODSEL_ONLY_INT		0x0
#define TC9562MAC_IOCTL_PPS_TRGTMODSEL_INT_ST		0x2
#define TC9562MAC_IOCTL_PPS_TRGTMODSEL_ONLY_ST		0x3

#define TC9562MAC_IOCTL_PPS_CMD_START_SINGLE_PULSE		0x1
#define TC9562MAC_IOCTL_PPS_CMD_START_PULSE_TRAIN		0x2
#define TC9562MAC_IOCTL_PPS_CMD_CANCEL_START		0x3
#define TC9562MAC_IOCTL_PPS_CMD_STOP_PULSE_TRAIN_TIME	0x4
#define TC9562MAC_IOCTL_PPS_CMD_STOP_PULSE_TRAIN		0x5
#define TC9562MAC_IOCTL_PPS_CMD_CANCEL_STOP_PULSE_TRAIN	0x6

struct tc9562mac_ioctl_pps_cfg {
	__u32 cmd;
	__u32 index;
	__u32 enabled;
	__u32 ctrl_cmd;
	__u32 trgtmodsel;
	__u32 target_time[2];
	__u32 interval;
	__u32 width;
	__u32 freq; /* in Hz */
};

struct tc9562_PPS_Config
{
  __u32 cmd;
  unsigned int ptpclk_freq;
  unsigned int ppsout_freq;
  unsigned int ppsout_duty;
  unsigned int ppsout_align_ns; // first output align to ppsout_align_ns in ns
  unsigned short ppsout_ch;
  bool  ppsout_align;  	// first output align 
};

#define TC9562_MAC0REG 0
#define TC9562_MAC1REG 1

#define TC9562_SA0_NONE		    ((TC9562_MAC0REG << 2) | 0) /* Do not include the SA */
#define TC9562_SA0_DESC_INSERT	((TC9562_MAC0REG << 2) | 1) /* Include/Insert the SA with value given in MAC Addr 0 Reg */
#define TC9562_SA0_DESC_REPLACE	((TC9562_MAC0REG << 2) | 2) /* Replace the SA with the value given in MAC Addr 0 Reg */
#define TC9562_SA0_REG_INSERT	((TC9562_MAC0REG << 2) | 2) /* Include/Insert the SA with value given in MAC Addr 0 Reg */
#define TC9562_SA0_REG_REPLACE	((TC9562_MAC0REG << 2) | 3) /* Replace the SA with the value given in MAC Addr 0 Reg */

#define TC9562_SA1_NONE		    ((TC9562_MAC1REG << 2) | 0) /* Do not include the SA */
#define TC9562_SA1_DESC_INSERT	((TC9562_MAC1REG << 2) | 1) /* Include/Insert the SA with value given in MAC Addr 1 Reg */
#define TC9562_SA1_DESC_REPLACE	((TC9562_MAC1REG << 2) | 2) /* Replace the SA with the value given in MAC Addr 1 Reg */
#define TC9562_SA1_REG_INSERT	((TC9562_MAC1REG << 2) | 2) /* Include/Insert the SA with value given in MAC Addr 1 Reg */
#define TC9562_SA1_REG_REPLACE	((TC9562_MAC1REG << 2) | 3) /* Replace the SA with the value given in MAC Addr 1 Reg */

struct tc9562_ioctl_sa_ins_cfg {
    __u32 cmd;
    unsigned int control_flag;
    unsigned char mac_addr[ETH_ALEN];
    
};

struct tc9562_ioctl_tx_qcnt {
	__u32 cmd;
	__u32 queue_idx;
 	__u32 *ptr;
};

struct tc9562_ioctl_rx_qcnt {
	__u32 cmd;
	__u32 queue_idx;
 	__u32 *ptr;
};

struct tc9562_ioctl_pcie_reg_rd_wr {
	__u32 cmd;
	__u32 addr;
	__u32 *ptr;
};

struct tc9562_ioctl_vlan_filter {
	__u32 cmd;
	/* 0 - disable and 1 - enable */
	int filter_enb_dis;
	/* 0 - perfect and 1 - hash filtering */
	int perfect_hash;
	/* 0 - perfect and 1 - inverse matching */
	int perfect_inverse_match;
};

/* for PTP offloading configuration */
#define TC9562_PTP_OFFLOADING_DISABLE 		    0
#define TC9562_PTP_OFFLOADING_ENABLE	 		1

#define TC9562_PTP_ORDINARY_SLAVE	 		    1
#define TC9562_PTP_ORDINARY_MASTER	 		    2
#define TC9562_PTP_TRASPARENT_SLAVE	 		    3
#define TC9562_PTP_TRASPARENT_MASTER	 		4
#define TC9562_PTP_PEER_TO_PEER_TRANSPARENT	    5

struct tc9562_config_ptpoffloading {
	__u32 cmd;
	int en_dis;
	int mode;
	int domain_num;
    int mc_uc; 
    unsigned char mc_uc_addr[ETH_ALEN];
};

struct tc9562mac_ioctl_tdm_config {
	uint32_t cmd;
	uint32_t queue_idx;
    TDM_I2S_CONF tdm_init_config;
};

#define SIOCSTIOCTL	SIOCDEVPRIVATE

#endif /* __TC9562MAC_IOCTL_H__ */
