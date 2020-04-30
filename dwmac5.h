/*
 * TC9562 ethernet driver.
 *
 * dwmac5.h
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
 *  26 Feb 2020 : Added 4.19 kernel support.
 *  VERSION     : 01-01
 *  30 Sep 2019 : Base lined
 *  VERSION     : 01-00
 */

#ifndef __DWMAC5_H__
#define __DWMAC5_H__

#define MAC_PRESN_TIME_NS		MAC_OFFSET + 0x00000240
#define MAC_PRESN_TIME_UPDT		MAC_OFFSET + 0x00000244
#define MAC_DPP_FSM_INT_STATUS	MAC_OFFSET + 0x00000140
#define FSMPES				BIT(24)
#define ARITES				BIT(19)
#define ATITES				BIT(18)
#define SLVTES				BIT(17)
#define MSTTES				BIT(16)
#define RVCTES				BIT(15)
#define R125ES				BIT(14)
#define T125ES				BIT(13)
#define PTES				BIT(12)
#define ATES				BIT(11)
#define CTES				BIT(10)
#define RTES				BIT(9)
#define TTES				BIT(8)
#define ASRPES				BIT(7)
#define CWPES				BIT(6)
#define ARPES				BIT(5)
#define MTSPES				BIT(4)
#define MPES				BIT(3)
#define RDPES				BIT(2)
#define TPES				BIT(1)
#define ATPES				BIT(0)
#define MAC_AXI_SLV_DPE_ADDR_STATUS	MAC_OFFSET + 0x00000144
#define ASPEAS				GENMASK(13, 0)
#define MAC_FSM_CONTROL				MAC_OFFSET + 0x00000148
#define RVCLGRNML			BIT(31)
#define R125LGRNML			BIT(30)
#define T125LGRNML			BIT(29)
#define PLGRNML				BIT(28)
#define ALGRNML				BIT(27)
#define CLGRNML				BIT(26)
#define RLGRNML				BIT(25)
#define TLGRNML				BIT(24)
#define RVCPEIN				BIT(23)
#define R125PEIN			BIT(22)
#define T125PEIN			BIT(21)
#define PPEIN				BIT(20)
#define APEIN				BIT(19)
#define CPEIN				BIT(18)
#define RPEIN				BIT(17)
#define TPEIN				BIT(16)
#define RVCTEIN				BIT(15)
#define R125TEIN			BIT(14)
#define T125TEIN			BIT(13)
#define PTEIN				BIT(12)
#define ATEIN				BIT(11)
#define CTEIN				BIT(10)
#define RTEIN				BIT(9)
#define TTEIN				BIT(8)
#define PRTYEN				BIT(1)
#define TMOUTEN				BIT(0)
#define MAC_FSM_ACT_TIMER	MAC_OFFSET + 0x0000014c

#define MAC_GPIO_CONTROL	MAC_OFFSET + 0x00000208
#define MAC_GPIO_STATUS		MAC_OFFSET + 0x0000020c
#define GPIO_GPO			GENMASK(31, 16)
#define GPIO_GPIS			GENMASK(15, 0)
#define MAC_PRESN_TIME_NS	MAC_OFFSET + 0x00000240

#define MAC_PPS_CONTROL		MAC_OFFSET + 0x00000b70
#define MCGRENx(x)			BIT(((x) * 8) + 7)
#define TRGTMODSELx(x)			GENMASK(((x) * 8) + 6, ((x) * 8) + 5)
#define PPSCMDx(x)			GENMASK(((x) * 8) + 3, ((x) * 8))
#define PPSEN0				BIT(4)

#define MAC_AUX_CTRL		MAC_OFFSET + 0x00000b40
#define MAC_PPSx_TARGET_TIME_SEC(x)	MAC_OFFSET + (0x00000b80 + ((x) * 0x10))
#define MAC_PPSx_TARGET_TIME_NSEC(x)	MAC_OFFSET + (0x00000b84 + ((x) * 0x10))
#define TRGTBUSY0			BIT(31)
#define TTSL0				GENMASK(30, 0)
#define MAC_PPSx_INTERVAL(x)	MAC_OFFSET + (0x0000b88 + ((x) * 0x10))
#define MAC_PPSx_WIDTH(x)		MAC_OFFSET + (0x0000b8c + ((x) * 0x10))

#define MTL_DBG_CTL			MAC_OFFSET + 0x00000c08
#define EIEC				GENMASK(18, 17)
#define EIEE				BIT(16)
#define MTL_RXP_CONTROL_STATUS	MAC_OFFSET + 0x00000ca0
#define RXPI				BIT(31)
#define NPE				GENMASK(23, 16)
#define NVE				GENMASK(7, 0)
#define MTL_RXP_INT_CONTROL_STATUS	MAC_OFFSET + 0x00000ca4
#define MTL_RXP_DROP_CNT			MAC_OFFSET + 0x00000ca8
#define MTL_RXP_ERROR_CNT			MAC_OFFSET + 0x00000cac
#define MTL_RXP_IACC_CTRL_STATUS	MAC_OFFSET + 0x00000cb0
#define STARTBUSY			BIT(31)
#define RXPEIEC				GENMASK(22, 21)
#define RXPEIEE				BIT(20)
#define WRRDN				BIT(16)
#define ADDR				GENMASK(15,0)
#define MTL_RXP_IACC_DATA	MAC_OFFSET + 0x00000cb4
#define MTL_ECC_CONTROL		MAC_OFFSET + 0x00000cc0
#define TSOEE				BIT(4)
#define MRXPEE				BIT(3)
#define MESTEE				BIT(2)
#define MRXEE				BIT(1)
#define MTXEE				BIT(0)

#define MTL_SAFETY_INT_STATUS	MAC_OFFSET + 0x00000cc4
#define MCSIS				BIT(31)
#define MEUIS				BIT(1)
#define MECIS				BIT(0)
#define MTL_ECC_INT_ENABLE	MAC_OFFSET + 0x00000cc8
#define RPCEIE				BIT(12)
#define ECEIE				BIT(8)
#define RXCEIE				BIT(4)
#define TXCEIE				BIT(0)
#define MTL_ECC_INT_STATUS	MAC_OFFSET + 0x00000ccc
#define RPUES				BIT(14)
#define RPAMS				BIT(13)
#define RPCES				BIT(12)
#define EUES				BIT(10)
#define EAMS				BIT(9)
#define ECES				BIT(8)
#define RXUES				BIT(6)
#define RXAMS				BIT(5)
#define RXCES				BIT(4)
#define TXUES				BIT(2)
#define TXAMS				BIT(1)
#define TXCES				BIT(0)
#define MTL_ECC_ERR_STS_RCTL		MAC_OFFSET + 0x00000cd0
#define CUES				BIT(5)
#define CCES				BIT(4)
#define EMS				GENMASK(3, 1)
#define EESRE				BIT(0)
#define MTL_ECC_ERR_ADDR_STATUS		MAC_OFFSET + 0x00000cd4
#define EUEAS				GENMASK(30, 16)
#define ECEAS				GENMASK(14, 0)
#define MTL_ECC_ERR_CNTR_STATUS		MAC_OFFSET + 0x00000cd8
#define EUECS				GENMASK(19, 16)
#define ECECS				GENMASK(7, 0)
#define MTL_DPP_CONTROL			MAC_OFFSET + 0x00000ce0
#define IPECW				BIT(13)
#define IPEASW				BIT(12)
#define IPERD				BIT(11)
#define IPETD				BIT(10)
#define IPETSO				BIT(9)
#define IPEDDC				BIT(8)
#define IPEMRF				BIT(7)
#define IPEMTS				BIT(6)
#define IPEMC				BIT(5)
#define IPEID				BIT(4)
#define EPSI				BIT(2)
#define OPE				BIT(1)
#define EDPP				BIT(0)

#define DMA_SAFETY_INT_STATUS		MAC_OFFSET + 0x00001080
#define MSUIS				BIT(29)
#define MSCIS				BIT(28)
#define DEUIS				BIT(1)
#define DECIS				BIT(0)
#define DMA_ECC_INT_ENABLE		MAC_OFFSET + 0x00001084
#define TCEIE				BIT(0)
#define DMA_ECC_INT_STATUS		MAC_OFFSET + 0x00001088

#define DMA_CHx_RXP_ACCEPT_CNT(i)	MAC_OFFSET + (0x00001168 + ((i) * 0x80))

int dwmac5_est_init(struct net_device *ndev, struct mac_device_info *hw,
		struct tc9562mac_est_cfg *cfg, unsigned int estsel,
		unsigned int estdep, unsigned int estwid);
int dwmac5_est_read(struct mac_device_info *hw, u32 reg, u32 *val,
		bool is_gcla);
int dwmac5_safety_feat_init(struct net_device *ndev,
		struct mac_device_info *hw, unsigned int asp);
bool dwmac5_safety_feat_irq_status(struct net_device *ndev,
		struct mac_device_info *hw, unsigned int asp);
void dwmac5_safety_feat_set(struct net_device *ndev, struct mac_device_info *hw,
		unsigned int asp, bool enabled, bool inject, u32 where,
		bool correctable);
int dwmac5_rx_parser_init(struct net_device *ndev, struct mac_device_info *hw,
		unsigned int spram, unsigned int frpsel, unsigned int frpes,
		struct tc9562mac_rx_parser_cfg *cfg);
int dwmac5_pps_init(struct net_device *ndev, struct mac_device_info *hw,
		int index, struct tc9562mac_pps_cfg *cfg);
int dwmac5_mcgr_init(struct net_device *ndev, struct mac_device_info *hw,
		struct tc9562mac_mcgr_cfg *cfg, int count);
void dwmac5_mcgr_intr(struct mac_device_info *hw,
		struct tc9562mac_mcgr_cfg *cfg, int count);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,15))
int dwmac5_rxp_config(void *p, void *ent,
		      unsigned int count);
#endif
#endif /* __DWMAC5_H__ */
