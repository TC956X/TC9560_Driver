/*
 * TC9562 ethernet driver.
 *
 * tc9562mac_tsn.c
 *
 * Copyright (C) 2017 Synopsys, Inc. and/or its affiliates.
 * Copyright (C) 2019 Toshiba Electronic Devices & Storage Corporation
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
 *  30 Sep 2019 : Base lined
 *  VERSION     : 01-00
 */

#include "tc9562mac.h"
#include "tc9562mac_ptp.h"
#include "dwmac4.h"
#include <linux/module.h>

#define MAX_FRAME_SIZE	1522
#define MIN_FRAME_SIZE	64

extern u8 dev_addr[6], dev_cm3_addr[6], dev_bc_addr[6], dev_mc_addr[6] ;

#ifndef TC9562_DEFINED

static void tc9562mac_tsn_fp_configure(struct tc9562mac_priv *priv)
{
	u32 control;

	DBGPR_FUNC_TSN("-->tc9562mac_tsn_fp_configure\n");

	/* Enable FPE */
	control = readl(priv->ioaddr + GMAC_FPE_CTRL_STS);
	control |= GMAC_FPE_EFPE;
	writel(control, priv->ioaddr + GMAC_FPE_CTRL_STS);

	DBGPR_FUNC_TSN("<--tc9562mac_tsn_fp_configure\n");
}

int tc9562mac_tsn_capable(struct net_device *ndev)
{
	struct tc9562mac_priv *priv = netdev_priv(ndev);

	DBGPR_FUNC_TSN("-->tc9562mac_tsn_capable\n");
	
	return priv->tsn_ready == 1;

	DBGPR_FUNC_TSN("<--tc9562mac_tsn_capable\n");
}

int tc9562mac_tsn_link_configure(struct net_device *ndev, enum sr_class class,
			      u16 framesize, u16 vid, u8 add_link, u8 pcp_hi,
			      u8 pcp_lo)
{
	struct tc9562mac_priv *priv = netdev_priv(ndev);
	u32 mode_to_use;
	s32 port_rate;
	u32 queue;
	int err;
	s32 bw;

	DBGPR_FUNC_TSN("<--tc9562mac_tsn_link_configure\n");
	if (!tc9562mac_tsn_capable(ndev)) {
		pr_err("%s:  NIC not capable\n", __func__);
		return -EINVAL;
	}
	
	if (framesize > MAX_FRAME_SIZE || framesize < MIN_FRAME_SIZE) {
		pr_err("%s: framesize (%u) must be [%d,%d]\n", __func__,
		       framesize, MIN_FRAME_SIZE, MAX_FRAME_SIZE);
		return -EINVAL;
	}

	if (add_link && !priv->tsn_vlan_added) {
		rtnl_lock();
		pr_info("%s: adding VLAN %u to HW filter on device %s\n",
			__func__, vid, ndev->name);
		err = vlan_vid_add(ndev, htons(ETH_P_8021Q), vid);
		if (err != 0)
			pr_err("%s: error adding vlan %u, res=%d\n",
				__func__, vid, err);
		rtnl_unlock();

		priv->pcp_hi = pcp_hi & 0x7;
		priv->pcp_lo = pcp_lo & 0x7;
		priv->tsn_vlan_added = 1;
	}

	/* Choose port rate */
	if (priv->plat->interface == PHY_INTERFACE_MODE_SGMII)
		port_rate = PORT_RATE_SGMII;
	else
		port_rate = PORT_RATE_GMII;

	switch(class) {
	case SR_CLASS_A:
		/* Transmit Port Rate Class A */
		queue = AVB_CLASSA_CH;
		bw = AVB_CLASSA_BW;
		break;
	case SR_CLASS_B:
		/* Transmit Port Rate Class B */
		queue = AVB_CLASSB_CH;
		bw = AVB_CLASSB_BW;
		break;
	default:
		pr_err("tc9562mac_tsn: unknown traffic-class, aborting configuration\n");
		return -EINVAL;
	}

	mode_to_use = priv->plat->tx_queues_cfg[queue].mode_to_use;
	if (mode_to_use != MTL_QUEUE_AVB) {
		pr_err("tc9562mac_tsn: Queue %d is not AVB / TSN\n", queue);
		return -EINVAL;
	}

	if (priv->synopsys_id >= DWMAC_CORE_5_00 && priv->dma_cap.tsn) {
		if (priv->dma_cap.fpesel && priv->plat->fp_en)
			tc9562mac_tsn_fp_configure(priv);
	}

	DBGPR_FUNC_TSN("<--tc9562mac_tsn_link_configure\n");

	return 0;
}
#endif

u16 tc9562mac_tsn_select_queue(struct net_device *ndev, struct sk_buff *skb,
			    void *accel_priv, select_queue_fallback_t fallback)
{

#ifndef TC9562_DEFINED
	struct tc9562mac_priv *priv = netdev_priv(ndev);
	if (!priv)
		return fallback(ndev, skb);

	if (tc9562mac_tsn_capable(ndev)) {
		switch (vlan_get_protocol(skb)) {
		case htons(ETH_P_TSN):
			if (skb->priority == priv->pcp_hi)
				return AVB_CLASSA_CH; /* Class A traffic */
			if (skb->priority == priv->pcp_lo)
				return AVB_CLASSB_CH; /* Class B traffic */
			if (skb->priority == 0x1)
				return AVB_PTPCP_CH;
			return HOST_BEST_EFF_CH;
		case htons(ETH_P_1588):
			/* PTP traffic */
			return AVB_PTPCP_CH;
		default:
			/* best effort queue */
			return HOST_BEST_EFF_CH;
		}
	}
	return fallback(ndev, skb);
#endif

	static u16 txqueue_select = 0;
	unsigned int eth_or_vlan_tag;
	unsigned int avb_priority;
	unsigned int eth_type;

	DBGPR_FUNC("-->tc9562mac_tsn_select_queue\n");

	/* TX Channel assignment based on Vlan tag and protocol type */ 
	eth_or_vlan_tag = htons(((skb->data[13]<<8) | skb->data[12]));
	//NMSGPR_ALERT( "ETH TYPE or VLAN TAG : %#x\n", eth_or_vlan_tag);
	if(eth_or_vlan_tag == TC9562_VLAN_TAG)
	{
		eth_type = htons(((skb->data[17]<<8) | skb->data[16]));
		//NMSGPR_INFO("VLAN ETH TYPE : %#x\n", eth_type);
		if(eth_type == TC9562_ETH_TYPE_AVB)
		{
			/* Extract VLAN priority field from the tx data */
			avb_priority = htons((skb->data[15]<<8) | skb->data[14]);
			//NMSGPR_INFO("VID : %#x\n", avb_priority);
			avb_priority >>= 13;
			//NMSGPR_INFO("AVB Priority : %#x\n", avb_priority);
			if(avb_priority == TC9562_AVB_PRIORITY_CLASS_A){           
				txqueue_select = AVB_CLASSA_CH;
			}else if(avb_priority == TC9562_AVB_PRIORITY_CLASS_B){     
				txqueue_select = AVB_CLASSB_CH;
			}
			else if(avb_priority == TC9562_PRIORITY_CLASS_CDT){     
				txqueue_select = CLASS_CDT;
			}
			else{
				txqueue_select = HOST_BEST_EFF_CH;
			}
		}else{
			/* VLAN but not AVB send it to Q0 */
			//printk("%x %x \n",dev_cm3_addr[0] , skb->data[0]);
				if((dev_cm3_addr[0] == skb->data[0] ) && (dev_cm3_addr[1] == skb->data[1] )&& (dev_cm3_addr[2] == skb->data[2]) && (dev_cm3_addr[3] == skb->data[3] ) && (dev_cm3_addr[4] == skb->data[4] )&& (dev_cm3_addr[5] == skb->data[5] ) )
					txqueue_select = CM3_BEST_EFF_CH;
				else
					txqueue_select = HOST_BEST_EFF_CH;	
		}
	}
	else
	{       
		/* It's protcol (ether type) field */
		eth_type = eth_or_vlan_tag;
		switch(eth_type){
			case TC9562_GPTP_ETH_TYPE:         
				txqueue_select = AVB_PTPCP_CH;
				break;
			default:
				//printk("%x %x \n",dev_cm3_addr[0] , skb->data[0]);
				if((dev_cm3_addr[0] == skb->data[0] ) && (dev_cm3_addr[1] == skb->data[1] )&& (dev_cm3_addr[2] == skb->data[2]) && (dev_cm3_addr[3] == skb->data[3] ) && (dev_cm3_addr[4] == skb->data[4] )&& (dev_cm3_addr[5] == skb->data[5] ) )
					txqueue_select = CM3_BEST_EFF_CH;
				else
					txqueue_select = HOST_BEST_EFF_CH;	
				break;
		}
	}

	DBGPR_FUNC("<--DWC_ETH_QOS_select_queue txqueue-select:%d\n",
		txqueue_select);

	return txqueue_select;
}
