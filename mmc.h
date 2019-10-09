/*
 * TC9562 ethernet driver.
 *
 * mmc.h
 *
 * Copyright (C) 2011 STMicroelectronics Ltd
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

#ifndef __MMC_H__
#define __MMC_H__

/* MMC control register */
/* When set, all counter are reset */
#define MMC_CNTRL_COUNTER_RESET		0x1
/* When set, do not roll over zero after reaching the max value*/
#define MMC_CNTRL_COUNTER_STOP_ROLLOVER	0x2
#define MMC_CNTRL_RESET_ON_READ		0x4	/* Reset after reading */
#define MMC_CNTRL_COUNTER_FREEZER	0x8	/* Freeze counter values to the
						 * current value.*/
#define MMC_CNTRL_PRESET		0x10
#define MMC_CNTRL_FULL_HALF_PRESET	0x20
#define MMC_GMAC4_OFFSET		0x700 + MAC_OFFSET
#define MMC_GMAC3_X_OFFSET		0x100

/* MMC Interrupt registers */
#define MMC_RX_INTR		        0x04	/* MMC RX Interrupt */
#define MMC_TX_INTR		        0x08	/* MMC TX Interrupt */
#define MMC_RX_IPC_INTR			0x108   /* MMC RX IPC Interrupt */
#define MMC_FPE_TX_INTR         0x1a0   /* MMC FPE Tx Interrupt */
#define MMC_FPE_RX_INTR         0x1c0   /* MMC FPE Rx Interrupt */

/* FPE */

#define MMC_FPE_TX_INTR_MASK        0x1a4
#define MMC_FPE_TX_DEFAULT_MASK     0x3

#define MMC_TX_FPE_FRAGMENT         0x1a8
#define MMC_TX_HOLD_REQ             0x1ac

#define MMC_FPE_RX_INTR_MASK        0x1c4
#define MMC_FPE_RX_DEFAULT_MASK     0xf

#define MMC_RX_PKT_ASSEMBLY_ERR     0x1c8
#define MMC_RX_PKT_SMD_ERR          0x1cc
#define MMC_RX_PKT_ASSEMBLY_OK      0x1d0
#define MMC_RX_FPE_FRAGMENT         0x1d4

struct tc9562mac_counters {
	unsigned int mmc_tx_octetcount_gb;
	unsigned int mmc_tx_framecount_gb;
	unsigned int mmc_tx_broadcastframe_g;
	unsigned int mmc_tx_multicastframe_g;
	unsigned int mmc_tx_64_octets_gb;
	unsigned int mmc_tx_65_to_127_octets_gb;
	unsigned int mmc_tx_128_to_255_octets_gb;
	unsigned int mmc_tx_256_to_511_octets_gb;
	unsigned int mmc_tx_512_to_1023_octets_gb;
	unsigned int mmc_tx_1024_to_max_octets_gb;
	unsigned int mmc_tx_unicast_gb;
	unsigned int mmc_tx_multicast_gb;
	unsigned int mmc_tx_broadcast_gb;
	unsigned int mmc_tx_underflow_error;
	unsigned int mmc_tx_singlecol_g;
	unsigned int mmc_tx_multicol_g;
	unsigned int mmc_tx_deferred;
	unsigned int mmc_tx_latecol;
	unsigned int mmc_tx_exesscol;
	unsigned int mmc_tx_carrier_error;
	unsigned int mmc_tx_octetcount_g;
	unsigned int mmc_tx_framecount_g;
	unsigned int mmc_tx_excessdef;
	unsigned int mmc_tx_pause_frame;
	unsigned int mmc_tx_vlan_frame_g;

	/* MMC RX counter registers */
	unsigned int mmc_rx_framecount_gb;
	unsigned int mmc_rx_octetcount_gb;
	unsigned int mmc_rx_octetcount_g;
	unsigned int mmc_rx_broadcastframe_g;
	unsigned int mmc_rx_multicastframe_g;
	unsigned int mmc_rx_crc_error;
	unsigned int mmc_rx_align_error;
	unsigned int mmc_rx_run_error;
	unsigned int mmc_rx_jabber_error;
	unsigned int mmc_rx_undersize_g;
	unsigned int mmc_rx_oversize_g;
	unsigned int mmc_rx_64_octets_gb;
	unsigned int mmc_rx_65_to_127_octets_gb;
	unsigned int mmc_rx_128_to_255_octets_gb;
	unsigned int mmc_rx_256_to_511_octets_gb;
	unsigned int mmc_rx_512_to_1023_octets_gb;
	unsigned int mmc_rx_1024_to_max_octets_gb;
	unsigned int mmc_rx_unicast_g;
	unsigned int mmc_rx_length_error;
	unsigned int mmc_rx_autofrangetype;
	unsigned int mmc_rx_pause_frames;
	unsigned int mmc_rx_fifo_overflow;
	unsigned int mmc_rx_vlan_frames_gb;
	unsigned int mmc_rx_watchdog_error;
	/* IPC */
	unsigned int mmc_rx_ipc_intr_mask;
	unsigned int mmc_rx_ipc_intr;
	/* IPv4 */
	unsigned int mmc_rx_ipv4_gd;
	unsigned int mmc_rx_ipv4_hderr;
	unsigned int mmc_rx_ipv4_nopay;
	unsigned int mmc_rx_ipv4_frag;
	unsigned int mmc_rx_ipv4_udsbl;

	unsigned int mmc_rx_ipv4_gd_octets;
	unsigned int mmc_rx_ipv4_hderr_octets;
	unsigned int mmc_rx_ipv4_nopay_octets;
	unsigned int mmc_rx_ipv4_frag_octets;
	unsigned int mmc_rx_ipv4_udsbl_octets;

	/* IPV6 */
	unsigned int mmc_rx_ipv6_gd_octets;
	unsigned int mmc_rx_ipv6_hderr_octets;
	unsigned int mmc_rx_ipv6_nopay_octets;

	unsigned int mmc_rx_ipv6_gd;
	unsigned int mmc_rx_ipv6_hderr;
	unsigned int mmc_rx_ipv6_nopay;

	/* Protocols */
	unsigned int mmc_rx_udp_gd;
	unsigned int mmc_rx_udp_err;
	unsigned int mmc_rx_tcp_gd;
	unsigned int mmc_rx_tcp_err;
	unsigned int mmc_rx_icmp_gd;
	unsigned int mmc_rx_icmp_err;

	unsigned int mmc_rx_udp_gd_octets;
	unsigned int mmc_rx_udp_err_octets;
	unsigned int mmc_rx_tcp_gd_octets;
	unsigned int mmc_rx_tcp_err_octets;
	unsigned int mmc_rx_icmp_gd_octets;
	unsigned int mmc_rx_icmp_err_octets;
	
	/* FPE counters */
	unsigned int mmc_tx_fpe_fragments;
	unsigned int mmc_tx_hold_req;
	
	unsigned int mmc_rx_pkt_assembly_err;
    unsigned int mmc_rx_pkt_smd_err;
    unsigned int mmc_rx_pkt_assembly_ok;
    unsigned int mmc_rx_fpe_fragment;	
	
};

void dwmac_mmc_ctrl(void __iomem *ioaddr, unsigned int mode);
void dwmac_mmc_intr_all_mask(void __iomem *ioaddr);
void dwmac_mmc_read(void __iomem *ioaddr, struct tc9562mac_counters *mmc);
#ifdef FPE
unsigned int read_mmc_tx_fpe_frag_cntr(void __iomem *mmcaddr);
unsigned int read_mmc_tx_hold_req_cntr(void __iomem *mmcaddr);
unsigned int read_mmc_rx_pkt_asmbly_ok_cntr(void __iomem *mmcaddr);
unsigned int read_mmc_rx_fpe_frag_cntr(void __iomem *mmcaddr);
#endif

#endif /* __MMC_H__ */
