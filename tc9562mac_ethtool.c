/*
 * TC9562 ethernet driver.
 *
 * tc9562mac_ethtool.c
 *
 * Copyright (C) 2007-2009 STMicroelectronics Ltd
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
 *  26 Feb 2020 : Minor update.
 *  VERSION     : 01-01
 *  30 Sep 2019 : Base lined
 *  VERSION     : 01-00
 */

#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/net_tstamp.h>
#include <asm/io.h>

#include "tc9562mac.h"
#include "dwmac_dma.h"

#define REG_SPACE_SIZE	0x1060
#define MAC100_ETHTOOL_NAME	"tc9562_mac100"
#define GMAC_ETHTOOL_NAME	"tc9562_gmac"

#define ETHTOOL_DMA_OFFSET	55

struct tc9562mac_stats {
	char stat_string[ETH_GSTRING_LEN];
	int sizeof_stat;
	int stat_offset;
};

#define TC9562MAC_STAT(m)	\
	{ #m, FIELD_SIZEOF(struct tc9562mac_extra_stats, m),	\
	offsetof(struct tc9562mac_priv, xstats.m)}

static const struct tc9562mac_stats tc9562mac_gstrings_stats[] = {
	/* Transmit errors */
	TC9562MAC_STAT(tx_underflow),
	TC9562MAC_STAT(tx_carrier),
	TC9562MAC_STAT(tx_losscarrier),
	TC9562MAC_STAT(vlan_tag),
	TC9562MAC_STAT(tx_deferred),
	TC9562MAC_STAT(tx_vlan),
	TC9562MAC_STAT(tx_jabber),
	TC9562MAC_STAT(tx_frame_flushed),
	TC9562MAC_STAT(tx_payload_error),
	TC9562MAC_STAT(tx_ip_header_error),
	/* Receive errors */
	TC9562MAC_STAT(rx_desc),
	TC9562MAC_STAT(sa_filter_fail),
	TC9562MAC_STAT(overflow_error),
	TC9562MAC_STAT(ipc_csum_error),
	TC9562MAC_STAT(rx_collision),
	TC9562MAC_STAT(rx_crc_errors),
	TC9562MAC_STAT(dribbling_bit),
	TC9562MAC_STAT(rx_length),
	TC9562MAC_STAT(rx_mii),
	TC9562MAC_STAT(rx_multicast),
	TC9562MAC_STAT(rx_gmac_overflow),
	TC9562MAC_STAT(rx_watchdog),
	TC9562MAC_STAT(da_rx_filter_fail),
	TC9562MAC_STAT(sa_rx_filter_fail),
	TC9562MAC_STAT(rx_missed_cntr),
	TC9562MAC_STAT(rx_overflow_cntr),
	TC9562MAC_STAT(rx_vlan),
	/* Tx/Rx IRQ error info */
	TC9562MAC_STAT(tx_undeflow_irq),
	TC9562MAC_STAT(tx_process_stopped_irq),
	TC9562MAC_STAT(tx_jabber_irq),
	TC9562MAC_STAT(rx_overflow_irq),
	TC9562MAC_STAT(rx_buf_unav_irq),
	TC9562MAC_STAT(rx_process_stopped_irq),
	TC9562MAC_STAT(rx_watchdog_irq),
	TC9562MAC_STAT(tx_early_irq),
	TC9562MAC_STAT(fatal_bus_error_irq),
	/* Tx/Rx IRQ Events */
	TC9562MAC_STAT(rx_early_irq),
	TC9562MAC_STAT(threshold),
	TC9562MAC_STAT(tx_pkt_n),
	TC9562MAC_STAT(rx_pkt_n),
	TC9562MAC_STAT(normal_irq_n),
	TC9562MAC_STAT(rx_normal_irq_n),
	TC9562MAC_STAT(napi_poll),
	TC9562MAC_STAT(tx_normal_irq_n),
	TC9562MAC_STAT(tx_clean),
	TC9562MAC_STAT(tx_set_ic_bit),
	TC9562MAC_STAT(irq_receive_pmt_irq_n),
	/* MMC info */
	TC9562MAC_STAT(mmc_tx_irq_n),
	TC9562MAC_STAT(mmc_rx_irq_n),
	TC9562MAC_STAT(mmc_rx_csum_offload_irq_n),
	/* EEE */
	TC9562MAC_STAT(irq_tx_path_in_lpi_mode_n),
	TC9562MAC_STAT(irq_tx_path_exit_lpi_mode_n),
	TC9562MAC_STAT(irq_rx_path_in_lpi_mode_n),
	TC9562MAC_STAT(irq_rx_path_exit_lpi_mode_n),
	TC9562MAC_STAT(phy_eee_wakeup_error_n),
	/* Extended RDES status */
	TC9562MAC_STAT(ip_hdr_err),
	TC9562MAC_STAT(ip_payload_err),
	TC9562MAC_STAT(ip_csum_bypassed),
	TC9562MAC_STAT(ipv4_pkt_rcvd),
	TC9562MAC_STAT(ipv6_pkt_rcvd),
	TC9562MAC_STAT(no_ptp_rx_msg_type_ext),
	TC9562MAC_STAT(ptp_rx_msg_type_sync),
	TC9562MAC_STAT(ptp_rx_msg_type_follow_up),
	TC9562MAC_STAT(ptp_rx_msg_type_delay_req),
	TC9562MAC_STAT(ptp_rx_msg_type_delay_resp),
	TC9562MAC_STAT(ptp_rx_msg_type_pdelay_req),
	TC9562MAC_STAT(ptp_rx_msg_type_pdelay_resp),
	TC9562MAC_STAT(ptp_rx_msg_type_pdelay_follow_up),
	TC9562MAC_STAT(ptp_rx_msg_type_announce),
	TC9562MAC_STAT(ptp_rx_msg_type_management),
	TC9562MAC_STAT(ptp_rx_msg_pkt_reserved_type),
	TC9562MAC_STAT(ptp_frame_type),
	TC9562MAC_STAT(ptp_ver),
	TC9562MAC_STAT(timestamp_dropped),
	TC9562MAC_STAT(av_pkt_rcvd),
	TC9562MAC_STAT(av_tagged_pkt_rcvd),
	TC9562MAC_STAT(vlan_tag_priority_val),
	TC9562MAC_STAT(l3_filter_match),
	TC9562MAC_STAT(l4_filter_match),
	TC9562MAC_STAT(l3_l4_filter_no_match),
	/* PCS */
	TC9562MAC_STAT(irq_pcs_ane_n),
	TC9562MAC_STAT(irq_pcs_link_n),
	TC9562MAC_STAT(irq_rgmii_n),
	/* DEBUG */
	TC9562MAC_STAT(mtl_tx_status_fifo_full),
	TC9562MAC_STAT(mtl_tx_fifo_not_empty),
	TC9562MAC_STAT(mmtl_fifo_ctrl),
	TC9562MAC_STAT(mtl_tx_fifo_read_ctrl_write),
	TC9562MAC_STAT(mtl_tx_fifo_read_ctrl_wait),
	TC9562MAC_STAT(mtl_tx_fifo_read_ctrl_read),
	TC9562MAC_STAT(mtl_tx_fifo_read_ctrl_idle),
	TC9562MAC_STAT(mac_tx_in_pause),
	TC9562MAC_STAT(mac_tx_frame_ctrl_xfer),
	TC9562MAC_STAT(mac_tx_frame_ctrl_idle),
	TC9562MAC_STAT(mac_tx_frame_ctrl_wait),
	TC9562MAC_STAT(mac_tx_frame_ctrl_pause),
	TC9562MAC_STAT(mac_gmii_tx_proto_engine),
	TC9562MAC_STAT(mtl_rx_fifo_fill_level_full),
	TC9562MAC_STAT(mtl_rx_fifo_fill_above_thresh),
	TC9562MAC_STAT(mtl_rx_fifo_fill_below_thresh),
	TC9562MAC_STAT(mtl_rx_fifo_fill_level_empty),
	TC9562MAC_STAT(mtl_rx_fifo_read_ctrl_flush),
	TC9562MAC_STAT(mtl_rx_fifo_read_ctrl_read_data),
	TC9562MAC_STAT(mtl_rx_fifo_read_ctrl_status),
	TC9562MAC_STAT(mtl_rx_fifo_read_ctrl_idle),
	TC9562MAC_STAT(mtl_rx_fifo_ctrl_active),
	TC9562MAC_STAT(mac_rx_frame_ctrl_fifo),
	TC9562MAC_STAT(mac_gmii_rx_proto_engine),
	/* TSO */
	TC9562MAC_STAT(tx_tso_frames),
	TC9562MAC_STAT(tx_tso_nfrags),
};
#define TC9562MAC_STATS_LEN ARRAY_SIZE(tc9562mac_gstrings_stats)

/* HW MAC Management counters (if supported) */
#define TC9562MAC_MMC_STAT(m)	\
	{ #m, FIELD_SIZEOF(struct tc9562mac_counters, m),	\
	offsetof(struct tc9562mac_priv, mmc.m)}

static const struct tc9562mac_stats tc9562mac_mmc[] = {
	TC9562MAC_MMC_STAT(mmc_tx_octetcount_gb),
	TC9562MAC_MMC_STAT(mmc_tx_framecount_gb),
	TC9562MAC_MMC_STAT(mmc_tx_broadcastframe_g),
	TC9562MAC_MMC_STAT(mmc_tx_multicastframe_g),
	TC9562MAC_MMC_STAT(mmc_tx_64_octets_gb),
	TC9562MAC_MMC_STAT(mmc_tx_65_to_127_octets_gb),
	TC9562MAC_MMC_STAT(mmc_tx_128_to_255_octets_gb),
	TC9562MAC_MMC_STAT(mmc_tx_256_to_511_octets_gb),
	TC9562MAC_MMC_STAT(mmc_tx_512_to_1023_octets_gb),
	TC9562MAC_MMC_STAT(mmc_tx_1024_to_max_octets_gb),
	TC9562MAC_MMC_STAT(mmc_tx_unicast_gb),
	TC9562MAC_MMC_STAT(mmc_tx_multicast_gb),
	TC9562MAC_MMC_STAT(mmc_tx_broadcast_gb),
	TC9562MAC_MMC_STAT(mmc_tx_underflow_error),
	TC9562MAC_MMC_STAT(mmc_tx_singlecol_g),
	TC9562MAC_MMC_STAT(mmc_tx_multicol_g),
	TC9562MAC_MMC_STAT(mmc_tx_deferred),
	TC9562MAC_MMC_STAT(mmc_tx_latecol),
	TC9562MAC_MMC_STAT(mmc_tx_exesscol),
	TC9562MAC_MMC_STAT(mmc_tx_carrier_error),
	TC9562MAC_MMC_STAT(mmc_tx_octetcount_g),
	TC9562MAC_MMC_STAT(mmc_tx_framecount_g),
	TC9562MAC_MMC_STAT(mmc_tx_excessdef),
	TC9562MAC_MMC_STAT(mmc_tx_pause_frame),
	TC9562MAC_MMC_STAT(mmc_tx_vlan_frame_g),
	TC9562MAC_MMC_STAT(mmc_rx_framecount_gb),
	TC9562MAC_MMC_STAT(mmc_rx_octetcount_gb),
	TC9562MAC_MMC_STAT(mmc_rx_octetcount_g),
	TC9562MAC_MMC_STAT(mmc_rx_broadcastframe_g),
	TC9562MAC_MMC_STAT(mmc_rx_multicastframe_g),
	TC9562MAC_MMC_STAT(mmc_rx_crc_error),
	TC9562MAC_MMC_STAT(mmc_rx_align_error),
	TC9562MAC_MMC_STAT(mmc_rx_run_error),
	TC9562MAC_MMC_STAT(mmc_rx_jabber_error),
	TC9562MAC_MMC_STAT(mmc_rx_undersize_g),
	TC9562MAC_MMC_STAT(mmc_rx_oversize_g),
	TC9562MAC_MMC_STAT(mmc_rx_64_octets_gb),
	TC9562MAC_MMC_STAT(mmc_rx_65_to_127_octets_gb),
	TC9562MAC_MMC_STAT(mmc_rx_128_to_255_octets_gb),
	TC9562MAC_MMC_STAT(mmc_rx_256_to_511_octets_gb),
	TC9562MAC_MMC_STAT(mmc_rx_512_to_1023_octets_gb),
	TC9562MAC_MMC_STAT(mmc_rx_1024_to_max_octets_gb),
	TC9562MAC_MMC_STAT(mmc_rx_unicast_g),
	TC9562MAC_MMC_STAT(mmc_rx_length_error),
	TC9562MAC_MMC_STAT(mmc_rx_autofrangetype),
	TC9562MAC_MMC_STAT(mmc_rx_pause_frames),
	TC9562MAC_MMC_STAT(mmc_rx_fifo_overflow),
	TC9562MAC_MMC_STAT(mmc_rx_vlan_frames_gb),
	TC9562MAC_MMC_STAT(mmc_rx_watchdog_error),
	TC9562MAC_MMC_STAT(mmc_rx_ipc_intr_mask),
	TC9562MAC_MMC_STAT(mmc_rx_ipc_intr),
	TC9562MAC_MMC_STAT(mmc_rx_ipv4_gd),
	TC9562MAC_MMC_STAT(mmc_rx_ipv4_hderr),
	TC9562MAC_MMC_STAT(mmc_rx_ipv4_nopay),
	TC9562MAC_MMC_STAT(mmc_rx_ipv4_frag),
	TC9562MAC_MMC_STAT(mmc_rx_ipv4_udsbl),
	TC9562MAC_MMC_STAT(mmc_rx_ipv4_gd_octets),
	TC9562MAC_MMC_STAT(mmc_rx_ipv4_hderr_octets),
	TC9562MAC_MMC_STAT(mmc_rx_ipv4_nopay_octets),
	TC9562MAC_MMC_STAT(mmc_rx_ipv4_frag_octets),
	TC9562MAC_MMC_STAT(mmc_rx_ipv4_udsbl_octets),
	TC9562MAC_MMC_STAT(mmc_rx_ipv6_gd_octets),
	TC9562MAC_MMC_STAT(mmc_rx_ipv6_hderr_octets),
	TC9562MAC_MMC_STAT(mmc_rx_ipv6_nopay_octets),
	TC9562MAC_MMC_STAT(mmc_rx_ipv6_gd),
	TC9562MAC_MMC_STAT(mmc_rx_ipv6_hderr),
	TC9562MAC_MMC_STAT(mmc_rx_ipv6_nopay),
	TC9562MAC_MMC_STAT(mmc_rx_udp_gd),
	TC9562MAC_MMC_STAT(mmc_rx_udp_err),
	TC9562MAC_MMC_STAT(mmc_rx_tcp_gd),
	TC9562MAC_MMC_STAT(mmc_rx_tcp_err),
	TC9562MAC_MMC_STAT(mmc_rx_icmp_gd),
	TC9562MAC_MMC_STAT(mmc_rx_icmp_err),
	TC9562MAC_MMC_STAT(mmc_rx_udp_gd_octets),
	TC9562MAC_MMC_STAT(mmc_rx_udp_err_octets),
	TC9562MAC_MMC_STAT(mmc_rx_tcp_gd_octets),
	TC9562MAC_MMC_STAT(mmc_rx_tcp_err_octets),
	TC9562MAC_MMC_STAT(mmc_rx_icmp_gd_octets),
	TC9562MAC_MMC_STAT(mmc_rx_icmp_err_octets),
	TC9562MAC_MMC_STAT(mmc_tx_fpe_fragments),
	TC9562MAC_MMC_STAT(mmc_tx_hold_req),
    TC9562MAC_MMC_STAT(mmc_rx_pkt_assembly_err),
    TC9562MAC_MMC_STAT(mmc_rx_pkt_smd_err),
    TC9562MAC_MMC_STAT(mmc_rx_pkt_assembly_ok),
    TC9562MAC_MMC_STAT(mmc_rx_fpe_fragment)
};
#define TC9562MAC_MMC_STATS_LEN ARRAY_SIZE(tc9562mac_mmc)

static void tc9562mac_ethtool_getdrvinfo(struct net_device *dev,
				      struct ethtool_drvinfo *info)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
    tc9562_version_t *fw_version;
    int reg;
    char fw_version_str[32];
    
	DBGPR_FUNC_ETHTOOL("-->tc9562mac_ethtool_getdrvinfo\n");

    reg = readl(priv->tc9562_SRAM_pci_base_addr + M3_SRAM_FW_VER_OFFSET);
    fw_version = (struct tc9562_version_s *)(&reg);
	sprintf(fw_version_str, "Firmware Version %s_%d.%d-%d", (fw_version->rel_dbg == 'D')?"DBG":"REL", fw_version->major, fw_version->minor, fw_version->sub_minor);
    
    strlcpy(info->fw_version, fw_version_str, sizeof(info->fw_version));
    
 	if (priv->plat->has_gmac || priv->plat->has_gmac4)
		strlcpy(info->driver, GMAC_ETHTOOL_NAME, sizeof(info->driver));
	else
		strlcpy(info->driver, MAC100_ETHTOOL_NAME, sizeof(info->driver));

	strlcpy(info->version, DRV_MODULE_VERSION, sizeof(info->version));
    
	DBGPR_FUNC_ETHTOOL("<--tc9562mac_ethtool_getdrvinfo\n");
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 10, 0))

static int tc9562mac_ethtool_getsettings(struct net_device *dev,
				      struct ethtool_cmd *cmd)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	//struct phy_device *phy = priv->phydev;
	struct phy_device *phy = dev->phydev;
	
	int rc;

	if (priv->hw->pcs & TC9562MAC_PCS_RGMII ||
	    priv->hw->pcs & TC9562MAC_PCS_SGMII) {
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE	    
		struct rgmii_adv adv;

		if (!priv->xstats.pcs_link) {
			ethtool_cmd_speed_set(cmd, SPEED_UNKNOWN);
			cmd->duplex = DUPLEX_UNKNOWN;
			return 0;
		}
		cmd->duplex = priv->xstats.pcs_duplex;

		ethtool_cmd_speed_set(cmd, priv->xstats.pcs_speed);

		/* Get and convert ADV/LP_ADV from the HW AN registers */
		if (!priv->hw->mac->pcs_get_adv_lp)
			return -EOPNOTSUPP;	/* should never happen indeed */

		priv->hw->mac->pcs_get_adv_lp(priv->ioaddr, &adv);

		/* Encoding of PSE bits is defined in 802.3z, 37.2.1.4 */

		if (adv.pause & TC9562MAC_PCS_PAUSE)
			cmd->advertising |= ADVERTISED_Pause;
		if (adv.pause & TC9562MAC_PCS_ASYM_PAUSE)
			cmd->advertising |= ADVERTISED_Asym_Pause;
		if (adv.lp_pause & TC9562MAC_PCS_PAUSE)
			cmd->lp_advertising |= ADVERTISED_Pause;
		if (adv.lp_pause & TC9562MAC_PCS_ASYM_PAUSE)
			cmd->lp_advertising |= ADVERTISED_Asym_Pause;

		/* Reg49[3] always set because ANE is always supported */
		cmd->autoneg = ADVERTISED_Autoneg;
		cmd->supported |= SUPPORTED_Autoneg;
		cmd->advertising |= ADVERTISED_Autoneg;
		cmd->lp_advertising |= ADVERTISED_Autoneg;

		if (adv.duplex) {
			cmd->supported |= (SUPPORTED_1000baseT_Full |
					   SUPPORTED_100baseT_Full |
					   SUPPORTED_10baseT_Full);
			cmd->advertising |= (ADVERTISED_1000baseT_Full |
					     ADVERTISED_100baseT_Full |
					     ADVERTISED_10baseT_Full);
		} else {
			cmd->supported |= (SUPPORTED_1000baseT_Half |
					   SUPPORTED_100baseT_Half |
					   SUPPORTED_10baseT_Half);
			cmd->advertising |= (ADVERTISED_1000baseT_Half |
					     ADVERTISED_100baseT_Half |
					     ADVERTISED_10baseT_Half);
		}
		if (adv.lp_duplex)
			cmd->lp_advertising |= (ADVERTISED_1000baseT_Full |
						ADVERTISED_100baseT_Full |
						ADVERTISED_10baseT_Full);
		else
			cmd->lp_advertising |= (ADVERTISED_1000baseT_Half |
						ADVERTISED_100baseT_Half |
						ADVERTISED_10baseT_Half);
		cmd->port = PORT_OTHER;
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
		return 0;
	}

	if (phy == NULL) {
		pr_err("%s: %s: PHY is not registered\n",
		       __func__, dev->name);
		return -ENODEV;
	}
	if (!netif_running(dev)) {
		pr_err("%s: interface is disabled: we cannot track "
		"link speed / duplex setting\n", dev->name);
		return -EBUSY;
	}
	cmd->transceiver = XCVR_INTERNAL;
	rc = phy_ethtool_gset(phy, cmd);
	return rc;
}

static int tc9562mac_ethtool_setsettings(struct net_device *dev,
				      struct ethtool_cmd *cmd)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	//struct phy_device *phy = priv->phydev;
	struct phy_device *phy = dev->phydev;
	
	int rc;

	if (priv->hw->pcs & TC9562MAC_PCS_RGMII ||
	    priv->hw->pcs & TC9562MAC_PCS_SGMII) {
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
		u32 mask = ADVERTISED_Autoneg | ADVERTISED_Pause;

		/* Only support ANE */
		if (cmd->autoneg != AUTONEG_ENABLE)
			return -EINVAL;

		mask &= (ADVERTISED_1000baseT_Half |
			ADVERTISED_1000baseT_Full |
			ADVERTISED_100baseT_Half |
			ADVERTISED_100baseT_Full |
			ADVERTISED_10baseT_Half |
			ADVERTISED_10baseT_Full);

		spin_lock(&priv->lock);

		if (priv->hw->mac->pcs_ctrl_ane)
			priv->hw->mac->pcs_ctrl_ane(priv->ioaddr, 1,
						    priv->hw->ps, 0);

		spin_unlock(&priv->lock);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */		
		return 0;
	}

	spin_lock(&priv->lock);
	rc = phy_ethtool_sset(phy, cmd);
	spin_unlock(&priv->lock);

	return rc;
}


#else /*#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 10, 0))*/
static int tc9562mac_ethtool_get_link_ksettings(struct net_device *dev,
					     struct ethtool_link_ksettings *cmd)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	struct phy_device *phy = dev->phydev;

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_ethtool_get_link_ksettings\n");
	
	if (priv->hw->pcs & TC9562MAC_PCS_RGMII ||
	    priv->hw->pcs & TC9562MAC_PCS_SGMII) {
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE		    
		struct rgmii_adv adv;
		u32 supported, advertising, lp_advertising;

		if (!priv->xstats.pcs_link) {
			cmd->base.speed = SPEED_UNKNOWN;
			cmd->base.duplex = DUPLEX_UNKNOWN;
			return 0;
		}
		cmd->base.duplex = priv->xstats.pcs_duplex;

		cmd->base.speed = priv->xstats.pcs_speed;

		/* Get and convert ADV/LP_ADV from the HW AN registers */
		if (!priv->hw->mac->pcs_get_adv_lp)
			return -EOPNOTSUPP;	/* should never happen indeed */

		priv->hw->mac->pcs_get_adv_lp(priv->ioaddr, &adv);

		/* Encoding of PSE bits is defined in 802.3z, 37.2.1.4 */

		ethtool_convert_link_mode_to_legacy_u32(
			&supported, cmd->link_modes.supported);
		ethtool_convert_link_mode_to_legacy_u32(
			&advertising, cmd->link_modes.advertising);
		ethtool_convert_link_mode_to_legacy_u32(
			&lp_advertising, cmd->link_modes.lp_advertising);

		if (adv.pause & TC9562MAC_PCS_PAUSE)
			advertising |= ADVERTISED_Pause;
		if (adv.pause & TC9562MAC_PCS_ASYM_PAUSE)
			advertising |= ADVERTISED_Asym_Pause;
		if (adv.lp_pause & TC9562MAC_PCS_PAUSE)
			lp_advertising |= ADVERTISED_Pause;
		if (adv.lp_pause & TC9562MAC_PCS_ASYM_PAUSE)
			lp_advertising |= ADVERTISED_Asym_Pause;

		/* Reg49[3] always set because ANE is always supported */
		cmd->base.autoneg = ADVERTISED_Autoneg;
		supported |= SUPPORTED_Autoneg;
		advertising |= ADVERTISED_Autoneg;
		lp_advertising |= ADVERTISED_Autoneg;

		if (adv.duplex) {
			supported |= (SUPPORTED_1000baseT_Full |
				      SUPPORTED_100baseT_Full |
				      SUPPORTED_10baseT_Full);
			advertising |= (ADVERTISED_1000baseT_Full |
					ADVERTISED_100baseT_Full |
					ADVERTISED_10baseT_Full);
		} else {
			supported |= (SUPPORTED_1000baseT_Half |
				      SUPPORTED_100baseT_Half |
				      SUPPORTED_10baseT_Half);
			advertising |= (ADVERTISED_1000baseT_Half |
					ADVERTISED_100baseT_Half |
					ADVERTISED_10baseT_Half);
		}
		if (adv.lp_duplex)
			lp_advertising |= (ADVERTISED_1000baseT_Full |
					   ADVERTISED_100baseT_Full |
					   ADVERTISED_10baseT_Full);
		else
			lp_advertising |= (ADVERTISED_1000baseT_Half |
					   ADVERTISED_100baseT_Half |
					   ADVERTISED_10baseT_Half);
		cmd->base.port = PORT_OTHER;

		ethtool_convert_legacy_u32_to_link_mode(
			cmd->link_modes.supported, supported);
		ethtool_convert_legacy_u32_to_link_mode(
			cmd->link_modes.advertising, advertising);
		ethtool_convert_legacy_u32_to_link_mode(
			cmd->link_modes.lp_advertising, lp_advertising);

#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
		return 0;
	}

	if (phy == NULL) {
		pr_err("%s: %s: PHY is not registered\n",
		       __func__, dev->name);
		return -ENODEV;
	}
	if (!netif_running(dev)) {
		pr_err("%s: interface is disabled: we cannot track "
		"link speed / duplex setting\n", dev->name);
		return -EBUSY;
	}
	phy_ethtool_ksettings_get(phy, cmd);

	DBGPR_FUNC_ETHTOOL("<--tc9562mac_ethtool_get_link_ksettings\n");
	
	return 0;
}


static int
tc9562mac_ethtool_set_link_ksettings(struct net_device *dev,
				  const struct ethtool_link_ksettings *cmd)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	struct phy_device *phy = dev->phydev;
	int rc;

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_ethtool_set_link_ksettings\n");

	if (priv->hw->pcs & TC9562MAC_PCS_RGMII ||
	    priv->hw->pcs & TC9562MAC_PCS_SGMII) {
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE	    
		u32 mask = ADVERTISED_Autoneg | ADVERTISED_Pause;

		/* Only support ANE */
		if (cmd->base.autoneg != AUTONEG_ENABLE)
			return -EINVAL;

		mask &= (ADVERTISED_1000baseT_Half |
			ADVERTISED_1000baseT_Full |
			ADVERTISED_100baseT_Half |
			ADVERTISED_100baseT_Full |
			ADVERTISED_10baseT_Half |
			ADVERTISED_10baseT_Full);

		spin_lock(&priv->lock);

		if (priv->hw->mac->pcs_ctrl_ane)
			priv->hw->mac->pcs_ctrl_ane(priv->ioaddr, 1,
						    priv->hw->ps, 0);

		spin_unlock(&priv->lock);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
		return 0;
	}

	rc = phy_ethtool_ksettings_set(phy, cmd);

	DBGPR_FUNC_ETHTOOL("<--tc9562mac_ethtool_set_link_ksettings\n");
	
	return rc;
}
#endif
static u32 tc9562mac_ethtool_getmsglevel(struct net_device *dev)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	DBGPR_FUNC_ETHTOOL("-->tc9562mac_ethtool_getmsglevel\n");
	return priv->msg_enable;
}

static void tc9562mac_ethtool_setmsglevel(struct net_device *dev, u32 level)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	DBGPR_FUNC_ETHTOOL("-->tc9562mac_ethtool_setmsglevel\n");
	priv->msg_enable = level;
	DBGPR_FUNC_ETHTOOL("<--tc9562mac_ethtool_setmsglevel\n");
}

static int tc9562mac_check_if_running(struct net_device *dev)
{
	DBGPR_FUNC_ETHTOOL("-->tc9562mac_check_if_running\n");
	if (!netif_running(dev))
		return -EBUSY;
	DBGPR_FUNC_ETHTOOL("<--tc9562mac_check_if_running\n");
	return 0;
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static int tc9562mac_ethtool_get_regs_len(struct net_device *dev)
{
	DBGPR_FUNC_ETHTOOL("-->tc9562mac_ethtool_get_regs_len\n");
	return REG_SPACE_SIZE;
}

static void tc9562mac_ethtool_gregs(struct net_device *dev,
			  struct ethtool_regs *regs, void *space)
{
	u32 *reg_space = (u32 *) space;

	struct tc9562mac_priv *priv = netdev_priv(dev);

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_ethtool_gregs\n");

	memset(reg_space, 0x0, REG_SPACE_SIZE);

	priv->hw->mac->dump_regs(priv->hw, reg_space);
	priv->hw->dma->dump_regs(priv->ioaddr, reg_space);
	/* Copy DMA registers to where ethtool expects them */
	memcpy(&reg_space[ETHTOOL_DMA_OFFSET], &reg_space[DMA_BUS_MODE / 4],
	       NUM_DWMAC1000_DMA_REGS * 4);
	DBGPR_FUNC_ETHTOOL("<--tc9562mac_ethtool_gregs\n");
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

static void
tc9562mac_get_pauseparam(struct net_device *netdev,
		      struct ethtool_pauseparam *pause)
{
	struct tc9562mac_priv *priv = netdev_priv(netdev);

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_get_pauseparam\n");
	
	pause->rx_pause = 0;
	pause->tx_pause = 0;

	if (priv->hw->pcs && priv->hw->mac->pcs_get_adv_lp) {
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE		
		struct rgmii_adv adv_lp;

		pause->autoneg = 1;
		priv->hw->mac->pcs_get_adv_lp(priv->ioaddr, &adv_lp);
		if (!adv_lp.pause)
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */				
			return;
	} else {
		if (!(netdev->phydev->supported & SUPPORTED_Pause) ||
		    !(netdev->phydev->supported & SUPPORTED_Asym_Pause))
			return;
	}

	pause->autoneg = netdev->phydev->autoneg;

	if (priv->flow_ctrl & FLOW_RX)
		pause->rx_pause = 1;
	if (priv->flow_ctrl & FLOW_TX)
		pause->tx_pause = 1;

	DBGPR_FUNC_ETHTOOL("<--tc9562mac_get_pauseparam\n");

}

static int
tc9562mac_set_pauseparam(struct net_device *netdev,
		      struct ethtool_pauseparam *pause)
{
	struct tc9562mac_priv *priv = netdev_priv(netdev);
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	struct phy_device *phy = netdev->phydev;
	int new_pause = FLOW_OFF;

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_set_pauseparam\n");
		
	if (priv->hw->pcs && priv->hw->mac->pcs_get_adv_lp) {
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE		
		struct rgmii_adv adv_lp;

		pause->autoneg = 1;
		priv->hw->mac->pcs_get_adv_lp(priv->ioaddr, &adv_lp);
		if (!adv_lp.pause)
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
			return -EOPNOTSUPP;
	} else {
		if (!(phy->supported & SUPPORTED_Pause) ||
		    !(phy->supported & SUPPORTED_Asym_Pause))
			return -EOPNOTSUPP;
	}

	if (pause->rx_pause)
		new_pause |= FLOW_RX;
	if (pause->tx_pause)
		new_pause |= FLOW_TX;

	priv->flow_ctrl = new_pause;
	phy->autoneg = pause->autoneg;

	if (phy->autoneg) {
		if (netif_running(netdev))
			return phy_start_aneg(phy);
	}

	priv->hw->mac->flow_ctrl(priv->hw, phy->duplex, priv->flow_ctrl,
				 priv->pause, tx_cnt);

	DBGPR_FUNC_ETHTOOL("<--tc9562mac_set_pauseparam\n");
	
	return 0;
}

static void tc9562mac_get_ethtool_stats(struct net_device *dev,
				 struct ethtool_stats *dummy, u64 *data)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	int i, j = 0;
	unsigned long mmc_flags;

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_get_ethtool_stats\n");
	
	/* Update the DMA HW counters for dwmac10/100 */
	if (priv->hw->dma->dma_diagnostic_fr)
		priv->hw->dma->dma_diagnostic_fr(&dev->stats,
						 (void *) &priv->xstats,
						 priv->ioaddr);
	else {
		/* If supported, for new GMAC chips expose the MMC counters */
		if (priv->dma_cap.rmon) {
		    spin_lock_irqsave(&priv->mmc_lock, mmc_flags);
			dwmac_mmc_read(priv->mmcaddr, &priv->mmc);
			spin_unlock_irqrestore(&priv->mmc_lock, mmc_flags);

			for (i = 0; i < TC9562MAC_MMC_STATS_LEN; i++) {
				char *p;
				p = (char *)priv + tc9562mac_mmc[i].stat_offset;

				data[j++] = (tc9562mac_mmc[i].sizeof_stat ==
					     sizeof(u64)) ? (*(u64 *)p) :
					     (*(u32 *)p);
			}
		}
		if (priv->eee_enabled) {
			int val = phy_get_eee_err(dev->phydev);
			if (val)
				priv->xstats.phy_eee_wakeup_error_n = val;
		}

		if ((priv->hw->mac->debug) &&
		    (priv->synopsys_id >= DWMAC_CORE_3_50))
			priv->hw->mac->debug(priv->ioaddr,
					     (void *)&priv->xstats,
					     rx_queues_count, tx_queues_count);
	}
	for (i = 0; i < TC9562MAC_STATS_LEN; i++) {
		char *p = (char *)priv + tc9562mac_gstrings_stats[i].stat_offset;
		data[j++] = (tc9562mac_gstrings_stats[i].sizeof_stat ==
			     sizeof(u64)) ? (*(u64 *)p) : (*(u32 *)p);
	}

	DBGPR_FUNC_ETHTOOL("<--tc9562mac_get_ethtool_stats\n");
}

static int tc9562mac_get_sset_count(struct net_device *netdev, int sset)
{
	struct tc9562mac_priv *priv = netdev_priv(netdev);
	int len;

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_get_sset_count\n");

	switch (sset) {
	case ETH_SS_STATS:
		len = TC9562MAC_STATS_LEN;

		if (priv->dma_cap.rmon)
			len += TC9562MAC_MMC_STATS_LEN;
		return len;
	default:
		return -EOPNOTSUPP;
	}

	DBGPR_FUNC_ETHTOOL("<--tc9562mac_get_sset_count\n");
	
}

static void tc9562mac_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
	int i;
	u8 *p = data;
	struct tc9562mac_priv *priv = netdev_priv(dev);

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_get_strings\n");
	
	switch (stringset) {
	case ETH_SS_STATS:
		if (priv->dma_cap.rmon)
			for (i = 0; i < TC9562MAC_MMC_STATS_LEN; i++) {
				memcpy(p, tc9562mac_mmc[i].stat_string,
				       ETH_GSTRING_LEN);
				p += ETH_GSTRING_LEN;
			}
		for (i = 0; i < TC9562MAC_STATS_LEN; i++) {
			memcpy(p, tc9562mac_gstrings_stats[i].stat_string,
				ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	default:
		WARN_ON(1);
		break;
	}

	DBGPR_FUNC_ETHTOOL("<--tc9562mac_get_strings\n");
}

/* Currently only support WOL through Magic packet. */
static void tc9562mac_get_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_get_wol\n");
	spin_lock_irq(&priv->lock);
	if (device_can_wakeup(priv->device)) {
		wol->supported = WAKE_MAGIC | WAKE_UCAST;
		wol->wolopts = priv->wolopts;
	}
	spin_unlock_irq(&priv->lock);
	DBGPR_FUNC_ETHTOOL("<--tc9562mac_get_wol\n");
}

static int tc9562mac_set_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	u32 support = WAKE_MAGIC | WAKE_UCAST;

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_set_wol\n");
	
	/* By default almost all GMAC devices support the WoL via
	 * magic frame but we can disable it if the HW capability
	 * register shows no support for pmt_magic_frame. */
	if ((priv->hw_cap_support) && (!priv->dma_cap.pmt_magic_frame))
		wol->wolopts &= ~WAKE_MAGIC;

	if (!device_can_wakeup(priv->device))
		return -EINVAL;

	if (wol->wolopts & ~support)
		return -EINVAL;

	if (wol->wolopts) {
		pr_info("tc9562mac: wakeup enable\n");
		device_set_wakeup_enable(priv->device, 1);
		enable_irq_wake(priv->wol_irq);
	} else {
		device_set_wakeup_enable(priv->device, 0);
		disable_irq_wake(priv->wol_irq);
	}

	spin_lock_irq(&priv->lock);
	priv->wolopts = wol->wolopts;
	spin_unlock_irq(&priv->lock);

	DBGPR_FUNC_ETHTOOL("<--tc9562mac_set_wol\n");
	return 0;
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static int tc9562mac_ethtool_op_get_eee(struct net_device *dev,
				     struct ethtool_eee *edata)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_ethtool_op_get_eee\n");
	
	if (!priv->dma_cap.eee)
		return -EOPNOTSUPP;

	edata->eee_enabled = priv->eee_enabled;
	edata->eee_active = priv->eee_active;
	edata->tx_lpi_timer = priv->tx_lpi_timer;

	DBGPR_FUNC_ETHTOOL("<--tc9562mac_ethtool_op_get_eee\n");

	return phy_ethtool_get_eee(dev->phydev, edata);
}

static int tc9562mac_ethtool_op_set_eee(struct net_device *dev,
				     struct ethtool_eee *edata)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_ethtool_op_set_eee\n");

	priv->eee_enabled = edata->eee_enabled;

	if (!priv->eee_enabled)
		tc9562mac_disable_eee_mode(priv);
	else {
		/* We are asking for enabling the EEE but it is safe
		 * to verify all by invoking the eee_init function.
		 * In case of failure it will return an error.
		 */
		priv->eee_enabled = tc9562mac_eee_init(priv);
		if (!priv->eee_enabled)
			return -EOPNOTSUPP;

		/* Do not change tx_lpi_timer in case of failure */
		priv->tx_lpi_timer = edata->tx_lpi_timer;
	}

	DBGPR_FUNC_ETHTOOL("<--tc9562mac_ethtool_op_set_eee\n");

	return phy_ethtool_set_eee(dev->phydev, edata);
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

static u32 tc9562mac_usec2riwt(u32 usec, struct tc9562mac_priv *priv)
{
	unsigned long clk = TC9562_PTP_SYSCLOCK ; //clk_get_rate(priv->plat->tc9562mac_clk);

	if (!clk)
		return 0;

	return (usec * (clk / 1000000)) / 256;
}

static u32 tc9562mac_riwt2usec(u32 riwt, struct tc9562mac_priv *priv)
{
	unsigned long clk = TC9562_PTP_SYSCLOCK; // clk_get_rate(priv->plat->tc9562mac_clk);

	if (!clk)
		return 0;

	return (riwt * 256) / (clk / 1000000);
}

static int tc9562mac_get_coalesce(struct net_device *dev,
			       struct ethtool_coalesce *ec)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_get_coalesce\n");

	ec->tx_coalesce_usecs = priv->tx_coal_timer;
	ec->tx_max_coalesced_frames = priv->tx_coal_frames;

	if (priv->use_riwt)
		ec->rx_coalesce_usecs = tc9562mac_riwt2usec(priv->rx_riwt, priv);

	DBGPR_FUNC_ETHTOOL("<--tc9562mac_get_coalesce\n");

	return 0;
}

static int tc9562mac_set_coalesce(struct net_device *dev,
			       struct ethtool_coalesce *ec)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	u32 rx_cnt = priv->plat->rx_queues_to_use + 1;
	unsigned int rx_riwt;

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_set_coalesce\n");

	/* Check not supported parameters  */
	if ((ec->rx_max_coalesced_frames) || (ec->rx_coalesce_usecs_irq) ||
	    (ec->rx_max_coalesced_frames_irq) || (ec->tx_coalesce_usecs_irq) ||
	    (ec->use_adaptive_rx_coalesce) || (ec->use_adaptive_tx_coalesce) ||
	    (ec->pkt_rate_low) || (ec->rx_coalesce_usecs_low) ||
	    (ec->rx_max_coalesced_frames_low) || (ec->tx_coalesce_usecs_high) ||
	    (ec->tx_max_coalesced_frames_low) || (ec->pkt_rate_high) ||
	    (ec->tx_coalesce_usecs_low) || (ec->rx_coalesce_usecs_high) ||
	    (ec->rx_max_coalesced_frames_high) ||
	    (ec->tx_max_coalesced_frames_irq) ||
	    (ec->stats_block_coalesce_usecs) ||
	    (ec->tx_max_coalesced_frames_high) || (ec->rate_sample_interval))
		return -EOPNOTSUPP;

	if (ec->rx_coalesce_usecs == 0)
		return -EINVAL;

	if ((ec->tx_coalesce_usecs == 0) &&
	    (ec->tx_max_coalesced_frames == 0))
		return -EINVAL;

	if ((ec->tx_coalesce_usecs > TC9562MAC_MAX_COAL_TX_TICK) ||
	    (ec->tx_max_coalesced_frames > TC9562MAC_TX_MAX_FRAMES))
		return -EINVAL;

	rx_riwt = tc9562mac_usec2riwt(ec->rx_coalesce_usecs, priv);

	if ((rx_riwt > MAX_DMA_RIWT) || (rx_riwt < MIN_DMA_RIWT))
		return -EINVAL;
	else if (!priv->use_riwt)
		return -EOPNOTSUPP;

	/* Only copy relevant parameters, ignore all others. */
	priv->tx_coal_frames = ec->tx_max_coalesced_frames;
	priv->tx_coal_timer = ec->tx_coalesce_usecs;
	priv->rx_riwt = rx_riwt;
	priv->hw->dma->rx_watchdog(priv->ioaddr, priv->rx_riwt, rx_cnt);

	DBGPR_FUNC_ETHTOOL("<--tc9562mac_set_coalesce\n");
	
	return 0;
}

static int tc9562mac_get_ts_info(struct net_device *dev,
			      struct ethtool_ts_info *info)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_get_ts_info\n");
	
	if ((priv->dma_cap.time_stamp || priv->dma_cap.atime_stamp)) {

		info->so_timestamping = SOF_TIMESTAMPING_TX_SOFTWARE |
					SOF_TIMESTAMPING_TX_HARDWARE |
					SOF_TIMESTAMPING_RX_SOFTWARE |
					SOF_TIMESTAMPING_RX_HARDWARE |
					SOF_TIMESTAMPING_SOFTWARE |
					SOF_TIMESTAMPING_RAW_HARDWARE;

		if (priv->ptp_clock)
			info->phc_index = ptp_clock_index(priv->ptp_clock);

		info->tx_types = (1 << HWTSTAMP_TX_OFF) | (1 << HWTSTAMP_TX_ON);

		info->rx_filters = ((1 << HWTSTAMP_FILTER_NONE) |
				    (1 << HWTSTAMP_FILTER_PTP_V1_L4_EVENT) |
				    (1 << HWTSTAMP_FILTER_PTP_V1_L4_SYNC) |
				    (1 << HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_L4_EVENT) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_L4_SYNC) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_EVENT) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_SYNC) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_DELAY_REQ) |
				    (1 << HWTSTAMP_FILTER_ALL));
		return 0;
	} else
		return ethtool_op_get_ts_info(dev, info);

	DBGPR_FUNC_ETHTOOL("<--tc9562mac_get_ts_info\n");
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static int tc9562mac_get_tunable(struct net_device *dev,
			      const struct ethtool_tunable *tuna, void *data)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	int ret = 0;

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_get_tunable\n");

	switch (tuna->id) {
	case ETHTOOL_RX_COPYBREAK:
		*(u32 *)data = priv->rx_copybreak;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	DBGPR_FUNC_ETHTOOL("<--tc9562mac_get_tunable\n");

	return ret;
}

static int tc9562mac_set_tunable(struct net_device *dev,
			      const struct ethtool_tunable *tuna,
			      const void *data)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	int ret = 0;

	DBGPR_FUNC_ETHTOOL("-->tc9562mac_set_tunable\n");
		
	switch (tuna->id) {
	case ETHTOOL_RX_COPYBREAK:
		priv->rx_copybreak = *(u32 *)data;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	DBGPR_FUNC_ETHTOOL("<--tc9562mac_set_tunable\n");
	
	return ret;
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */


static const struct ethtool_ops tc9562mac_ethtool_ops = {
	.begin = tc9562mac_check_if_running,
	.get_drvinfo = tc9562mac_ethtool_getdrvinfo,
	.get_msglevel = tc9562mac_ethtool_getmsglevel,
	.set_msglevel = tc9562mac_ethtool_setmsglevel,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	.get_regs = tc9562mac_ethtool_gregs,
	.get_regs_len = tc9562mac_ethtool_get_regs_len,
#endif  /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */	
	.get_link = ethtool_op_get_link,
	
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0))
	.nway_reset = phy_ethtool_nway_reset,
#endif
	.get_pauseparam = tc9562mac_get_pauseparam,
	.set_pauseparam = tc9562mac_set_pauseparam,
	.get_ethtool_stats = tc9562mac_get_ethtool_stats,
	.get_strings = tc9562mac_get_strings,
	.get_wol = tc9562mac_get_wol,
	.set_wol = tc9562mac_set_wol,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE	
	.get_eee = tc9562mac_ethtool_op_get_eee,
	.set_eee = tc9562mac_ethtool_op_set_eee,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */	
	.get_sset_count	= tc9562mac_get_sset_count,
	.get_ts_info = tc9562mac_get_ts_info,
	.get_coalesce = tc9562mac_get_coalesce,
	.set_coalesce = tc9562mac_set_coalesce,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE	
	.get_tunable = tc9562mac_get_tunable,
	.set_tunable = tc9562mac_set_tunable,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 10, 0))
    .get_settings = tc9562mac_ethtool_getsettings,
    .set_settings = tc9562mac_ethtool_setsettings,
#else
    .get_link_ksettings = tc9562mac_ethtool_get_link_ksettings,
    .set_link_ksettings = tc9562mac_ethtool_set_link_ksettings,
#endif
	
};

void tc9562mac_set_ethtool_ops(struct net_device *netdev)
{
	DBGPR_FUNC_ETHTOOL("-->tc9562mac_set_ethtool_ops\n");
	netdev->ethtool_ops = &tc9562mac_ethtool_ops;
	DBGPR_FUNC_ETHTOOL("<--tc9562mac_set_ethtool_ops\n");
}
