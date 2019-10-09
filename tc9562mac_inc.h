/*
 * TC9562 ethernet driver.
 *
 * tc9562mac_inc.h
 *
 * Copyright (C) 2009 STMicroelectronics Ltd
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

#ifndef __TC9562MAC_PLATFORM_DATA
#define __TC9562MAC_PLATFORM_DATA

#include <linux/platform_device.h>

//#define FPE	1   

#define MTL_MAX_RX_QUEUES	8
#define MTL_MAX_TX_QUEUES	8

#define TC9562MAC_RX_COE_NONE	0
#define TC9562MAC_RX_COE_TYPE1	1
#define TC9562MAC_RX_COE_TYPE2	2

/* Define the macros for CSR clock range parameters to be passed by
 * platform code.
 * This could also be configured at run time using CPU freq framework. */

/* MDC Clock Selection define*/
#define	TC9562MAC_CSR_60_100M	0x0	/* MDC = clk_scr_i/42 */
#define	TC9562MAC_CSR_100_150M	0x1	/* MDC = clk_scr_i/62 */
#define	TC9562MAC_CSR_20_35M	0x2	/* MDC = clk_scr_i/16 */
#define	TC9562MAC_CSR_35_60M	0x3	/* MDC = clk_scr_i/26 */
#define	TC9562MAC_CSR_150_250M	0x4	/* MDC = clk_scr_i/102 */
#define	TC9562MAC_CSR_250_300M	0x5	/* MDC = clk_scr_i/122 */

/* MTL algorithms identifiers */
#define MTL_TX_ALGORITHM_WRR	0x0
#define MTL_TX_ALGORITHM_WFQ	0x1
#define MTL_TX_ALGORITHM_DWRR	0x2
#define MTL_TX_ALGORITHM_SP	0x3
#define MTL_RX_ALGORITHM_SP	0x4
#define MTL_RX_ALGORITHM_WSP	0x5

/* RX/TX Queue Mode */
#define MTL_QUEUE_AVB		0x0
#define MTL_QUEUE_DCB		0x1

/* The MDC clock could be set higher than the IEEE 802.3
 * specified frequency limit 0f 2.5 MHz, by programming a clock divider
 * of value different than the above defined values. The resultant MDIO
 * clock frequency of 12.5 MHz is applicable for the interfacing chips
 * supporting higher MDC clocks.
 * The MDC clock selection macros need to be defined for MDC clock rate
 * of 12.5 MHz, corresponding to the following selection.
 */
#define TC9562MAC_CSR_I_4		0x8	/* clk_csr_i/4 */
#define TC9562MAC_CSR_I_6		0x9	/* clk_csr_i/6 */
#define TC9562MAC_CSR_I_8		0xA	/* clk_csr_i/8 */
#define TC9562MAC_CSR_I_10		0xB	/* clk_csr_i/10 */
#define TC9562MAC_CSR_I_12		0xC	/* clk_csr_i/12 */
#define TC9562MAC_CSR_I_14		0xD	/* clk_csr_i/14 */
#define TC9562MAC_CSR_I_16		0xE	/* clk_csr_i/16 */
#define TC9562MAC_CSR_I_18		0xF	/* clk_csr_i/18 */

/* AXI DMA Burst length supported */
#define DMA_AXI_BLEN_4		(1 << 1)
#define DMA_AXI_BLEN_8		(1 << 2)
#define DMA_AXI_BLEN_16		(1 << 3)
#define DMA_AXI_BLEN_32		(1 << 4)
#define DMA_AXI_BLEN_64		(1 << 5)
#define DMA_AXI_BLEN_128	(1 << 6)
#define DMA_AXI_BLEN_256	(1 << 7)
#define DMA_AXI_BLEN_ALL (DMA_AXI_BLEN_4 | DMA_AXI_BLEN_8 | DMA_AXI_BLEN_16 \
			| DMA_AXI_BLEN_32 | DMA_AXI_BLEN_64 \
			| DMA_AXI_BLEN_128 | DMA_AXI_BLEN_256)

#define MTL_FPE_AFSZ_64		0
#define MTL_FPE_AFSZ_128	1
#define MTL_FPE_AFSZ_192	2
#define MTL_FPE_AFSZ_256	3
/* Platfrom data for platform device structure's platform_data field */

struct tc9562mac_mdio_bus_data {
	int (*phy_reset)(void *priv);
	unsigned int phy_mask;
	int *irqs;
	int probed_phy_irq;
#ifdef CONFIG_OF
	int reset_gpio, active_low;
	u32 delays[3];
#endif
};

struct tc9562mac_dma_cfg {
	int pbl;
	int txpbl;
	int rxpbl;
	bool pblx8;
	int fixed_burst;
	int mixed_burst;
	bool aal;
};

#define AXI_BLEN	7
struct tc9562mac_axi {
	bool axi_lpi_en;
	bool axi_xit_frm;
	u32 axi_wr_osr_lmt;
	u32 axi_rd_osr_lmt;
	bool axi_kbbe;
	u32 axi_blen[AXI_BLEN];
	bool axi_fb;
	bool axi_mb;
	bool axi_rb;
};

struct tc9562mac_rxq_cfg {
	u8 mode_to_use;
	u32 chan;
	u8 pkt_route;
	bool use_prio;
	u32 prio;
};

struct tc9562mac_txq_cfg {
	u32 weight;
	u8 mode_to_use;
	/* Credit Base Shaper parameters */
	u32 send_slope;
	u32 idle_slope;
	u32 high_credit;
	u32 low_credit;
	u32 percentage;
	bool use_prio;
	u32 prio;
};

#define TC9562MAC_EST_GCL_MAX_ENTRIES		256
struct tc9562mac_est_cfg {
	bool enable;
	u32 btr_offset[2];
	u32 ctr[2];
	u32 ter;
	u32 gcl[TC9562MAC_EST_GCL_MAX_ENTRIES];
	u32 gcl_size;
};

struct tc9562mac_fpe_cfg {
	bool enable;
	u32 pec_cfg;
	u32 afsz_cfg;	
        u32 RA_time;
        u32 HA_time;
};

struct tc9562mac_rx_parser_entry {
	__le32 match_data;
	__le32 match_en;
	u8 af:1;
	u8 rf:1;
	u8 im:1;
	u8 nc:1;
	u8 res1:4;
	u8 frame_offset;
	u8 ok_index;
	u8 dma_ch_no;
	__le32 res2;
} __packed;

#define TC9562MAC_RX_PARSER_MAX_ENTRIES		64
struct tc9562mac_rx_parser_cfg {
	bool enable;
	u32 nve;
	u32 npe;
	struct tc9562mac_rx_parser_entry entries[TC9562MAC_RX_PARSER_MAX_ENTRIES];
};

#define TC9562MAC_PPS_MAX_NUM			4
struct tc9562mac_pps_cfg {
	bool enable;
	u32 ctrl_cmd;
	u32 trgtmodsel;
	u32 target_time[2];
	u32 interval;
	u32 width;
};

struct tc9562mac_mcgr_cfg {
	bool enable;
	u32 ctrl;
	u32 debug_route;
};

struct plat_tc9562macenet_data {
	int bus_id;
	int phy_addr;
	int interface;
	struct tc9562mac_mdio_bus_data *mdio_bus_data;
	struct device_node *phy_node;
	struct device_node *mdio_node;
	struct tc9562mac_dma_cfg *dma_cfg;
	struct pci_dev *pdev;
	int clk_csr;
	int has_gmac;
	int enh_desc;
	int tx_coe;
	int rx_coe;
	int bugged_jumbo;
	int pmt;
	int force_sf_dma_mode;
	int force_thresh_dma_mode;
	int force_no_rx_coe;
	int force_no_tx_coe;
	int riwt_off;
	int max_speed;
	int maxmtu;
	int multicast_filter_bins;
	int unicast_filter_entries;
	int tx_fifo_size;
	int rx_fifo_size;
	u32 rx_queues_to_use;
	u32 tx_queues_to_use;
	u8 rx_sched_algorithm;
	u8 tx_sched_algorithm;
	struct tc9562mac_rxq_cfg rx_queues_cfg[MTL_MAX_RX_QUEUES];
	struct tc9562mac_txq_cfg tx_queues_cfg[MTL_MAX_TX_QUEUES];
	void (*fix_mac_speed)(void *priv, unsigned int speed);
	int (*init)(struct platform_device *pdev, void *priv);
	void (*exit)(struct platform_device *pdev, void *priv);
	struct mac_device_info *(*setup)(void *priv);
	void *bsp_priv;
	struct clk *tc9562mac_clk;
	struct clk *pclk;
	struct clk *clk_ptp_ref;
	unsigned int clk_ptp_rate;
	struct reset_control *tc9562mac_rst;
	struct tc9562mac_axi *axi;
	int has_gmac4;
	bool has_sun8i;
	bool tso_en;
	int mac_port_sel_speed;
	bool en_tx_lpi_clockgating;
	bool fp_en;
	struct tc9562mac_fpe_cfg fpe_cfg;
	struct tc9562mac_est_cfg est_cfg;
	struct tc9562mac_rx_parser_cfg rxp_cfg;
	struct tc9562mac_pps_cfg pps_cfg[TC9562MAC_PPS_MAX_NUM];
	struct tc9562mac_mcgr_cfg mcgr_cfg[TC9562MAC_PPS_MAX_NUM];
};
#endif
