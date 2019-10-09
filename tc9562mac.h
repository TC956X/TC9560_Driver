/*
 * TC9562 ethernet driver.
 *
 * tc9562mac.h
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

#ifndef __TC9562MAC_H__
#define __TC9562MAC_H__

#define TC9562_RESOURCE_NAME   "tc9562pci"
#define DRV_MODULE_VERSION	"V_01-00"

#include <linux/clk.h>
#include "tc9562mac_inc.h"
//#include <linux/tc9562mac.h>
#include <linux/phy.h>
#include <linux/pci.h>
#include "common.h"
#include <linux/ptp_clock_kernel.h>
#include <linux/reset.h>
#include <linux/ctype.h>

/* select any one of the following interface */
#define ENABLE_RMII_INTERFACE               0
#define ENABLE_RGMII_INTERFACE              1
#define ENABLE_SGMII_INTERFACE		    	2

#define INTERFACE_SELECTED                 ENABLE_RGMII_INTERFACE 
//#define INTERFACE_SELECTED                 ENABLE_SGMII_INTERFACE  

/* M3 debug address */
#define M3_SRAM_FW_VER_OFFSET 0x4f900 /* DMEM addrs 0x2000F900 */

struct tc9562mac_resources {
	void __iomem *addr;
	void __iomem *tc9562_SRAM_pci_base_addr;
	void __iomem *tc9562_FLASH_pci_base_addr;
	char *mac;
	int wol_irq;
	int lpi_irq;
	int irq;
};

struct tc9562mac_tx_info {
	dma_addr_t buf;
	bool map_as_page;
	unsigned len;
	bool last_segment;
	bool is_jumbo;
};

/* Frequently used values are kept adjacent for cache effect */
struct tc9562mac_tx_queue {
	u32 queue_index;
	struct tc9562mac_priv *priv_data;
	struct dma_extended_desc *dma_etx ____cacheline_aligned_in_smp;
	struct dma_desc *dma_tx;
	struct dma_enhanced_desc *dma_entx ____cacheline_aligned_in_smp;
	struct sk_buff **tx_skbuff;
	struct tc9562mac_tx_info *tx_skbuff_dma;
	unsigned int cur_tx;
	unsigned int dirty_tx;
	dma_addr_t dma_tx_phy;
	u32 tx_tail_addr;
};

struct tc9562mac_rx_queue {
	u32 queue_index;
	struct tc9562mac_priv *priv_data;
	struct dma_extended_desc *dma_erx;
	struct dma_desc *dma_rx ____cacheline_aligned_in_smp;
	struct sk_buff **rx_skbuff;
	dma_addr_t *rx_skbuff_dma;
	unsigned int cur_rx;
	unsigned int dirty_rx;
	u32 rx_zeroc_thresh;
	dma_addr_t dma_rx_phy;
	u32 rx_tail_addr;
	struct napi_struct napi ____cacheline_aligned_in_smp;
};

struct tc9562_cbs_params {
	u32 send_slope;
	u32 idle_slope;
	u32 high_credit;
	u32 low_credit;
	u32 percentage;
};

struct tc9562mac_priv {
	/* Frequently used values are kept adjacent for cache effect */
	u32 tx_count_frames;
	u32 tx_coal_frames;
	u32 tx_coal_timer;

	/* Adapter state */
	unsigned long state;
	struct workqueue_struct *wq;
	struct work_struct service_task;

	int tx_coalesce;
	int hwts_tx_en;
	bool tx_path_in_lpi_mode;
	struct timer_list txtimer;
	bool tso;

	unsigned int dma_buf_sz;
	unsigned int rx_copybreak;
	u32 rx_riwt;
	int hwts_rx_en;

	void __iomem *ioaddr;
	void __iomem *tc9562_SRAM_pci_base_addr;
	void __iomem *tc9562_FLASH_pci_base_addr;
	struct net_device *dev;
	struct device *device;
	struct mac_device_info *hw;
	spinlock_t lock;

	/* RX Queue */
	struct tc9562mac_rx_queue rx_queue[MTL_MAX_RX_QUEUES];

	/* TX Queue */
	struct tc9562mac_tx_queue tx_queue[MTL_MAX_TX_QUEUES];

	bool oldlink;
	int speed;
	int oldduplex;
	unsigned int flow_ctrl;
	unsigned int pause;
	struct mii_bus *mii;
	int mii_irq[PHY_MAX_ADDR];

	struct tc9562mac_extra_stats xstats ____cacheline_aligned_in_smp;
	struct plat_tc9562macenet_data *plat;
	struct dma_features dma_cap;
	struct tc9562mac_counters mmc;
	int hw_cap_support;
	int synopsys_id;
	u32 msg_enable;
	int wolopts;
	int wol_irq;
	int clk_csr;
	struct timer_list eee_ctrl_timer;
	int lpi_irq;
	int eee_enabled;
	int eee_active;
	int tx_lpi_timer;
	unsigned int mode;
	int extend_desc;
	struct ptp_clock *ptp_clock;
	struct ptp_clock_info ptp_clock_ops;
	unsigned int default_addend;
	u32 adv_ts;
	int use_riwt;
	int irq_wake;
	spinlock_t ptp_lock;
	spinlock_t mmc_lock;
	void __iomem *mmcaddr;
	void __iomem *ptpaddr;
	u32 mss;

#ifdef CONFIG_DEBUG_FS
	struct dentry *dbgfs_dir;
	struct dentry *dbgfs_rings_status;
	struct dentry *dbgfs_dma_cap;
#endif
	u8 pcp_hi;
	u8 pcp_lo;
	u8 tsn_ready:1;
	u8 tsn_vlan_added:1;
	/* Reserved BW for class A and B */
	s32 sra_idleslope_res;
	s32 srb_idleslope_res;

	/* Enhancements to Scheduling Traffic */
	bool est_enabled;

	/* Safety Features */
	bool ecc_enabled;
	bool ecc_err_inject;
	u32 ecc_err_where;
	bool ecc_err_correctable;

	/* RX Parser */
	bool rxp_enabled;
	/* l2 filtering mode */	
	unsigned int l2_filtering_mode; /* 0 - if perfect and 1 - if hash filtering */
	int irq_number;
	int csum_insertion;
	
	/* SA MAC addr insertion/replacement */
	u32 sa_vlan_ins_via_desc;
	u32 sa_vlan_ins_via_reg;
	unsigned char ins_mac_addr[ETH_ALEN];
	
	/* VLAN Filtering */
	unsigned int vlan_hash_filtering;
	
	/* set to 1 when ptp offload is enabled, else 0. */
	u32 ptp_offload;
	/* ptp offloading mode - ORDINARY_SLAVE, ORDINARY_MASTER,
     * TRANSPARENT_SLAVE, TRANSPARENT_MASTER, PTOP_TRANSPERENT.
     * */
	u32 ptp_offloading_mode;
	
	/* CBS configurations */
	struct tc9562_cbs_params cbs_speed100_cfg[6];
	struct tc9562_cbs_params cbs_speed1000_cfg[6];
	int cbs_cfg_status[6];
	u32 mac_loopback_mode;
	u32 phy_loopback_mode;
};

enum tc9562mac_state {
	TC9562MAC_DOWN,
	TC9562MAC_RESET_REQUESTED,
	TC9562MAC_RESETTING,
	TC9562MAC_SERVICE_SCHED,
};

typedef struct tc9562_version_s
{
        unsigned char rel_dbg; // 'R' for release, 'D' for debug
        unsigned char major;
        unsigned char minor;
        unsigned char sub_minor;
}tc9562_version_t;

void tc9562mac_init_tsn(struct net_device *ndev);
#ifndef TC9562_DEFINED
int tc9562mac_tsn_capable(struct net_device *ndev);
int tc9562mac_tsn_link_configure(struct net_device *ndev, enum sr_class class,
			      u16 framesize, u16 vid, u8 add_link, u8 pcp_hi,
			      u8 pcp_lo);
#endif
u16 tc9562mac_tsn_select_queue(struct net_device *netdev, struct sk_buff *skb,
			    void *accel_priv, select_queue_fallback_t fallback);
int tc9562mac_mdio_unregister(struct net_device *ndev);
int tc9562mac_mdio_register(struct net_device *ndev);
int tc9562mac_mdio_reset(struct mii_bus *mii);
void tc9562mac_set_ethtool_ops(struct net_device *netdev);

void tc9562mac_ptp_register(struct tc9562mac_priv *priv);
void tc9562mac_ptp_unregister(struct tc9562mac_priv *priv);
int tc9562mac_resume(struct device *dev);
int tc9562mac_suspend(struct device *dev);
int tc9562mac_dvr_remove(struct device *dev);
int tc9562mac_dvr_probe(struct device *device,
		     struct plat_tc9562macenet_data *plat_dat,
		     struct tc9562mac_resources *res);
void tc9562mac_disable_eee_mode(struct tc9562mac_priv *priv);
bool tc9562mac_eee_init(struct tc9562mac_priv *priv);
void tc9562mac_update_safety_feat(struct tc9562mac_priv *priv);
#ifdef NETIF_F_HW_VLAN_CTAG_RX
int tc9562_vlan_rx_add_vid(struct net_device *dev, __always_unused __be16 proto, u16 vid);
int tc9562_vlan_rx_kill_vid(struct net_device *dev, __always_unused __be16 proto, u16 vid);
#else
int tc9562_vlan_rx_add_vid(struct net_device *dev, u16 vid);
int tc9562_vlan_rx_kill_vid(struct net_device *dev, u16 vid);
#endif
int tc9562mac_mdio_read_direct(struct tc9562mac_priv *, int, int , int *);
int tc9562mac_mdio_write_direct(struct tc9562mac_priv *, int, int, u16);
int tc9562_load_firmware(struct device* dev, struct tc9562mac_resources* res);

#endif /* __TC9562MAC_H__ */
