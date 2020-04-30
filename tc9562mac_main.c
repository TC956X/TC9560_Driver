/*
 * TC9562 ethernet driver.
 *
 * tc9562mac_main.c
 *
 * Copyright (C) 2007-2011 STMicroelectronics Ltd
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
 *  26 Feb 2020 : 1. Added Suspend/Resume fix.
                  2. BCM Driver update for NTN2.
                  3. Added Unified Firmware feature.
                  4. Added SGMII Interface support.
                  5. Added 4.19 kernel support.
                  6. Added TC - CBS and launch time feature support.
 *  VERSION     : 01-001
 *  30 Sep 2019 : Base lined
 *  VERSION     : 01-00
 */

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/if.h>
#include <linux/if_vlan.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/prefetch.h>
#include <linux/pinctrl/consumer.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif /* CONFIG_DEBUG_FS */
#include <linux/net_tstamp.h>
#include "tc9562mac_ptp.h"
#include "tc9562mac.h"
#include <linux/reset.h>
#include <linux/of_mdio.h>
#ifdef TC9562_DEFINED
#include "dwmac4.h"
#include "dwmac4_dma.h"
#else
#include "dwmac1000.h"
#endif
#include "tc9562mac_ioctl.h"
#include "dwmac5.h"
#include "common.h"
#include "mmc.h"

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 19, 8))
#include <linux/sizes.h>
#include <linux/netdev_features.h>
#include <linux/mdio.h>
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,15))
#include <net/pkt_cls.h>
#endif
#define TC9562_MSIGEN_VERIFICATION
//#define ENABLE_EST
#define TEMP_SG_4TSO
#define TC9562MAC_ALIGN(x)	L1_CACHE_ALIGN(x)
#define	TSO_MAX_BUFF_SIZE	(SZ_16K - 1)
#define	UFO_USO_MSS_SIZE	(1200) /*Should be in multiple of 8 for UFO*/
#define TC9562_ADDL_BUF_SIZE (128) /* eMAC cannot write the received frame data to PCIe controller in some cases. Additional buffer size required to avoid this issue */

/* Module parameters */
#define TX_TIMEO	15000000//5000//15000000
#define PPS_START_DELAY			100000000   /* 100 ms, in unit of ns */
static int watchdog = TX_TIMEO;
module_param(watchdog, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(watchdog, "Transmit timeout in milliseconds (default 5s)");

static int debug = -1;
module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Message Level (-1: default, 0: no output, 16: all)");

static int phyaddr = -1;
module_param(phyaddr, int, S_IRUGO);
MODULE_PARM_DESC(phyaddr, "Physical device address");

#define TC9562MAC_TX_THRESH	(DMA_TX_SIZE / 4)
#define TC9562MAC_RX_THRESH	(DMA_RX_SIZE / 4)

static int flow_ctrl = FLOW_OFF;
module_param(flow_ctrl, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(flow_ctrl, "Flow control ability [on/off]");

static int pause = PAUSE_TIME;//should be valid small value and not 0xffff
module_param(pause, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(pause, "Flow Control Pause Time");

#define TC_DEFAULT 64
static int tc = TC_DEFAULT;
module_param(tc, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tc, "DMA threshold control value");

#define	DEFAULT_BUFSIZE	1536
static int buf_sz = DEFAULT_BUFSIZE;
module_param(buf_sz, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(buf_sz, "DMA buffer size");

#define	TC9562MAC_RX_COPYBREAK	256

static const u32 default_msg_level = (NETIF_MSG_DRV | NETIF_MSG_PROBE |
				      NETIF_MSG_LINK | NETIF_MSG_IFUP |
				      NETIF_MSG_IFDOWN | NETIF_MSG_TIMER);

#define TC9562MAC_DEFAULT_LPI_TIMER	1000
static int eee_timer = TC9562MAC_DEFAULT_LPI_TIMER;
module_param(eee_timer, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(eee_timer, "LPI tx expiration time in msec");
#define TC9562MAC_LPI_T(x) (jiffies + msecs_to_jiffies(x))

/* By default the driver will use the ring mode to manage tx and rx descriptors,
 * but allow user to force to use the chain instead of the ring
 */
static unsigned int chain_mode;
module_param(chain_mode, int, S_IRUGO);
MODULE_PARM_DESC(chain_mode, "To use chain instead of ring mode");

static irqreturn_t tc9562mac_interrupt(int irq, void *dev_id);
void tc9562_ptp_configuration(struct tc9562mac_priv *priv);

#ifdef CONFIG_DEBUG_FS
static int tc9562mac_init_fs(struct net_device *dev);
static void tc9562mac_exit_fs(struct net_device *dev);
#endif

#define TC9562MAC_COAL_TIMER(x) (jiffies + usecs_to_jiffies(x))
u8 dev_addr[6] = { 0xEC, 0x21, 0xE5, 0x10, 0x4F, 0xEA};
u8 dev_cm3_addr[6] = { 0xE8, 0xE0, 0xB7, 0xB5, 0x7D, 0xF8};
u8 dev_bc_addr[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
u8 dev_mc_addr[6] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};

typedef struct {
	char mdio_key[32];
	unsigned short mdio_key_len;
	char mac_key[32];
	unsigned short mac_key_len;
	unsigned short mac_str_len;
	char mac_str_def[20];
} config_param_list_t;

const config_param_list_t config_param_list[] = {
{"MDIOBUSID", 9, "MAC_ID", 6, 17, "00:00:00:00:00:00"},
};

uint16_t mdio_bus_id;
#define CONFIG_PARAM_NUM (sizeof(config_param_list)/sizeof(config_param_list[0]))

#ifdef TC9562_POLLING_METHOD
static void *polling_task_data;
static struct delayed_work task;
irqreturn_t tc9562mac_interrupt_dummy(int irq, void *dev_id);
static void polling_task(void)
{
	struct tc9562mac_priv *priv = netdev_priv(polling_task_data);

	DBGPR_FUNC("-->polling_task");

	if (priv == NULL) {
		NMSGPR_ALERT("polling_task data pointer is NULL");
		return;
	}

	disable_irq(priv->irq_number);
    tc9562mac_interrupt(priv->irq_number, polling_task_data);
	enable_irq(priv->irq_number);

	DBGPR_FUNC("<--polling_task");

	schedule_delayed_work(&task, usecs_to_jiffies(TC9562_POLL_DELAY_US));
	return;
}
#endif //TC9562_POLLING_METHOD

/**
 * tc9562mac_verify_args - verify the driver parameters.
 * Description: it checks the driver parameters and set a default in case of
 * errors.
 */
static void tc9562mac_verify_args(void)
{
	DBGPR_FUNC("-->tc9562mac_verify_args\n");
	
	if (unlikely(watchdog < 0))
		watchdog = TX_TIMEO;
	if (unlikely((buf_sz < DEFAULT_BUFSIZE) || (buf_sz > BUF_SIZE_16KiB)))
		buf_sz = DEFAULT_BUFSIZE;
	if (unlikely(flow_ctrl > 1))
		flow_ctrl = FLOW_AUTO;
	else if (likely(flow_ctrl < 0))
		flow_ctrl = FLOW_OFF;
	if (unlikely((pause < 0) || (pause > 0xffff)))
		pause = PAUSE_TIME;
	if (eee_timer < 0)
		eee_timer = TC9562MAC_DEFAULT_LPI_TIMER;
	
	DBGPR_FUNC("<--tc9562mac_verify_args\n");
}

/**
 * tc9562mac_disable_all_queues - Disable all queues
 * @priv: driver private structure
 */
static void tc9562mac_disable_all_queues(struct tc9562mac_priv *priv)
{
	u32 napi_queues_cnt = priv->plat->tx_queues_to_use;
	u32 queue;
	
	DBGPR_FUNC("-->tc9562mac_disable_all_queues\n");
	
	for (queue = 0; queue < napi_queues_cnt; queue++) {
		struct tc9562mac_rx_queue *rx_q = &priv->rx_queue[queue];

	    napi_disable(&rx_q->napi);
	}
	
	DBGPR_FUNC("<--tc9562mac_disable_all_queues\n");
}

/**			
 * tc9562mac_enable_all_queues - Enable all queues
 * @priv: driver private structure
 */
static void tc9562mac_enable_all_queues(struct tc9562mac_priv *priv)
{
	u32 napi_queues_cnt = priv->plat->tx_queues_to_use;
	u32 queue;
	
	DBGPR_FUNC("-->tc9562mac_enable_all_queues\n");
	
	for (queue = 0; queue < napi_queues_cnt; queue++) {
		struct tc9562mac_rx_queue *rx_q = &priv->rx_queue[queue];

		napi_enable(&rx_q->napi);
	}
	
	DBGPR_FUNC("<--tc9562mac_enable_all_queues\n");
}

/**
 * tc9562mac_stop_all_queues - Stop all queues
 * @priv: driver private structure
 */
static void tc9562mac_stop_all_queues(struct tc9562mac_priv *priv)
{
	u32 tx_queues_cnt = priv->plat->tx_queues_to_use;
	u32 queue;

	DBGPR_FUNC("-->tc9562mac_stop_all_queues\n");

	for (queue = 0; queue < tx_queues_cnt; queue++)
		netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, queue));

	DBGPR_FUNC("<--tc9562mac_stop_all_queues\n");
}

/**
 * tc9562mac_start_all_queues - Start all queues
 * @priv: driver private structure
 */
static void tc9562mac_start_all_queues(struct tc9562mac_priv *priv)
{
	u32 tx_queues_cnt = priv->plat->tx_queues_to_use;
	u32 queue;

	DBGPR_FUNC("-->tc9562mac_start_all_queues\n");

	for (queue = 0; queue < tx_queues_cnt; queue++)
		netif_tx_start_queue(netdev_get_tx_queue(priv->dev, queue));

	DBGPR_FUNC("<--tc9562mac_start_all_queues\n");
}

/**
 * tc9562mac_reset_queues_param - reset queue parameters
 * @dev: device pointer
 */
static void tc9562mac_reset_queues_param(struct tc9562mac_priv *priv)
{
	u32 rx_cnt = priv->plat->rx_queues_to_use + 1;
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	u32 queue;
	struct tc9562mac_rx_queue *rx_q;
	struct tc9562mac_tx_queue *tx_q;

	DBGPR_FUNC("-->tc9562mac_reset_queues_param\n");

	for (queue = 0; queue < rx_cnt; queue++) {
#ifdef UNIFIED_DRIVER
		if(priv->plat->rx_dma_ch_for_host[queue] != 0)    
			continue;
#endif
        rx_q = &priv->rx_queue[queue];

		rx_q->cur_rx = 0;
		rx_q->dirty_rx = 0;
	}

	for (queue = 0; queue < tx_cnt; queue++) {
#ifdef UNIFIED_DRIVER
		if(priv->plat->tx_dma_ch_for_host[queue] == 0)
			continue;
#endif
        tx_q = &priv->tx_queue[queue];

		tx_q->cur_tx = 0;
		tx_q->dirty_tx = 0;
	}

	DBGPR_FUNC("<--tc9562mac_reset_queues_param\n");
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
/**
 * tc9562mac_clk_csr_set - dynamically set the MDC clock
 * @priv: driver private structure
 * Description: this is to dynamically set the MDC clock according to the csr
 * clock input.
 * Note:
 *	If a specific clk_csr value is passed from the platform
 *	this means that the CSR Clock Range selection cannot be
 *	changed at run-time and it is fixed (as reported in the driver
 *	documentation). Viceversa the driver will try to set the MDC
 *	clock dynamically according to the actual clock input.
 */
static void tc9562mac_clk_csr_set(struct tc9562mac_priv *priv)
{
	u32 clk_rate;

	DBGPR_FUNC("-->tc9562mac_clk_csr_set\n");

	clk_rate = clk_get_rate(priv->plat->tc9562mac_clk);

	/* Platform provided default clk_csr would be assumed valid
	 * for all other cases except for the below mentioned ones.
	 * For values higher than the IEEE 802.3 specified frequency
	 * we can not estimate the proper divider as it is not known
	 * the frequency of clk_csr_i. So we do not change the default
	 * divider.
	 */
	if (!(priv->clk_csr & MAC_CSR_H_FRQ_MASK)) {
		if (clk_rate < CSR_F_35M)
			priv->clk_csr = TC9562MAC_CSR_20_35M;
		else if ((clk_rate >= CSR_F_35M) && (clk_rate < CSR_F_60M))
			priv->clk_csr = TC9562MAC_CSR_35_60M;
		else if ((clk_rate >= CSR_F_60M) && (clk_rate < CSR_F_100M))
			priv->clk_csr = TC9562MAC_CSR_60_100M;
		else if ((clk_rate >= CSR_F_100M) && (clk_rate < CSR_F_150M))
			priv->clk_csr = TC9562MAC_CSR_100_150M;
		else if ((clk_rate >= CSR_F_150M) && (clk_rate < CSR_F_250M))
			priv->clk_csr = TC9562MAC_CSR_150_250M;
		else if ((clk_rate >= CSR_F_250M) && (clk_rate < CSR_F_300M))
			priv->clk_csr = TC9562MAC_CSR_250_300M;
	}
#ifndef TC9562_DEFINED
	if (priv->plat->has_sun8i) {
		if (clk_rate > 160000000)
			priv->clk_csr = 0x03;
		else if (clk_rate > 80000000)
			priv->clk_csr = 0x02;
		else if (clk_rate > 40000000)
			priv->clk_csr = 0x01;
		else
			priv->clk_csr = 0;
	}
#endif
	DBGPR_FUNC("<--tc9562mac_clk_csr_set\n");
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

static void print_pkt(unsigned char *buf, int len)
{
	pr_debug("len = %d byte, buf addr: 0x%p\n", len, buf);
	print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, buf, len);
}

static inline u32 tc9562mac_tx_avail(struct tc9562mac_priv *priv, u32 queue)
{
	struct tc9562mac_tx_queue *tx_q = &priv->tx_queue[queue];
	u32 avail;

	DBGPR_FUNC("-->tc9562mac_tx_avail\n");

	if (tx_q->dirty_tx > tx_q->cur_tx)
		avail = tx_q->dirty_tx - tx_q->cur_tx;    
	else
		avail = DMA_TX_SIZE - tx_q->cur_tx + tx_q->dirty_tx;  

	DBGPR_FUNC("<--tc9562mac_tx_avail\n");

	return avail;
}

/**
 * tc9562mac_rx_dirty - Get RX queue dirty
 * @priv: driver private structure
 * @queue: RX queue index
 */
static inline u32 tc9562mac_rx_dirty(struct tc9562mac_priv *priv, u32 queue)
{
	struct tc9562mac_rx_queue *rx_q = &priv->rx_queue[queue];
	u32 dirty;

	DBGPR_FUNC("-->tc9562mac_rx_dirty\n");

	if (rx_q->dirty_rx <= rx_q->cur_rx)
		dirty = rx_q->cur_rx - rx_q->dirty_rx;
	else
		dirty = DMA_RX_SIZE - rx_q->dirty_rx + rx_q->cur_rx;

	DBGPR_FUNC("<--tc9562mac_rx_dirty\n");

	return dirty;
}

/**
 * tc9562mac_hw_fix_mac_speed - callback for speed selection
 * @priv: driver private structure
 * Description: on some platforms (e.g. ST), some HW system configuration
 * registers have to be set according to the link speed negotiated.
 */
static inline void tc9562mac_hw_fix_mac_speed(struct tc9562mac_priv *priv)
{
	struct net_device *ndev = priv->dev;
	struct phy_device *phydev = ndev->phydev;

	DBGPR_FUNC("-->tc9562mac_hw_fix_mac_speed\n");

	if (likely(priv->plat->fix_mac_speed))
		priv->plat->fix_mac_speed(priv->plat->bsp_priv, phydev->speed);

	DBGPR_FUNC("<--tc9562mac_hw_fix_mac_speed\n");
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
/**
 * tc9562mac_enable_eee_mode - check and enter in LPI mode
 * @priv: driver private structure
 * Description: this function is to verify and enter in LPI mode in case of
 * EEE.
 */
static void tc9562mac_enable_eee_mode(struct tc9562mac_priv *priv)
{
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	u32 queue;
	struct tc9562mac_tx_queue *tx_q;

	DBGPR_FUNC("-->tc9562mac_enable_eee_mode\n");

	/* check if all TX queues have the work finished */
	for (queue = 0; queue < tx_cnt; queue++) {
#ifdef UNIFIED_DRIVER
		if(priv->plat->tx_dma_ch_for_host[queue] == 0)
			continue;
#endif
		tx_q = &priv->tx_queue[queue];

		if (tx_q->dirty_tx != tx_q->cur_tx)
			return; /* still unfinished work */
	}

	/* Check and enter in LPI mode */
	if (!priv->tx_path_in_lpi_mode)
		priv->hw->mac->set_eee_mode(priv->hw,
					    priv->plat->en_tx_lpi_clockgating);

	DBGPR_FUNC("<--tc9562mac_enable_eee_mode\n");
}

/**
 * tc9562mac_disable_eee_mode - disable and exit from LPI mode
 * @priv: driver private structure
 * Description: this function is to exit and disable EEE in case of
 * LPI state is true. This is called by the xmit.
 */
void tc9562mac_disable_eee_mode(struct tc9562mac_priv *priv)
{
	DBGPR_FUNC("-->tc9562mac_disable_eee_mode\n");

	priv->hw->mac->reset_eee_mode(priv->hw);
	del_timer_sync(&priv->eee_ctrl_timer);
	priv->tx_path_in_lpi_mode = false;

	DBGPR_FUNC("<--tc9562mac_disable_eee_mode\n");
}

/**
 * tc9562mac_eee_ctrl_timer - EEE TX SW timer.
 * @arg : data hook
 * Description:
 *  if there is no data transfer and if we are not in LPI state,
 *  then MAC Transmitter can be moved to LPI state.
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,10))
static void tc9562mac_eee_ctrl_timer(struct timer_list *t)
{
	struct tc9562mac_priv *priv = from_timer(priv, t, eee_ctrl_timer);

	DBGPR_FUNC("-->tc9562mac_eee_ctrl_timer\n");

	tc9562mac_enable_eee_mode(priv);
	mod_timer(&priv->eee_ctrl_timer, TC9562MAC_LPI_T(eee_timer));

	DBGPR_FUNC("<--tc9562mac_eee_ctrl_timer\n");
}
#else
static void tc9562mac_eee_ctrl_timer(unsigned long arg)
{
	struct tc9562mac_priv *priv = (struct tc9562mac_priv *)arg;

	DBGPR_FUNC("-->tc9562mac_eee_ctrl_timer\n");

	tc9562mac_enable_eee_mode(priv);
	mod_timer(&priv->eee_ctrl_timer, TC9562MAC_LPI_T(eee_timer));

	DBGPR_FUNC("<--tc9562mac_eee_ctrl_timer\n");
}
#endif

/**
 * tc9562mac_eee_init - init EEE
 * @priv: driver private structure
 * Description:
 *  if the GMAC supports the EEE (from the HW cap reg) and the phy device
 *  can also manage EEE, this function enable the LPI state and start related
 *  timer.
 */
bool tc9562mac_eee_init(struct tc9562mac_priv *priv)
{
	struct net_device *ndev = priv->dev;
	int interface = priv->plat->interface;
	unsigned long flags;
	bool ret = false;
	
	DBGPR_FUNC("-->tc9562mac_eee_init\n");
	
	if ((interface != PHY_INTERFACE_MODE_MII) &&
	    (interface != PHY_INTERFACE_MODE_GMII) &&
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0))
        !phy_interface_mode_is_rgmii(interface)
#else
        !(interface >= PHY_INTERFACE_MODE_RGMII &&
        interface <= PHY_INTERFACE_MODE_RGMII_TXID)
#endif
	    )
	    
		goto out;

	/* Using PCS we cannot dial with the phy registers at this stage
	 * so we do not support extra feature like EEE.
	 */
	if ((priv->hw->pcs == TC9562MAC_PCS_RGMII) ||
	    (priv->hw->pcs == TC9562MAC_PCS_TBI) ||
	    (priv->hw->pcs == TC9562MAC_PCS_RTBI))
		goto out;

	/* MAC core supports the EEE feature. */
	if (priv->dma_cap.eee) {
		int tx_lpi_timer = priv->tx_lpi_timer;

		/* Check if the PHY supports EEE */
		if (phy_init_eee(ndev->phydev, 1)) {
			/* To manage at run-time if the EEE cannot be supported
			 * anymore (for example because the lp caps have been
			 * changed).
			 * In that case the driver disable own timers.
			 */
			spin_lock_irqsave(&priv->lock, flags);
			if (priv->eee_active) {
				netdev_dbg(priv->dev, "disable EEE\n");
				del_timer_sync(&priv->eee_ctrl_timer);
				priv->hw->mac->set_eee_timer(priv->hw, 0,
							     tx_lpi_timer);
			}
			priv->eee_active = 0;
			spin_unlock_irqrestore(&priv->lock, flags);
			goto out;
		}
		/* Activate the EEE and start timers */
		spin_lock_irqsave(&priv->lock, flags);
		if (!priv->eee_active) {
			priv->eee_active = 1;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,10))
			timer_setup(&priv->eee_ctrl_timer,
				    tc9562mac_eee_ctrl_timer,
				    0);
#else
			setup_timer(&priv->eee_ctrl_timer,
				    tc9562mac_eee_ctrl_timer,
				    (unsigned long)priv);
#endif
			mod_timer(&priv->eee_ctrl_timer,
				  TC9562MAC_LPI_T(eee_timer));

			priv->hw->mac->set_eee_timer(priv->hw,
						     TC9562MAC_DEFAULT_LIT_LS,
						     tx_lpi_timer);
		}
		/* Set HW EEE according to the speed */
		priv->hw->mac->set_eee_pls(priv->hw, ndev->phydev->link);

		ret = true;
		spin_unlock_irqrestore(&priv->lock, flags);

		netdev_dbg(priv->dev, "Energy-Efficient Ethernet initialized\n");
	}
	
	DBGPR_FUNC("<--tc9562mac_eee_init\n");
out:
	return ret;
}

void tc9562mac_update_safety_feat(struct tc9562mac_priv *priv)
{
	DBGPR_FUNC("-->tc9562mac_update_safety_feat\n");
	if (priv->hw->mac->safety_feat_set)
		priv->hw->mac->safety_feat_set(priv->dev, priv->hw,
				priv->dma_cap.asp, priv->ecc_enabled,
				priv->ecc_err_inject, priv->ecc_err_where,
				priv->ecc_err_correctable);
	DBGPR_FUNC("<--tc9562mac_update_safety_feat\n");
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

#define LE_PKT_RATE_DBG
/* tc9562mac_get_tx_hwtstamp - get HW TX timestamps
 * @priv: driver private structure
 * @p : descriptor pointer
 * @skb : the socket buffer
 * Description :
 * This function will read timestamp from the descriptor & pass it to stack.
 * and also perform some sanity checks.
 */
#if defined(TX_LOGGING_TRACE)
static void tc9562mac_get_tx_hwtstamp(struct tc9562mac_priv *priv,
				   struct dma_desc *p, struct sk_buff *skb, u32 qno)
#else
static void tc9562mac_get_tx_hwtstamp(struct tc9562mac_priv *priv,
				   struct dma_desc *p, struct sk_buff *skb)
#endif

{
	struct skb_shared_hwtstamps shhwtstamp;
	u64 ns;

#if defined(TX_LOGGING_TRACE)		
		static unsigned int ccnt1=0,ccnt2=0,ccnt3=0,ccnt4=0,ccnt5=0;
#endif
	DBGPR_FUNC("-->tc9562mac_get_tx_hwtstamp\n");
	if (!priv->hwts_tx_en)
		return;

#if !defined(TX_LOGGING_TRACE)		
	/* exit if skb doesn't support hw tstamp */
	if (likely(!skb || !(skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS)))
		return;

#endif
	/* check tx tstamp status */
	if (priv->hw->desc->get_tx_timestamp_status(p)) {
#ifdef LE_PKT_RATE_DBG
static unsigned int count=0;
static u64 prev_ns=0;
u64 rate;
#endif
		/* get the valid tstamp */
		ns = priv->hw->desc->get_timestamp(p, priv->adv_ts);
#ifdef LE_PKT_RATE_DBG
if (count % 8000 == 0)
{

	rate = (8000*1000 *1000000000ULL)/(unsigned int)(ns - prev_ns);
	//printk("count ndesc: %d, rate: %lld, delta %llu \n", count, rate, ns-prev_ns); 
#ifdef FPE
#if defined(TX_LOGGING_TRACE)
			printk("Tx FPE mmc_tx_frag_cnt:%d \n",priv->mmc.mmc_tx_fpe_fragments);
#endif
#endif
  

	prev_ns = ns;
}
count++;
#endif

#ifdef FPE
#if defined(TX_LOGGING_TRACE)
		if(priv->plat->fp_en)
		{
			priv->mmc.mmc_tx_fpe_fragments += readl(priv->mmcaddr + MMC_TX_FPE_FRAGMENT);
		}
#endif
#endif

#if defined(TX_LOGGING_TRACE)

		if(qno == AVB_CLASSA_CH) // For AVB
		{
			if(skb->data[20] == 0)                                                                            
			{
					ccnt1++;
			}
			if(priv->plat->fp_en)
			{
				trace_printk("[AVB]TS,%llu,%d,%03d,%02d,\n",ns,ccnt1,skb->data[20],qno);
			}
			else
			{			
				trace_printk("[AVB]TS,%llu,%d,%03d,%02d, , \n",ns,ccnt1,skb->data[20],qno);
			}
		}
		else if(qno == AVB_CLASSB_CH) 
		{
			if(skb->data[20] == 0)                                                                            
			{
					ccnt2++;
			}
			
			trace_printk("[AVB_B]TS,%llu,%d,%03d,%02d, , \n",ns,ccnt2,skb->data[20],qno);
		}
		else if(qno == CLASS_CDT) // For CDT
		{
			if(skb->data[20] == 0)                                                                            
			{
					ccnt3++;
			}
			if(priv->plat->fp_en)
			{
				trace_printk("[CDT]TS,%llu,%d,%03d,%02d,\n",ns,ccnt3,skb->data[20],qno);
			}
			else
			{	
				trace_printk("[CDT]TS,%llu,%d,%03d,%02d, , \n",ns,ccnt3,skb->data[20],qno);
			}
		}
		else if(qno == 1) // For queue 1 dummy packet
		{
			if(skb->data[20] == 0)                                                                            
			{
					/*trace_printk("[CDT]CYCLE = %d\n",ccnt3);*/                                               
					ccnt4++;
			}
			//[CDT]TS,<timestamp>,<cycle iteration>,<sequence no>,<queue no>"			;
			if(priv->plat->fp_en)
			{
				trace_printk("[DUM1]TS,%llu,%d,%03d,%02d,\n",ns,ccnt4,skb->data[20],qno);
			}
			else
			{	
				trace_printk("[DUM1]TS,%llu,%d,%03d,%02d, , \n",ns,ccnt4,skb->data[20],qno);
			}
		}
		else if(qno == 3) // For queue 3
		{
			if(skb->data[20] == 0)                                                                            
			{
					/*trace_printk("[CDT]CYCLE = %d\n",ccnt3);*/                                               
					ccnt5++;
			}
			//[CDT]TS,<timestamp>,<cycle iteration>,<sequence no>,<queue no>"			;
			if(priv->plat->fp_en)
			{
				trace_printk("[DUM3]TS,%llu,%d,%03d,%02d,\n",ns,ccnt5,skb->data[20],qno);
			}
			else
			{	
				trace_printk("[DUM3]TS,%llu,%d,%03d,%02d, , \n",ns,ccnt5,skb->data[20],qno);
			}
		}
		else if(qno == AVB_PTPCP_CH)
		{
		  u16 gPTP_ID=0;
			u16 MsgType = 0;
																																				   
			MsgType = skb->data[14] & 0x0F ;
			gPTP_ID = skb->data[44];
			gPTP_ID = (gPTP_ID<<8) | skb->data[45];
			
			if(MsgType != 0x0b)
				//trace_printk("[gPTP]TS = %019llu CH = %02d SEQ = %04d MSG_TYPE = 0x%x\n",ns,qno,gPTP_ID,MsgType);
				trace_printk("[gPTP]TS,%019llu,%04d,0x%x,%02d\n",ns,gPTP_ID,MsgType,qno);
		}
		else if(qno == HOST_BEST_EFF_CH)
		{
			if(priv->plat->fp_en)
			{
				trace_printk("[LE]TS,%llu,00,000,%02d,\n",ns,qno);
			}
			else
			{	
				trace_printk("[LE]TS,%llu,00,000,%02d, , \n",ns,qno);
			}
		}
		else
		{}
#endif
		memset(&shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
		shhwtstamp.hwtstamp = ns_to_ktime(ns);

		netdev_dbg(priv->dev, "get valid TX hw timestamp %llu\n", ns);
		/* pass tstamp to stack */
		skb_tstamp_tx(skb, &shhwtstamp);
	}
	DBGPR_FUNC("<--tc9562mac_get_tx_hwtstamp\n");
	return;
}


#ifdef UNIFIED_DRIVER
void tc9562_propagate_link_params(struct net_device *dev)
{
    struct tc9562mac_priv *priv = netdev_priv(dev);
    struct phy_device *phydev = dev->phydev;
    
    priv->link_param.link_state = phydev->link;
    
    if (phydev->speed == SPEED_1000)
    {
        priv->link_param.link_speed = 3;
    }
    else if (phydev->speed == SPEED_100)
    {
        priv->link_param.link_speed = 2;
    }
    else
    {
        priv->link_param.link_speed = 0;
    }
    
#ifdef UNIFIED_DRIVER_TEST_DBG1
    if (priv->link_param.link_state)
    {
        DBGPR_UNIFIED_1("Host Link, Speed = %dMbps, State = Up.\n", phydev->speed);
    }
    else
    {
        DBGPR_UNIFIED_1("Host Link, Speed = NA, State = Down.\n");
    }
#endif

#ifdef DEBUG_UNIFIED	
    NMSGPR_ALERT("Propagating Updated Link Params to CM3.\n");
#endif    

    /* Copy Link data to Shared Memory Region */
    memcpy((uint8_t *)(priv->tc9562_SRAM_pci_base_addr + HOST_SHARED_MEM_OFFSET + sizeof(priv->tdm_init_config)), (uint8_t *)&priv->link_param, sizeof(priv->link_param)); 
    
    /* Raise MCU Flag to CM3 for propogating the Updated Link Parameters */
    
    writel(1 << 9, priv->ioaddr + 0x8054);  /*MCU_FLAG:9 - LINK STATUS*/
}

int tc9562_ioctl_tdm_start(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562mac_ioctl_tdm_config tdm_config;
	
	
#ifdef DEBUG_UNIFIED	
    NMSGPR_ALERT("TDM Start IOCTL Handler\n");
#endif
    /* Copy Config data to Shared Memory Region */
    

    if(data != NULL)
    {
        if(copy_from_user(&tdm_config, data, sizeof(tdm_config)))
            return -1;
        memcpy((uint8_t *)&priv->tdm_init_config, (uint8_t *)&tdm_config.tdm_init_config, sizeof(tdm_config.tdm_init_config));
        priv->tdm_init_config.config = 1;
        priv->tdm_start = 1;
    }
    else
    {
#ifdef DEBUG_UNIFIED
        NMSGPR_ALERT("NULL Pointer in TDM START.\n");
#endif
    }
    
    {
        int k, l;

        NMSGPR_ALERT("Host TDM Config : \n");
        NMSGPR_ALERT("-----------------\n");
        NMSGPR_ALERT("config : %d\n", priv->tdm_init_config.config);    
        NMSGPR_ALERT("samp_freq : %d\n", priv->tdm_init_config.samp_freq);    
        NMSGPR_ALERT("op_mode : %d\n", priv->tdm_init_config.op_mode);    
        NMSGPR_ALERT("mcr_mode : %d\n\n", priv->tdm_init_config.mcr_mode);    
    
        NMSGPR_ALERT("\nIN : \n");
    
        for(l = 0; l < 4; l++)
        {
            NMSGPR_ALERT("samp_size        : %d\n", priv->tdm_init_config.in_conf[l].samp_size);    
            NMSGPR_ALERT("num_ch           : %d\n", priv->tdm_init_config.in_conf[l].num_ch);
            NMSGPR_ALERT("stream_id        : ");    
            for(k = 0; k < 8; k++)
            {
                NMSGPR_ALERT(KERN_CONT "%02x:", priv->tdm_init_config.in_conf[l].stream_id[k]);
            }
            NMSGPR_ALERT("\n");
            NMSGPR_ALERT("dst_mac_address  : " );    
            for(k = 0; k < 6; k++)
            {
                NMSGPR_ALERT(KERN_CONT "%02x:", priv->tdm_init_config.in_conf[l].dst_mac_address[k]);
            }
            NMSGPR_ALERT("\n");
            NMSGPR_ALERT("vlan_id          : %3hhx\n\n", priv->tdm_init_config.in_conf[l].vlan_id);
       }
       
        NMSGPR_ALERT("\nOUT: \n");
        NMSGPR_ALERT("samp_size : %d\n", priv->tdm_init_config.out_conf.samp_size);
        NMSGPR_ALERT("num_ch : %d\n", priv->tdm_init_config.out_conf.num_ch);
        NMSGPR_ALERT("stream_id        : ");    
        for(k = 0; k < 8; k++)
        {
            NMSGPR_ALERT(KERN_CONT "%02x:", priv->tdm_init_config.out_conf.stream_id[k]);
        }
        NMSGPR_ALERT("\n\n");

    
        NMSGPR_ALERT("crf_op_mode : %d\n", priv->tdm_init_config.crf_conf.crf_op_mode);
        NMSGPR_ALERT("stream_id        : ");    
        for(k = 0; k < 8; k++)
        {
            NMSGPR_ALERT(KERN_CONT "%02x:", priv->tdm_init_config.crf_conf.crf_stream_id[k]);
        }
        NMSGPR_ALERT("\n");
        NMSGPR_ALERT("dst_mac_address  : " );    
        for(k = 0; k < 6; k++)
        {
            NMSGPR_ALERT(KERN_CONT "%02x:", priv->tdm_init_config.crf_conf.crf_dst_mac_address[k]);
        }
        NMSGPR_ALERT("\n");
        
     }
    memcpy((uint8_t *)(priv->tc9562_SRAM_pci_base_addr + HOST_SHARED_MEM_OFFSET), (uint8_t *)&priv->tdm_init_config, sizeof(priv->tdm_init_config)); 


	/* Raise MCU Flag to CM3 for TDM Start */
    writel(1 << 11, priv->ioaddr + 0x8054);  /*MCU_FLAG:11 - TDM START*/

    DBGPR_FUNC("-->tc9562mac_ioctl_TDM_Start\n");
    
    return 0;
}

int tc9562_ioctl_tdm_stop(struct tc9562mac_priv *priv, void __user *data)
{	
#ifdef DEBUG_UNIFIED	
	NMSGPR_ALERT("TDM Stop IOCTL Handler\n");
#endif

    priv->tdm_init_config.config = 1;         
    priv->tdm_start = 0;

    priv->tdm_init_config.config = 0;         //For internal Start Stop Call 

    memcpy((uint8_t *)(priv->tc9562_SRAM_pci_base_addr + HOST_SHARED_MEM_OFFSET), (uint8_t *)&priv->tdm_init_config, sizeof(priv->tdm_init_config));
    
    
    /* Raise MCU Flag to CM3 for TDM Stop */
	writel(1 << 10, priv->ioaddr + 0x8054);  /*MCU_FLAG:10 - TDM STOP*/

	DBGPR_FUNC("-->tc9562mac_extension_ioctl\n");
    
    return 0;
}
#endif



#define AVB_PKT_RATE_DBG
#if defined(TX_LOGGING_TRACE)		
static void tc9562mac_get_etx_hwtstamp(struct tc9562mac_priv *priv,
				   struct dma_enhanced_desc *p, struct sk_buff *skb,uint16_t qno)
#else
static void tc9562mac_get_etx_hwtstamp(struct tc9562mac_priv *priv,
				   struct dma_enhanced_desc *p, struct sk_buff *skb)
#endif
{
	struct skb_shared_hwtstamps shhwtstamp;
	u64 ns;
	static u64 pns = 0;
  u64 delta = 0;
#if defined(TX_LOGGING_TRACE)		
		static unsigned int ccnt1,ccnt2,ccnt3,ccnt4,ccnt5;
#endif

	if (!priv->hwts_tx_en)
	{
		return;
	}

	/* check tx tstamp status */
	if (priv->hw->desc->get_etx_timestamp_status(p)) {

#ifdef AVB_PKT_RATE_DBG
static unsigned int count=0;
static u64 prev_ns=0;
u64 rate;
#endif

		/* get the valid tstamp */
		ns = priv->hw->desc->get_en_timestamp(p, priv->adv_ts);

#ifdef TEST
static u64 prev_ns1=0;
printk("diff: %llx %d\n",ns-prev_ns1,(u32)ns-(u32)prev_ns1);   
prev_ns1 = ns;
#endif

#ifdef AVB_PKT_RATE_DBG
if (count % 8000 == 0)
{

	rate = (8000*1000 *1000000000ULL)/(unsigned int)(ns - prev_ns);
	//printk("count edesc: %d, rate: %lld, delta %llu \n", count, rate, ns-prev_ns);   

#ifdef FPE
#if defined(TX_LOGGING_TRACE)
		printk("Tx FPE mmc_tx_frag_cnt:%d\n", priv->mmc.mmc_tx_fpe_fragments);
#endif  
#endif

	prev_ns = ns;
}
count++;
#endif
		delta = ns - pns;
		//printk("sn: %d, Tx ns:%llx, Del:%llu ns\n", skb->data[20], ns, delta);
        pns = ns;

		memset(&shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
		shhwtstamp.hwtstamp = ns_to_ktime(ns);

		netdev_dbg(priv->dev, "get valid TX hw timestamp %llu\n", ns);
		//printk("get valid TX hw timestamp %lld\n", ns);
		/* pass tstamp to stack */
		skb_tstamp_tx(skb, &shhwtstamp);
#ifdef FPE
#if defined(TX_LOGGING_TRACE)
		if(priv->plat->fp_en)
		{
			priv->mmc.mmc_tx_fpe_fragments += readl(priv->mmcaddr + MMC_TX_FPE_FRAGMENT);
		}
#endif
#endif 

#if defined(TX_LOGGING_TRACE)
		if(qno == AVB_CLASSA_CH) // For AVB
		{
			if(skb->data[20] == 0)                                                                            
			{
					/*trace_printk("[AVB_A]CYCLE = %d\n",ccnt1);*/                                               
					ccnt1++;
			}
			if(priv->plat->fp_en)
			{
				trace_printk("[AVB]TS,%llu,%d,%03d,%02d,",ns,ccnt1,skb->data[20],qno);
			}
			else
			{			
				trace_printk("[AVB]TS,%llu,%d,%03d,%02d, , \n",ns,ccnt1,skb->data[20],qno);
			}
		}
		else if(qno == AVB_CLASSB_CH) 
		{
			if(skb->data[20] == 0)                                                                            
			{
					/*trace_printk("[AVB_B]CYCLE = %d\n",ccnt2);*/                                               
					ccnt2++;
			}
			trace_printk("[AVB_B]TS,%llu,%d,%03d,%02d, , \n",ns,ccnt2,skb->data[20],qno);
		
		}
		else if(qno == CLASS_CDT) // For CDT
		{
			if(skb->data[20] == 0)                                                                            
			{
					/*trace_printk("[CDT]CYCLE = %d\n",ccnt3);*/                                               
					ccnt3++;
			}
			trace_printk("[CDT]TS,%llu,%d,%03d,%02d, , \n",ns,ccnt3,skb->data[20],qno);
		}
		else if(qno == 1) // For queue 1 dummy packet
		{
			if(skb->data[20] == 0)                                                                            
			{
					/*trace_printk("[CDT]CYCLE = %d\n",ccnt3);*/                                               
					ccnt4++;
			}
			//[CDT]TS,<timestamp>,<cycle iteration>,<sequence no>,<queue no>"			;
			if(priv->plat->fp_en)
			{
				trace_printk("[DUM1]TS,%llu,%d,%03d,%02d,\n",ns,ccnt4,skb->data[20],qno);
			}
			else
			{	
				trace_printk("[DUM1]TS,%llu,%d,%03d,%02d, , \n",ns,ccnt4,skb->data[20],qno);
			}
		}
		else if(qno == 3) // For queue 3
		{
			if(skb->data[20] == 0)                                                                            
			{
					/*trace_printk("[CDT]CYCLE = %d\n",ccnt3);*/                                               
					ccnt5++;
			}
			//[CDT]TS,<timestamp>,<cycle iteration>,<sequence no>,<queue no>"			;
			if(priv->plat->fp_en)
			{
				trace_printk("[DUM3]TS,%llu,%d,%03d,%02d,\n",ns,ccnt5,skb->data[20],qno);
			}
			else
			{	
				trace_printk("[DUM3]TS,%llu,%d,%03d,%02d, , \n",ns,ccnt5,skb->data[20],qno);
			}
		}
		else if(qno == AVB_PTPCP_CH)
		{
		    u16 gPTP_ID=0;
			u16 MsgType = 0;
																																				   
			MsgType = skb->data[14] & 0x0F ;
			gPTP_ID = skb->data[44];
			gPTP_ID = (gPTP_ID<<8) | skb->data[45];
			
			if(MsgType != 0x0b)
				trace_printk("[gPTP]TS,%019llu,%04d,0x%x,%02d\n",ns,gPTP_ID,MsgType,qno);
		}
		else
		{}
#endif
	}

	return;
}

#define RDES3_CONTEXT_DESCRIPTOR        BIT(30)
#define RDES3_CONTEXT_DESCRIPTOR_SHIFT  30
#define RDES3_OWN                       BIT(31)
#if 0
int dwmac4_rx_check_timestamp_local(void *desc)
{
        struct dma_desc *p = (struct dma_desc *)desc;
        u32 own, ctxt;
        int ret = 1;

        DBGPR_FUNC("-->dwmac4_rx_check_timestamp\n");

        own = p->des3 & RDES3_OWN;
        ctxt = ((p->des3 & RDES3_CONTEXT_DESCRIPTOR)
                >> RDES3_CONTEXT_DESCRIPTOR_SHIFT);

        if (likely(!own && ctxt)) {
                if ((p->des0 == 0xffffffff) && (p->des1 == 0xffffffff))
                        /* Corrupted value */
                        ret = -EINVAL;
                else
                        /* A valid Timestamp is ready to be read */
                        ret = 0;
        }

        DBGPR_FUNC("<--dwmac4_rx_check_timestamp\n");
        /* Timestamp not ready */
        return ret;
}
#endif 

/* tc9562mac_get_rx_hwtstamp - get HW RX timestamps
 * @priv: driver private structure
 * @p : descriptor pointer
 * @np : next descriptor pointer
 * @skb : the socket buffer
 * Description :
 * This function will read received packet's timestamp from the descriptor
 * and pass it to stack. It also perform some sanity checks.
 */
#if defined(RX_LOGGING_TRACE)
static void tc9562mac_get_rx_hwtstamp(struct tc9562mac_priv *priv, struct dma_desc *p,
				   struct dma_desc *np, struct sk_buff *skb, u32 qno)
#else
static void tc9562mac_get_rx_hwtstamp(struct tc9562mac_priv *priv, struct dma_desc *p,
				   struct dma_desc *np, struct sk_buff *skb)
#endif
{
	struct skb_shared_hwtstamps *shhwtstamp = NULL;
	struct dma_desc *desc = p;
	u64 ns;
#if defined(RX_LOGGING_TRACE)
		static unsigned int ccnt1, ccnt2;
		unsigned int proto = 0;
#endif	

	DBGPR_FUNC("-->tc9562mac_get_rx_hwtstamp\n");
#ifdef FPE  
#if defined(RX_LOGGING_TRACE)
	 //if(priv->plat->fp_en)
    {
       priv->mmc.mmc_rx_pkt_assembly_ok += readl(priv->mmcaddr + MMC_RX_PKT_ASSEMBLY_OK);
       priv->mmc.mmc_rx_fpe_fragment += readl(priv->mmcaddr + MMC_RX_FPE_FRAGMENT);
    }
#endif 
#endif

	if (!priv->hwts_rx_en)
		return;
	/* For GMAC4, the valid timestamp is from CTX next desc. */
	if (priv->plat->has_gmac4)
		desc = np;

	/* Check if timestamp is available */
if (priv->hw->desc->get_rx_timestamp_status(p, np, priv->adv_ts)) {
#ifdef FPE
#if defined(RX_LOGGING_TRACE)
        static unsigned int count=0;
    
        if (count % 8000 == 0)
        {
                 printk("Rx FPE asmbly_ok_cnt:%d,frag_cnt:%d\n",priv->mmc.mmc_rx_pkt_assembly_ok, priv->mmc.mmc_rx_fpe_fragment);
        }
        count++;
#endif
#endif

		ns = priv->hw->desc->get_timestamp(desc, priv->adv_ts);
		netdev_dbg(priv->dev, "get valid RX hw timestamp %llu\n", ns);
		shhwtstamp = skb_hwtstamps(skb);
		memset(shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
		shhwtstamp->hwtstamp = ns_to_ktime(ns);
#if defined(RX_LOGGING_TRACE)
		
		//proto = eth_type_trans(skb, priv->dev);
    		
		proto = htons(((skb->data[13]<<8) | skb->data[12]));
		if(proto == TC9562_VLAN_TAG) {
	        proto = htons(((skb->data[17]<<8) | skb->data[16]));
	        if(proto == TC9562_ETH_TYPE_AVB) {
	            if(skb->data[24] == 0 && skb->data[25] == 2) //Differentiated by stream IDs 2 for CDT
			    {
	            	if((unsigned char)skb->data[16] == 0)
				    {
						    ccnt1++;
				    }			
		
				    trace_printk("[CDT]TS,%llu,%d,%03d,%02d \n",ns,ccnt1,skb->data[16],qno);
			    }
			    else if(skb->data[24] == 0 && skb->data[25] == 1) //Differentiated by stream IDs 1 for AVB
			    {
		            	if((unsigned char)skb->data[16] == 0)	
				    {
						    ccnt2++;
				    }
				    trace_printk("[AVB]TS,%llu,%d,%03d,%02d \n",ns,ccnt2,skb->data[16],qno);
			    }
	        }
	        else {}
		}
		else if(proto == TC9562_GPTP_ETH_TYPE) {
			if((skb->data[14]&0x0F) != 0xb)
			{
				u16 MsgType = skb->data[14] & 0x0F;
				u16 gPTP_ID = (skb->data[44] << 8) | skb->data[45];
				trace_printk("[gPTP]TS,%019llu,%04d,0x%x,%02d\n",ns,gPTP_ID,MsgType,qno);
			}
		}
		else
		{}
#endif
	} else  {
		printk("cannot get RX hw timestamp\n");
	}
	
	DBGPR_FUNC("<--tc9562mac_get_rx_hwtstamp\n");
}

/**
 *  tc9562mac_hwtstamp_ioctl - control hardware timestamping.
 *  @dev: device pointer.
 *  @ifr: An IOCTL specific structure, that can contain a pointer to
 *  a proprietary structure used to pass information to the driver.
 *  Description:
 *  This function configures the MAC to enable/disable both outgoing(TX)
 *  and incoming(RX) packets time stamping based on user input.
 *  Return Value:
 *  0 on success and an appropriate -ve integer on failure.
 */
static int tc9562mac_hwtstamp_ioctl(struct net_device *dev, struct ifreq *ifr)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	struct hwtstamp_config config;
	u32 ptp_v2 = 0;
	u32 tstamp_all = 0;
	u32 ptp_over_ipv4_udp = 0;
	u32 ptp_over_ipv6_udp = 0;
	u32 ptp_over_ethernet = 0;
	u32 snap_type_sel = 0;
	u32 ts_master_en = 0;
	u32 ts_event_en = 0;
	u32 value = 0;
	

	DBGPR_FUNC("-->tc9562mac_hwtstamp_ioctl\n");
	
	if (!(priv->dma_cap.time_stamp || priv->adv_ts)) {
		netdev_alert(priv->dev, "No support for HW time stamping\n");
		priv->hwts_tx_en = 0;
		priv->hwts_rx_en = 0;

		return -EOPNOTSUPP;
	}

	if (copy_from_user(&config, ifr->ifr_data,
			   sizeof(struct hwtstamp_config)))
		return -EFAULT;

	netdev_dbg(priv->dev, "%s config flags:0x%x, tx_type:0x%x, rx_filter:0x%x\n",
		   __func__, config.flags, config.tx_type, config.rx_filter);

	/* reserved for future extensions */
	if (config.flags)
		return -EINVAL;

	if (config.tx_type != HWTSTAMP_TX_OFF &&
	    config.tx_type != HWTSTAMP_TX_ON)
		return -ERANGE;

	if (priv->adv_ts) {
		switch (config.rx_filter) {
		case HWTSTAMP_FILTER_NONE:
			/* time stamp no incoming packet at all */
			config.rx_filter = HWTSTAMP_FILTER_NONE;
			break;

		case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
			/* PTP v1, UDP, any kind of event packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_EVENT;
			/* take time stamp for all event messages */
			if (priv->plat->has_gmac4)
				//snap_type_sel = PTP_GMAC4_TCR_SNAPTYPSEL_1;
				// SNAPTYPESEL is 01 for all case 
				snap_type_sel = PTP_TCR_SNAPTYPSEL_1;
			else
				snap_type_sel = PTP_TCR_SNAPTYPSEL_1;

			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
			/* PTP v1, UDP, Sync packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_SYNC;
			/* take time stamp for SYNC messages only */
			ts_event_en = PTP_TCR_TSEVNTENA;

			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
			/* PTP v1, UDP, Delay_req packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ;
			/* take time stamp for Delay_Req messages only */
			ts_master_en = PTP_TCR_TSMSTRENA;
			ts_event_en = PTP_TCR_TSEVNTENA;

			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
			/* PTP v2, UDP, any kind of event packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			/* take time stamp for all event messages */
			if (priv->plat->has_gmac4)
				//snap_type_sel = PTP_GMAC4_TCR_SNAPTYPSEL_1;
				// SNAPTYPESEL is 01 for all case 
				snap_type_sel = PTP_TCR_SNAPTYPSEL_1;
			else
				snap_type_sel = PTP_TCR_SNAPTYPSEL_1;

			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
			/* PTP v2, UDP, Sync packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_SYNC;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			/* take time stamp for SYNC messages only */
			ts_event_en = PTP_TCR_TSEVNTENA;

			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
			/* PTP v2, UDP, Delay_req packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			/* take time stamp for Delay_Req messages only */
			ts_master_en = PTP_TCR_TSMSTRENA;
			ts_event_en = PTP_TCR_TSEVNTENA;

			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_EVENT:
			/* PTP v2/802.AS1 any layer, any kind of event packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			/* take time stamp for all event messages */
			if (priv->plat->has_gmac4)
				//snap_type_sel = PTP_GMAC4_TCR_SNAPTYPSEL_1;
				// SNAPTYPESEL is 01 for all case 
				snap_type_sel = PTP_TCR_SNAPTYPSEL_1;
			else
				snap_type_sel = PTP_TCR_SNAPTYPSEL_1;

			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			ptp_over_ethernet = PTP_TCR_TSIPENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_SYNC:
			/* PTP v2/802.AS1, any layer, Sync packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_SYNC;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			/* take time stamp for SYNC messages only */
			ts_event_en = PTP_TCR_TSEVNTENA;

			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			ptp_over_ethernet = PTP_TCR_TSIPENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
			/* PTP v2/802.AS1, any layer, Delay_req packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_DELAY_REQ;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			/* take time stamp for Delay_Req messages only */
			ts_master_en = PTP_TCR_TSMSTRENA;
			ts_event_en = PTP_TCR_TSEVNTENA;

			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			ptp_over_ethernet = PTP_TCR_TSIPENA;
			break;
			
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0))
		case HWTSTAMP_FILTER_NTP_ALL:
#endif		
		case HWTSTAMP_FILTER_ALL:
			/* time stamp any incoming packet */
			config.rx_filter = HWTSTAMP_FILTER_ALL;
			tstamp_all = PTP_TCR_TSENALL;
			break;

		default:
			return -ERANGE;
		}
	} else {
		switch (config.rx_filter) {
		case HWTSTAMP_FILTER_NONE:
			config.rx_filter = HWTSTAMP_FILTER_NONE;
			break;
		default:
			/* PTP v1, UDP, any kind of event packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_EVENT;
			break;
		}
	}
	priv->hwts_rx_en = ((config.rx_filter == HWTSTAMP_FILTER_NONE) ? 0 : 1);
	priv->hwts_tx_en = config.tx_type == HWTSTAMP_TX_ON;

	if (!priv->hwts_tx_en && !priv->hwts_rx_en)
		priv->hw->ptp->config_hw_tstamping(priv->ptpaddr, 0);
	else {
		//value = (PTP_TCR_TSENA | PTP_TCR_TSCFUPDT | PTP_TCR_TSCTRLSSR |
		//	 tstamp_all | ptp_v2 | ptp_over_ethernet |
		//	 ptp_over_ipv6_udp | ptp_over_ipv4_udp | ts_event_en |
		//	 ts_master_en | snap_type_sel);

		//  PTP_TCR_ASMEN = 1
		value = (PTP_TCR_TSENA | PTP_TCR_TSCFUPDT | PTP_TCR_TSCTRLSSR |
			 tstamp_all | ptp_v2 | ptp_over_ethernet |
			 ptp_over_ipv6_udp | ptp_over_ipv4_udp | ts_event_en |
			 ts_master_en | snap_type_sel | PTP_TCR_ASMEN);

		value = readl(priv->ptpaddr + PTP_TCR);
		if(!(value & 0x00000001))
		{
			tc9562_ptp_configuration(priv);
			printk("tc9562mac_hwtstamp_ioctl : ptp configuration");	
		}
		
	}

	return copy_to_user(ifr->ifr_data, &config,
			    sizeof(struct hwtstamp_config)) ? -EFAULT : 0;

	DBGPR_FUNC("<--tc9562mac_hwtstamp_ioctl\n");
}

/**
 * tc9562mac_init_ptp - init PTP
 * @priv: driver private structure
 * Description: this is to verify if the HW supports the PTPv1 or PTPv2.
 * This is done by looking at the HW cap. register.
 * This function also registers the ptp driver.
 */
static int tc9562mac_init_ptp(struct tc9562mac_priv *priv)
{
	DBGPR_FUNC("-->tc9562mac_init_ptp\n");

	if (!(priv->dma_cap.time_stamp || priv->dma_cap.atime_stamp))
		return -EOPNOTSUPP;

	priv->adv_ts = 0;
	/* Check if adv_ts can be enabled for dwmac 4.x core */
	if (priv->plat->has_gmac4 && priv->dma_cap.atime_stamp)
		priv->adv_ts = 1;
	/* Dwmac 3.x core with extend_desc can support adv_ts */
	else if (priv->extend_desc && priv->dma_cap.atime_stamp)
		priv->adv_ts = 1;

	if (priv->dma_cap.time_stamp)
		netdev_info(priv->dev, "IEEE 1588-2002 Timestamp supported\n");

	if (priv->adv_ts)
		netdev_info(priv->dev,
			    "IEEE 1588-2008 Advanced Timestamp supported\n");

	priv->hw->ptp = &tc9562mac_ptp;
	priv->hwts_tx_en = 0;
	priv->hwts_rx_en = 0;

	tc9562mac_ptp_register(priv);
	
	DBGPR_FUNC("<--tc9562mac_init_ptp\n");

	return 0;
}

static void tc9562mac_release_ptp(struct tc9562mac_priv *priv)
{
	DBGPR_FUNC("-->tc9562mac_release_ptp\n");

	if (priv->plat->clk_ptp_ref)
		clk_disable_unprepare(priv->plat->clk_ptp_ref);
	tc9562mac_ptp_unregister(priv);

	DBGPR_FUNC("<--tc9562mac_release_ptp\n");
}

/**
 *  tc9562mac_mac_flow_ctrl - Configure flow control in all queues
 *  @priv: driver private structure
 *  Description: It is used for configuring the flow control in all queues
 */
static void tc9562mac_mac_flow_ctrl(struct tc9562mac_priv *priv, u32 duplex)
{
	u32 tx_cnt = priv->plat->tx_queues_to_use;

	DBGPR_FUNC("-->tc9562mac_mac_flow_ctrl\n");

	priv->hw->mac->flow_ctrl(priv->hw, duplex, priv->flow_ctrl,
				 priv->pause, tx_cnt);

	DBGPR_FUNC("<--tc9562mac_mac_flow_ctrl\n");
}

static void tc9562_reload_cbs(struct tc9562mac_priv *priv)
{
    u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 queue;
	DBGPR_FUNC("-->tc9562_reload_cbs\n");

/* queue 0 is reserved for legacy traffic */
//	for (queue = 0; queue < tx_queues_count; queue++) {
    for (queue = 1; queue < tx_queues_count; queue++) {
#ifndef UNIFIED_DRIVER
		if ((priv->plat->tx_queues_cfg[queue].mode_to_use == MTL_QUEUE_AVB) && (priv->cbs_cfg_status[queue] == 1)) {
#else
		if ((priv->plat->tx_queues_cfg[queue].mode_to_use == MTL_QUEUE_AVB) && (priv->cbs_cfg_status[queue] == 1) &&
			(priv->plat->tx_dma_ch_for_host[queue] == 1)) {
#endif
		
		    if(priv->speed == SPEED_100) {
		        priv->plat->tx_queues_cfg[queue].send_slope = priv->cbs_speed100_cfg[queue].send_slope;
		        priv->plat->tx_queues_cfg[queue].idle_slope = priv->cbs_speed100_cfg[queue].idle_slope;
		        priv->plat->tx_queues_cfg[queue].high_credit = priv->cbs_speed100_cfg[queue].high_credit;
		        priv->plat->tx_queues_cfg[queue].low_credit = priv->cbs_speed100_cfg[queue].low_credit;
		        priv->plat->tx_queues_cfg[queue].percentage = priv->cbs_speed100_cfg[queue].percentage;
		    }
		    else if(priv->speed == SPEED_1000) {
		        priv->plat->tx_queues_cfg[queue].send_slope = priv->cbs_speed1000_cfg[queue].send_slope;
		        priv->plat->tx_queues_cfg[queue].idle_slope = priv->cbs_speed1000_cfg[queue].idle_slope;
		        priv->plat->tx_queues_cfg[queue].high_credit = priv->cbs_speed1000_cfg[queue].high_credit;
		        priv->plat->tx_queues_cfg[queue].low_credit = priv->cbs_speed1000_cfg[queue].low_credit;
		        priv->plat->tx_queues_cfg[queue].percentage = priv->cbs_speed1000_cfg[queue].percentage;

		    }
		    
		    priv->hw->mac->config_cbs(priv->hw,
				    priv->plat->tx_queues_cfg[queue].send_slope,
				    priv->plat->tx_queues_cfg[queue].idle_slope,
				    priv->plat->tx_queues_cfg[queue].high_credit,
				    priv->plat->tx_queues_cfg[queue].low_credit,
				    queue);
		}
	}
    
	DBGPR_FUNC("<--tc9562_reload_cbs\n");

}

/**
 * tc9562mac_adjust_link - adjusts the link parameters
 * @dev: net device structure
 * Description: this is the helper called by the physical abstraction layer
 * drivers to communicate the phy link status. According the speed and duplex
 * this driver can invoke registered glue-logic as well.
 * It also invoke the eee initialization because it could happen when switch
 * on different networks (that are eee capable).
 */
static void tc9562mac_adjust_link(struct net_device *dev)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	struct phy_device *phydev = dev->phydev;
	unsigned long flags;
	bool new_state = false;

	DBGPR_FUNC("-->tc9562mac_adjust_link \n");
	if (!phydev)
		return;

	spin_lock_irqsave(&priv->lock, flags);

	if (phydev->link) {
		u32 ctrl = readl(priv->ioaddr + GMAC_CONFIG);
        u32 emac_ctrl = readl(priv->ioaddr + NEMACCTL_OFFSET);
        
		/* Now we make sure that we can be in full duplex mode.
		 * If not, we operate in half-duplex mode. */
		if (phydev->duplex != priv->oldduplex) {
			new_state = true;
			if (!phydev->duplex)
				ctrl &= ~priv->hw->link.duplex;
			else
				ctrl |= priv->hw->link.duplex;
			priv->oldduplex = phydev->duplex;
		}
		/* Flow Control operation */
		if (phydev->pause)
			tc9562mac_mac_flow_ctrl(priv, phydev->duplex);

		if (phydev->speed != priv->speed) {
			new_state = true;
			ctrl &= ~priv->hw->link.speed_mask;
		    emac_ctrl &= (~(3 << 0)); /* Bit pos [1:0] of NEMACCTL corresponds to speed select */
			switch (phydev->speed) {
			case SPEED_1000:
				ctrl |= priv->hw->link.speed1000;
				emac_ctrl |= 0; 
				break;
			case SPEED_100:
				ctrl |= priv->hw->link.speed100;
			    emac_ctrl |= 2;
				break;
			case SPEED_10:
				ctrl |= priv->hw->link.speed10;
				emac_ctrl |= 3;
				break;
			default:
				netif_warn(priv, link, priv->dev,
					   "broken speed: %d\n", phydev->speed);
				phydev->speed = SPEED_UNKNOWN;
				break;
			}
			if (phydev->speed != SPEED_UNKNOWN)
				tc9562mac_hw_fix_mac_speed(priv);
			priv->speed = phydev->speed;
		}

		writel(ctrl, priv->ioaddr + GMAC_CONFIG);
        writel(emac_ctrl, priv->ioaddr + NEMACCTL_OFFSET);
        
		if (!priv->oldlink) {
			new_state = true;
			priv->oldlink = true;
		}
		
#ifdef UNIFIED_DRIVER
		if(new_state)
		{	
            tc9562_propagate_link_params(dev);
            if (priv->tdm_start == 1)
            {
                void __user *data = NULL;
                tc9562_ioctl_tdm_start(priv, data);    
            }
        }
#endif
		
	} else if (priv->oldlink) {
		new_state = true;
		priv->oldlink = false;
		priv->speed = SPEED_UNKNOWN;
		priv->oldduplex = DUPLEX_UNKNOWN;
	#ifdef UNIFIED_DRIVER
		if(new_state)
		{
            tc9562_propagate_link_params(dev);
            if (priv->tdm_start == 1)
            {
                void __user *data = NULL;
                tc9562_ioctl_tdm_stop(priv, data);
                priv->tdm_start = 1;
            }
        }
      #endif
	}

	if (new_state && netif_msg_link(priv))
		phy_print_status(phydev);

    if(new_state) {
        tc9562_reload_cbs(priv);
    }
	spin_unlock_irqrestore(&priv->lock, flags);
	
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0))
	if (phydev->is_pseudo_fixed_link)
	{
		/* Stop PHY layer to call the hook to adjust the link in case
		 * of a switch is attached to the tc9562mac driver.
		 */
		phydev->irq = PHY_IGNORE_INTERRUPT;
	}
	else
#endif
    {
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE    
		/* At this stage, init the EEE if supported.
		 * Never called in case of fixed_link.
		 */
		priv->eee_enabled = tc9562mac_eee_init(priv);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
	}
	DBGPR_FUNC("<--tc9562mac_adjust_link \n");
}

/**
 * tc9562mac_check_pcs_mode - verify if RGMII/SGMII is supported
 * @priv: driver private structure
 * Description: this is to verify if the HW supports the PCS.
 * Physical Coding Sublayer (PCS) interface that can be used when the MAC is
 * configured for the TBI, RTBI, or SGMII PHY interface.
 */
static void tc9562mac_check_pcs_mode(struct tc9562mac_priv *priv)
{
	int interface = priv->plat->interface;

	DBGPR_FUNC("-->tc9562mac_check_pcs_mode\n");

	if (priv->dma_cap.pcs) {
		if ((interface == PHY_INTERFACE_MODE_RGMII) ||
		    (interface == PHY_INTERFACE_MODE_RGMII_ID) ||
		    (interface == PHY_INTERFACE_MODE_RGMII_RXID) ||
		    (interface == PHY_INTERFACE_MODE_RGMII_TXID)) {
			netdev_dbg(priv->dev, "PCS RGMII support enabled\n");
 			//priv->hw->pcs = TC9562MAC_PCS_RGMII;  
			priv->hw->pcs = 0;
		} else if (interface == PHY_INTERFACE_MODE_SGMII) {
			netdev_dbg(priv->dev, "PCS SGMII support enabled\n");
			//priv->hw->pcs = TC9562MAC_PCS_SGMII;
			priv->hw->pcs = 0; //PCS disabled
		}
	}else{
		NDBGPR_L2("PCS not enabled \n");
	}
	DBGPR_FUNC("<--tc9562mac_check_pcs_mode\n");
}

/**
 * tc9562mac_init_phy - PHY initialization
 * @dev: net device structure
 * Description: it initializes the driver's PHY state, and attaches the PHY
 * to the mac driver.
 *  Return value:
 *  0 on success
 */
static int tc9562mac_init_phy(struct net_device *dev)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	struct phy_device *phydev;
	char phy_id_fmt[MII_BUS_ID_SIZE + 3];
	char bus_id[MII_BUS_ID_SIZE];
	struct tc9562mac_mdio_bus_data *mdio_bus_data = priv->plat->mdio_bus_data;
	int interface = priv->plat->interface;
	int max_speed = priv->plat->max_speed;
	priv->oldlink = false;
	priv->speed = SPEED_UNKNOWN;
	priv->oldduplex = DUPLEX_UNKNOWN;

	DBGPR_FUNC("--> tc9562mac_init_phy \n");
	if (priv->plat->phy_node) {
		phydev = of_phy_connect(dev, priv->plat->phy_node,
					&tc9562mac_adjust_link, 0, interface);
	} else {
		snprintf(bus_id, MII_BUS_ID_SIZE, "tc9562-%x",
			 priv->plat->bus_id);

		snprintf(phy_id_fmt, MII_BUS_ID_SIZE + 3, PHY_ID_FMT, bus_id,
			 priv->plat->phy_addr);
		netdev_dbg(priv->dev, "%s: trying to attach to %s\n", __func__,
			   phy_id_fmt);

		phydev = phy_connect(dev, phy_id_fmt, &tc9562mac_adjust_link,
				     interface);
	}

    /*
     * If an IRQ was provided to be assigned after
     * the bus probe, do it here.
     */
    if (!mdio_bus_data->irqs &&
        (mdio_bus_data->probed_phy_irq > 0)) {
        NMSGPR_ALERT("probed_phy_irq\n");
        phydev->irq = mdio_bus_data->probed_phy_irq;
    }

	if (IS_ERR_OR_NULL(phydev)) {
		netdev_err(priv->dev, "Could not attach to PHY\n");
		if (!phydev)
			return -ENODEV;

		return PTR_ERR(phydev);
	}else{
		NDBGPR_L2(" PHY attached to network device \n");	
	}

	/* Stop Advertising 1000BASE Capability if interface is not GMII */
	if ((interface == PHY_INTERFACE_MODE_MII) ||
	    (interface == PHY_INTERFACE_MODE_RMII) ||
		(max_speed < 1000 && max_speed > 0))
		phydev->advertising &= ~(SUPPORTED_1000baseT_Half |
					 SUPPORTED_1000baseT_Full);

	/*
	 * Broken HW is sometimes missing the pull-up resistor on the
	 * MDIO line, which results in reads to non-existent devices returning
	 * 0 rather than 0xffff. Catch this here and treat 0 as a non-existent
	 * device as well.
	 * Note: phydev->phy_id is the result of reading the UID PHY registers.
	 */
	if (!priv->plat->phy_node && phydev->phy_id == 0) {
		phy_disconnect(phydev);
		return -ENODEV;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0))
	/* tc9562mac_adjust_link will change this to PHY_IGNORE_INTERRUPT to avoid
	 * subsequent PHY polling, make sure we force a link transition if
	 * we have a UP/DOWN/UP transition
	 */
	if (phydev->is_pseudo_fixed_link)
		phydev->irq = PHY_POLL;
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0))
	phy_attached_info(phydev);
#else
  	netdev_dbg(priv->dev, "%s: attached to PHY (UID 0x%x) Link = %d\n",
  		   __func__, phydev->phy_id, phydev->link);

#endif
	DBGPR_FUNC("<-- tc9562mac_init_phy \n");
	return 0;
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static void tc9562mac_display_rx_rings(struct tc9562mac_priv *priv)
{
	u32 rx_cnt = priv->plat->rx_queues_to_use + 1;
	void *head_rx;
	u32 queue;
	struct tc9562mac_rx_queue *rx_q;

	DBGPR_FUNC("-->tc9562mac_display_rx_rings\n");

	/* Display RX rings */
	for (queue = 0; queue < rx_cnt; queue++) {
#ifdef UNIFIED_DRIVER
		if(priv->plat->rx_dma_ch_for_host[queue] == 0)
			continue;
#endif
		rx_q = &priv->rx_queue[queue];

		pr_info("\tRX Queue %u rings\n", queue);

		if (priv->extend_desc)
			head_rx = (void *)rx_q->dma_erx;
		else
			head_rx = (void *)rx_q->dma_rx;

		/* Display RX ring */
		priv->hw->desc->display_ring(head_rx, DMA_RX_SIZE, true);
	}

	DBGPR_FUNC("<--tc9562mac_display_rx_rings\n");
}

static void tc9562mac_display_tx_rings(struct tc9562mac_priv *priv)
{
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	void *head_tx;
	u32 queue;
	struct tc9562mac_tx_queue *tx_q;

	DBGPR_FUNC("-->tc9562mac_display_tx_rings\n");

	/* Display TX rings */
	for (queue = 0; queue < tx_cnt; queue++) {
#ifdef UNIFIED_DRIVER
		if(priv->plat->tx_dma_ch_for_host[queue] == 0)
			continue;
#endif
		tx_q = &priv->tx_queue[queue];

		pr_info("\tTX Queue %d rings\n", queue);

		if (priv->extend_desc)
			head_tx = (void *)tx_q->dma_etx;
		else{			
			if((queue == AVB_CLASSA_CH) || (queue == AVB_CLASSB_CH) || (queue == CLASS_CDT))
				head_tx = (void *)tx_q->dma_entx;
			else
				head_tx = (void *)tx_q->dma_tx;
		}

		priv->hw->desc->display_ring(head_tx, DMA_TX_SIZE, false);
	}
	
	DBGPR_FUNC("<--tc9562mac_display_tx_rings\n");
}

static void tc9562mac_display_rings(struct tc9562mac_priv *priv)
{
	DBGPR_FUNC("-->tc9562mac_display_rings\n");

	/* Display RX ring */
	tc9562mac_display_rx_rings(priv);

	/* Display TX ring */
	tc9562mac_display_tx_rings(priv);

	DBGPR_FUNC("<--tc9562mac_display_rings\n");
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

static int tc9562mac_set_bfsize(int mtu, int bufsize)
{
	int ret = bufsize;

	DBGPR_FUNC("-->tc9562mac_set_bfsize\n");

	if (mtu >= BUF_SIZE_4KiB)
		ret = BUF_SIZE_8KiB;
	else if (mtu >= BUF_SIZE_2KiB)
		ret = BUF_SIZE_4KiB;
	else if (mtu > DEFAULT_BUFSIZE)
		ret = BUF_SIZE_2KiB;
	else
		ret = DEFAULT_BUFSIZE;

	DBGPR_FUNC("<--tc9562mac_set_bfsize\n");

	return ret;
}

/**
 * tc9562mac_clear_rx_descriptors - clear RX descriptors
 * @priv: driver private structure
 * @queue: RX queue index
 * Description: this function is called to clear the RX descriptors
 * in case of both basic and extended descriptors are used.
 */
static void tc9562mac_clear_rx_descriptors(struct tc9562mac_priv *priv, u32 queue)
{
	struct tc9562mac_rx_queue *rx_q = &priv->rx_queue[queue];
	int i;
	
	DBGPR_FUNC("-->tc9562mac_clear_rx_descriptors\n");

	/* Clear the RX descriptors */
	for (i = 0; i < DMA_RX_SIZE; i++)
		if (priv->extend_desc)
			priv->hw->desc->init_rx_desc(&rx_q->dma_erx[i].basic,
						     priv->use_riwt, priv->mode,
						     (i == DMA_RX_SIZE - 1));
		else
			priv->hw->desc->init_rx_desc(&rx_q->dma_rx[i],
						     priv->use_riwt, priv->mode,
						     (i == DMA_RX_SIZE - 1));

	DBGPR_FUNC("<--tc9562mac_clear_rx_descriptors\n");
}

/**
 * tc9562mac_clear_tx_descriptors - clear tx descriptors
 * @priv: driver private structure
 * @queue: TX queue index.
 * Description: this function is called to clear the TX descriptors
 * in case of both basic and extended descriptors are used.
 */
static void tc9562mac_clear_tx_descriptors(struct tc9562mac_priv *priv, u32 queue)
{
	struct tc9562mac_tx_queue *tx_q = &priv->tx_queue[queue];
	int i;
	DBGPR_FUNC("-->tc9562mac_clear_tx_descriptors\n");
    tx_q = &priv->tx_queue[queue];
	if( (tx_q->queue_index == AVB_CLASSA_CH) || (tx_q->queue_index == AVB_CLASSB_CH) || (tx_q->queue_index == CLASS_CDT) )
	{
		/* Enable Enhanced Descriptor (EDSE) */
		
		for (i = 0; i < DMA_TX_SIZE; i++)
		priv->hw->desc->init_etx_desc(&tx_q->dma_entx[i],
						 priv->mode,
						 (i == DMA_TX_SIZE - 1));
	}

	else
	{
	/* Clear the TX descriptors */
	for (i = 0; i < DMA_TX_SIZE; i++)
		if (priv->extend_desc)
			priv->hw->desc->init_tx_desc(&tx_q->dma_etx[i].basic,
						     priv->mode,
						     (i == DMA_TX_SIZE - 1));
		else
			priv->hw->desc->init_tx_desc(&tx_q->dma_tx[i],
						     priv->mode,
						     (i == DMA_TX_SIZE - 1));
	}
	DBGPR_FUNC("<--tc9562mac_clear_tx_descriptors\n");
}

/**
 * tc9562mac_clear_descriptors - clear descriptors
 * @priv: driver private structure
 * Description: this function is called to clear the TX and RX descriptors
 * in case of both basic and extended descriptors are used.
 */
static void tc9562mac_clear_descriptors(struct tc9562mac_priv *priv)
{
	u32 rx_queue_cnt = priv->plat->rx_queues_to_use + 1;
	u32 tx_queue_cnt = priv->plat->tx_queues_to_use;
	u32 queue;
	
	DBGPR_FUNC("-->tc9562mac_clear_descriptors\n");

	/* Clear the RX descriptors */
	for (queue = 0; queue < rx_queue_cnt; queue++)
	{
#ifdef UNIFIED_DRIVER
		if(priv->plat->rx_dma_ch_for_host[queue] == 0)
			continue;
#endif
		tc9562mac_clear_rx_descriptors(priv, queue);
	}
	/* Clear the TX descriptors */
	for (queue = 0; queue < tx_queue_cnt; queue++)
	{
#ifdef UNIFIED_DRIVER
		if(priv->plat->tx_dma_ch_for_host[queue] == 0)
			continue;
#endif
		tc9562mac_clear_tx_descriptors(priv, queue);
	}
	DBGPR_FUNC("<--tc9562mac_clear_descriptors\n");
}

/**
 * tc9562mac_init_rx_buffers - init the RX descriptor buffer.
 * @priv: driver private structure
 * @p: descriptor pointer
 * @i: descriptor index
 * @flags: gfp flag
 * @queue: RX queue index
 * Description: this function is called to allocate a receive buffer, perform
 * the DMA mapping and init the descriptor.
 */
static int tc9562mac_init_rx_buffers(struct tc9562mac_priv *priv, struct dma_desc *p,
				  int i, gfp_t flags, u32 queue)
{
	struct tc9562mac_rx_queue *rx_q = &priv->rx_queue[queue];
	struct sk_buff *skb;

	DBGPR_FUNC("-->tc9562mac_init_rx_buffers\n");

	skb = __netdev_alloc_skb_ip_align(priv->dev, 
	            priv->dma_buf_sz + priv->plat->axi->axi_wr_osr_lmt * TC9562_ADDL_BUF_SIZE, flags);
	if (!skb) {
		netdev_err(priv->dev,
			   "%s: Rx init fails; skb is NULL\n", __func__);
		return -ENOMEM;
	}
	rx_q->rx_skbuff[i] = skb;
	rx_q->rx_skbuff_dma[i] = dma_map_single(priv->device, skb->data,
						priv->dma_buf_sz,
						DMA_FROM_DEVICE);
	if (dma_mapping_error(priv->device, rx_q->rx_skbuff_dma[i])) {
		netdev_err(priv->dev, "%s: DMA mapping error\n", __func__);
		dev_kfree_skb_any(skb);
		return -EINVAL;
	}
	if (priv->synopsys_id >= DWMAC_CORE_4_00) {
#ifdef TC9562_DEFINED
		p->des0 = (rx_q->rx_skbuff_dma[i] & 0xFFFFFFFF);
		p->des1 = HOST_PHYSICAL_ADRS_MASK | ((rx_q->rx_skbuff_dma[i] >> 32) & 0xF);
#else
		p->des0 = cpu_to_le32(rx_q->rx_skbuff_dma[i]);
#endif
	}
	else
		p->des2 = cpu_to_le32(rx_q->rx_skbuff_dma[i]);

	if ((priv->hw->mode->init_desc3) &&
	    (priv->dma_buf_sz == BUF_SIZE_16KiB))
		priv->hw->mode->init_desc3(p);

	DBGPR_FUNC("<--tc9562mac_init_rx_buffers\n");

	return 0;
}

/**
 * tc9562mac_free_rx_buffer - free RX dma buffers
 * @priv: private structure
 * @queue: RX queue index
 * @i: buffer index.
 */
static void tc9562mac_free_rx_buffer(struct tc9562mac_priv *priv, u32 queue, int i)
{
	struct tc9562mac_rx_queue *rx_q = &priv->rx_queue[queue];

	DBGPR_FUNC("-->tc9562mac_free_rx_buffer\n");

	if (rx_q->rx_skbuff[i]) {
		dma_unmap_single(priv->device, rx_q->rx_skbuff_dma[i],
				 priv->dma_buf_sz, DMA_FROM_DEVICE);
		dev_kfree_skb_any(rx_q->rx_skbuff[i]);
	}
	rx_q->rx_skbuff[i] = NULL;

	DBGPR_FUNC("<--tc9562mac_free_rx_buffer\n");
}

/**
 * tc9562mac_free_tx_buffer - free RX dma buffers
 * @priv: private structure
 * @queue: RX queue index
 * @i: buffer index.
 */
static void tc9562mac_free_tx_buffer(struct tc9562mac_priv *priv, u32 queue, int i)
{
	struct tc9562mac_tx_queue *tx_q = &priv->tx_queue[queue];

	DBGPR_FUNC("-->tc9562mac_free_tx_buffer\n");

	if (tx_q->tx_skbuff_dma[i].buf) {
		if (tx_q->tx_skbuff_dma[i].map_as_page)
			dma_unmap_page(priv->device,
				       tx_q->tx_skbuff_dma[i].buf,
				       tx_q->tx_skbuff_dma[i].len,
				       DMA_TO_DEVICE);
		else
			dma_unmap_single(priv->device,
					 tx_q->tx_skbuff_dma[i].buf,
					 tx_q->tx_skbuff_dma[i].len,
					 DMA_TO_DEVICE);
	}

	if (tx_q->tx_skbuff[i]) {
		dev_kfree_skb_any(tx_q->tx_skbuff[i]);
		tx_q->tx_skbuff[i] = NULL;
		tx_q->tx_skbuff_dma[i].buf = 0;
		tx_q->tx_skbuff_dma[i].map_as_page = false;
	}

	DBGPR_FUNC("<--tc9562mac_free_tx_buffer\n");
}

/**
 * init_dma_rx_desc_rings - init the RX descriptor rings
 * @dev: net device structure
 * @flags: gfp flag.
 * Description: this function initializes the DMA RX descriptors
 * and allocates the socket buffers. It supports the chained and ring
 * modes.
 */
static int init_dma_rx_desc_rings(struct net_device *dev, gfp_t flags)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	u32 rx_count = priv->plat->rx_queues_to_use + 1;
	unsigned int bfsize = 0;
	int ret = -ENOMEM;
	int queue;
	int i;
	struct tc9562mac_rx_queue *rx_q;
	
	DBGPR_FUNC("-->init_dma_rx_desc_rings\n");

	if (priv->hw->mode->set_16kib_bfsize)
		bfsize = priv->hw->mode->set_16kib_bfsize(dev->mtu);

	if (bfsize < BUF_SIZE_16KiB)
		bfsize = tc9562mac_set_bfsize(dev->mtu, priv->dma_buf_sz);

	priv->dma_buf_sz = bfsize;

	NDBGPR_L2("priv->dma_buf_sz : %d \n", priv->dma_buf_sz);
	/* RX INITIALIZATION */
	netif_dbg(priv, probe, priv->dev,
		  "SKB addresses:\nskb\t\tskb data\tdma data\n");

	for (queue = 0; queue < rx_count; queue++) {
#ifdef UNIFIED_DRIVER
		if(priv->plat->rx_dma_ch_for_host[queue] == 0)
			continue;
#endif
		rx_q = &priv->rx_queue[queue];

		netif_dbg(priv, probe, priv->dev,
			  "(%s) dma_rx_phy=0x%08x\n", __func__,
			  (u32)rx_q->dma_rx_phy);

		for (i = 0; i < DMA_RX_SIZE; i++) {
			struct dma_desc *p;

			if (priv->extend_desc)
				p = &((rx_q->dma_erx + i)->basic);
			else
				p = rx_q->dma_rx + i;

			ret = tc9562mac_init_rx_buffers(priv, p, i, flags,
						     queue);
			if (ret)
				goto err_init_rx_buffers;

			netif_dbg(priv, probe, priv->dev, "[%p]\t[%p]\t[%x]\n",
				  rx_q->rx_skbuff[i], rx_q->rx_skbuff[i]->data,
				  (unsigned int)rx_q->rx_skbuff_dma[i]);
		}

		rx_q->cur_rx = 0;
		rx_q->dirty_rx = (unsigned int)(i - DMA_RX_SIZE);

		tc9562mac_clear_rx_descriptors(priv, queue);

		/* Setup the chained descriptor addresses */
		if (priv->mode == TC9562MAC_CHAIN_MODE) {
			if (priv->extend_desc)
				priv->hw->mode->init(rx_q->dma_erx,
						     rx_q->dma_rx_phy,
						     DMA_RX_SIZE, 1);
			else
				priv->hw->mode->init(rx_q->dma_rx,
						     rx_q->dma_rx_phy,
						     DMA_RX_SIZE, 0);
		}
	}

	buf_sz = bfsize;

	DBGPR_FUNC("<--init_dma_rx_desc_rings\n");

	return 0;

err_init_rx_buffers:
	while (queue >= 0) {
		while (--i >= 0)
			tc9562mac_free_rx_buffer(priv, queue, i);

		if (queue == 0)
			break;

		i = DMA_RX_SIZE;
		queue--;
	}

	return ret;
}

/**
 * init_dma_tx_desc_rings - init the TX descriptor rings
 * @dev: net device structure.
 * Description: this function initializes the DMA TX descriptors
 * and allocates the socket buffers. It supports the chained and ring
 * modes.
 */
static int init_dma_tx_desc_rings(struct net_device *dev)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	u32 tx_queue_cnt = priv->plat->tx_queues_to_use;
	u32 queue;
	int i;
	struct tc9562mac_tx_queue *tx_q;

	DBGPR_FUNC("-->init_dma_tx_desc_rings\n");

	for (queue = 0; queue < tx_queue_cnt; queue++) {
#ifdef UNIFIED_DRIVER
		if(priv->plat->tx_dma_ch_for_host[queue] == 0)
			continue;
#endif
		tx_q = &priv->tx_queue[queue];

		netif_dbg(priv, probe, priv->dev,
			  "(%s) dma_tx_phy=0x%08x\n", __func__,
			 (u32)tx_q->dma_tx_phy);

		/* Setup the chained descriptor addresses */
		if (priv->mode == TC9562MAC_CHAIN_MODE) {
			if (priv->extend_desc)
				priv->hw->mode->init(tx_q->dma_etx,
						     tx_q->dma_tx_phy,
						     DMA_TX_SIZE, 1);
			else
				priv->hw->mode->init(tx_q->dma_tx,
						     tx_q->dma_tx_phy,
						     DMA_TX_SIZE, 0);
			}
					
		for (i = 0; i < DMA_TX_SIZE; i++) {
			struct dma_desc *p;
			struct dma_enhanced_desc *ep;
			if (priv->extend_desc){
				p = &((tx_q->dma_etx + i)->basic);}
			else{
				if( queue == AVB_CLASSA_CH || queue == AVB_CLASSB_CH || queue == CLASS_CDT )
					{
					ep = tx_q->dma_entx + i;
						if (priv->synopsys_id >= DWMAC_CORE_4_00) {
							ep->basic.des0 = 0;
							ep->basic.des1 = 0;
							ep->basic.des2 = 0;
							ep->basic.des3 = 0;
							ep->des4 = 0;
							ep->des5 = 0;
							ep->des6 = 0;
							ep->des7 = 0;
						} else {
						ep->basic.des2 = 0;
						}		

					}
				else
					{
						p = tx_q->dma_tx + i;
						if (priv->synopsys_id >= DWMAC_CORE_4_00) {
							p->des0 = 0;
							p->des1 = 0;
							p->des2 = 0;
							p->des3 = 0;
						} else {
							p->des2 = 0;
						}
					}
				
			}
			
			tx_q->tx_skbuff_dma[i].buf = 0;
			tx_q->tx_skbuff_dma[i].map_as_page = false;
			tx_q->tx_skbuff_dma[i].len = 0;
			tx_q->tx_skbuff_dma[i].last_segment = false;
			tx_q->tx_skbuff[i] = NULL;
		}

		tx_q->dirty_tx = 0;
		tx_q->cur_tx = 0;

		netdev_tx_reset_queue(netdev_get_tx_queue(priv->dev, queue));
	}


	DBGPR_FUNC("<--init_dma_tx_desc_rings\n");

	return 0;
}

/**
 * init_dma_desc_rings - init the RX/TX descriptor rings
 * @dev: net device structure
 * @flags: gfp flag.
 * Description: this function initializes the DMA RX/TX descriptors
 * and allocates the socket buffers. It supports the chained and ring
 * modes.
 */
static int init_dma_desc_rings(struct net_device *dev, gfp_t flags)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	int ret;

	DBGPR_FUNC("-->init_dma_desc_rings\n");

	ret = init_dma_rx_desc_rings(dev, flags);
	if (ret){
		NDBGPR_L2("DMA RX ring init error \n");
		return ret;
	}

	ret = init_dma_tx_desc_rings(dev);
	if(ret)
		NDBGPR_L2("DMA TX ring init error \n");

	tc9562mac_clear_descriptors(priv);

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	if (netif_msg_hw(priv))
		tc9562mac_display_rings(priv);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

	DBGPR_FUNC("<--init_dma_desc_rings\n");

	return ret;
}

/**
 * dma_free_rx_skbufs - free RX dma buffers
 * @priv: private structure
 * @queue: RX queue index
 */
static void dma_free_rx_skbufs(struct tc9562mac_priv *priv, u32 queue)
{
	int i;

	DBGPR_FUNC("-->dma_free_rx_skbufs\n");

	for (i = 0; i < DMA_RX_SIZE; i++)
		tc9562mac_free_rx_buffer(priv, queue, i);

	DBGPR_FUNC("<--dma_free_rx_skbufs\n");
}

/**
 * dma_free_tx_skbufs - free TX dma buffers
 * @priv: private structure
 * @queue: TX queue index
 */
static void dma_free_tx_skbufs(struct tc9562mac_priv *priv, u32 queue)
{
	int i;

	DBGPR_FUNC("-->dma_free_tx_skbufs\n");

	for (i = 0; i < DMA_TX_SIZE; i++)
		tc9562mac_free_tx_buffer(priv, queue, i);

	DBGPR_FUNC("<--dma_free_tx_skbufs\n");
}

/**
 * free_dma_rx_desc_resources - free RX dma desc resources
 * @priv: private structure
 */
static void free_dma_rx_desc_resources(struct tc9562mac_priv *priv)
{
	u32 rx_count = priv->plat->rx_queues_to_use + 1;
	u32 queue;
	struct tc9562mac_rx_queue *rx_q;
	
	DBGPR_FUNC("-->free_dma_rx_desc_resources\n");

	/* Free RX queue resources */
	for (queue = 0; queue < rx_count; queue++) {
#ifdef UNIFIED_DRIVER
		if(priv->plat->rx_dma_ch_for_host[queue] == 0) 
			continue;
#endif
		rx_q = &priv->rx_queue[queue];

		/* Release the DMA RX socket buffers */
		dma_free_rx_skbufs(priv, queue);

		/* Free DMA regions of consistent memory previously allocated */
		if (!priv->extend_desc)
			dma_free_coherent(priv->device,
					  DMA_RX_SIZE * sizeof(struct dma_desc),
					  rx_q->dma_rx, rx_q->dma_rx_phy);
		else
			dma_free_coherent(priv->device, DMA_RX_SIZE *
					  sizeof(struct dma_extended_desc),
					  rx_q->dma_erx, rx_q->dma_rx_phy);

		kfree(rx_q->rx_skbuff_dma);
		kfree(rx_q->rx_skbuff);
	}

	DBGPR_FUNC("<--free_dma_rx_desc_resources\n");
}

/**
 * free_dma_tx_desc_resources - free TX dma desc resources
 * @priv: private structure
 */
static void free_dma_tx_desc_resources(struct tc9562mac_priv *priv)
{
	u32 tx_count = priv->plat->tx_queues_to_use;
	u32 queue;
	struct tc9562mac_tx_queue *tx_q;
	
	DBGPR_FUNC("-->free_dma_tx_desc_resources\n");

	/* Free TX queue resources */
	for (queue = 0; queue < tx_count; queue++) {
#ifdef UNIFIED_DRIVER
		if(priv->plat->tx_dma_ch_for_host[queue] == 0)
			continue;
#endif
		tx_q = &priv->tx_queue[queue];

		/* Release the DMA TX socket buffers */
		dma_free_tx_skbufs(priv, queue);

		/* Free DMA regions of consistent memory previously allocated */
		if (!priv->extend_desc){		
			if( queue == AVB_CLASSA_CH || queue == AVB_CLASSB_CH || queue == CLASS_CDT ) {
    			dma_free_coherent(priv->device,
					  DMA_TX_SIZE * sizeof(struct dma_enhanced_desc),
					  tx_q->dma_entx, tx_q->dma_tx_phy);
			}
			else {
			    dma_free_coherent(priv->device,
					  DMA_TX_SIZE * sizeof(struct dma_desc),
					  tx_q->dma_tx, tx_q->dma_tx_phy);
			}
		}
		else
			dma_free_coherent(priv->device, DMA_TX_SIZE *
					  sizeof(struct dma_extended_desc),
					  tx_q->dma_etx, tx_q->dma_tx_phy);

		kfree(tx_q->tx_skbuff_dma);
		kfree(tx_q->tx_skbuff);
	}
	
	DBGPR_FUNC("<--free_dma_tx_desc_resources\n");
}

/**
 * alloc_dma_rx_desc_resources - alloc RX resources.
 * @priv: private structure
 * Description: according to which descriptor can be used (extend or basic)
 * this function allocates the resources for TX and RX paths. In case of
 * reception, for example, it pre-allocated the RX socket buffer in order to
 * allow zero-copy mechanism.
 */
static int alloc_dma_rx_desc_resources(struct tc9562mac_priv *priv)
{
	u32 rx_count = priv->plat->rx_queues_to_use + 1;
	int ret = -ENOMEM;
	u32 queue;

	DBGPR_FUNC("<--alloc_dma_rx_desc_resources\n");

	/* RX queues buffers and DMA */
	for (queue = 0; queue < rx_count + 1; queue++) {
		struct tc9562mac_rx_queue *rx_q = &priv->rx_queue[queue];

		rx_q->queue_index = queue;
		rx_q->priv_data = priv;

#ifdef UNIFIED_DRIVER
	/* napi channel 3 is used for Tx interrupt handling. So initialize only queue_index and 
	priv_data and skip rest of the allocation for Rx channel 3 */
		if(priv->plat->rx_dma_ch_for_host[queue] == 0)
			continue;
#endif
	/* napi channel 6 is used for Tx interrupt handling. So initialize only queue_index and 
	priv_data and skip rest of the allocation for Rx channel 6 */
        if(queue == 5) continue;
		rx_q->rx_skbuff_dma = kmalloc_array(DMA_RX_SIZE,
						    sizeof(dma_addr_t),
						    GFP_KERNEL);
		if (!rx_q->rx_skbuff_dma)
			goto err_dma;

		rx_q->rx_skbuff = kmalloc_array(DMA_RX_SIZE,
						sizeof(struct sk_buff *),
						GFP_KERNEL);
		if (!rx_q->rx_skbuff)
			goto err_dma;

		if (priv->extend_desc) {
			rx_q->dma_erx = dma_zalloc_coherent(priv->device,
							    DMA_RX_SIZE *
							    sizeof(struct
							    dma_extended_desc),
							    &rx_q->dma_rx_phy,
							    GFP_KERNEL);
			if (!rx_q->dma_erx)
				goto err_dma;

		} else {
			rx_q->dma_rx = dma_zalloc_coherent(priv->device,
							   DMA_RX_SIZE *
							   sizeof(struct
							   dma_desc),
							   &rx_q->dma_rx_phy,
							   GFP_KERNEL);
			if (!rx_q->dma_rx)
				goto err_dma;
		}
	}

	DBGPR_FUNC("<--alloc_dma_rx_desc_resources\n");

	return 0;

err_dma:
	free_dma_rx_desc_resources(priv);

	return ret;
}

/**
 * alloc_dma_tx_desc_resources - alloc TX resources.
 * @priv: private structure
 * Description: according to which descriptor can be used (extend or basic)
 * this function allocates the resources for TX and RX paths. In case of
 * reception, for example, it pre-allocated the RX socket buffer in order to
 * allow zero-copy mechanism.
 */
static int alloc_dma_tx_desc_resources(struct tc9562mac_priv *priv)
{
	u32 tx_count = priv->plat->tx_queues_to_use;
	int ret = -ENOMEM;
	u32 queue;
	struct tc9562mac_tx_queue *tx_q;

	DBGPR_FUNC("-->alloc_dma_tx_desc_resources\n");

	/* TX queues buffers and DMA */
	for (queue = 0; queue < tx_count; queue++) {
#ifdef UNIFIED_DRIVER
		if(priv->plat->tx_dma_ch_for_host[queue] == 0)
			continue;
#endif
        tx_q = &priv->tx_queue[queue];

		tx_q->queue_index = queue;
		tx_q->priv_data = priv;

		tx_q->tx_skbuff_dma = kmalloc_array(DMA_TX_SIZE,
						    sizeof(*tx_q->tx_skbuff_dma),
						    GFP_KERNEL);
		if (!tx_q->tx_skbuff_dma)
			goto err_dma;

		tx_q->tx_skbuff = kmalloc_array(DMA_TX_SIZE,
						sizeof(struct sk_buff *),
						GFP_KERNEL);
		if (!tx_q->tx_skbuff)
			goto err_dma;

		if (priv->extend_desc) {
			tx_q->dma_etx = dma_zalloc_coherent(priv->device,
							    DMA_TX_SIZE *
							    sizeof(struct
							    dma_extended_desc),
							    &tx_q->dma_tx_phy,
							    GFP_KERNEL);
		if (!tx_q->dma_etx)
				goto err_dma;
		} else{
			if ( queue == AVB_CLASSA_CH || queue == AVB_CLASSB_CH || queue == CLASS_CDT) {
    			tx_q->dma_entx = dma_zalloc_coherent(priv->device,
							   DMA_TX_SIZE *
							   sizeof(struct
								  dma_enhanced_desc),
							   &tx_q->dma_tx_phy,
							   GFP_KERNEL);
					if (!tx_q->dma_entx)
					goto err_dma;
			} else {
				tx_q->dma_tx = dma_zalloc_coherent(priv->device,
							   DMA_TX_SIZE *
							   sizeof(struct
								  dma_desc),
							   &tx_q->dma_tx_phy,
							   GFP_KERNEL);
	            if (!tx_q->dma_tx)
		            goto err_dma;
		    }
		}
	}	

	DBGPR_FUNC("<--alloc_dma_tx_desc_resources\n");

	return 0;

err_dma:
	free_dma_tx_desc_resources(priv);

	return ret;
}

/**
 * alloc_dma_desc_resources - alloc TX/RX resources.
 * @priv: private structure
 * Description: according to which descriptor can be used (extend or basic)
 * this function allocates the resources for TX and RX paths. In case of
 * reception, for example, it pre-allocated the RX socket buffer in order to
 * allow zero-copy mechanism.
 */
static int alloc_dma_desc_resources(struct tc9562mac_priv *priv)
{
	/* RX Allocation */
	int ret = 0;
	
	DBGPR_FUNC("-->alloc_dma_desc_resources\n");

	ret = alloc_dma_rx_desc_resources(priv);

	if (ret){
		NDBGPR_L2("RX Allocation failure \n");
		return ret;
	}
	ret = alloc_dma_tx_desc_resources(priv);
	if (ret)
		NDBGPR_L2("TX Allocation failure \n");
		
	DBGPR_FUNC("<--alloc_dma_desc_resources\n");

	return ret;
}

/**
 * free_dma_desc_resources - free dma desc resources
 * @priv: private structure
 */
static void free_dma_desc_resources(struct tc9562mac_priv *priv)
{
	DBGPR_FUNC("-->free_dma_desc_resources\n");
	
	/* Release the DMA RX socket buffers */
	free_dma_rx_desc_resources(priv);

	/* Release the DMA TX socket buffers */
	free_dma_tx_desc_resources(priv);

	DBGPR_FUNC("<--free_dma_desc_resources\n");
}

/**
 *  tc9562mac_mac_enable_rx_queues - Enable MAC rx queues
 *  @priv: driver private structure
 *  Description: It is used for enabling the rx queues in the MAC
 */
static void tc9562mac_mac_enable_rx_queues(struct tc9562mac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	int queue;
	u8 mode;

	DBGPR_FUNC("-->tc9562mac_mac_enable_rx_queues\n");

	for (queue = 0; queue < rx_queues_count; queue++) {
		mode = priv->plat->rx_queues_cfg[queue].mode_to_use;
		priv->hw->mac->rx_queue_enable(priv->hw, mode, queue);
	}
	NDBGPR_L1("RXQ_Ctrl0 = %x\n", readl(priv->ioaddr + GMAC_RXQ_CTRL0));
	DBGPR_FUNC("<--tc9562mac_mac_enable_rx_queues\n");
}

/**
 * tc9562mac_start_rx_dma - start RX DMA channel
 * @priv: driver private structure
 * @chan: RX channel index
 * Description:
 * This starts a RX DMA channel
 */
static void tc9562mac_start_rx_dma(struct tc9562mac_priv *priv, u32 chan)
{
	DBGPR_FUNC("-->tc9562mac_start_rx_dma\n");

	netdev_dbg(priv->dev, "DMA RX processes started in channel %d\n", chan);
	priv->hw->dma->start_rx(priv->ioaddr, chan);

	DBGPR_FUNC("<--tc9562mac_start_rx_dma\n");
}

/**
 * tc9562mac_start_tx_dma - start TX DMA channel
 * @priv: driver private structure
 * @chan: TX channel index
 * Description:
 * This starts a TX DMA channel
 */
static void tc9562mac_start_tx_dma(struct tc9562mac_priv *priv, u32 chan)
{
	DBGPR_FUNC("-->tc9562mac_start_tx_dma\n");

	netdev_dbg(priv->dev, "DMA TX processes started in channel %d\n", chan);
	priv->hw->dma->start_tx(priv->ioaddr, chan);
	
	DBGPR_FUNC("<--tc9562mac_start_tx_dma\n");
}

/**
 * tc9562mac_stop_rx_dma - stop RX DMA channel
 * @priv: driver private structure
 * @chan: RX channel index
 * Description:
 * This stops a RX DMA channel
 */
static void tc9562mac_stop_rx_dma(struct tc9562mac_priv *priv, u32 chan)
{
	DBGPR_FUNC("-->tc9562mac_stop_rx_dma\n");

	netdev_dbg(priv->dev, "DMA RX processes stopped in channel %d\n", chan);
	priv->hw->dma->stop_rx(priv->ioaddr, chan);
	
	DBGPR_FUNC("<--tc9562mac_stop_rx_dma\n");
}

/**
 * tc9562mac_stop_tx_dma - stop TX DMA channel
 * @priv: driver private structure
 * @chan: TX channel index
 * Description:
 * This stops a TX DMA channel
 */
static void tc9562mac_stop_tx_dma(struct tc9562mac_priv *priv, u32 chan)
{
	DBGPR_FUNC("-->tc9562mac_stop_tx_dma\n");

	netdev_dbg(priv->dev, "DMA TX processes stopped in channel %d\n", chan);
	priv->hw->dma->stop_tx(priv->ioaddr, chan);

	DBGPR_FUNC("<--tc9562mac_stop_tx_dma\n");
}

/**
 * tc9562mac_start_all_dma - start all RX and TX DMA channels
 * @priv: driver private structure
 * Description:
 * This starts all the RX and TX DMA channels
 */
static void tc9562mac_start_all_dma(struct tc9562mac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_queues_to_use + 1;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	u32 chan = 0;

	DBGPR_FUNC("-->tc9562mac_start_all_dma\n");

	for (chan = 0; chan < rx_channels_count; chan++)
	{
#ifdef UNIFIED_DRIVER
		if(priv->plat->rx_dma_ch_for_host[chan] == 0)
			continue;
#endif
		tc9562mac_start_rx_dma(priv, chan);
	}

	for (chan = 0; chan < tx_channels_count; chan++)
	{
#ifdef UNIFIED_DRIVER
		if(priv->plat->tx_dma_ch_for_host[chan] == 0)
			continue;
#endif
		tc9562mac_start_tx_dma(priv, chan);
	}

	DBGPR_FUNC("<--tc9562mac_start_all_dma\n");
}

/**
 * tc9562mac_stop_all_dma - stop all RX and TX DMA channels
 * @priv: driver private structure
 * Description:
 * This stops the RX and TX DMA channels
 */
static void tc9562mac_stop_all_dma(struct tc9562mac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_queues_to_use + 1;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	u32 chan = 0;

	DBGPR_FUNC("-->tc9562mac_stop_all_dma\n");

	for (chan = 0; chan < rx_channels_count; chan++)
	{
#ifdef UNIFIED_DRIVER
		if(priv->plat->rx_dma_ch_for_host[chan] == 0)
			continue;
#endif
		tc9562mac_stop_rx_dma(priv, chan);
	}

	for (chan = 0; chan < tx_channels_count; chan++)
	{
#ifdef UNIFIED_DRIVER
	if(priv->plat->tx_dma_ch_for_host[chan] == 0)
		continue;
#endif
		tc9562mac_stop_tx_dma(priv, chan);
	}

	DBGPR_FUNC("<--tc9562mac_stop_all_dma\n");
}

/**
 *  tc9562mac_dma_operation_mode - HW DMA operation mode
 *  @priv: driver private structure
 *  Description: it is used for configuring the DMA operation mode register in
 *  order to program the tx/rx DMA thresholds or Store-And-Forward mode.
 */
static void tc9562mac_dma_operation_mode(struct tc9562mac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_queues_to_use;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	int rxfifosz = priv->plat->rx_fifo_size;
	int txfifosz = priv->plat->tx_fifo_size;
	u32 txmode = 0;
	u32 rxmode = 0;
	u32 chan = 0;
	u8 qmode = 0;

	DBGPR_FUNC("-->tc9562mac_dma_operation_mode\n");

	if (rxfifosz == 0)
		rxfifosz = priv->dma_cap.rx_fifo_size;
	if (txfifosz == 0)
		txfifosz = priv->dma_cap.tx_fifo_size;

	/* Adjust for real per queue fifo size */
	rxfifosz /= rx_channels_count;
	txfifosz /= tx_channels_count;

	if (priv->plat->force_thresh_dma_mode) {
		txmode = tc;
		rxmode = tc;
	} else if (priv->plat->force_sf_dma_mode || priv->plat->tx_coe) { /*MAC_HW_feature 14 th bit set, tx_coe is set */
		/*
		 * In case of GMAC, SF mode can be enabled
		 * to perform the TX COE in HW. This depends on:
		 * 1) TX COE if actually supported
		 * 2) There is no bugged Jumbo frame support
		 *    that needs to not insert csum in the TDES.
		 */
		txmode = SF_DMA_MODE;
		rxmode = SF_DMA_MODE;
		priv->xstats.threshold = SF_DMA_MODE;
	} else {
		txmode = tc;
		rxmode = SF_DMA_MODE;
	}

	/* configure all channels */
	if (priv->synopsys_id >= DWMAC_CORE_4_00) {
		for (chan = 0; chan < rx_channels_count; chan++) {
#ifdef TC9562_DEFINED
			switch (chan) {
			case 0: /* 512 *2 = 1024  B*/
				rxfifosz = 1024;
				break;	
			case 1: /* 1536 *2 = 3072  B*/
				rxfifosz = 3072;
				break;
			case 2: /* 1536 *2 = 3072  B*/
				rxfifosz = 3072;
				break;
			case 3: /* 1536 *2 = 3072  B*/
				rxfifosz = 3072;
				break;
			}
#endif			
			qmode = priv->plat->rx_queues_cfg[chan].mode_to_use;

			priv->hw->dma->dma_rx_mode(priv->ioaddr, rxmode, chan,
						   rxfifosz, qmode);
		}

		for (chan = 0; chan < tx_channels_count; chan++) {
#ifdef TC9562_DEFINED
#ifndef UNIFIED_DRIVER
			switch (chan) {
			case 0: /* 1536 *2 = 3072  B*/
				txfifosz = 3072;
				break;
			case 1: /* 512 *2 = 1024  B*/
				txfifosz = 1024;
				break;
			case 2: /* 512 *2 = 1024  B*/
				txfifosz = 1024;
				break;
			case 3: /* 1536 *2 = 3072  B*/
				txfifosz = 3072;
				break;
			case 4: /* 1536 *2 = 3072  B*/
				txfifosz = 3072;
				break;
			case 5: /* 1536 *2 = 3072  B*/
				txfifosz = 3072;
				break;
			}
#else
			switch (chan) {
			case 0: 
				txfifosz = 3072;
				break;
			case 1: 
				txfifosz = 2048;
				break;
			case 2: 
				txfifosz = 1024;
				break;
			case 3: 
				txfifosz = 3072;
				break;
			case 4: 
				txfifosz = 3072;
				break;
			case 5: 
				txfifosz = 3072;
				break;
			}

#endif
#endif
			qmode = priv->plat->tx_queues_cfg[chan].mode_to_use;

			priv->hw->dma->dma_tx_mode(priv->ioaddr, txmode, chan,
						   txfifosz, qmode);
		}
	} else {
		priv->hw->dma->dma_mode(priv->ioaddr, txmode, rxmode,
					rxfifosz);
	}

	DBGPR_FUNC("<--tc9562mac_dma_operation_mode\n");
}

/**
 * tc9562mac_tx_clean - to manage the transmission completion
 * @priv: driver private structure
 * @queue: TX queue index
 * Description: it reclaims the transmit resources after transmission completes.
 */
static void tc9562mac_tx_clean(struct tc9562mac_priv *priv, u32 queue)
{
	struct tc9562mac_tx_queue *tx_q = &priv->tx_queue[queue];
	unsigned int bytes_compl = 0, pkts_compl = 0;
	unsigned int entry;
	
	DBGPR_FUNC("-->tc9562mac_tx_clean\n");
	netif_tx_lock(priv->dev);

	priv->xstats.tx_clean++;

	entry = tx_q->dirty_tx;
	while (entry != tx_q->cur_tx) {
		struct sk_buff *skb = tx_q->tx_skbuff[entry];
		struct dma_desc *p = NULL;
		struct dma_enhanced_desc *ep = NULL;
		int status;
	if( (tx_q->queue_index == AVB_CLASSA_CH) || (tx_q->queue_index == AVB_CLASSB_CH) || (tx_q->queue_index == CLASS_CDT))
	{
		ep = tx_q->dma_entx + entry;
		status = priv->hw->desc->etx_status(&priv->dev->stats,
						      &priv->xstats, ep,
						      priv->ioaddr);
	}
	else 
	{
		if (priv->extend_desc)
			p = (struct dma_desc *)(tx_q->dma_etx + entry);
		else
			p = tx_q->dma_tx + entry;

		status = priv->hw->desc->tx_status(&priv->dev->stats,
						      &priv->xstats, p,
						      priv->ioaddr);
	}
		/* Check if the descriptor is owned by the DMA */
		if (unlikely(status & tx_dma_own))
			break;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,10))
		dma_rmb(); 
#endif
		/* Just consider the last segment and ...*/
		if (likely(!(status & tx_not_ls))) {
			/* ... verify the status error condition */
			if (unlikely(status & tx_err)) {
				priv->dev->stats.tx_errors++;
				printk("tx error : %x, queue: %x\n", status, queue);
			} else {
				priv->dev->stats.tx_packets++;
				priv->xstats.tx_pkt_n++;
			}
    			if( (tx_q->queue_index == AVB_CLASSA_CH) || (tx_q->queue_index == AVB_CLASSB_CH) || (tx_q->queue_index == CLASS_CDT))
				{
#if defined(TX_LOGGING_TRACE)
					tc9562mac_get_etx_hwtstamp(priv, ep, skb, queue);
#else
					tc9562mac_get_etx_hwtstamp(priv, ep, skb);
#endif
				}
				else
				{
#if defined (TX_LOGGING_TRACE)
					tc9562mac_get_tx_hwtstamp(priv, p, skb, queue);
#else
					tc9562mac_get_tx_hwtstamp(priv, p, skb);
#endif
				}
		}

		if (likely(tx_q->tx_skbuff_dma[entry].buf)) {
			if (tx_q->tx_skbuff_dma[entry].map_as_page)
				dma_unmap_page(priv->device,
					       tx_q->tx_skbuff_dma[entry].buf,
					       tx_q->tx_skbuff_dma[entry].len,
					       DMA_TO_DEVICE);
			else
				dma_unmap_single(priv->device,
						 tx_q->tx_skbuff_dma[entry].buf,
						 tx_q->tx_skbuff_dma[entry].len,
						 DMA_TO_DEVICE);
			tx_q->tx_skbuff_dma[entry].buf = 0;
			tx_q->tx_skbuff_dma[entry].len = 0;
			tx_q->tx_skbuff_dma[entry].map_as_page = false;
		}

		if (priv->hw->mode->clean_desc3)
			priv->hw->mode->clean_desc3(tx_q, p);

		tx_q->tx_skbuff_dma[entry].last_segment = false;
		tx_q->tx_skbuff_dma[entry].is_jumbo = false;

		if (likely(skb != NULL)) {
			pkts_compl++;
			bytes_compl += skb->len;
			dev_consume_skb_any(skb);
			tx_q->tx_skbuff[entry] = NULL;
		}
		if( (tx_q->queue_index == AVB_CLASSA_CH) || (tx_q->queue_index == AVB_CLASSB_CH) || (tx_q->queue_index == CLASS_CDT))
		{
			priv->hw->desc->release_etx_desc(ep, priv->mode);
		}
		else
		{
			priv->hw->desc->release_tx_desc(p, priv->mode);
		}

		entry = TC9562MAC_GET_ENTRY(entry, DMA_TX_SIZE);
	}
	tx_q->dirty_tx = entry;

	netdev_tx_completed_queue(netdev_get_tx_queue(priv->dev, queue),
				  pkts_compl, bytes_compl);

	if (unlikely(netif_tx_queue_stopped(netdev_get_tx_queue(priv->dev,
								queue))) &&
	    tc9562mac_tx_avail(priv, queue) > TC9562MAC_TX_THRESH) {

		netif_dbg(priv, tx_done, priv->dev,
			  "%s: restart transmit\n", __func__);
		netif_tx_wake_queue(netdev_get_tx_queue(priv->dev, queue));
	}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	if ((priv->eee_enabled) && (!priv->tx_path_in_lpi_mode)) {
		tc9562mac_enable_eee_mode(priv);
		mod_timer(&priv->eee_ctrl_timer, TC9562MAC_LPI_T(eee_timer));
	}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */	
	netif_tx_unlock(priv->dev);

	DBGPR_FUNC("<--tc9562mac_tx_clean\n");
}

static inline void tc9562mac_enable_dma_irq(struct tc9562mac_priv *priv, u32 chan)
{
	DBGPR_FUNC("-->tc9562mac_enable_dma_irq\n");
	priv->hw->dma->enable_dma_irq(priv->ioaddr, chan);
	DBGPR_FUNC("<--tc9562mac_enable_dma_irq\n");
}

static inline void tc9562mac_disable_dma_irq(struct tc9562mac_priv *priv, u32 chan)
{
	DBGPR_FUNC("-->tc9562mac_disable_dma_irq\n");
	priv->hw->dma->disable_dma_irq(priv->ioaddr, chan);
	DBGPR_FUNC("<--tc9562mac_disable_dma_irq\n");
}

/**
 * tc9562mac_tx_err - to manage the tx error
 * @priv: driver private structure
 * @chan: channel index
 * Description: it cleans the descriptors and restarts the transmission
 * in case of transmission errors.
 */
static void tc9562mac_tx_err(struct tc9562mac_priv *priv, u32 chan)
{
	struct tc9562mac_tx_queue *tx_q = &priv->tx_queue[chan];
	DBGPR_FUNC("-->tc9562mac_tx_err\n");
	netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, chan));

	tc9562mac_stop_tx_dma(priv, chan);
	dma_free_tx_skbufs(priv, chan);
	tc9562mac_clear_tx_descriptors(priv, chan);
	tx_q->dirty_tx = 0;
	tx_q->cur_tx = 0;
	netdev_tx_reset_queue(netdev_get_tx_queue(priv->dev, chan));
	tc9562mac_start_tx_dma(priv, chan);

	priv->dev->stats.tx_errors++;
	netif_tx_wake_queue(netdev_get_tx_queue(priv->dev, chan));
	DBGPR_FUNC("<--tc9562mac_tx_err\n");
}
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
/**
 *  tc9562mac_set_dma_operation_mode - Set DMA operation mode by channel
 *  @priv: driver private structure
 *  @txmode: TX operating mode
 *  @rxmode: RX operating mode
 *  @chan: channel index
 *  Description: it is used for configuring of the DMA operation mode in
 *  runtime in order to program the tx/rx DMA thresholds or Store-And-Forward
 *  mode.
 */
static void tc9562mac_set_dma_operation_mode(struct tc9562mac_priv *priv, u32 txmode,
					  u32 rxmode, u32 chan)
{
	u8 rxqmode = priv->plat->rx_queues_cfg[chan].mode_to_use;
	u8 txqmode = priv->plat->tx_queues_cfg[chan].mode_to_use;
	u32 rx_channels_count = priv->plat->rx_queues_to_use;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	int rxfifosz = priv->plat->rx_fifo_size;
	int txfifosz = priv->plat->tx_fifo_size;

	DBGPR_FUNC("-->tc9562mac_set_dma_operation_mode\n");

	if (rxfifosz == 0)
		rxfifosz = priv->dma_cap.rx_fifo_size;
	if (txfifosz == 0)
		txfifosz = priv->dma_cap.tx_fifo_size;

	/* Adjust for real per queue fifo size */
	rxfifosz /= rx_channels_count;
	txfifosz /= tx_channels_count;

	if (priv->synopsys_id >= DWMAC_CORE_4_00) {
		priv->hw->dma->dma_rx_mode(priv->ioaddr, rxmode, chan,
					   rxfifosz, rxqmode);
		priv->hw->dma->dma_tx_mode(priv->ioaddr, txmode, chan,
					   txfifosz, txqmode);
	} else {
		priv->hw->dma->dma_mode(priv->ioaddr, txmode, rxmode,
					rxfifosz);
	}

	DBGPR_FUNC("<--tc9562mac_set_dma_operation_mode\n");

}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

static void tc9562mac_global_err(struct tc9562mac_priv *priv);

/**
 * tc9562mac_safety_feat_interrupt - Safety Features ISR
 * @priv: driver private structure
 * Description: this is the safety features ISR. It is called by the main ISR.
 * It calls the dwmac safety feature interrupt and if an error is returned
 * then a reset is performed
 */
static bool tc9562mac_safety_feat_interrupt(struct tc9562mac_priv *priv)
{
	bool ret = false;

	DBGPR_FUNC("-->tc9562mac_safety_feat_interrupt\n");

	/* Safety features are only available in cores >= 5.10 */
	if (priv->synopsys_id < DWMAC_CORE_5_10)
		return ret;

	if (priv->hw->mac->safety_feat_irq_status)
		ret = priv->hw->mac->safety_feat_irq_status(priv->dev, priv->hw,
				priv->dma_cap.asp);

	if (ret)
		tc9562mac_global_err(priv);

	DBGPR_FUNC("<--tc9562mac_safety_feat_interrupt\n");
	return ret;
}

/*MCGR is not applicable for TC9562, so commented the same*/
#ifndef TC9562_DEFINED
static void tc9562mac_mcgr_interrupt(struct tc9562mac_priv *priv)
{
	DBGPR_FUNC("-->tc9562mac_mcgr_interrupt\n");

	if (priv->hw->mac->mcgr_intr)
	{
		priv->hw->mac->mcgr_intr(priv->hw, priv->plat->mcgr_cfg,
				priv->dma_cap.ppsoutnum);
	}

	DBGPR_FUNC("<--tc9562mac_mcgr_interrupt\n");
}
#endif

/**
 * tc9562mac_dma_interrupt - DMA ISR
 * @priv: driver private structure
 * Description: this is the DMA ISR. It is called by the main ISR.
 * It calls the dwmac dma routine and schedule poll method in case of some
 * work can be done.
 */
static void tc9562mac_dma_interrupt(struct tc9562mac_priv *priv)
{
	u32 tx_channel_count = priv->plat->tx_queues_to_use;
	int status;
	u32 chan;
	struct tc9562mac_rx_queue *rx_q;

	DBGPR_FUNC("-->tc9562mac_dma_interrupt\n");

	for (chan = 0; chan < tx_channel_count; chan++) {
#ifdef UNIFIED_DRIVER
		/* Tx 1,2,3 and Rx ,1 ,2,3,5 handled by CM3, Tx - 0,4,5 and Rx 0, 4 handled by Host */
		
		/*So incase of Tx 3, raise napi but dont process RxQ = 3 */
		if((priv->plat->tx_dma_ch_for_host[chan] == 0) && (priv->plat->rx_dma_ch_for_host[chan] == 0))
			continue;
#endif

		rx_q = &priv->rx_queue[chan];

		status = priv->hw->dma->dma_interrupt(priv->ioaddr,
						      &priv->xstats, chan);
						      
#ifdef UNIFIED_DRIVER
		if((priv->plat->tx_dma_ch_for_host[chan] == 1) && (priv->plat->rx_dma_ch_for_host[chan] == 1))
		{
		if (likely((status & handle_rx)) || (status & handle_tx)) {
			if (likely(napi_schedule_prep(&rx_q->napi))) {
				tc9562mac_disable_dma_irq(priv, chan);
				__napi_schedule(&rx_q->napi);
			}
		}
		}
		else if(priv->plat->tx_dma_ch_for_host[chan] == 1)	/*Chan 3 Rx is used by CM3, so handle Host Tx Ch3*/
		{				
			if (status & handle_tx) {
				if (likely(napi_schedule_prep(&rx_q->napi))) {
					tc9562mac_disable_dma_irq(priv, chan);
					__napi_schedule(&rx_q->napi);
				}
			}
			
		}
		else {
		    NMSGPR_ALERT("Rx channel %d being handled by Host\n", chan);
		}
#else
		if (likely((status & handle_rx)) || (status & handle_tx)) {
			if (likely(napi_schedule_prep(&rx_q->napi))) {
				tc9562mac_disable_dma_irq(priv, chan);
				__napi_schedule(&rx_q->napi);
			}
		}
#endif

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
		if (unlikely(status & tx_hard_error_bump_tc)) {
			/* Try to bump up the dma threshold on this failure */
			if (unlikely(priv->xstats.threshold != SF_DMA_MODE) &&
			    (tc <= 256)) {
				tc += 64;
				if (priv->plat->force_thresh_dma_mode)
					tc9562mac_set_dma_operation_mode(priv,
								      tc,
								      tc,
								      chan);
				else
					tc9562mac_set_dma_operation_mode(priv,
								    tc,
								    SF_DMA_MODE,
								    chan);
				priv->xstats.threshold = tc;
			}
		} else if (unlikely(status == tx_hard_error)) {
#else
		if (unlikely(status == tx_hard_error)) {
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
			tc9562mac_tx_err(priv, chan);
		}
	}

	DBGPR_FUNC("<--tc9562mac_dma_interrupt\n");
}

/**
 * tc9562mac_mmc_setup: setup the Mac Management Counters (MMC)
 * @priv: driver private structure
 * Description: this masks the MMC irq, in fact, the counters are managed in SW.
 */
static void tc9562mac_mmc_setup(struct tc9562mac_priv *priv)
{
	unsigned int mode = MMC_CNTRL_RESET_ON_READ | MMC_CNTRL_COUNTER_RESET | MMC_CNTRL_COUNTER_STOP_ROLLOVER;

	DBGPR_FUNC("-->tc9562mac_mmc_setup\n");

	if (priv->synopsys_id >= DWMAC_CORE_4_00) {
		priv->ptpaddr = priv->ioaddr + PTP_GMAC4_OFFSET;
		priv->mmcaddr = priv->ioaddr + MMC_GMAC4_OFFSET;
	} else {
		priv->ptpaddr = priv->ioaddr + PTP_GMAC3_X_OFFSET;
		priv->mmcaddr = priv->ioaddr + MMC_GMAC3_X_OFFSET;
	}

	dwmac_mmc_intr_all_mask(priv->mmcaddr);

	if (priv->dma_cap.rmon) {
		dwmac_mmc_ctrl(priv->mmcaddr, mode);
		memset(&priv->mmc, 0, sizeof(struct tc9562mac_counters));
	} else
		netdev_info(priv->dev, "No MAC Management Counters available\n");

	DBGPR_FUNC("<--tc9562mac_mmc_setup\n");
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
/**
 * tc9562mac_selec_desc_mode - to select among: normal/alternate/extend descriptors
 * @priv: driver private structure
 * Description: select the Enhanced/Alternate or Normal descriptors.
 * In case of Enhanced/Alternate, it checks if the extended descriptors are
 * supported by the HW capability register.
 */
static void tc9562mac_selec_desc_mode(struct tc9562mac_priv *priv)
{
	DBGPR_FUNC("-->tc9562mac_selec_desc_mode\n");

	if (priv->plat->enh_desc) {
		dev_info(priv->device, "Enhanced/Alternate descriptors\n");

		/* GMAC older than 3.50 has no extended descriptors */
		if (priv->synopsys_id >= DWMAC_CORE_3_50) {
			dev_info(priv->device, "Enabled extended descriptors\n");
			priv->extend_desc = 1;
		} else
			dev_warn(priv->device, "Extended descriptors not supported\n");

		priv->hw->desc = &enh_desc_ops;
	} else {
		dev_info(priv->device, "Normal descriptors\n");
		priv->hw->desc = &ndesc_ops;
	}

	DBGPR_FUNC("<--tc9562mac_selec_desc_mode\n");
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

/**
 * tc9562mac_get_hw_features - get MAC capabilities from the HW cap. register.
 * @priv: driver private structure
 * Description:
 *  new GMAC chip generations have a new register to indicate the
 *  presence of the optional feature/functions.
 *  This can be also used to override the value passed through the
 *  platform and necessary for old MAC10/100 and GMAC chips.
 */
static int tc9562mac_get_hw_features(struct tc9562mac_priv *priv)
{
	u32 ret = 0;

	DBGPR_FUNC("-->tc9562mac_get_hw_features\n");

	if (priv->hw->dma->get_hw_feature) {
		priv->hw->dma->get_hw_feature(priv->ioaddr,
					      &priv->dma_cap,
					      priv->synopsys_id);
		ret = 1;
	}

	DBGPR_FUNC("<--tc9562mac_get_hw_features\n");

	return ret;
}

/**
 * tc9562mac_check_ether_addr - check if the MAC addr is valid
 * @priv: driver private structure
 * Description:
 * it is to verify if the MAC address is valid, in case of failures it
 * generates a random MAC address
 */
static void tc9562mac_check_ether_addr(struct tc9562mac_priv *priv)
{
	DBGPR_FUNC("-->tc9562mac_check_ether_addr\n");

	if (!is_valid_ether_addr(priv->dev->dev_addr)) {
#ifndef UNIFIED_DRIVER
		priv->hw->mac->get_umac_addr(priv->hw,
					     priv->dev->dev_addr, 0);
#else
		priv->hw->mac->get_umac_addr(priv->hw,
					     priv->dev->dev_addr, HOST_MAC_ADDR_OFFSET);
#endif
		if (!is_valid_ether_addr(priv->dev->dev_addr))
			eth_hw_addr_random(priv->dev);
		netdev_info(priv->dev, "device MAC address %pM\n",
			    priv->dev->dev_addr);
	}

	DBGPR_FUNC("<--tc9562mac_check_ether_addr\n");
}

/**
 * tc9562mac_init_dma_engine - DMA init.
 * @priv: driver private structure
 * Description:
 * It inits the DMA invoking the specific MAC/GMAC callback.
 * Some DMA parameters can be passed from the platform;
 * in case of these are not passed a default is kept for the MAC or GMAC.
 */
static int tc9562mac_init_dma_engine(struct tc9562mac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_queues_to_use + 1;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	struct tc9562mac_rx_queue *rx_q;
	struct tc9562mac_tx_queue *tx_q;
	u32 dummy_dma_rx_phy = 0;
	u32 dummy_dma_tx_phy = 0;
	u32 chan = 0;
	int atds = 0;
	int ret = 0;

	DBGPR_FUNC("-->tc9562mac_init_dma_engine\n");

	if (!priv->plat->dma_cfg || !priv->plat->dma_cfg->txpbl || !priv->plat->dma_cfg->rxpbl) {
		dev_err(priv->device, "Invalid DMA configuration\n");
		return -EINVAL;
	}

	if (priv->extend_desc && (priv->mode == TC9562MAC_RING_MODE))
		atds = 1;
	ret = priv->hw->dma->reset(priv->ioaddr);
	if (ret) {
		dev_err(priv->device, "Failed to reset the dma\n");
		return ret;
	}

	if (priv->synopsys_id >= DWMAC_CORE_4_00) {
		/* DMA Configuration */
		priv->hw->dma->init(priv->ioaddr, priv->plat->dma_cfg,
				    dummy_dma_tx_phy, dummy_dma_rx_phy, atds);

		/* DMA RX Channel Configuration */
		for (chan = 0; chan < rx_channels_count; chan++) {
#ifdef UNIFIED_DRIVER
				if(priv->plat->rx_dma_ch_for_host[chan] == 0)
					continue;
#endif
			rx_q = &priv->rx_queue[chan];

			priv->hw->dma->init_rx_chan(priv->ioaddr,
						    priv->plat->dma_cfg,
						    rx_q->dma_rx_phy, priv->dma_buf_sz, chan);

			rx_q->rx_tail_addr = rx_q->dma_rx_phy +
				    (DMA_RX_SIZE * sizeof(struct dma_desc));
			priv->hw->dma->set_rx_tail_ptr(priv->ioaddr,
						       rx_q->rx_tail_addr,
						       chan);
		}

		/* DMA TX Channel Configuration */
		for (chan = 0; chan < tx_channels_count; chan++) {
#ifdef UNIFIED_DRIVER
			if(priv->plat->tx_dma_ch_for_host[chan] == 0)
				continue;
#endif
			tx_q = &priv->tx_queue[chan];

			priv->hw->dma->init_chan(priv->ioaddr,
						 priv->plat->dma_cfg,
						 chan);

			priv->hw->dma->init_tx_chan(priv->ioaddr,
						    priv->plat->dma_cfg,
						    tx_q->dma_tx_phy, chan);
			if( (chan == AVB_CLASSA_CH) || (chan == AVB_CLASSB_CH) || (chan == CLASS_CDT))
			{
				tx_q->tx_tail_addr = tx_q->dma_tx_phy +
				    (DMA_TX_SIZE * sizeof(struct dma_enhanced_desc));
				priv->hw->dma->set_tx_tail_ptr(priv->ioaddr,
						       tx_q->tx_tail_addr,
						       chan);
			}
			else {
				tx_q->tx_tail_addr = tx_q->dma_tx_phy +
				    (DMA_TX_SIZE * sizeof(struct dma_desc));
				priv->hw->dma->set_tx_tail_ptr(priv->ioaddr,
						       tx_q->tx_tail_addr,
						       chan);
			}
		}
	} else {
		rx_q = &priv->rx_queue[chan];
		tx_q = &priv->tx_queue[chan];
		priv->hw->dma->init(priv->ioaddr, priv->plat->dma_cfg,
				    tx_q->dma_tx_phy, rx_q->dma_rx_phy, atds);
	}

	if (priv->plat->axi && priv->hw->dma->axi)
		priv->hw->dma->axi(priv->ioaddr, priv->plat->axi);

	DBGPR_FUNC("<--tc9562mac_init_dma_engine\n");

	return ret;
}

/**
 * tc9562mac_tx_timer - mitigation sw timer for tx.
 * @data: data pointer
 * Description:
 * This is the timer handler to directly invoke the tc9562mac_tx_clean.
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,10))
static void tc9562mac_tx_timer(struct timer_list *timer)
{
	struct tc9562mac_priv *priv = from_timer(priv, timer, txtimer);
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 queue;

	DBGPR_FUNC("-->tc9562mac_tx_timer\n");

	/* let's scan all the tx queues */
	for (queue = 0; queue < tx_queues_count; queue++)
		tc9562mac_tx_clean(priv, queue);

	DBGPR_FUNC("<--tc9562mac_tx_timer\n");
}
#else
static void tc9562mac_tx_timer(unsigned long data)
{
	struct tc9562mac_priv *priv = (struct tc9562mac_priv *)data;
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 queue;

	DBGPR_FUNC("-->tc9562mac_tx_timer\n");

	/* let's scan all the tx queues */
	for (queue = 0; queue < tx_queues_count; queue++)
	{
#ifdef UNIFIED_DRIVER
	if(priv->plat->tx_dma_ch_for_host[queue] == 0)
		continue;
#endif
		tc9562mac_tx_clean(priv, queue);
	}

	DBGPR_FUNC("<--tc9562mac_tx_timer\n");
}
#endif

/**
 * tc9562mac_init_tx_coalesce - init tx mitigation options.
 * @priv: driver private structure
 * Description:
 * This inits the transmit coalesce parameters: i.e. timer rate,
 * timer handler and default threshold used for enabling the
 * interrupt on completion bit.
 */
static void tc9562mac_init_tx_coalesce(struct tc9562mac_priv *priv)
{
	DBGPR_FUNC("-->tc9562mac_init_tx_coalesce\n");

	priv->tx_coal_frames = TC9562MAC_TX_FRAMES;
	priv->tx_coal_timer = TC9562MAC_COAL_TX_TIMER;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,10))
	priv->txtimer.expires = TC9562MAC_COAL_TIMER(priv->tx_coal_timer);
	timer_setup(&priv->txtimer, tc9562mac_tx_timer, 0);
	/* mod_timer(&priv->txtimer, priv->tx_coal_timer); */
	add_timer(&priv->txtimer);
#else
	init_timer(&priv->txtimer);
	priv->txtimer.expires = TC9562MAC_COAL_TIMER(priv->tx_coal_timer);
	priv->txtimer.data = (unsigned long)priv;
	priv->txtimer.function = tc9562mac_tx_timer;
	add_timer(&priv->txtimer);
#endif

	DBGPR_FUNC("<--tc9562mac_init_tx_coalesce\n");
}

static void tc9562mac_set_rings_length(struct tc9562mac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_queues_to_use + 1;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	u32 chan;

	DBGPR_FUNC("-->tc9562mac_set_rings_length\n");

	/* set TX ring length */
	if (priv->hw->dma->set_tx_ring_len) {
		for (chan = 0; chan < tx_channels_count; chan++)
		{
#ifdef UNIFIED_DRIVER
			if(priv->plat->tx_dma_ch_for_host[chan] == 0)
				continue;
#endif
			priv->hw->dma->set_tx_ring_len(priv->ioaddr,
						       (DMA_TX_SIZE - 1), chan);
		}
	}

	/* set RX ring length */
	if (priv->hw->dma->set_rx_ring_len) {
		for (chan = 0; chan < rx_channels_count; chan++)
		{
#ifdef UNIFIED_DRIVER
			if(priv->plat->rx_dma_ch_for_host[chan] == 0) 
				continue;
#endif
			priv->hw->dma->set_rx_ring_len(priv->ioaddr,
						       (DMA_RX_SIZE - 1), chan);
		}
	}

	DBGPR_FUNC("<--tc9562mac_set_rings_length\n");
}

/**
 *  tc9562mac_set_tx_queue_weight - Set TX queue weight
 *  @priv: driver private structure
 *  Description: It is used for setting TX queues weight
 */
static void tc9562mac_set_tx_queue_weight(struct tc9562mac_priv *priv)
{
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 weight;
	u32 queue;

	DBGPR_FUNC("-->tc9562mac_set_tx_queue_weight\n");

	for (queue = 0; queue < tx_queues_count; queue++) {
#ifdef UNIFIED_DRIVER
	if(priv->plat->tx_dma_ch_for_host[queue] == 0)
		continue;
#endif
		weight = priv->plat->tx_queues_cfg[queue].weight;
		priv->hw->mac->set_mtl_tx_queue_weight(priv->hw, weight, queue);
	}

	DBGPR_FUNC("<--tc9562mac_set_tx_queue_weight\n");
}

/**
 *  tc9562mac_configure_cbs - Configure CBS in TX queue
 *  @priv: driver private structure
 *  Description: It is used for configuring CBS in AVB TX queues
 */
static u32 tc9562mac_configure_cbs(struct tc9562mac_priv *priv)
{
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 queues_av = 0;
	u32 mode_to_use;
	u32 queue;

	DBGPR_FUNC("-->tc9562mac_configure_cbs\n");

    priv->cbs_cfg_status[0] = -1;
	/* queue 0 is reserved for legacy traffic */
	for (queue = 1; queue < tx_queues_count; queue++) {
#ifdef UNIFIED_DRIVER
		if(priv->plat->tx_dma_ch_for_host[queue] == 0) {
			priv->cbs_cfg_status[queue] = -1;
			continue;
		}
#endif
		mode_to_use = priv->plat->tx_queues_cfg[queue].mode_to_use;
		if (mode_to_use == MTL_QUEUE_DCB) {
		    priv->cbs_cfg_status[queue] = -1;
	    	continue;
		}
		else
			queues_av++;

        priv->cbs_cfg_status[queue] = 1;
		priv->hw->mac->config_cbs(priv->hw,
				priv->plat->tx_queues_cfg[queue].send_slope,
				priv->plat->tx_queues_cfg[queue].idle_slope,
				priv->plat->tx_queues_cfg[queue].high_credit,
				priv->plat->tx_queues_cfg[queue].low_credit,
				queue);
				
        priv->cbs_speed100_cfg[queue].send_slope = priv->plat->tx_queues_cfg[queue].send_slope;
        priv->cbs_speed100_cfg[queue].idle_slope = priv->plat->tx_queues_cfg[queue].idle_slope;
        priv->cbs_speed100_cfg[queue].high_credit = priv->plat->tx_queues_cfg[queue].high_credit;
        priv->cbs_speed100_cfg[queue].low_credit = priv->plat->tx_queues_cfg[queue].low_credit;

        priv->cbs_speed1000_cfg[queue].send_slope = priv->plat->tx_queues_cfg[queue].send_slope;
        priv->cbs_speed1000_cfg[queue].idle_slope = priv->plat->tx_queues_cfg[queue].idle_slope;
        priv->cbs_speed1000_cfg[queue].high_credit = priv->plat->tx_queues_cfg[queue].high_credit;
        priv->cbs_speed1000_cfg[queue].low_credit = priv->plat->tx_queues_cfg[queue].low_credit;
	
	}

	DBGPR_FUNC("<--tc9562mac_configure_cbs\n");

	return queues_av;
}

/**
 *  tc9562mac_rx_queue_dma_chan_map - Map RX queue to RX dma channel
 *  @priv: driver private structure
 *  Description: It is used for mapping RX queues to RX dma channels
 */
static void tc9562mac_rx_queue_dma_chan_map(struct tc9562mac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 queue;
	u32 chan;
	
	DBGPR_FUNC("-->tc9562mac_rx_queue_dma_chan_map\n");

	for (queue = 0; queue < rx_queues_count; queue++) {
		chan = priv->plat->rx_queues_cfg[queue].chan;
		priv->hw->mac->map_mtl_to_dma(priv->hw, queue, chan);
	}

	DBGPR_FUNC("<--tc9562mac_rx_queue_dma_chan_map\n");
}

/**
 *  tc9562mac_mac_config_rx_queues_prio - Configure RX Queue priority
 *  @priv: driver private structure
 *  Description: It is used for configuring the RX Queue Priority
 */
static void tc9562mac_mac_config_rx_queues_prio(struct tc9562mac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 queue;
	u32 prio;
	
	DBGPR_FUNC("-->tc9562mac_mac_config_rx_queues_prio\n");

	for (queue = 0; queue < rx_queues_count; queue++) {
		if (!priv->plat->rx_queues_cfg[queue].use_prio)
			continue;

		prio = priv->plat->rx_queues_cfg[queue].prio;
		priv->hw->mac->rx_queue_prio(priv->hw, prio, queue);
	}

	DBGPR_FUNC("<--tc9562mac_mac_config_rx_queues_prio\n");
}

/**
 *  tc9562mac_mac_config_tx_queues_prio - Configure TX Queue priority
 *  @priv: driver private structure
 *  Description: It is used for configuring the TX Queue Priority
 */
static void tc9562mac_mac_config_tx_queues_prio(struct tc9562mac_priv *priv)
{
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 queue;
	u32 prio;

	DBGPR_FUNC("-->tc9562mac_mac_config_tx_queues_prio\n");

	for (queue = 0; queue < tx_queues_count; queue++) {
		if (!priv->plat->tx_queues_cfg[queue].use_prio)
			continue;

		prio = priv->plat->tx_queues_cfg[queue].prio;
		priv->hw->mac->tx_queue_prio(priv->hw, prio, queue);
	}

	DBGPR_FUNC("<--tc9562mac_mac_config_tx_queues_prio\n");
}

/**
 *  tc9562mac_mac_config_rx_queues_routing - Configure RX Queue Routing
 *  @priv: driver private structure
 *  Description: It is used for configuring the RX queue routing
 */
static void tc9562mac_mac_config_rx_queues_routing(struct tc9562mac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 queue;
	u8 packet;

	DBGPR_FUNC("-->tc9562mac_mac_config_rx_queues_routing\n");

	for (queue = 0; queue < rx_queues_count; queue++) {
		if(queue == 0){
			packet = PACKET_PTPQ;
			priv->hw->mac->rx_queue_routing(priv->hw, packet, queue);
			packet = PACKET_AVCPQ;
			priv->hw->mac->rx_queue_routing(priv->hw, packet, queue);			
		}else if(queue == 3) {
			packet = PACKET_UPQ;
			priv->hw->mac->rx_queue_routing(priv->hw, packet, queue);			
			/* packet = PACKET_DCBCPQ;
			priv->hw->mac->rx_queue_routing(priv->hw, packet, queue);			
			packet = PACKET_MCBCQ;
			priv->hw->mac->rx_queue_routing(priv->hw, packet, queue); */
            packet = PACKET_FPE_RESIDUE;
            priv->hw->mac->rx_queue_routing(priv->hw, packet, queue);           
			
		}else {
			continue; /* no specific packet type routing specified for the queue */
		}
	}
	NDBGPR_L1("RXQ_Ctrl1 = %x\n", readl(priv->ioaddr + GMAC_RXQ_CTRL1));
	DBGPR_FUNC("<--tc9562mac_mac_config_rx_queues_routing\n");
}

static void tc9562mac_configure_tsn(struct tc9562mac_priv *priv)
{
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 queues_av = 0;

	DBGPR_FUNC("-->tc9562mac_configure_tsn\n");

	/* Initialize AVB / TSN settings */
	priv->sra_idleslope_res = 0;
	priv->srb_idleslope_res = 0;
	priv->tsn_vlan_added = 0;
	priv->tsn_ready = 0;

	if (!priv->dma_cap.av)
		return;

	/* Configure CBS in AVB / TSN TX queues */
	if (tx_queues_count > 1 && priv->hw->mac->config_cbs)
		queues_av = tc9562mac_configure_cbs(priv);

#ifndef UNIFIED_DRIVER
	if (queues_av < MIN_AVB_QUEUES) {
		pr_err("Not enough queues for AVB (only %d)\n", queues_av);
		return;
	}
#endif
	priv->tsn_ready = 1;

	DBGPR_FUNC("<--tc9562mac_configure_tsn\n");
}

/**
 *  tc9562mac_mtl_configuration - Configure MTL
 *  @priv: driver private structure
 *  Description: It is used for configurring MTL
 */
static void tc9562mac_mtl_configuration(struct tc9562mac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 tx_queues_count = priv->plat->tx_queues_to_use;

	DBGPR_FUNC("-->tc9562mac_mtl_configuration\n");

	if (tx_queues_count > 1 && priv->hw->mac->set_mtl_tx_queue_weight)
		tc9562mac_set_tx_queue_weight(priv);

	/* Configure MTL RX algorithms */
	if (rx_queues_count > 1 && priv->hw->mac->prog_mtl_rx_algorithms)
		priv->hw->mac->prog_mtl_rx_algorithms(priv->hw,
						priv->plat->rx_sched_algorithm);

	/* Configure MTL TX algorithms */
	if (tx_queues_count > 1 && priv->hw->mac->prog_mtl_tx_algorithms)
		priv->hw->mac->prog_mtl_tx_algorithms(priv->hw,
						priv->plat->tx_sched_algorithm);

	/* Initial AVB configuration */
	tc9562mac_configure_tsn(priv);
	
	/* Map RX MTL to DMA channels */
	if (priv->hw->mac->map_mtl_to_dma)
		tc9562mac_rx_queue_dma_chan_map(priv);

	/* Enable MAC RX Queues */
	if (priv->hw->mac->rx_queue_enable)
		tc9562mac_mac_enable_rx_queues(priv);

	/* Set RX priorities */
	if (rx_queues_count > 1 && priv->hw->mac->rx_queue_prio)
		tc9562mac_mac_config_rx_queues_prio(priv);

	/* Set TX priorities */
	if (tx_queues_count > 1 && priv->hw->mac->tx_queue_prio)
		tc9562mac_mac_config_tx_queues_prio(priv);

	/* Set RX routing */
	if (rx_queues_count > 1 && priv->hw->mac->rx_queue_routing)
		tc9562mac_mac_config_rx_queues_routing(priv);

	DBGPR_FUNC("<--tc9562mac_mtl_configuration\n");
}

static void Launch_time_parameters_init(struct tc9562mac_priv *priv)
{
	u32 reg_data = 0;
	/* fetch time offset register setting */
	reg_data = readl(priv->ioaddr + DMA_TBS_CTRL);
	//reg_data |= (FTOS << 8);/* | MTL_TBS_FEOV;*/ // Fetch Offset Valid bit:  disabled for testing
	reg_data |= (100 << 8) | (1 << 4) | (1 << 0);
	writel(reg_data, priv->ioaddr + DMA_TBS_CTRL);

	/* expiry time offset register setting */
	reg_data = readl(priv->ioaddr + MTL_TBS_CTRL);
	//reg_data |= MTL_TBS_LEOV; Expiry time disabled for testing
	if ((readl(priv->ioaddr + MTL_TBS_CTRL)) & MTL_TBS_ESTM)
		reg_data |=  (EST_LEOS << 8);/* Launch expiry offset (~124us) in units of 256ns, Launch Expiry Offset Valid bit */
	else
		reg_data |=  (ABSOLUTE_LEOS << 8);/* Launch expiry offset (~50ms) in units of 256ns, Launch Expiry Offset Valid bit */
	
	writel(reg_data, priv->ioaddr + MTL_TBS_CTRL);
}
/**
 * tc9562_ptp_configuration - Configure PTP
 * @priv: driver private structure
 * Description: It is used for configuring the PTP
 * Return value:
 * 0 on success or negative error number.
 */
void tc9562_ptp_configuration(struct tc9562mac_priv *priv)
{
	struct timespec64 now;	
	u32 control, sec_inc;
	u64 temp;

	ktime_get_real_ts64(&now);

	control = PTP_TCR_TSENA | PTP_TCR_TSENALL | PTP_TCR_TSCTRLSSR | PTP_TCR_TSCFUPDT;
	control |= PTP_TCR_PTGE;
	control |= 0x10013e03;
	priv->hw->ptp->config_hw_tstamping(priv->ptpaddr, control);

	/* program Sub Second Increment reg */
	sec_inc = priv->hw->ptp->config_sub_second_increment(
		priv->ptpaddr, priv->plat->clk_ptp_rate,
		priv->plat->has_gmac4);
	temp = div_u64(1000000000ULL, sec_inc);

	/* calculate default added value:
	 * formula is :
	 * addend = (2^32)/freq_div_ratio;
	 * where, freq_div_ratio = 1e9ns/sec_inc
	 */
	temp = (u64)(temp << 32);
#ifdef PTP_CHANGE
	priv->default_addend = div_u64(temp, TC9562_PTP_SYSCLOCK);
#else
	priv->default_addend = div_u64(temp, priv->plat->clk_ptp_rate);
#endif
	priv->hw->ptp->config_addend(priv->ptpaddr, priv->default_addend);
	priv->hw->ptp->init_systime(priv->ptpaddr, (u32)now.tv_sec, now.tv_nsec);
	control |= PTP_TCR_TSINIT;
	priv->hw->ptp->config_hw_tstamping(priv->ptpaddr, control);

}

/**
 * tc9562mac_est_configuration - Configure EST
 * @priv: driver private structure
 * Description: It is used for configuring the EST
 * Return value:
 * 0 on success or negative error number.
 */
static int tc9562mac_est_configuration(struct tc9562mac_priv *priv)
{
	u32 control;
	int ret = -EINVAL;
	uint value = 0;
	DBGPR_FUNC("-->tc9562mac_est_configuration\n");
#ifndef UNIFIED_DRIVER
	value = readl(priv->ptpaddr + PTP_TCR);
	if(!(value & 0x00000001))
	{
		tc9562_ptp_configuration(priv);
	}
#endif
    priv->hwts_tx_en = 1;
    priv->hwts_rx_en = 1;

    /* Disable EST */
    control = 0;
    control = readl(priv->ioaddr + MTL_EST_CONTROL);
    control &= ~MTL_EST_EEST;
    writel(control, priv->ioaddr + MTL_EST_CONTROL);

	ret = 1; //Absolute/Normal mode always used

	if (ret) {
		priv->est_enabled = false;
		value = readl(priv->ioaddr + MTL_TBS_CTRL);
		writel(value & (~MTL_TBS_ESTM),
				   priv->ioaddr + MTL_TBS_CTRL);
		printk("-->>TBS Disabled\n\r");

	} else {
		priv->est_enabled = true;
		value = readl(priv->ioaddr + MTL_TBS_CTRL);
		writel(value | MTL_TBS_ESTM,
				   priv->ioaddr + MTL_TBS_CTRL);
		printk("-->>TBS Enabled\n\r");
	}
	DBGPR_FUNC("<--tc9562mac_est_configuration\n");

	return ret;
}

static int tc9562mac_est_configuration_ioctl(struct tc9562mac_priv *priv)
{
    int ret = -EINVAL;
    uint value = 0;
    DBGPR_FUNC("-->tc9562mac_est_configuration\n");

    if (!(priv->dma_cap.time_stamp || priv->adv_ts)) {
    	netdev_alert(priv->dev, "No HW time stamping: Disabling EST\n");
    	return -EINVAL;
    }


     if (priv->hw->mac->est_init)
    	ret = priv->hw->mac->est_init(priv->dev, priv->hw,
    			&priv->plat->est_cfg, priv->dma_cap.estsel,
    			priv->dma_cap.estdep, priv->dma_cap.estwid);

    ret = 1; //ESTM disabled to test EST without launch time

    if (ret) {
    	priv->est_enabled = false;
    	value = readl(priv->ioaddr + MTL_TBS_CTRL);
    	writel(value & (~MTL_TBS_ESTM),
    			   priv->ioaddr + MTL_TBS_CTRL);
    	printk("-->>TBS Disabled\n\r");

    } else {
    	priv->est_enabled = true;
    	value = readl(priv->ioaddr + MTL_TBS_CTRL);
    	writel(value | MTL_TBS_ESTM,
    			   priv->ioaddr + MTL_TBS_CTRL);
    	printk("-->>TBS Enabled\n\r");
    }
    DBGPR_FUNC("<--tc9562mac_est_configuration\n");

    return ret;
}


/**
 * tc9562mac_safety_feat_configuration - Configure Safety Features
 * @priv: driver private structure
 * Description: It is used for configuring the safety features
 */
static void tc9562mac_safety_feat_configuration(struct tc9562mac_priv *priv)
{
	int ret = -EINVAL;

	DBGPR_FUNC("-->tc9562mac_safety_feat_configuration\n");

	if (priv->hw->mac->safety_feat_init)
		ret = priv->hw->mac->safety_feat_init(priv->dev, priv->hw,
				priv->dma_cap.asp);

	if (ret) {
		priv->ecc_enabled = false;
	} else {
		priv->ecc_enabled = true;
	}

	priv->ecc_err_inject = false;
	priv->ecc_err_where = 0;
	priv->ecc_err_correctable = false;

	DBGPR_FUNC("<--tc9562mac_safety_feat_configuration\n");
}

/**
 * tc9562mac_rx_parser_configuration - Configure RX Parser
 * @priv: driver private structure
 * Description: It is used for configuring the RX Parser
 * Return value:
 * 0 on success or negative error number.
 */
static int tc9562mac_rx_parser_configuration(struct tc9562mac_priv *priv)
{
	int ret = -EINVAL;

	DBGPR_FUNC("-->tc9562mac_rx_parser_configuration\n");

	if (priv->hw->mac->rx_parser_init && priv->plat->rxp_cfg.enable)
		ret = priv->hw->mac->rx_parser_init(priv->dev, priv->hw,
				priv->dma_cap.spram, priv->dma_cap.frpsel,
				priv->dma_cap.frpes, &priv->plat->rxp_cfg);

	if (ret) {
		DBGPR_TEST("FRP disabled \n");
		priv->rxp_enabled = false;
	} else {
		DBGPR_TEST("FRP enabled \n");
		priv->rxp_enabled = true;
	}

	DBGPR_FUNC("<--tc9562mac_rx_parser_configuration\n");

	return ret;
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
/**
 * tc9562mac_mcgr_configuration - Configure MCGR
 * @priv: driver private structure
 * Description: It is used for configuring the MCGR
 * Return value:
 * 0 on success or negative error number.
 */
static int tc9562mac_mcgr_configuration(struct tc9562mac_priv *priv)
{
	int ret = -EINVAL;

	DBGPR_FUNC("-->tc9562mac_mcgr_configuration\n");

	if (priv->hw->mac->mcgr_init)
		ret = priv->hw->mac->mcgr_init(priv->dev, priv->hw,
				priv->plat->mcgr_cfg, priv->dma_cap.ppsoutnum);

	DBGPR_FUNC("<--tc9562mac_mcgr_configuration\n");
	return ret;
}

/**
 * tc9562mac_pps_configuration - Configure PPS
 * @priv: driver private structure
 * @index: PPS output index
 * Description: It is used for configuring the PPS
 * Return value:
 * 0 on success or negative error number.
 */
static int tc9562mac_pps_configuration(struct tc9562mac_priv *priv, int index)
{
	struct tc9562mac_pps_cfg *cfg;
	int ret = -EINVAL;

	DBGPR_FUNC("-->tc9562mac_pps_configuration\n");

	if (index >= priv->dma_cap.ppsoutnum)
		return -EINVAL;

	cfg = &priv->plat->pps_cfg[index];
	if (priv->hw->mac->pps_init)
		ret = priv->hw->mac->pps_init(priv->dev, priv->hw, index, cfg);

	DBGPR_FUNC("<--tc9562mac_pps_configuration\n");
	
	return ret;
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

/**
 * tc9562mac_hw_setup - setup mac in a usable state.
 *  @dev : pointer to the device structure.
 *  Description:
 *  this is the main function to setup the HW in a usable state because the
 *  dma engine is reset, the core registers are configured (e.g. AXI,
 *  Checksum features, timers). The DMA is ready to start receiving and
 *  transmitting.
 *  Return value:
 *  0 on success and an appropriate (-)ve integer as defined in errno.h
 *  file on failure.
 */
static int tc9562mac_hw_setup(struct net_device *dev, bool init_ptp)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	u32 rx_cnt = priv->plat->rx_queues_to_use + 1;
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	u32 chan, value = 0;
	int ret;

#ifdef UNIFIED_DRIVER
	NMSGPR_ALERT("Unified Driver Loading..\n");
#endif
	DBGPR_FUNC("-->tc9562mac_hw_setup\n");
	/* DMA initialization and SW reset */
	ret = tc9562mac_init_dma_engine(priv);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: DMA engine initialization failed\n",
			   __func__);
		return ret;
	}
#ifdef TC9562_PKT_DUP
	/* Enable Packet Duplication PDC= 1,  */
	value = readl(priv->ioaddr + GMAC_EXT_CONFIG);
	value |= GMAC_EXT_CONFIG_PDC;
	writel(value, priv->ioaddr + GMAC_EXT_CONFIG);
	NDBGPR_L1("MAC Ext Configuration = %x\n", readl(priv->ioaddr + GMAC_EXT_CONFIG));
#endif
	/* Enable Perfect Address Filter HPF= 1, Pass all multicast PM =1 */
	value = readl(priv->ioaddr + GMAC_PACKET_FILTER);
	value = (value & (GMAC_MPFR_RES_Wr_Mask_22))|((( 0) & (GMAC_MPFR_Mask_22))<<22);
	value = (value & (GMAC_MPFR_RES_Wr_Mask_17))|((( 0) & (GMAC_MPFR_Mask_17))<<17);
	value = (value & (GMAC_MPFR_RES_Wr_Mask_11))|((( 0) & (GMAC_MPFR_Mask_11))<<11);
	value = ((value & GMAC_MPFR_HPF_Wr_Mask) | ((0x1 & GMAC_MPFR_HPF_Mask)<<10));
	value |= GMAC_PACKET_FILTER_PM;
	writel(value, priv->ioaddr + GMAC_PACKET_FILTER);
	NDBGPR_L1("MAC Packet Filter = %x\n", readl(priv->ioaddr + GMAC_PACKET_FILTER));

 /* Copy the MAC addr into the HW  */
 #ifdef TC9562_PKT_DUP
    /*If PDC bit is set (i.e packet duplication is on), one hot representation*/
	priv->hw->mac->set_umac_addr(priv->hw, dev->dev_addr, 2, 1); /* PCIe Host MAC Address, Channel 0*/
	priv->hw->mac->set_umac_addr(priv->hw, dev_cm3_addr, 3, 2); /*CM3 MAC Address, Channel 1 */

#else
#ifdef UNIFIED_DRIVER
    /*If PDC bit is not set (i.e packet duplication is off), binary representation*/
	priv->hw->mac->set_umac_addr(priv->hw, dev->dev_addr, HOST_MAC_ADDR_OFFSET, 0); //2/* PCIe Host MAC Address, Channel 0*/
#else
	priv->hw->mac->set_umac_addr(priv->hw, dev->dev_addr, 2, 0); /* PCIe Host MAC Address, Channel 0*/
	priv->hw->mac->set_umac_addr(priv->hw, dev_cm3_addr, 3, 1); /*CM3 MAC Address, Channel 1 */
#endif
#endif
	/* PS and related bits will be programmed according to the speed */
	if (priv->hw->pcs) {
		int speed = priv->plat->mac_port_sel_speed;

		if ((speed == SPEED_10) || (speed == SPEED_100) ||
		    (speed == SPEED_1000)) {
			priv->hw->ps = speed;
		} else {
			dev_warn(priv->device, "invalid port speed\n");
			priv->hw->ps = 0;
		}
	}

	/* Initialize the MAC Core */
	priv->hw->mac->core_init(dev, priv->hw, dev->mtu);

	/* Initialize MTL*/
	if (priv->synopsys_id >= DWMAC_CORE_4_00)
		tc9562mac_mtl_configuration(priv);

	/* Initialize Safety Features and RX Parser */
	if (priv->synopsys_id >= DWMAC_CORE_5_10) {
		tc9562mac_safety_feat_configuration(priv);
		ret = tc9562mac_rx_parser_configuration(priv);
		NDBGPR_L2("\n RX Parser Configuration failed");
	}

	ret = priv->hw->mac->rx_ipc(priv->hw);
	if (!ret) {
		netdev_warn(priv->dev, "RX IPC Checksum Offload disabled\n");
		priv->plat->rx_coe = TC9562MAC_RX_COE_NONE;
		priv->hw->rx_csum = 0;
	}
#ifndef UNIFIED_DRIVER
	/* Enable the MAC Rx/Tx */
	priv->hw->mac->set_mac(priv->ioaddr, true);
#else
	priv->hw->mac->set_mac(priv->ioaddr, false);
#endif
	/* Set the HW DMA mode and the COE */
	tc9562mac_dma_operation_mode(priv);

	tc9562mac_mmc_setup(priv);

	if (init_ptp) {
		ret = clk_prepare_enable(priv->plat->clk_ptp_ref);
		if (ret < 0)
			netdev_warn(priv->dev, "failed to enable PTP reference clock: %d\n", ret);

		ret = tc9562mac_init_ptp(priv);
		if (ret == -EOPNOTSUPP)
			netdev_warn(priv->dev, "PTP not supported by HW\n");
		else if (ret)
			netdev_warn(priv->dev, "PTP init failed\n");
	}
#ifndef UNIFIED_DRIVER
	value = readl(priv->ptpaddr + PTP_TCR);
	if(!(value & 0x00000001))
	{
		tc9562_ptp_configuration(priv);
	}
#endif
	/* Initialize EST */
	if (priv->synopsys_id >= DWMAC_CORE_5_00){
		ret = tc9562mac_est_configuration(priv);
		if(ret)
			NDBGPR_L2("EST Configuration failed\n");
	}
#ifdef CONFIG_DEBUG_FS
	ret = tc9562mac_init_fs(dev);
	if (ret < 0)
		netdev_warn(priv->dev, "%s: failed debugFS registration\n",
			    __func__);
#endif
	/* Start the ball rolling... */
	tc9562mac_start_all_dma(priv);

	priv->tx_lpi_timer = TC9562MAC_DEFAULT_TWT_LS;

	if ((priv->use_riwt) && (priv->hw->dma->rx_watchdog)) {
		priv->rx_riwt = MAX_DMA_RIWT;
		priv->hw->dma->rx_watchdog(priv->ioaddr, MAX_DMA_RIWT, rx_cnt);
	}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	if (priv->hw->pcs && priv->hw->mac->pcs_ctrl_ane)
		priv->hw->mac->pcs_ctrl_ane(priv->hw, 1, priv->hw->ps, 0);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

	/* set TX and RX rings length */
	tc9562mac_set_rings_length(priv);

	/* Enable TSO */
	if (priv->tso) {
		for (chan = 0; chan < tx_cnt; chan++)
			if(chan == 0)
				priv->hw->dma->enable_tso(priv->ioaddr, 1, chan);
	}

	DBGPR_FUNC("<--tc9562mac_hw_setup\n");

	return 0;
}

static void tc9562mac_hw_teardown(struct net_device *dev)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);

	DBGPR_FUNC("-->tc9562mac_hw_teardown\n");

	clk_disable_unprepare(priv->plat->clk_ptp_ref);
	DBGPR_FUNC("<--tc9562mac_hw_teardown\n");
}

/**
 *  tc9562mac_set_rx_mode - entry point for multicast addressing
 *  @dev : pointer to the device structure
 *  Description:
 *  This function is a driver entry point which gets called by the kernel
 *  whenever multicast addresses must be enabled/disabled.
 *  Return value:
 *  void.
 */
static void tc9562mac_set_rx_mode(struct net_device *dev)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);

	DBGPR_FUNC("---->tc9562mac_set_rx_mode\n");
	priv->hw->mac->set_filter(priv->hw, dev,priv->l2_filtering_mode);
	DBGPR_TEST("<----tc9562mac_set_rx_mode\n");
}

#ifdef TC9562_DUMP_MAC_REG
/*!
 * \details This debug function can be used to dump the MAC Core 
 * register space. It is mainly used during development phase for debug
 * purpose.  Use of these functions may affect the performance during normal
 * operation.
 *
 * \param[in] pdata:  pointer to private data structure.
 *
 * \return void
 */
void tc9562_dump_emac_registers(struct tc9562mac_priv *priv)
{
	long unsigned int reg_val;
	unsigned int reg_addr = 0;	

	reg_addr = 0;
  	printk("**********************************\n");
  	printk("**********************************\n");
  	printk("*********** EMAC    CORE *********\n");
	do {
	   reg_val = readl(priv->ioaddr + MAC_OFFSET + reg_addr);
  	   printk("EMAC Core addr: 0x4000 %x   val: %lx\n",
	                      (MAC_OFFSET + reg_addr), reg_val);
	   reg_addr += 4;
	} while(reg_addr < 0x404);
	
	reg_addr = 0x700;
	do {
	   reg_val = readl(priv->ioaddr + MAC_OFFSET + reg_addr);
  	   printk("EMAC Core addr: 0x4000 %x   val: %lx\n",
	                      (MAC_OFFSET + reg_addr), reg_val);
	   reg_addr += 4;
	} while(reg_addr < 0x14E8);
}
#endif

/**
 *  tc9562mac_open - open entry point of the driver
 *  @dev : pointer to the device structure.
 *  Description:
 *  This function is the open entry point of the driver.
 *  Return value:
 *  0 on success and an appropriate (-)ve integer as defined in errno.h
 *  file on failure.
 */
static int tc9562mac_open(struct net_device *dev)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	int ret, rd_val;
	DBGPR_FUNC("--> tc9562mac_open \n");

	tc9562mac_check_ether_addr(priv);

	 if (priv->hw->pcs != TC9562MAC_PCS_RGMII &&
	    priv->hw->pcs != TC9562MAC_PCS_TBI &&
	    priv->hw->pcs != TC9562MAC_PCS_RTBI) {
		ret = tc9562mac_init_phy(dev);
		if (ret) {
			netdev_err(priv->dev,
				   "%s: Cannot attach to PHY (error: %d)\n",
				   __func__, ret);
			return ret;
		}
	}

    /* To put the phydev in PHY_HALTED state */
	if (dev->phydev) {
		phy_stop(dev->phydev);
	}

	/* Extra statistics */
	memset(&priv->xstats, 0, sizeof(struct tc9562mac_extra_stats));
	priv->xstats.threshold = tc;

    if ((dev->features & NETIF_F_IP_CSUM) || (dev->features & NETIF_F_IPV6_CSUM))
		priv->csum_insertion = 1;
	else
		priv->csum_insertion = 0;

	priv->dma_buf_sz = TC9562MAC_ALIGN(buf_sz);
	priv->rx_copybreak = TC9562MAC_RX_COPYBREAK;
	priv->mss = 0;

	dev->phydev->autoneg = AUTONEG_DISABLE;
	phy_start_aneg(dev->phydev);
	mdelay(1);
	
	ret = alloc_dma_desc_resources(priv);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: DMA descriptors allocation failed\n",
			   __func__);
		goto dma_desc_error;
	}

	ret = init_dma_desc_rings(dev, GFP_KERNEL);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: DMA descriptors initialization failed\n",
			   __func__);
		goto init_error;
	}

	ret = tc9562mac_hw_setup(dev, true);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: Hw setup failed\n", __func__);
		goto init_error;
	}
	
	dev->phydev->autoneg = AUTONEG_ENABLE;
	phy_start_aneg(dev->phydev);
	
        /* Enable interrupt  */
#ifdef TC9562_MSIGEN_VERIFICATION
#ifdef TC9562_MSI_GEN_SW_AGENT
        /* In case of SW MSI interrupt, eMAC to MCU interrupts are enabled 
              * FW will be modified to generate SW MSI interrupt (for testing purpose)
              */
        rd_val = readl(priv->ioaddr + INTMCUMASK1);
        rd_val &= 0xc0000000;
        rd_val |= 0xFF0004FF;//RX DMA interrupts for channel 5,6,7 not enabled.
        writel(rd_val, priv->ioaddr + INTMCUMASK1);

        rd_val = readl(priv->ioaddr + INTMCUMASK2);
        rd_val = 0xFFC17A00;
        writel(rd_val , priv->ioaddr + INTMCUMASK2);

#else
        /* mask all eMAC interrupts for MCU */
        rd_val = readl(priv->ioaddr + INTMCUMASK1);
        rd_val |= 0x00ffff00;
        writel(rd_val, priv->ioaddr + INTMCUMASK1);

#endif    
        /* Enable MSIGEN */
        rd_val = readl(priv->ioaddr + NCLKCTRL_OFFSET);
        rd_val |= 0x00040000; // MSIGENCEN=1
        writel(rd_val, priv->ioaddr + NCLKCTRL_OFFSET);
        rd_val = readl(priv->ioaddr + NRSTCTRL_OFFSET);
        rd_val &= ~0x00040000; // MSIGENSRST=0
        writel(rd_val, priv->ioaddr + NRSTCTRL_OFFSET);
    
        /* Initialize MSIGEN */
        writel(0x00000000, priv->ioaddr + 0xf000); // MSI_OUT_EN: Disable all first
        writel(0xfffffffe, priv->ioaddr + 0xf008); // MSI_MASK_SET: mask all vectors other than vector 0
        writel(0x00000001, priv->ioaddr + 0xf00c); // MSI_MASK_CLR: unmask vector 0
        writel(0x00000000, priv->ioaddr + 0xf020); // MSI_VECT_SET0: All INTs mapped to vector 0
        writel(0x00000000, priv->ioaddr + 0xf024); // MSI_VECT_SET1: All INTs mapped to vector 0
        writel(0x00000000, priv->ioaddr + 0xf028); // MSI_VECT_SET2: All INTs mapped to vector 0
        writel(0x00000000, priv->ioaddr + 0xf02c); // MSI_VECT_SET3: All INTs mapped to vector 0
        writel(0x00000000, priv->ioaddr + 0xf030); // MSI_VECT_SET4: All INTs mapped to vector 0
        writel(0x00000000, priv->ioaddr + 0xf034); // MSI_VECT_SET5: All INTs mapped to vector 0
        writel(0x00000000, priv->ioaddr + 0xf038); // MSI_VECT_SET6: All INTs mapped to vector 0
        writel(0x00000000, priv->ioaddr + 0xf03c); // MSI_VECT_SET7: All INTs mapped to vector 0

#ifdef TC9562_MSI_GEN_SW_AGENT
        writel(0x1000000, priv->ioaddr + 0xf000); // MSI_OUT_EN: Enable only SW int bit[24]       

#else
#ifndef UNIFIED_DRIVER
        writel(0x0001f9ff, priv->ioaddr + 0xf000); // MSI_OUT_EN: Enable All mac int
#else
		/* Enable only Host related channels - LPI, PMT, Tx - 0,4,5, Rx - 0,4*/
		writel(0x10088CB, priv->ioaddr + 0xf000); /*Also Enable SW-MSI*/
#endif

#endif

#else /*TC9562_MSIGEN_VERIFICATION*/

	/* Enable interrupt  */
	rd_val = readl(priv->ioaddr + INTMCUMASK1);
	rd_val &= 0xc0000000;
#ifndef UNIFIED_DRIVER
	rd_val |= 0xFF0000FF;//RX DMA interrupts for channel 5,6,7 not enabled.
#else
	rd_val |= 0xFE8ECCFF; /* LPI, PMT, Tx - 0,4,5, Rx - 0,4 are enabled */
#endif
	writel(rd_val, priv->ioaddr + INTMCUMASK1);

	rd_val = readl(priv->ioaddr + INTMCUMASK2);
	rd_val = 0xFFC17A00;
	writel(rd_val , priv->ioaddr + INTMCUMASK2);
#endif
	Launch_time_parameters_init(priv);
	tc9562mac_init_tx_coalesce(priv);

	if (dev->phydev)
		phy_start(dev->phydev);

	/* Request the IRQ lines */
#ifdef TC9562_POLLING_METHOD
	priv->irq_number = dev->irq ;
	ret = request_irq(dev->irq, tc9562mac_interrupt_dummy,
				  IRQF_SHARED, dev->name, dev);
#else
	ret = request_irq(dev->irq, tc9562mac_interrupt,
			  IRQF_SHARED, dev->name, dev);
#endif
	if (unlikely(ret < 0)) {
		netdev_err(priv->dev,
			   "%s: ERROR: allocating the IRQ %d (error: %d)\n",
			   __func__, dev->irq, ret);
		goto irq_error;
	}
#ifndef TC9562_DEFINED
	/* Request the Wake IRQ in case of another line is used for WoL */
	if (priv->wol_irq != dev->irq) {
		ret = request_irq(priv->wol_irq, tc9562mac_interrupt,
				  IRQF_SHARED, dev->name, dev);
		if (unlikely(ret < 0)) {
			netdev_err(priv->dev,
				   "%s: ERROR: allocating the WoL IRQ %d (%d)\n",
				   __func__, priv->wol_irq, ret);
			goto wolirq_error;
		}
	}

	/* Request the IRQ lines */
	if (priv->lpi_irq > 0) {
		ret = request_irq(priv->lpi_irq, tc9562mac_interrupt, IRQF_SHARED,
				  dev->name, dev);
		if (unlikely(ret < 0)) {
			netdev_err(priv->dev,
				   "%s: ERROR: allocating the LPI IRQ %d (%d)\n",
				   __func__, priv->lpi_irq, ret);
			goto lpiirq_error;
		}
	}
#endif

	tc9562mac_enable_all_queues(priv);
	tc9562mac_start_all_queues(priv);
	tc9562mac_set_rx_mode(dev);
#ifdef TC9562_POLLING_METHOD
		polling_task_data = (void *)dev;
		INIT_DELAYED_WORK(&task, (void *)polling_task);
		NMSGPR_INFO("Jiffies : %ld\n", usecs_to_jiffies(TC9562_POLL_DELAY_US));
		schedule_delayed_work(&task, usecs_to_jiffies(TC9562_POLL_DELAY_US));
#endif //TC9562_POLLING_METHOD

#ifdef TC9562_DUMP_MAC_REG
    tc9562_dump_emac_registers(priv);
#endif

#ifdef UNIFIED_DRIVER
	/* Raise MCU Flag so CM3 FW can start operation*/
	writel(1 << 12, priv->ioaddr + 0x8054);

#endif
	DBGPR_FUNC("<-- tc9562mac_open \n");
	return 0;

#ifndef TC9562_DEFINED
lpiirq_error:
	if (priv->wol_irq != dev->irq)
		free_irq(priv->wol_irq, dev);
wolirq_error:
	free_irq(dev->irq, dev);
#endif
irq_error:
	if (dev->phydev)
		phy_stop(dev->phydev);

	del_timer_sync(&priv->txtimer);
	tc9562mac_hw_teardown(dev);
init_error:
	free_dma_desc_resources(priv);
dma_desc_error:
	if (dev->phydev)
		phy_disconnect(dev->phydev);

	return ret;
}

/**
 *  tc9562mac_release - close entry point of the driver
 *  @dev : device pointer.
 *  Description:
 *  This is the stop entry point of the driver.
 */
static int tc9562mac_release(struct net_device *dev)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
#ifdef UNIFIED_DRIVER	
	int rd_val = 0, reg;
    int reset_default_value = 0x02F700B0, clock_default_value = 0x00003A05;
#endif
	DBGPR_FUNC("-->tc9562mac_release\n");

	if (priv->eee_enabled)
		del_timer_sync(&priv->eee_ctrl_timer);

	/* Stop and disconnect the PHY */
	if (dev->phydev) {
		phy_stop(dev->phydev);
        priv->oldlink = false;
	    priv->speed = SPEED_UNKNOWN;
     	priv->oldduplex = DUPLEX_UNKNOWN;
		phy_disconnect(dev->phydev);
	}

	tc9562mac_stop_all_queues(priv);

	tc9562mac_disable_all_queues(priv);

	del_timer_sync(&priv->txtimer);

	/* Free the IRQ lines */
	free_irq(dev->irq, dev);
	if (priv->wol_irq != dev->irq)
		free_irq(priv->wol_irq, dev);
	if (priv->lpi_irq != dev->irq)
		free_irq(priv->lpi_irq, dev);

	/* Stop TX/RX DMA and clear the descriptors */
	tc9562mac_stop_all_dma(priv);

	/* Release and free the Rx/Tx resources */
	free_dma_desc_resources(priv);

	/* Disable the MAC Rx/Tx */
	priv->hw->mac->set_mac(priv->ioaddr, false);

	netif_carrier_off(dev);

#ifdef CONFIG_DEBUG_FS
	tc9562mac_exit_fs(dev);
#endif

	tc9562mac_release_ptp(priv);
	
#ifdef UNIFIED_DRIVER
    
#ifdef UNIFIED_DRIVER
/* Stopping TDM before removing Driver */
    if (priv->tdm_start == 1)       
    {
        void __user *data = NULL;
        tc9562_ioctl_tdm_stop(priv, data);    
    }
#endif
	
    {
        /* In FLASH Mode the FW is loaded only during boot. CM3 Reset Required in Flash Mode to Sync Firmware and Host Driver States.*/
        {
            /* CM3 System Reset for Managing States in Flash Mode*/	

            rd_val = readl(priv->ioaddr + 0xa000);      
            rd_val &= ~0x00000003;
            writel(rd_val, priv->ioaddr + 0xa000);      /*Clear Gloabal EMAC TX RX */
	        writel( clock_default_value, priv->ioaddr + 0x1004); 
            rd_val = readl(priv->ioaddr + 0x1004);      

            rd_val = readl(priv->ioaddr + 0x1008);
            rd_val |= (reset_default_value | 0x00000001); /*Assert */
            writel( rd_val, priv->ioaddr + 0x1008); 

#ifdef DEBUG_UNIFIED
            rd_val = readl(priv->ioaddr + 0x1004);
            NMSGPR_ALERT("1004 : %x\n",rd_val);	
            rd_val = readl(priv->ioaddr + 0x1008);
            NMSGPR_ALERT("1008 : %x\n",rd_val);	
#endif
            rd_val = readl(priv->ioaddr + 0x1008);
            
            rd_val = readl(priv->ioaddr + 0x1008);
            rd_val &= (~0x00000001); /*De-assert*/	
            writel( rd_val, priv->ioaddr + 0x1008); 
	
            DBGPR_FUNC("CM3 System Reset Complete\n");
#ifdef DEBUG_UNIFIED
            rd_val = readl(priv->ioaddr + 0x1004);
            NMSGPR_ALERT("1004 : %x\n",rd_val);
	
            rd_val = readl(priv->ioaddr + 0x1008);
            NMSGPR_ALERT("1008 : %x\n",rd_val);
#endif
	    }
	}
	
	/* The CM3 Firmware will be loaded during ndo_close when Host Initiated Boot Mode is selected.*/
	
	/* This is added to support ifconfig up/down scenario for UNIFIED_DRIVER.
       During ndo_open DMA_Mode SWR Bit is set which resets EMAC, MTL, DMA. 
       To Re-Initialize FW part of EMAC the following Code is added*/
    
    reg = readl(priv->ioaddr + NMODESTS_OFFSET);
    NMSGPR_ALERT("0x%x\n", reg);
    if((reg & TC9562_NMODESTS_HOST_BOOT_MASK) == TC9562_NMODESTS_HOST_BOOT_MASK) 
    {
        int ret;
        struct tc9562mac_resources res;
    
        res.addr = priv->ioaddr;
        res.tc9562_SRAM_pci_base_addr = priv->tc9562_SRAM_pci_base_addr;
	    res.tc9562_FLASH_pci_base_addr = priv->tc9562_FLASH_pci_base_addr;
	    res.irq = priv->dev->irq ;
	    
        ret = tc9562_load_firmware(priv->device, &res);
        if(ret < 0) 
        {
            NMSGPR_ERR("Firmware load failed\n");
        }
    }
	
#endif


#ifdef TC9562_POLLING_METHOD
	cancel_delayed_work_sync(&task);
#endif //TC9562_POLLING_METHOD
	DBGPR_FUNC("<--tc9562mac_release\n");
	return 0;
}

/**
 *  tc9562mac_tso_allocator - close entry point of the driver
 *  @priv: driver private structure
 *  @des: buffer start address
 *  @total_len: total length to fill in descriptors
 *  @last_segmant: condition for the last descriptor
 *  @queue: TX queue index
 *  Description:
 *  This function fills descriptor and request new descriptors according to
 *  buffer length to fill
 */
static void tc9562mac_tso_allocator(struct tc9562mac_priv *priv, u64 des,
				 int total_len, bool last_segment, u32 queue)
{
	struct tc9562mac_tx_queue *tx_q = &priv->tx_queue[queue];
	struct dma_desc *desc;
	u32 buff_size;
	int tmp_len;

	DBGPR_FUNC("-->tc9562mac_tso_allocator\n");

	tmp_len = total_len;

	while (tmp_len > 0) {
		tx_q->cur_tx = TC9562MAC_GET_ENTRY(tx_q->cur_tx, DMA_TX_SIZE);
		desc = tx_q->dma_tx + tx_q->cur_tx;

#ifdef TC9562_DEFINED
		desc->des0 = ((des + (total_len - tmp_len)) & 0xFFFFFFFF);
		desc->des1 = HOST_PHYSICAL_ADRS_MASK | (((des + (total_len - tmp_len)) >> 32) & 0xF);
#else
		desc->des0 = cpu_to_le32(des + (total_len - tmp_len));
#endif
		buff_size = tmp_len >= TSO_MAX_BUFF_SIZE ?
			    TSO_MAX_BUFF_SIZE : tmp_len;

		priv->hw->desc->prepare_tso_tx_desc(desc, 0, buff_size,
			0, 1,
			(last_segment) && (tmp_len <= TSO_MAX_BUFF_SIZE),
			0, 0);

		tmp_len -= TSO_MAX_BUFF_SIZE;
	}

	DBGPR_FUNC("<--tc9562mac_tso_allocator\n");
}

/**
 *  tc9562mac_tso_xmit - Tx entry point of the driver for oversized frames (TSO)
 *  @skb : the socket buffer
 *  @dev : device pointer
 *  Description: this is the transmit function that is called on TSO frames
 *  (support available on GMAC4 and newer chips).
 *  Diagram below show the ring programming in case of TSO frames:
 *
 *  First Descriptor
 *   --------
 *   | DES0 |---> buffer1 = L2/L3/L4 header
 *   | DES1 |---> TCP Payload (can continue on next descr...)
 *   | DES2 |---> buffer 1 and 2 len
 *   | DES3 |---> must set TSE, TCP hdr len-> [22:19]. TCP payload len [17:0]
 *   --------
 *	|
 *     ...
 *	|
 *   --------
 *   | DES0 | --| Split TCP Payload on Buffers 1 and 2
 *   | DES1 | --|
 *   | DES2 | --> buffer 1 and 2 len
 *   | DES3 |
 *   --------
 *
 * mss is fixed when enable tso, so w/o programming the TDES3 ctx field.
 */
static netdev_tx_t tc9562mac_tso_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct dma_desc *desc, *first, *mss_desc = NULL;
	struct tc9562mac_priv *priv = netdev_priv(dev);
	int nfrags = skb_shinfo(skb)->nr_frags;
	u32 queue = skb_get_queue_mapping(skb);
	unsigned int first_entry;
	u64 des;
	struct tc9562mac_tx_queue *tx_q;
	int tmp_pay_len = 0;
	u32 pay_len, mss;
	u8 proto_hdr_len;
	int i;

	DBGPR_FUNC("-->tc9562mac_tso_xmit\n");

	tx_q = &priv->tx_queue[queue];

	/* Compute header lengths */
	proto_hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);

	/* Desc availability based on threshold should be enough safe */
	if (unlikely(tc9562mac_tx_avail(priv, queue) <
		(((skb->len - proto_hdr_len) / TSO_MAX_BUFF_SIZE + 1)))) {
		if (!netif_tx_queue_stopped(netdev_get_tx_queue(dev, queue))) {
			netif_tx_stop_queue(netdev_get_tx_queue(priv->dev,
								queue));
			/* This is a hard error, log it. */
			netdev_err(priv->dev,
				   "%s: Tx Ring full when queue awake\n",
				   __func__);
		}
		return NETDEV_TX_BUSY;
	}

	pay_len = skb_headlen(skb) - proto_hdr_len; /* no frags */

	mss = skb_shinfo(skb)->gso_size;

	/* set new MSS value if needed */
	if (mss != priv->mss) {
		mss_desc = tx_q->dma_tx + tx_q->cur_tx;
		priv->hw->desc->set_mss(mss_desc, mss);
		priv->mss = mss;
		tx_q->cur_tx = TC9562MAC_GET_ENTRY(tx_q->cur_tx, DMA_TX_SIZE);
	}

	if (netif_msg_tx_queued(priv)) {
		pr_info("%s: tcphdrlen %d, hdr_len %d, pay_len %d, mss %d\n",
			__func__, tcp_hdrlen(skb), proto_hdr_len, pay_len, mss);
		pr_info("\tskb->len %d, skb->data_len %d\n", skb->len,
			skb->data_len);
	}

	first_entry = tx_q->cur_tx;

	desc = tx_q->dma_tx + first_entry;
	first = desc;

	/* first descriptor: fill Headers on Buf1 */
	des = dma_map_single(priv->device, skb->data, skb_headlen(skb),
			     DMA_TO_DEVICE);
	if (dma_mapping_error(priv->device, des))
		goto dma_map_err;

	tx_q->tx_skbuff_dma[first_entry].buf = des;
	tx_q->tx_skbuff_dma[first_entry].len = skb_headlen(skb);
	tx_q->tx_skbuff[first_entry] = skb;

#ifdef TC9562_DEFINED
	first->des0 = (des & 0xFFFFFFFF);
	first->des1 = HOST_PHYSICAL_ADRS_MASK | ((des >> 32) & 0xF);
	tmp_pay_len = pay_len;
#else
	first->des0 = cpu_to_le32(des);

	/* Fill start of payload in buff2 of first descriptor */
	if (pay_len)
		first->des1 = cpu_to_le32(des + proto_hdr_len);
	
	/* If needed take extra descriptors to fill the remaining payload */
	tmp_pay_len = pay_len - TSO_MAX_BUFF_SIZE;

#endif
	
	tc9562mac_tso_allocator(priv, (des + proto_hdr_len), tmp_pay_len, (nfrags == 0), queue);

	/* Prepare fragments */
	for (i = 0; i < nfrags; i++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

		des = skb_frag_dma_map(priv->device, frag, 0,
				       skb_frag_size(frag),
				       DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err;

		tc9562mac_tso_allocator(priv, des, skb_frag_size(frag),
				     (i == nfrags - 1), queue);

		tx_q->tx_skbuff_dma[tx_q->cur_tx].buf = des;
		tx_q->tx_skbuff_dma[tx_q->cur_tx].len = skb_frag_size(frag);
		tx_q->tx_skbuff[tx_q->cur_tx] = NULL;
		tx_q->tx_skbuff_dma[tx_q->cur_tx].map_as_page = true;
	}

	tx_q->tx_skbuff_dma[tx_q->cur_tx].last_segment = true;

	tx_q->cur_tx = TC9562MAC_GET_ENTRY(tx_q->cur_tx, DMA_TX_SIZE);

	if (unlikely(tc9562mac_tx_avail(priv, queue) <= (MAX_SKB_FRAGS + 1))) {
		netif_dbg(priv, hw, priv->dev, "%s: stop transmitted packets\n",
			  __func__);
		netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, queue));
	}

	dev->stats.tx_bytes += skb->len;
	priv->xstats.tx_tso_frames++;
	priv->xstats.tx_tso_nfrags += nfrags;

	/* Manage tx mitigation */
	priv->tx_count_frames += nfrags + 1;
	if (likely(priv->tx_coal_frames > priv->tx_count_frames)) {
		mod_timer(&priv->txtimer,
			  TC9562MAC_COAL_TIMER(priv->tx_coal_timer));
	} else {
		priv->tx_count_frames = 0;
		priv->hw->desc->set_tx_ic(desc);
		priv->xstats.tx_set_ic_bit++;
	}

	skb_tx_timestamp(skb);

	if (unlikely((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
		     priv->hwts_tx_en)) {
		/* declare that device is doing timestamping */
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		priv->hw->desc->enable_tx_timestamp(first);
	}

	/* Complete the first descriptor before granting the DMA */
	priv->hw->desc->prepare_tso_tx_desc(first, 1,
			proto_hdr_len,
			0 /*pay_len*/,
			1, tx_q->tx_skbuff_dma[first_entry].last_segment,
			tcp_hdrlen(skb) / 4, (skb->len - proto_hdr_len));

	/* If context desc is used to change MSS */
	if (mss_desc)
    {
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 18, 140))
		dma_wmb();
#endif
		priv->hw->desc->set_tx_owner(mss_desc);
    }

	/* The own bit must be the latest setting done when prepare the
	 * descriptor and then barrier is needed to make sure that
	 * all is coherent before granting the DMA engine.
	 */
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 18, 140))
    wmb();
#else
	dma_wmb();
#endif
	if (netif_msg_pktdata(priv)) {
		pr_info("%s: curr=%d dirty=%d f=%d, e=%d, f_p=%p, nfrags %d\n",
			__func__, tx_q->cur_tx, tx_q->dirty_tx, first_entry,
			tx_q->cur_tx, first, nfrags);

		priv->hw->desc->display_ring((void *)tx_q->dma_tx, DMA_TX_SIZE,
					     0);

		pr_info(">>> frame to be transmitted: ");
		print_pkt(skb->data, skb_headlen(skb));
	}

	netdev_tx_sent_queue(netdev_get_tx_queue(dev, queue), skb->len);

	priv->hw->dma->set_tx_tail_ptr(priv->ioaddr, tx_q->tx_tail_addr,
				       queue);

	DBGPR_FUNC("<--tc9562mac_tso_xmit\n");

	return NETDEV_TX_OK;

dma_map_err:
	dev_err(priv->device, "Tx dma map failed\n");
	dev_kfree_skb(skb);
	priv->dev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}


/**
 *  tc9562mac_uso_xmit - Tx entry point of the driver for oversized frames (USO)
 *  @skb : the socket buffer
 *  @dev : device pointer
 *  Description: this is the transmit function that is called on TSO frames
 *  (support available on GMAC4 and newer chips).
 *  Diagram below show the ring programming in case of USO frames:
 *
 *  First Descriptor
 *   --------
 *   | DES0 |---> buffer1 = L2/L3/L4 header
 *   | DES1 |---> UDP Payload (can continue on next descr...)
 *   | DES2 |---> buffer 1 and 2 len
 *   | DES3 |---> must set TSE, TCP hdr len-> [22:19]. UDP payload len [17:0]
 *   --------
 *	|
 *     ...
 *	|
 *   --------
 *   | DES0 | --| Split UDP Payload on Buffers 1 and 2
 *   | DES1 | --|
 *   | DES2 | --> buffer 1 and 2 len
 *   | DES3 |
 *   --------
 *
 * mss is fixed when enable tso, so w/o programming the TDES3 ctx field.
 */
static netdev_tx_t tc9562mac_uso_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct dma_desc *desc, *first, *mss_desc = NULL;
	struct tc9562mac_priv *priv = netdev_priv(dev);
	int nfrags = skb_shinfo(skb)->nr_frags;
	u32 queue = skb_get_queue_mapping(skb);
	unsigned int first_entry;
	u64 des;
	struct tc9562mac_tx_queue *tx_q;
	int tmp_pay_len = 0;
	u32 pay_len, mss;
	u8 proto_hdr_len;
	int i;

	DBGPR_TEST("-->tc9562mac_uso_xmit\n");

	DBGPR_FUNC("-->dwmac4_dma_init_tx_chan\n");

	tx_q = &priv->tx_queue[queue];

	/* Compute header lengths */
	proto_hdr_len = skb_transport_offset(skb) + /*tcp_hdrlen(skb)*/2*4; //header len 2 * 4 bytes in case of UDP header

	/* Desc availability based on threshold should be enough safe */
	if (unlikely(tc9562mac_tx_avail(priv, queue) <
		(((skb->len - proto_hdr_len) / TSO_MAX_BUFF_SIZE + 1)))) {
		if (!netif_tx_queue_stopped(netdev_get_tx_queue(dev, queue))) {
			netif_tx_stop_queue(netdev_get_tx_queue(priv->dev,
								queue));
			/* This is a hard error, log it. */
			netdev_err(priv->dev,
				   "%s: Tx Ring full when queue awake\n",
				   __func__);
		}
		return NETDEV_TX_BUSY;
	}

	pay_len = skb_headlen(skb) - proto_hdr_len; /* no frags */

	DBGPR_TEST("-->headlen: %d payhdr_len: %d\n", skb_headlen(skb), proto_hdr_len);

	//mss = skb_shinfo(skb)->gso_size;
	/*MSS is hardcoded as 'gso_size' will not be set in case of UFO/USO*/
	mss = UFO_USO_MSS_SIZE; 
	DBGPR_TEST("-->mss: %d pay_len: %d\n", mss, pay_len);

	/* set new MSS value if needed */
	if (mss != priv->mss) {
		mss_desc = tx_q->dma_tx + tx_q->cur_tx;
		priv->hw->desc->set_mss(mss_desc, mss);
		priv->mss = mss;
		tx_q->cur_tx = TC9562MAC_GET_ENTRY(tx_q->cur_tx, DMA_TX_SIZE);
	}

	if (netif_msg_tx_queued(priv)) {
		pr_info("%s: tcphdrlen %d, hdr_len %d, pay_len %d, mss %d\n",
			__func__, tcp_hdrlen(skb), proto_hdr_len, pay_len, mss);
		pr_info("\tskb->len %d, skb->data_len %d\n", skb->len,
			skb->data_len);
	}

	first_entry = tx_q->cur_tx;

	desc = tx_q->dma_tx + first_entry;
	first = desc;

	/* first descriptor: fill Headers on Buf1 */
	des = dma_map_single(priv->device, skb->data, skb_headlen(skb),
			     DMA_TO_DEVICE);
	if (dma_mapping_error(priv->device, des))
		goto dma_map_err;

	tx_q->tx_skbuff_dma[first_entry].buf = des;
	tx_q->tx_skbuff_dma[first_entry].len = skb_headlen(skb);
	tx_q->tx_skbuff[first_entry] = skb;

#ifdef TC9562_DEFINED
	first->des0 = (des & 0xFFFFFFFF);
	first->des1 = HOST_PHYSICAL_ADRS_MASK | ((des >> 32) & 0xF);
	tmp_pay_len = pay_len;
#else
	first->des0 = cpu_to_le32(des);

	/* Fill start of payload in buff2 of first descriptor */
	if (pay_len)
		first->des1 = cpu_to_le32(des + proto_hdr_len);
	
	/* If needed take extra descriptors to fill the remaining payload */
	tmp_pay_len = pay_len - TSO_MAX_BUFF_SIZE;

#endif
	
	tc9562mac_tso_allocator(priv, (des + proto_hdr_len), tmp_pay_len, (nfrags == 0), queue);

	/* Prepare fragments */
	DBGPR_TEST("-->nfrags: %d\n", nfrags);
	for (i = 0; i < nfrags; i++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

		des = skb_frag_dma_map(priv->device, frag, 0,
				       skb_frag_size(frag),
				       DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err;

		tc9562mac_tso_allocator(priv, des, skb_frag_size(frag),
				     (i == nfrags - 1), queue);

		tx_q->tx_skbuff_dma[tx_q->cur_tx].buf = des;
		tx_q->tx_skbuff_dma[tx_q->cur_tx].len = skb_frag_size(frag);
		tx_q->tx_skbuff[tx_q->cur_tx] = NULL;
		tx_q->tx_skbuff_dma[tx_q->cur_tx].map_as_page = true;
	}

	tx_q->tx_skbuff_dma[tx_q->cur_tx].last_segment = true;

	tx_q->cur_tx = TC9562MAC_GET_ENTRY(tx_q->cur_tx, DMA_TX_SIZE);

	if (unlikely(tc9562mac_tx_avail(priv, queue) <= (MAX_SKB_FRAGS + 1))) {
		netif_dbg(priv, hw, priv->dev, "%s: stop transmitted packets\n",
			  __func__);
		netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, queue));
	}

	dev->stats.tx_bytes += skb->len;
	priv->xstats.tx_tso_frames++;
	priv->xstats.tx_tso_nfrags += nfrags;

	/* Manage tx mitigation */
	priv->tx_count_frames += nfrags + 1;
	if (likely(priv->tx_coal_frames > priv->tx_count_frames)) {
		mod_timer(&priv->txtimer,
			  TC9562MAC_COAL_TIMER(priv->tx_coal_timer));
	} else {
		priv->tx_count_frames = 0;
		priv->hw->desc->set_tx_ic(desc);
		priv->xstats.tx_set_ic_bit++;
	}

	skb_tx_timestamp(skb);

	if (unlikely((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
		     priv->hwts_tx_en)) {
		/* declare that device is doing timestamping */
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		priv->hw->desc->enable_tx_timestamp(first);
	}

	/* Complete the first descriptor before granting the DMA */
	priv->hw->desc->prepare_tso_tx_desc(first, 1,
			proto_hdr_len,
			0 /*pay_len*/,
			1, tx_q->tx_skbuff_dma[first_entry].last_segment,
			/*tcp_hdrlen(skb) / 4*/2, (skb->len - proto_hdr_len));

	/* If context desc is used to change MSS */
	if (mss_desc)
		priv->hw->desc->set_tx_owner(mss_desc);

	/* The own bit must be the latest setting done when prepare the
	 * descriptor and then barrier is needed to make sure that
	 * all is coherent before granting the DMA engine.
	 */
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 18, 140))
    wmb();
#else
    dma_wmb();
#endif


	if (netif_msg_pktdata(priv)) {
		pr_info("%s: curr=%d dirty=%d f=%d, e=%d, f_p=%p, nfrags %d\n",
			__func__, tx_q->cur_tx, tx_q->dirty_tx, first_entry,
			tx_q->cur_tx, first, nfrags);

		priv->hw->desc->display_ring((void *)tx_q->dma_tx, DMA_TX_SIZE,
					     0);

		pr_info(">>> frame to be transmitted: ");
		print_pkt(skb->data, skb_headlen(skb));
	}

	netdev_tx_sent_queue(netdev_get_tx_queue(dev, queue), skb->len);

	priv->hw->dma->set_tx_tail_ptr(priv->ioaddr, tx_q->tx_tail_addr,
				       queue);

	DBGPR_TEST("<--tc9562mac_uso_xmit\n");

	return NETDEV_TX_OK;

dma_map_err:
	dev_err(priv->device, "Tx dma map failed\n");
	dev_kfree_skb(skb);
	priv->dev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

#define AVB_TX_PKT_DROP_DBG
static netdev_tx_t tc9562mac_enxmit(struct sk_buff *skb, struct net_device *dev)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	unsigned int nopaged_len = skb_headlen(skb);
	int i, csum_insertion = 0, is_jumbo = 0;
	unsigned int queue = skb_get_queue_mapping(skb);
	int nfrags = skb_shinfo(skb)->nr_frags;
	int entry, chan = 0;
	unsigned int GSN1 = 0, GSN = 0;
	unsigned int chInx = 0;
	unsigned int first_entry;
	struct dma_enhanced_desc *edesc, *efirst;
	struct tc9562mac_tx_queue *tx_q;
	struct tc9562mac_est_cfg *cfg = &priv->plat->est_cfg;
	unsigned int enh_desc;
	u64 des=0, real_btr=0, ctr=0;
	unsigned int app_launch_time = 0, launch_time =0, Presentation_time, reg_data = 0;
	unsigned int tx_cnt = priv->plat->tx_queues_to_use;
	int tv_flag =0;
	int traverse_time = 0;
#ifdef AVB_TX_PKT_DROP_DBG
	static int prev_sn = 0;
#endif

	tx_q = &priv->tx_queue[queue];
	chInx = skb_get_queue_mapping(skb);

  DBGPR_FUNC("-->tc9562mac_enxmit\n");

#ifdef AVB_TX_PKT_DROP_DBG
  if ( ( (prev_sn+1) & 0xFF) != skb->data[20])
  {
   	//printk(" *** prev->sn: %d -> %d ***\n", prev_sn, skb->data[20]);
  }
	prev_sn = skb->data[20];
#endif 

	/* Enable Enhanced Descriptors (EDSE) */
	
	for (chan = 3; chan < tx_cnt; chan++)
	priv->hw->dma->enable_edse(priv->ioaddr, 1, chan);

	/* Manage oversized TCP frames for GMAC4 device */
	if (skb_is_gso(skb) && priv->tso && (chInx == 0)) {
		if (skb_shinfo(skb)->gso_type & (SKB_GSO_TCPV4 | SKB_GSO_TCPV6))
			return tc9562mac_tso_xmit(skb, dev);
	}
	if (unlikely(tc9562mac_tx_avail(priv, queue) < nfrags + 1)) {
		if (!netif_tx_queue_stopped(netdev_get_tx_queue(dev, queue))) {
			netif_tx_stop_queue(netdev_get_tx_queue(priv->dev,
								queue));
            
			/* This is a hard error, log it. */
			netdev_err(priv->dev,
				   "%s: Tx Ring full when queue awake\n",
				   __func__);
		}
		return NETDEV_TX_BUSY;
	}
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	if (priv->tx_path_in_lpi_mode)
		tc9562mac_disable_eee_mode(priv);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */	

	entry = tx_q->cur_tx;
	first_entry = entry;
	csum_insertion = (skb->ip_summed == CHECKSUM_PARTIAL);
	csum_insertion = 1;
	edesc = (struct dma_enhanced_desc *)(tx_q->dma_entx + entry);
	efirst = edesc;

	tx_q->tx_skbuff[first_entry] = skb;

	enh_desc = priv->plat->enh_desc;
	for (i = 0; i < nfrags; i++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		int len = skb_frag_size(frag);
		bool last_segment = (i == (nfrags - 1));

		entry = TC9562MAC_GET_ENTRY(entry, DMA_TX_SIZE);
		edesc = (struct dma_enhanced_desc *)(tx_q->dma_entx + entry);


		des = skb_frag_dma_map(priv->device, frag, 0, len,
				       DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err; /* should reuse desc w/o issues */

		tx_q->tx_skbuff[entry] = NULL;

		tx_q->tx_skbuff_dma[entry].buf = des;
		if (unlikely(priv->synopsys_id >= DWMAC_CORE_4_00)){
#ifdef TC9562_DEFINED
			edesc->basic.des0 = (des & 0xFFFFFFFF);
			edesc->basic.des1 = HOST_PHYSICAL_ADRS_MASK | ((des >> 32) & 0xF);
#else
			edesc->basic.des0 = cpu_to_le32(des);
#endif
		}
		else
			edesc->basic.des2 = cpu_to_le32(des);

		tx_q->tx_skbuff_dma[entry].map_as_page = true;
		tx_q->tx_skbuff_dma[entry].len = len;
		tx_q->tx_skbuff_dma[entry].last_segment = last_segment;
	/* Extract the launch time from the packet if it is AVB packet */
		/* Enable Enhanced Descriptors */
		if( (chInx == AVB_CLASSA_CH) || (chInx == AVB_CLASSB_CH) || (chInx == CLASS_CDT))
		{
			Presentation_time = skb->data[30];
			Presentation_time = (Presentation_time<<8) | skb->data[31];
			Presentation_time = (Presentation_time<<8) | skb->data[32];
			Presentation_time = (Presentation_time<<8) | skb->data[33];

			app_launch_time = Presentation_time - 20000; /* Launch time = (Presentation time - 2ms ) */
		
			/* Read the value of GSN if ESTM bit is enabled */
			if (readl(priv->ioaddr + MTL_TBS_CTRL) & MTL_TBS_ESTM)
			{
				
				/* Read the Current GSN value from the register */
				GSN = readl(priv->ioaddr + MTL_EST_STATUS) & MTL_EST_CGSN;
				
				/* GSN is calculated for EST mode */
				dwmac5_est_read(priv->hw, MTL_EST_BTR_LOW, &cfg->btr_offset[0], false);
				dwmac5_est_read(priv->hw, MTL_EST_BTR_HIGH, &cfg->btr_offset[1], false);
				dwmac5_est_read(priv->hw, MTL_EST_CTR_LOW, &cfg->ctr[0], false);
				dwmac5_est_read(priv->hw, MTL_EST_CTR_HIGH, &cfg->ctr[1], false);
				real_btr = ((u64)cfg->btr_offset[1] << 32) | cfg->btr_offset[0];
				ctr = ((u64)cfg->ctr[1] << 32 ) | cfg->ctr[0];
				GSN = GSN + ((app_launch_time << 8)-real_btr) / ctr;
				
				if (15 == GSN)
					GSN = 0;
				
				/* Launch time and GSN is updated for EST mode */
				reg_data = readl(priv->ioaddr + DMA_TBS_CTRL);
				reg_data &= (~FGOS);/* Fetch GSN offset set to 0 */
				writel(reg_data, priv->ioaddr + DMA_TBS_CTRL);
				
				reg_data = readl(priv->ioaddr + MTL_TBS_CTRL);
				reg_data |= (LEGOS << 4);/* Launch Expiry GSN Offset */
				writel(reg_data, priv->ioaddr + MTL_TBS_CTRL);
				
				launch_time = EST_LAUNCH_TIME_OFFSET;
				
			}else{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,15))
				if((tx_q->launch_time) && (skb->tstamp)){

					launch_time = skb->tstamp;

				}else{
#endif						
				launch_time = app_launch_time;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,15))
				}
#endif
			}

#ifdef TEMP_SG_4TSO
			        priv->hw->desc->prepare_etx_desc(edesc, 0, len, csum_insertion,
						priv->mode, 1, last_segment,
						skb->len, GSN, launch_time);
#else
			/* Prepare the enhanced descriptor and set the own bit too */
		priv->hw->desc->prepare_etx_desc(edesc, (i == 0), skb->len, csum_insertion,
						priv->mode, 1, last_segment,
						skb->len, GSN, launch_time);
#endif
		}
	}

	if(skb->data[19] & 0x01)//if timestamp_valid bit enabled
	{
	    static u64 diffbtr = 0;
		Presentation_time = skb->data[30];
		Presentation_time = (Presentation_time<<8) | skb->data[31];
		Presentation_time = (Presentation_time<<8) | skb->data[32];
		Presentation_time = (Presentation_time<<8) | skb->data[33];

#if 1
		/* Launch time = (Presentation time - 2ms ) */
		if(AVB_CLASSB_CH == chInx)
		    traverse_time = 50000000;//classB 50ms
		else if(AVB_CLASSA_CH == chInx)
		    traverse_time = 2000000;//classA 2ms 
		      
		if (Presentation_time >= traverse_time)
			app_launch_time = Presentation_time - traverse_time; 
		else
			app_launch_time = 0x100000000ULL - traverse_time + Presentation_time; 
#else
			app_launch_time = Presentation_time ; 
#endif
		
		/* Read the value of GSN if ESTM bit is enabled */
		if (readl(priv->ioaddr + MTL_TBS_CTRL) & MTL_TBS_ESTM)
			{

				static u64 ns;
				static int start =0;
				u64 delta;
				u32 lt_s, lt_ns;
				
				do {
					/* Read the Current GSN value from the register */
					GSN1 = (readl(priv->ioaddr + MTL_EST_STATUS) & MTL_EST_CGSN) >> 16;
					/* GSN is calculated for EST mode */
					dwmac5_est_read(priv->hw, MTL_EST_BTR_LOW, &cfg->btr_offset[0], false);
					dwmac5_est_read(priv->hw, MTL_EST_BTR_HIGH, &cfg->btr_offset[1], false);
					GSN = (readl(priv->ioaddr + MTL_EST_STATUS) & MTL_EST_CGSN) >> 16;
				}while (GSN1 != GSN);

				dwmac5_est_read(priv->hw, MTL_EST_CTR_LOW, &cfg->ctr[0], false);
				dwmac5_est_read(priv->hw, MTL_EST_CTR_HIGH, &cfg->ctr[1], false);

				real_btr = ((u64)cfg->btr_offset[1] * 1000000000ULL) + cfg->btr_offset[0];
				ctr = ((u64)cfg->ctr[1] << 32 ) | cfg->ctr[0];


                
                if (start ==0)
                {
	                start = 1;
	                ns = priv->hw->ptp->get_systime(priv->ptpaddr);
	                printk("ptp = %lld, btr = %lld\n", app_launch_time - (ns & 0xFFFFFFFF), app_launch_time - (real_btr & 0xFFFFFFFF));
	                printk("app lt = %d, ptp = %lld, btr = %lld\n", app_launch_time, (ns & 0xFFFFFFFF),(real_btr & 0xFFFFFFFF));
	                printk("time diff = %lld\n", ns - real_btr );
	                diffbtr = ns - real_btr;
                }

                diffbtr = 0;
                
                delta = app_launch_time - (real_btr & 0xFFFFFFFF) - diffbtr; 
                if(((signed long long )(app_launch_time) - (signed long long) ((real_btr & 0xFFFFFFFF) - diffbtr)) < 0 )
                {
                    delta += (1UL << 32);
                }
                
		        GSN  =  (GSN + delta/ctr ) % 16;

				
#if 0  
				/* Launch time and GSN is updated for EST mode */
				reg_data = readl(priv->ioaddr + DMA_TBS_CTRL);
				reg_data &= (~FGOS);/* Fetch GSN offset set to 0 */
				writel(reg_data, priv->ioaddr + DMA_TBS_CTRL);
				
				reg_data = readl(priv->ioaddr + MTL_TBS_CTRL);
				reg_data |= (LEGOS << 4);/* Launch Expiry GSN Offset */
				writel(reg_data, priv->ioaddr + MTL_TBS_CTRL);
#endif
				
				lt_s  = ((delta % ctr) / 1000000000) & 0xFF;
				lt_ns = ((delta % ctr) % 1000000000) >> 8; // in 256 ns 
				launch_time = (lt_s << 24) | lt_ns;
				
			}else{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,15))

				if((tx_q->launch_time) && (skb->tstamp)){

					launch_time = skb->tstamp;

				}else{
#endif
				static u64 ns;
				u64 lt;
				u32 lt_s, lt_ns;

				ns = priv->hw->ptp->get_systime(priv->ptpaddr);

				lt = ((ns>>32) << 32) | app_launch_time;
                if ( ((signed long)app_launch_time - (signed long)(ns & 0xFFFFFFFF)) < 0 ) {
                      if(((ns & 0xFFFFFFFF) - app_launch_time) > ((1ULL << 32) / 2)){   // If the difference is 2 seconds apart, it is judged as rollover.
                            lt += (u64)( 1UL << 32);
                      }
                }
				lt_s  = (lt / 1000000000) & 0xFF;
				lt_ns = (lt % 1000000000) >> 8; // in 256 ns 

				launch_time = (lt_s << 24) | lt_ns;
			}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,15))

			}
#endif
		//efirst->des4 |= (0x1 << 31);//LTV enabled
		tv_flag = 1; 
		}
    else{
		//efirst->des4 &= 0x7fffffff;//LTV disabled
		tv_flag = 0;
	}
   		entry = TC9562MAC_GET_ENTRY(entry, DMA_TX_SIZE);
		tx_q->cur_tx = entry;
		
		if (netif_msg_pktdata(priv)) {
			void *tx_head;

			netdev_dbg(priv->dev,
		  	 "%s: curr=%d dirty=%d f=%d, e=%d, first=%p, nfrags=%d",
		  	 __func__, tx_q->cur_tx, tx_q->dirty_tx, first_entry,
		  	 entry, efirst, nfrags);

			tx_head = (void *)tx_q->dma_entx;		

			priv->hw->desc->display_en_ring(tx_head, DMA_TX_SIZE, false);

			netdev_dbg(priv->dev, ">>> frame to be transmitted: ");
			print_pkt(skb->data, skb->len);
		}
  
    		if (unlikely(tc9562mac_tx_avail(priv, queue) <= (MAX_SKB_FRAGS + 1))) {
			netif_dbg(priv, hw, priv->dev, "%s: stop transmitted packets\n",
			  __func__);
			netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, queue));
		}

		dev->stats.tx_bytes += skb->len;

		/* According to the coalesce parameter the IC bit for the latest
		 * segment is reset and the timer re-started to clean the tx status.
		 * This approach takes care about the fragments: desc is the first
		 * element in case of no SG.
		 */
		priv->tx_count_frames += nfrags + 1;
		if (likely(priv->tx_coal_frames > priv->tx_count_frames)) {
			mod_timer(&priv->txtimer,
			  TC9562MAC_COAL_TIMER(priv->tx_coal_timer));
		} else {
			priv->tx_count_frames = 0;
			priv->hw->desc->set_etx_ic(edesc);
			priv->xstats.tx_set_ic_bit++;
		}

		skb_tx_timestamp(skb);
	/* Ready to fill the first descriptor and set the OWN bit w/o any
	 * problems because all the descriptors are actually ready to be
	 * passed to the DMA engine.
	 */
     
     if (likely(!is_jumbo)) {
		bool last_segment = (nfrags == 0);

		des = dma_map_single(priv->device, skb->data,
				     nopaged_len, DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err;

		tx_q->tx_skbuff_dma[first_entry].buf = des;
		if (unlikely(priv->synopsys_id >= DWMAC_CORE_4_00)) {
#ifdef TC9562_DEFINED
			efirst->basic.des0 = (des & 0xFFFFFFFF);
			efirst->basic.des1 = HOST_PHYSICAL_ADRS_MASK | ((des >> 32) & 0xF);
#else
			efirst->basic.des0 = cpu_to_le32(des);
#endif
		}
		else
			efirst->basic.des2 = cpu_to_le32(des);



		tx_q->tx_skbuff_dma[first_entry].len = nopaged_len;
		tx_q->tx_skbuff_dma[first_entry].last_segment = last_segment;

		if (unlikely((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
			     priv->hwts_tx_en)) {
			/* declare that device is doing timestamping */
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
			priv->hw->desc->enable_etx_timestamp(efirst);
		}
		/* Prepare the first descriptor setting the OWN bit too */
		priv->hw->desc->prepare_etx_desc(efirst, 1, nopaged_len,
						//csum_insertion, priv->mode, 1,
						csum_insertion, tv_flag, 1,
						last_segment, skb->len, GSN, launch_time);
	
	    efirst->basic.des3 |= ((priv->sa_vlan_ins_via_desc << 23) & GENMASK(25, 23) );
		/* The own bit must be the latest setting done when prepare the
		 * descriptor and then barrier is needed to make sure that
		 * all is coherent before granting the DMA engine.
		 */
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 18, 140))
        wmb();
#else
        dma_wmb();
#endif
	}
	
		/*pr_info("%d [0x%x]: 0x%x 0x%x 0x%x 0x%x",
			i, (unsigned int)virt_to_phys(efirst),
			(efirst->basic.des0), (efirst->basic.des1),
			(efirst->basic.des2), (efirst->basic.des3));
		pr_info(" : 0x%x 0x%x 0x%x 0x%x\n",
			(efirst->des4), (efirst->des5),
			(efirst->des6),(efirst->des7));*/

	netdev_tx_sent_queue(netdev_get_tx_queue(dev, queue), skb->len);

	if (priv->synopsys_id < DWMAC_CORE_4_00)
		priv->hw->dma->enable_dma_transmission(priv->ioaddr);
	else
	{
		priv->hw->dma->set_tx_tail_ptr(priv->ioaddr, tx_q->tx_tail_addr,
					       queue);
	}
    DBGPR_FUNC("<--tc9562mac_enxmit\n");

	return NETDEV_TX_OK;

dma_map_err:
	netdev_err(priv->dev, "Tx DMA map failed\n");
	dev_kfree_skb(skb);
	priv->dev->stats.tx_dropped++;
	return NETDEV_TX_OK;
    }
	

static netdev_tx_t tc9562mac_nxmit(struct sk_buff *skb, struct net_device *dev)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	unsigned int nopaged_len = skb_headlen(skb);
	int i, /*csum_insertion = 0,*/ is_jumbo = 0;
	u32 queue = skb_get_queue_mapping(skb);
	int nfrags = skb_shinfo(skb)->nr_frags;
	int entry;
	unsigned int first_entry;
	struct dma_desc *desc, *first;
	struct tc9562mac_tx_queue *tx_q;
	unsigned int enh_desc;
	u64 des;
    //int ii;
    

	DBGPR_FUNC("-->tc9562mac_nxmit\n");

#ifdef UNIFIED_DRIVER
	if(queue == 1 || queue == 2 || queue == 5)
    {
		printk("Trying to send data on CM3 owned Tx Channel = %d\n", queue);
		return NETDEV_TX_BUSY;
	}
#endif
	tx_q = &priv->tx_queue[queue];

	/* Manage oversized TCP frames for GMAC4 device */
	if (skb_is_gso(skb) && priv->tso && (queue == 0) ) { //only for channel 0
		if (skb_shinfo(skb)->gso_type & (SKB_GSO_TCPV4 | SKB_GSO_TCPV6)) {
            	//DBGPR_TEST("-->TCP packet of size: %d\n", skb->len);
               //for (ii= 0; ii< 100; ii++)
                    //DBGPR_TEST("-->skb data[%d]: %x\n",ii, skb->data[ii]);  
                
			return tc9562mac_tso_xmit(skb, dev);
          }
	} else if((PROTOCOL_TYPE_UDP == skb->data[23]) && (skb->len > SIZE_FOR_SEGMENTATION) && (0 == queue)){ //only for channel 0
      	DBGPR_TEST("-->UDP packet of size: %d gso_type : %d\n", skb->len, skb_shinfo(skb)->gso_type);	
        //for (ii= 0; ii< 100; ii++)
          //  DBGPR_TEST("-->skb data[%d]: %x\n",ii, skb->data[ii]);	
        return tc9562mac_uso_xmit(skb, dev);
    }
	if (unlikely(tc9562mac_tx_avail(priv, queue) < nfrags + 1)) {
		if (!netif_tx_queue_stopped(netdev_get_tx_queue(dev, queue))) {
			netif_tx_stop_queue(netdev_get_tx_queue(priv->dev,
								queue));
			/* This is a hard error, log it. */
			netdev_err(priv->dev,
				   "%s: Tx Ring full when queue awake\n",
				   __func__);
		}
		return NETDEV_TX_BUSY;
	}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	if (priv->tx_path_in_lpi_mode)
		tc9562mac_disable_eee_mode(priv);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */	

	entry = tx_q->cur_tx;
	first_entry = entry;

	/* csum_insertion = (skb->ip_summed == CHECKSUM_PARTIAL);*/
	// csum_insertion = 1;  forced for checksum 


	if (likely(priv->extend_desc))
		desc = (struct dma_desc *)(tx_q->dma_etx + entry);
	else
		desc = tx_q->dma_tx + entry;

	first = desc;

	tx_q->tx_skbuff[first_entry] = skb;

	enh_desc = priv->plat->enh_desc;
	/* To program the descriptors according to the size of the frame */
	if (enh_desc)
		is_jumbo = priv->hw->mode->is_jumbo_frm(skb->len, enh_desc);

	if (unlikely(is_jumbo) && likely(priv->synopsys_id <
					 DWMAC_CORE_4_00)) {
		entry = priv->hw->mode->jumbo_frm(tx_q, skb, priv->csum_insertion);
		if (unlikely(entry < 0))
			goto dma_map_err;
	}


	for (i = 0; i < nfrags; i++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		int len = skb_frag_size(frag);
		bool last_segment = (i == (nfrags - 1));

        //printk("ZZ SG %d\n", nfrags);
		entry = TC9562MAC_GET_ENTRY(entry, DMA_TX_SIZE);

		if (likely(priv->extend_desc))
			desc = (struct dma_desc *)(tx_q->dma_etx + entry);
		else
			desc = tx_q->dma_tx + entry;

		des = skb_frag_dma_map(priv->device, frag, 0, len,
				       DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err; /* should reuse desc w/o issues */

		tx_q->tx_skbuff[entry] = NULL;

		tx_q->tx_skbuff_dma[entry].buf = des;
		if (unlikely(priv->synopsys_id >= DWMAC_CORE_4_00)){
#ifdef TC9562_DEFINED
			desc->des0 = (des & 0xFFFFFFFF);
			desc->des1 = HOST_PHYSICAL_ADRS_MASK | ((des >> 32) & 0xF);
#else
			desc->des0 = cpu_to_le32(des);
#endif
		}
		else
			desc->des2 = cpu_to_le32(des);

		tx_q->tx_skbuff_dma[entry].map_as_page = true;
		tx_q->tx_skbuff_dma[entry].len = len;
		tx_q->tx_skbuff_dma[entry].last_segment = last_segment;

		/* Prepare the descriptor and set the own bit too */
#ifdef TEMP_SG_4TSO
		priv->hw->desc->prepare_tx_desc(desc, 0, len, priv->csum_insertion,
						priv->mode, 1, last_segment,
						skb->len);
#else
		priv->hw->desc->prepare_tx_desc(desc, (i == 0), skb->len, priv->csum_insertion,
						priv->mode, 1, last_segment,
						skb->len);
#endif
	}

	entry = TC9562MAC_GET_ENTRY(entry, DMA_TX_SIZE);
	tx_q->cur_tx = entry;

	if (netif_msg_pktdata(priv)) {
		void *tx_head;

		netdev_dbg(priv->dev,
			   "%s: curr=%d dirty=%d f=%d, e=%d, first=%p, nfrags=%d",
			   __func__, tx_q->cur_tx, tx_q->dirty_tx, first_entry,
			   entry, first, nfrags);

		if (priv->extend_desc)
			tx_head = (void *)tx_q->dma_etx;
		else
			tx_head = (void *)tx_q->dma_tx;

		priv->hw->desc->display_ring(tx_head, DMA_TX_SIZE, false);

		netdev_dbg(priv->dev, ">>> frame to be transmitted: ");
		print_pkt(skb->data, skb->len);
	}

	if (unlikely(tc9562mac_tx_avail(priv, queue) <= (MAX_SKB_FRAGS + 1))) {
		netif_dbg(priv, hw, priv->dev, "%s: stop transmitted packets\n",
			  __func__);
		netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, queue));
	}

	dev->stats.tx_bytes += skb->len;

	/* According to the coalesce parameter the IC bit for the latest
	 * segment is reset and the timer re-started to clean the tx status.
	 * This approach takes care about the fragments: desc is the first
	 * element in case of no SG.
	 */
	priv->tx_count_frames += nfrags + 1;
	if (likely(priv->tx_coal_frames > priv->tx_count_frames)) {
		mod_timer(&priv->txtimer,
			  TC9562MAC_COAL_TIMER(priv->tx_coal_timer));
	} else {
		priv->tx_count_frames = 0;
		priv->hw->desc->set_tx_ic(desc);
		priv->xstats.tx_set_ic_bit++;
	}

	skb_tx_timestamp(skb);

	/* Ready to fill the first descriptor and set the OWN bit w/o any
	 * problems because all the descriptors are actually ready to be
	 * passed to the DMA engine.
	 */
	if (likely(!is_jumbo)) {
		bool last_segment = (nfrags == 0);

		des = dma_map_single(priv->device, skb->data,
				     nopaged_len, DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err;

		tx_q->tx_skbuff_dma[first_entry].buf = des;
		if (unlikely(priv->synopsys_id >= DWMAC_CORE_4_00)) {
#ifdef TC9562_DEFINED
			first->des0 = (des & 0xFFFFFFFF);
			first->des1 = HOST_PHYSICAL_ADRS_MASK | ((des >> 32) & 0xF);
#else
			first->des0 = cpu_to_le32(des);
#endif
		}
		else
			first->des2 = cpu_to_le32(des);

		tx_q->tx_skbuff_dma[first_entry].len = nopaged_len;
		tx_q->tx_skbuff_dma[first_entry].last_segment = last_segment;

		if (unlikely((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
			     priv->hwts_tx_en)) {
			/* declare that device is doing timestamping */
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
			priv->hw->desc->enable_tx_timestamp(first);
		}

		/* Prepare the first descriptor setting the OWN bit too */
		priv->hw->desc->prepare_tx_desc(first, 1, nopaged_len,
						priv->csum_insertion, priv->mode, 1,
						last_segment, skb->len);

        first->des3 |= ((priv->sa_vlan_ins_via_desc << 23) & GENMASK(25, 23) );
		/* The own bit must be the latest setting done when prepare the
		 * descriptor and then barrier is needed to make sure that
		 * all is coherent before granting the DMA engine.
		 */
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 18, 140))
        wmb();
#else
        dma_wmb();
#endif
	}

	netdev_tx_sent_queue(netdev_get_tx_queue(dev, queue), skb->len);

	if (priv->synopsys_id < DWMAC_CORE_4_00)
		priv->hw->dma->enable_dma_transmission(priv->ioaddr);
	else
		priv->hw->dma->set_tx_tail_ptr(priv->ioaddr, tx_q->tx_tail_addr,
					       queue);
		DBGPR_FUNC("<--tc9562mac_nxmit\n");

	return NETDEV_TX_OK;

dma_map_err:
	netdev_err(priv->dev, "Tx DMA map failed\n");
	dev_kfree_skb(skb);
	priv->dev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

/**
 *  tc9562mac_xmit - Tx entry point of the driver
 *  @skb : the socket buffer
 *  @dev : device pointer
 *  Description : this is the tx entry point of the driver.
 *  It programs the chain or the ring and supports oversized frames
 *  and SG feature.
 */
static netdev_tx_t tc9562mac_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct tc9562mac_priv *priv = netdev_priv(dev);
    u32 queue = skb_get_queue_mapping(skb);
    unsigned int chInx = 0;
    struct tc9562mac_tx_queue *tx_q;
    chInx = skb_get_queue_mapping(skb); 
    tx_q = &priv->tx_queue[queue];

	DBGPR_FUNC("-->tc9562mac_xmit\n");
    if( (tx_q->queue_index == AVB_CLASSA_CH) || (tx_q->queue_index == AVB_CLASSB_CH) || (tx_q->queue_index == CLASS_CDT))
	{
        	return tc9562mac_enxmit(skb,dev);
	}
    else
	{
	        return tc9562mac_nxmit(skb,dev);
	}

	DBGPR_FUNC("<--tc9562mac_xmit\n");
}

static void tc9562mac_rx_vlan(struct net_device *dev, struct sk_buff *skb)
{
	struct ethhdr *ehdr;
	u16 vlanid;

	DBGPR_FUNC("-->tc9562mac_rx_vlan\n");

	if ((dev->features & NETIF_F_HW_VLAN_CTAG_RX) ==
	    NETIF_F_HW_VLAN_CTAG_RX &&
	    !__vlan_get_tag(skb, &vlanid)) {
		/* pop the vlan tag */
		ehdr = (struct ethhdr *)skb->data;
		memmove(skb->data + VLAN_HLEN, ehdr, ETH_ALEN * 2);
		skb_pull(skb, VLAN_HLEN);
		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), vlanid);
	}

	DBGPR_FUNC("<--tc9562mac_rx_vlan\n");
}


static inline int tc9562mac_rx_threshold_count(struct tc9562mac_rx_queue *rx_q)
{
	DBGPR_FUNC("-->tc9562mac_rx_threshold_count\n");
	if (rx_q->rx_zeroc_thresh < TC9562MAC_RX_THRESH)
		return 0;

	DBGPR_FUNC("<--tc9562mac_rx_threshold_count\n");

	return 1;
}

/**
 * tc9562mac_rx_refill - refill used skb preallocated buffers
 * @priv: driver private structure
 * @queue: RX queue index
 * Description : this is to reallocate the skb for the reception process
 * that is based on zero-copy.
 */
static inline void tc9562mac_rx_refill(struct tc9562mac_priv *priv, u32 queue)
{
	struct tc9562mac_rx_queue *rx_q = &priv->rx_queue[queue];
	int dirty = tc9562mac_rx_dirty(priv, queue);
	unsigned int entry = rx_q->dirty_rx;

	int bfsize = priv->dma_buf_sz;

	DBGPR_FUNC("-->tc9562mac_rx_refill\n");

	while (dirty-- > 0) {
		struct dma_desc *p;

		if (priv->extend_desc)
			p = (struct dma_desc *)(rx_q->dma_erx + entry);
		else
			p = rx_q->dma_rx + entry;

		if (likely(!rx_q->rx_skbuff[entry])) {
			struct sk_buff *skb;

			skb = netdev_alloc_skb_ip_align(priv->dev, bfsize + priv->plat->axi->axi_wr_osr_lmt * TC9562_ADDL_BUF_SIZE) ;
			if (unlikely(!skb)) {
				/* so for a while no zero-copy! */
				rx_q->rx_zeroc_thresh = TC9562MAC_RX_THRESH;
				if (unlikely(net_ratelimit()))
					dev_err(priv->device,
						"fail to alloc skb entry %d\n",
						entry);
				break;
			}

			rx_q->rx_skbuff[entry] = skb;
			rx_q->rx_skbuff_dma[entry] =
			    dma_map_single(priv->device, skb->data, bfsize,
					   DMA_FROM_DEVICE);
			if (dma_mapping_error(priv->device,
					      rx_q->rx_skbuff_dma[entry])) {
				netdev_err(priv->dev, "Rx DMA map failed\n");
				dev_kfree_skb(skb);
				break;
			}

			if (unlikely(priv->synopsys_id >= DWMAC_CORE_4_00)) {
#ifdef TC9562_DEFINED
				p->des0 = (rx_q->rx_skbuff_dma[entry]) & 0xFFFFFFFF;
				p->des1 = HOST_PHYSICAL_ADRS_MASK | ((rx_q->rx_skbuff_dma[entry] >> 32) & 0xF);
#else
				p->des0 = cpu_to_le32(rx_q->rx_skbuff_dma[entry]);
				p->des1 = 0;
#endif
			} else {
				p->des2 = cpu_to_le32(rx_q->rx_skbuff_dma[entry]);
			}
			if (priv->hw->mode->refill_desc3)
				priv->hw->mode->refill_desc3(rx_q, p);

			if (rx_q->rx_zeroc_thresh > 0)
				rx_q->rx_zeroc_thresh--;

			netif_dbg(priv, rx_status, priv->dev,
				  "refill entry #%d\n", entry);
		}
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 18, 140))
        wmb();
#else
        dma_wmb();
#endif

		if (unlikely(priv->synopsys_id >= DWMAC_CORE_4_00))
			priv->hw->desc->init_rx_desc(p, priv->use_riwt, 0, 0);
		else
			priv->hw->desc->set_rx_owner(p);

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 18, 140))
        wmb();
#else
        dma_wmb();
#endif

		entry = TC9562MAC_GET_ENTRY(entry, DMA_RX_SIZE);
	}
	rx_q->dirty_rx = entry;

	DBGPR_FUNC("<--tc9562mac_rx_refill\n");
}

/**
 * tc9562mac_rx - manage the receive process
 * @priv: driver private structure
 * @limit: napi bugget
 * @queue: RX queue index.
 * Description :  this the function called by the napi poll method.
 * It gets all the frames inside the ring.
 */
static int tc9562mac_rx(struct tc9562mac_priv *priv, int limit, u32 queue)
{
	struct tc9562mac_rx_queue *rx_q = &priv->rx_queue[queue];
	unsigned int entry = rx_q->cur_rx;
	int coe = priv->hw->rx_csum;
	unsigned int next_entry;
	unsigned int count = 0;

	DBGPR_FUNC("-->tc9562mac_rx\n");

	if (netif_msg_rx_status(priv)){
		void *rx_head;

		netdev_dbg(priv->dev, "%s: descriptor ring:\n", __func__);
		if (priv->extend_desc)
			rx_head = (void *)rx_q->dma_erx;
		else
			rx_head = (void *)rx_q->dma_rx;

		priv->hw->desc->display_ring(rx_head, DMA_RX_SIZE, true);
	}
	while (count < limit) {
		int status;
		struct dma_desc *p;
		struct dma_desc *np;

		if (priv->extend_desc)
			p = (struct dma_desc *)(rx_q->dma_erx + entry);
		else
			p = rx_q->dma_rx + entry;

		/* read the status of the incoming frame */
		status = priv->hw->desc->rx_status(&priv->dev->stats,
						   &priv->xstats, p);
		/* check if managed by the DMA otherwise go ahead */
		if (unlikely(status & dma_own))
			break;

		count++;

		rx_q->cur_rx = TC9562MAC_GET_ENTRY(rx_q->cur_rx, DMA_RX_SIZE);
		next_entry = rx_q->cur_rx;

		if (priv->extend_desc)
			np = (struct dma_desc *)(rx_q->dma_erx + next_entry);
		else
			np = rx_q->dma_rx + next_entry;

		prefetch(np);

		if ((priv->extend_desc) && (priv->hw->desc->rx_extended_status))
			priv->hw->desc->rx_extended_status(&priv->dev->stats,
							   &priv->xstats,
							   rx_q->dma_erx +
							   entry);
		if (unlikely(status == discard_frame)) {

            /*Check for Context Descriptor; if so, dont increment the rx error counter */
            unsigned int rdes3 = le32_to_cpu(p->des3);

            if (!(rdes3 & RDES3_CONTEXT_DESCRIPTOR)) {
    			priv->dev->stats.rx_errors++;                
            }
            
    		if (priv->hwts_rx_en && !priv->extend_desc) {
				/* DESC2 & DESC3 will be overwritten by device
				 * with timestamp value, hence reinitialize
				 * them in tc9562mac_rx_refill() function so that
				 * device can reuse it.
				 */
				dev_kfree_skb_any(rx_q->rx_skbuff[entry]);
				rx_q->rx_skbuff[entry] = NULL;
				dma_unmap_single(priv->device,
						 rx_q->rx_skbuff_dma[entry],
						 priv->dma_buf_sz,
						 DMA_FROM_DEVICE);
			}
		} else {
			struct sk_buff *skb;
			int frame_len;
			u64 des;
			if (unlikely(priv->synopsys_id >= DWMAC_CORE_4_00)){
#ifdef TC9562_DEFINED
				des = p->des1;
				des = ((des << 32) | (p->des0 & 0xFFFFFFFF));
#else
				des = le32_to_cpu(p->des0);
#endif
			}
			else
				des = le32_to_cpu(p->des2);

			frame_len = priv->hw->desc->get_rx_frame_len(p, coe);

			/*  If frame length is greater than skb buffer size
			 *  (preallocated during init) then the packet is
			 *  ignored
			 */
			if (frame_len > priv->dma_buf_sz) {
				netdev_err(priv->dev,
					   "len %d larger than size (%d)\n",
					   frame_len, priv->dma_buf_sz);
				priv->dev->stats.rx_length_errors++;
				break;
			}

			/* ACS is set; GMAC core strips PAD/FCS for IEEE 802.3
			 * Type frames (LLC/LLC-SNAP)
			 */
			if (unlikely(status != llc_snap))
				frame_len -= ETH_FCS_LEN;

			if (netif_msg_rx_status(priv)) {
				netdev_dbg(priv->dev, "\tdesc: %p [entry %d] buff=0x%llx\n",
					   p, entry, des);
				if (frame_len > ETH_FRAME_LEN)
					netdev_dbg(priv->dev, "frame size %d, COE: %d\n",
						   frame_len, status);
			}

			/* The zero-copy is always used for all the sizes
			 * in case of GMAC4 because it needs
			 * to refill the used descriptors, always.
			 */
			if (unlikely(!priv->plat->has_gmac4 &&
				     ((frame_len < priv->rx_copybreak) ||
				     tc9562mac_rx_threshold_count(rx_q)))) {
				skb = netdev_alloc_skb_ip_align(priv->dev,
								frame_len + priv->plat->axi->axi_wr_osr_lmt * TC9562_ADDL_BUF_SIZE);
				if (unlikely(!skb)) {
					if (net_ratelimit())
						dev_warn(priv->device,
							 "packet dropped\n");
					priv->dev->stats.rx_dropped++;
					break;
				}

				dma_sync_single_for_cpu(priv->device,
							rx_q->rx_skbuff_dma
							[entry], frame_len,
							DMA_FROM_DEVICE);
				skb_copy_to_linear_data(skb,
							rx_q->
							rx_skbuff[entry]->data,
							frame_len);

				skb_put(skb, frame_len);
				dma_sync_single_for_device(priv->device,
							   rx_q->rx_skbuff_dma
							   [entry], frame_len,
							   DMA_FROM_DEVICE);
			} else {
				skb = rx_q->rx_skbuff[entry];
				if (unlikely(!skb)) {
					netdev_err(priv->dev,
						   "%s: Inconsistent Rx chain\n",
						   priv->dev->name);
					priv->dev->stats.rx_dropped++;
					break;
				}
				prefetch(skb->data - NET_IP_ALIGN);
				rx_q->rx_skbuff[entry] = NULL;
				rx_q->rx_zeroc_thresh++;

				skb_put(skb, frame_len);
				dma_unmap_single(priv->device,
						 rx_q->rx_skbuff_dma[entry],
						 priv->dma_buf_sz,
						 DMA_FROM_DEVICE);
			}

			if (netif_msg_pktdata(priv)) {
				netdev_dbg(priv->dev, "frame received (%dbytes)",
					   frame_len);
				print_pkt(skb->data, frame_len);
			}
#ifdef UNIFIED_DRIVER
            if(priv->plat->rx_dma_ch_for_host[HOST_GPTP_CHANNEL] == 1)      /*In Unified Rx Timestamp All is Disabled, GPTP is handled in CM3*/
            {
#endif
#if defined(RX_LOGGING_TRACE)
			tc9562mac_get_rx_hwtstamp(priv, p, np, skb, queue);
#else
			tc9562mac_get_rx_hwtstamp(priv, p, np, skb);
#endif

#ifdef UNIFIED_DRIVER
            }
#endif
			tc9562mac_rx_vlan(priv->dev, skb);

			skb->protocol = eth_type_trans(skb, priv->dev);

			if (unlikely(!coe))
				skb_checksum_none_assert(skb);
			else
				skb->ip_summed = CHECKSUM_UNNECESSARY;

			napi_gro_receive(&rx_q->napi, skb);

			priv->dev->stats.rx_packets++;
			priv->dev->stats.rx_bytes += frame_len;
		}
		entry = next_entry;
	}	
	
	tc9562mac_rx_refill(priv, queue);
	priv->hw->dma->set_rx_tail_ptr(priv->ioaddr, rx_q->rx_tail_addr, queue);
	
	priv->xstats.rx_pkt_n += count;

	DBGPR_FUNC("<--tc9562mac_rx\n");

	return count;
}

/**
 *  tc9562mac_poll - tc9562mac poll method (NAPI)
 *  @napi : pointer to the napi structure.
 *  @budget : maximum number of packets that the current CPU can receive from
 *	      all interfaces.
 *  Description :
 *  To look at the incoming frames and clear the tx resources.
 */
static int tc9562mac_poll(struct napi_struct *napi, int budget)
{
	struct tc9562mac_rx_queue *rx_q =
		container_of(napi, struct tc9562mac_rx_queue, napi);
	struct tc9562mac_priv *priv = rx_q->priv_data;
	u32 tx_count = priv->plat->tx_queues_to_use;
	u32 chan = rx_q->queue_index;
	int work_done = 0;
	u32 queue;

	DBGPR_FUNC("-->tc9562mac_poll\n");

	priv->xstats.napi_poll++;

	/* check all the queues */
	for (queue = 0; queue < tx_count; queue++)
	{
#ifdef UNIFIED_DRIVER
		if(priv->plat->tx_dma_ch_for_host[queue] == 0)
			continue;
#endif
		tc9562mac_tx_clean(priv, queue);

    }
    
#ifdef UNIFIED_DRIVER
	if(priv->plat->rx_dma_ch_for_host[chan] == 1)
	{
#endif
    /* Rx Ch 6 not configured. Skip it */
    if(chan != (tx_count - 1)) {
    	work_done = tc9562mac_rx(priv, budget, rx_q->queue_index);
	}
#ifdef UNIFIED_DRIVER
	}
#endif
	
	if (work_done < budget) {
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 18, 140))
		napi_complete(napi);
#else
		napi_complete_done(napi, work_done);
#endif	
		tc9562mac_enable_dma_irq(priv, chan);
	}

	DBGPR_FUNC("<--tc9562mac_poll\n");
	return work_done;
}
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static void tc9562mac_service_event_schedule(struct tc9562mac_priv *priv)
{
	DBGPR_FUNC("-->tc9562mac_service_event_schedule\n");

	if (!test_bit(TC9562MAC_DOWN, &priv->state) &&
	    !test_and_set_bit(TC9562MAC_SERVICE_SCHED, &priv->state))
		queue_work(priv->wq, &priv->service_task);

	DBGPR_FUNC("<--tc9562mac_service_event_schedule\n");
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

/**
 * tc9562mac_global_err - to manage a global error
 * @priv: driver private structure
 * Description: it cleans the descriptors and restarts the reception
 * and transmission in case of global errors.
 */
static void tc9562mac_global_err(struct tc9562mac_priv *priv)
{
	DBGPR_FUNC("-->tc9562mac_global_err\n");

	set_bit(TC9562MAC_RESET_REQUESTED, &priv->state);
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE	
	tc9562mac_service_event_schedule(priv);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

	DBGPR_FUNC("<--tc9562mac_global_err\n");
}

/**
 *  tc9562mac_tx_timeout
 *  @dev : Pointer to net device structure
 *  Description: this function is called when a packet transmission fails to
 *   complete within a reasonable time. The driver will mark the error in the
 *   netdev structure and arrange for the device to be reset to a sane state
 *   in order to transmit a new packet.
 */
static void tc9562mac_tx_timeout(struct net_device *dev)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);

	DBGPR_FUNC("-->tc9562mac_tx_timeout\n");

	tc9562mac_global_err(priv);
	
	DBGPR_FUNC("<--tc9562mac_tx_timeout\n");
}

/*!
 * \brief API to read register
 * \param[in] address offset as per tc9562 data-sheet
 * \param[in] bar number
 * \return register value
 */
static int tc9562_reg_rd(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562_ioctl_reg_rd_wr ioctl_data;
	u32 val;
	DBGPR_FUNC("-->tc9562_reg_rd\n");
	if (copy_from_user(&ioctl_data, data, sizeof(ioctl_data)))
			return -EFAULT;

   	if (2 == ioctl_data.bar_num)
		val = readl((void *)(priv->tc9562_SRAM_pci_base_addr + ioctl_data.addr));
   	else if (4 == ioctl_data.bar_num)
		val = readl((void *)(priv->tc9562_FLASH_pci_base_addr + ioctl_data.addr));
   	else //(0 == bar_num)
		val = readl((void *)(priv->dev->base_addr + ioctl_data.addr));

	if (copy_to_user(ioctl_data.ptr ,&val, sizeof(unsigned int)))
		return -EFAULT;
	DBGPR_FUNC("<--tc9562_reg_rd\n");
	return 0;
}
/*!
 * \brief API to write register
 * \param[in] address offset as per tc9562 data-sheet
 * \return register
 */
static int tc9562_reg_wr(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562_ioctl_reg_rd_wr ioctl_data;
	u32 val;
	DBGPR_FUNC("-->tc9562_reg_wr\n");
	if (copy_from_user(&ioctl_data, data, sizeof(ioctl_data)))
			return -EFAULT;
			
    if (copy_from_user(&val, ioctl_data.ptr, sizeof(unsigned int)))
            return -EFAULT;
			
   	if (0 == ioctl_data.bar_num)
   		writel(val, (void *)(priv->dev->base_addr + ioctl_data.addr));   		
   	if (2 == ioctl_data.bar_num)
       	writel(val, (void *)(priv->tc9562_SRAM_pci_base_addr + ioctl_data.addr));       	
   	if (4 == ioctl_data.bar_num)
       	writel(val, (void *)(priv->tc9562_FLASH_pci_base_addr + ioctl_data.addr));       	
       	
	DBGPR_FUNC("<--tc9562_reg_wr\n");
	return 0;
}

/**
* tc9562_vlan_rx_add_vid-  API to add vid to HW filter.
*
* This function is invoked by upper layer when a new VALN id is
* registered. This function updates the HW filter with new VLAN id.
* New vlan id can be added with vconfig -
* vconfig add <interface_name > <vlan_id>
*
* @dev - pointer to net_device structure
* @vid - new vlan id.
*
* Return void
*/
#ifdef NETIF_F_HW_VLAN_CTAG_RX
int tc9562_vlan_rx_add_vid(struct net_device *dev, __always_unused __be16 proto, u16 vid)
#else
int tc9562_vlan_rx_add_vid(struct net_device *dev, u16 vid)
#endif
{
    struct tc9562mac_priv *priv = netdev_priv(dev);
	unsigned short new_index, old_index;
	int crc32_val = 0;
	unsigned int enb_12bit_vhash;

	DBGPR_FUNC("-->tc9562_vlan_rx_add_vid: vid = %d\n", vid);

	if (priv->vlan_hash_filtering) {
		/* The upper 4 bits of the calculated CRC are used to
		 * index the content of the VLAN Hash Table Reg.
		 * */
		crc32_val = (bitrev32(~crc32_le(~0, (unsigned char *)&vid, 2)) >> 28);

		/* These 4(0xF) bits determines the bit within the
		 * VLAN Hash Table Reg 0
		 * */
		enb_12bit_vhash = (readl(priv->ioaddr + GMAC_VLAN_TAG) & GMAC_VLANTR_ETV) >> GMAC_VLANTR_ETV_LPOS;

		if (enb_12bit_vhash) {
			new_index = (1 << (crc32_val & 0xF));  //(1 << (~crc32_val & 0xF));
		} else {
		    /* If ETV is set, set one's complement of the 4 bit hash value */
			new_index = (1 << (~crc32_val & 0xF)); //(1 << (crc32_val & 0xF));
		}

		old_index = (readl(priv->ioaddr + GMAC_VLAN_HASH_TBL) & GMAC_VHT_VLHT) >> GMAC_VHT_VLHT_LPOS;
		old_index |= new_index;
		writel(old_index, priv->ioaddr + GMAC_VLAN_HASH_TBL);

	} else {
		writel(vid, priv->ioaddr + GMAC_VLAN_TAG);
	}

	DBGPR_FUNC("<--tc9562_vlan_rx_add_vid\n");

	return 0;
}

/**
* tc9562_vlan_rx_kill_vid-  API to add vid to HW filter.
*
* This function is invoked by upper layer when a VLAN id is removed.
* This function deletes the VLAN id from the HW filter.
* vlan id can be removed with vconfig -
* vconfig rem <interface_name > <vlan_id>
*
* @dev - pointer to net_device structure
* @vid - new vlan id.
*
* Return void
*/
#ifdef NETIF_F_HW_VLAN_CTAG_RX
int tc9562_vlan_rx_kill_vid(struct net_device *dev, __always_unused __be16 proto, u16 vid)
#else
int tc9562_vlan_rx_kill_vid(struct net_device *dev, u16 vid)
#endif
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	unsigned short new_index, old_index;
	int crc32_val = 0;
	unsigned int enb_12bit_vhash;

	DBGPR_FUNC("-->tc9562_vlan_rx_kill_vid: vid = %d\n", vid);

	if (priv->vlan_hash_filtering) {
		crc32_val = (bitrev32(~crc32_le(~0, (unsigned char *)&vid, 2)) >> 28);

		enb_12bit_vhash = (readl(priv->ioaddr + GMAC_VLAN_TAG) & GMAC_VLANTR_ETV) >> GMAC_VLANTR_ETV_LPOS;
		
		if (enb_12bit_vhash) {
			
			new_index = (1 << (crc32_val & 0xF));
		} else {
		    /* If ETV is set, set one's complement of the 4 bit hash value */
			new_index = (1 << (~crc32_val & 0xF));
		}

		old_index = (readl(priv->ioaddr + GMAC_VLAN_HASH_TBL) & GMAC_VHT_VLHT) >> GMAC_VHT_VLHT_LPOS;
		old_index &= ~new_index;
		writel(old_index, priv->ioaddr + GMAC_VLAN_HASH_TBL);
	} else {
		/* By default, receive only VLAN pkt with VID = 1
		 * becasue writting 0 will pass all VLAN pkt */
		writel(1, priv->ioaddr + GMAC_VLAN_TAG);
	}

	DBGPR_FUNC("<--tc9562_vlan_rx_kill_vid\n");

	return 0;
}

/**
 *  tc9562mac_change_mtu - entry point to change MTU size for the device.
 *  @dev : device pointer.
 *  @new_mtu : the new MTU size for the device.
 *  Description: the Maximum Transfer Unit (MTU) is used by the network layer
 *  to drive packet transmission. Ethernet has an MTU of 1500 octets
 *  (ETH_DATA_LEN). This value can be changed with ifconfig.
 *  Return value:
 *  0 on success and an appropriate (-)ve integer as defined in errno.h
 *  file on failure.
 */
static int tc9562mac_change_mtu(struct net_device *dev, int new_mtu)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
    int max_frame = (new_mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 10, 0))	
	int max_mtu, min_mtu;
#endif
    
	DBGPR_FUNC("-->tc9562mac_change_mtu\n");

	if (netif_running(dev)) {
		netdev_err(priv->dev, "must be stopped to change its MTU\n");
		return -EBUSY;
	}

    if (dev->mtu == new_mtu) {
		NMSGPR_ALERT("%s: is already configured to %d mtu\n",
		       dev->name, new_mtu);
		return 0;
	}

/*for version >= 4.10, kernel will check for valid range before calling ndo_change_mtu*/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 10, 0))
	min_mtu = MIN_SUPPORTED_MTU;
	if ((priv->plat->enh_desc) || (priv->synopsys_id >= DWMAC_CORE_4_00))
		max_mtu = JUMBO_LEN;
	else
		max_mtu = SKB_MAX_HEAD(NET_SKB_PAD + NET_IP_ALIGN);
	/* Will not overwrite ndev->max_mtu if plat->maxmtu > ndev->max_mtu
	 * as well as plat->maxmtu < ndev->min_mtu which is a invalid range.
	 */
	if ((priv->plat->maxmtu < max_mtu) &&
	    (priv->plat->maxmtu >= min_mtu))
		max_mtu = priv->plat->maxmtu;
	else if (priv->plat->maxmtu < min_mtu)
		dev_warn(priv->device,
			 "%s: warning: maxmtu having invalid value (%d)\n",
			 __func__, priv->plat->maxmtu);



	/* MTU must be positive, and in range */
	if (new_mtu < 0 || new_mtu < min_mtu) {
		net_err_ratelimited("%s: Invalid MTU %d requested, hw min %d\n",
				    dev->name, new_mtu, min_mtu);
		return -EINVAL;
	}

	if (max_mtu > 0 && new_mtu > max_mtu) {
		net_err_ratelimited("%s: Invalid MTU %d requested, hw max %d\n",
				    dev->name, new_mtu, max_mtu);
		return -EINVAL;
	}

#endif

	/* Supported frame sizes */
	if ((new_mtu < MIN_SUPPORTED_MTU) ||
	    (max_frame > MAX_SUPPORTED_MTU)) {
		NMSGPR_ALERT(
		       "%s: invalid MTU, min %d and max %d MTU are supported\n",
		       dev->name, MIN_SUPPORTED_MTU,(MAX_SUPPORTED_MTU - (ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN)));
		return -EINVAL;
	}

	NMSGPR_ALERT("changing MTU from %d to %d\n", dev->mtu, new_mtu);
	
	dev->mtu = new_mtu;

	netdev_update_features(dev);

	DBGPR_FUNC("<--tc9562mac_change_mtu\n");

	return 0;
}

static netdev_features_t tc9562mac_fix_features(struct net_device *dev,
					     netdev_features_t features)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);

	DBGPR_FUNC("-->tc9562mac_fix_features\n");

	if (priv->plat->rx_coe == TC9562MAC_RX_COE_NONE)
		features &= ~NETIF_F_RXCSUM;

	if (!priv->plat->tx_coe) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0))	
		features &= ~NETIF_F_CSUM_MASK;
#else
        features &= ~NETIF_F_ALL_CSUM;
#endif
	}

	/* Some GMAC devices have a bugged Jumbo frame support that
	 * needs to have the Tx COE disabled for oversized frames
	 * (due to limited buffer sizes). In this case we disable
	 * the TX csum insertion in the TDES and not use SF.
	 */
	if (priv->plat->bugged_jumbo && (dev->mtu > ETH_DATA_LEN)) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0))	
        features &= ~NETIF_F_CSUM_MASK;
#else
        features &= ~NETIF_F_ALL_CSUM;
#endif
	}

	/* Disable tso if asked by ethtool */
	if ((priv->plat->tso_en) && (priv->dma_cap.tsoen)) {
		if (features & NETIF_F_TSO)
			priv->tso = true;
		else
			priv->tso = false;
	}

	DBGPR_FUNC("<--tc9562mac_fix_features\n");

	return features;
}

static int tc9562mac_set_features(struct net_device *netdev,
			       netdev_features_t features)
{
	struct tc9562mac_priv *priv = netdev_priv(netdev);

	DBGPR_FUNC("-->tc9562mac_set_features\n");

	/* Keep the COE Type in case of csum is supporting */
	if (features & NETIF_F_RXCSUM)
		priv->hw->rx_csum = priv->plat->rx_coe;
	else
		priv->hw->rx_csum = 0;
	/* No check needed because rx_coe has been set before and it will be
	 * fixed in case of issue.
	 */
	priv->hw->mac->rx_ipc(priv->hw);
	if ((features & NETIF_F_IP_CSUM) || (features & NETIF_F_IPV6_CSUM))
		priv->csum_insertion = 1;
	else
		priv->csum_insertion = 0;
	
	DBGPR_FUNC("<--tc9562mac_set_features\n");

	return 0;
}

#ifdef TC9562_POLLING_METHOD
irqreturn_t tc9562mac_interrupt_dummy(int irq, void *dev_id)
{
	printk("Dummy ISR called\n");
	return IRQ_HANDLED;
}
#endif

/**
 *  tc9562mac_interrupt - main ISR
 *  @irq: interrupt number.
 *  @dev_id: to pass the net device pointer.
 *  Description: this is the main driver interrupt service routine.
 *  It can call:
 *  o DMA service routine (to manage incoming frame reception and transmission
 *    status)
 *  o Core interrupts to manage: remote wake-up, management counter, LPI
 *    interrupts.
 */
static irqreturn_t tc9562mac_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct tc9562mac_priv *priv = netdev_priv(dev);
	u32 rx_cnt = priv->plat->rx_queues_to_use + 1;
	/* u32 tx_cnt = priv->plat->tx_queues_to_use; */
	u32 queues_count;
	int mtl_status;
	int status;
	u32 queue;
	u32 tc9562_intc_intstatus;
	unsigned long mmc_irq_flags;
	struct tc9562mac_rx_queue *rx_q;
#ifdef UNIFIED_DRIVER
	int rd_val;
#endif

	DBGPR_FUNC("-->tc9562mac_interrupt\n");
	/* queues_count = (rx_cnt > tx_cnt) ? rx_cnt : tx_cnt; */
	queues_count = rx_cnt;
	if (priv->irq_wake)
		pm_wakeup_event(priv->device, 0);

	if (unlikely(!dev)) {
		netdev_err(priv->dev, "%s: invalid dev pointer\n", __func__);
		return IRQ_NONE;
	}
#ifdef TC9562_DEFINED
	/* Check if adapter is up */
	if (test_bit(TC9562MAC_DOWN, &priv->state)){
		printk("TC9562 down \n");
		return IRQ_HANDLED;
	}
#endif
	/* To handle safety features interrupts */
	if (tc9562mac_safety_feat_interrupt(priv)) {
		netif_carrier_off(dev);
		return IRQ_HANDLED;
	}

    /*MCGR is not applicable for TC9562, so commented the same*/
#ifndef TC9562_DEFINED
	/* To handle MCGR interrupts */
	tc9562mac_mcgr_interrupt(priv);
#endif

	/* To handle GMAC own interrupts */
	if ((priv->plat->has_gmac) || (priv->plat->has_gmac4)) {
		status = priv->hw->mac->host_irq_status(priv->hw,
							    &priv->xstats);

		if (unlikely(status)) {
			/* For LPI we need to save the tx status */
			if (status & CORE_IRQ_TX_PATH_IN_LPI_MODE)
				priv->tx_path_in_lpi_mode = true;
			if (status & CORE_IRQ_TX_PATH_EXIT_LPI_MODE)
				priv->tx_path_in_lpi_mode = false;
		}

		if (priv->synopsys_id >= DWMAC_CORE_4_00) {
			for (queue = 0; queue < queues_count; queue++) {
#ifdef UNIFIED_DRIVER
	/* Host Tx - 0, 4, 5, Host Rx - 0, 4. TxCh5 needs to be handled by napi. 
	Both Tx and Rx are handled in NAPI. so handle the Host related channels in NAPI function */
			if(priv->plat->rx_dma_ch_for_host[queue] != 1)
				continue;
#endif
				rx_q =
				&priv->rx_queue[queue];

				mtl_status = status |
				priv->hw->mac->host_mtl_irq_status(priv->hw,
								   queue);

				if (mtl_status & CORE_IRQ_MTL_RX_OVERFLOW &&
				    priv->hw->dma->set_rx_tail_ptr)
					priv->hw->dma->set_rx_tail_ptr(priv->ioaddr,
								rx_q->rx_tail_addr,
								queue);
			}
		}

		/* PCS link status */
		if (priv->hw->pcs) {
			if (priv->xstats.pcs_link)
				netif_carrier_on(dev);
			else
				netif_carrier_off(dev);
		}
	}

    tc9562_intc_intstatus = readl(priv->ioaddr + INTC_INTSTATUS);
	/* To handle DMA interrupts */
	tc9562mac_dma_interrupt(priv);
	
#ifdef UNIFIED_DRIVER

	rd_val = readl(priv->ioaddr + 0xf054);
	
    if(rd_val) 
	{
#ifdef UNIFIED_DRIVER_TEST_DBG2
        {
            static unsigned int test = 0, s, ns;
            
            if(test < 20)
            {
                ns = readl(priv->ioaddr + 0xab0c);
                s = readl(priv->ioaddr + 0xab08);

	            DBGPR_UNIFIED_2("C: %d\n ns: %d\n s: %d\n", test++);
            }
      	}
#endif
		/* Clear SW MSI mask for vector 0 */
		DBGPR_TEST("Clear SW MSI mask\n");
		writel( 0x1, priv->ioaddr + 0xf054); /* SW_MSI_CLR, Clear Software MSI (internal interrupt24).*/
	}
#endif

#ifndef UNIFIED_DRIVER
	/* Handle MAC Events */
	if(tc9562_intc_intstatus & INTC_INTSTS_MAC_EVENT) {
	    /* If any of the MMC Interrupt occured, update all values */
	    if( (priv->mmcaddr + MMC_RX_INTR) || (priv->mmcaddr + MMC_TX_INTR) 
	        || (priv->mmcaddr + MMC_RX_IPC_INTR) || (priv->mmcaddr + MMC_FPE_TX_INTR) || (priv->mmcaddr + MMC_FPE_RX_INTR) ) {
#endif
            /* To Reduce the frequency of Shared MAC Event Interrupt, Read-Clear MMC Register values at every interrupt*/
	        if (priv->dma_cap.rmon) {
	            spin_lock_irqsave(&priv->mmc_lock, mmc_irq_flags);
    			dwmac_mmc_read(priv->mmcaddr, &priv->mmc);
    			spin_unlock_irqrestore(&priv->mmc_lock, mmc_irq_flags);
			}
#ifndef UNIFIED_DRIVER
	    }
	    
	}
#endif
    	
#ifdef TC9562_MSIGEN_VERIFICATION

#ifdef TC9562_MSI_GEN_SW_AGENT /*Code under macro is used when SW is used to generate MSI interrupt (bit[24])*/
    {
    
    	int rd_val;
    	rd_val = readl(priv->ioaddr + 0xf054);

        if(1 == rd_val) {
        	/* Clear SW MSI mask for vector 0 */
        	DBGPR_TEST("Clear SW MSI mask\n");
        	writel( 0x1, priv->ioaddr + 0xf054); /* SW_MSI_CLR, Clear Software MSI (internal interrupt24).*/
        }

        /* It is required to enable interrupt and mask all vectors other than vector 0 */
        rd_val = readl(priv->ioaddr + INTMCUMASK1);
        rd_val &= 0xc0000000;
        rd_val |= 0xFF0004FF;//RX DMA interrupts for channel 5,6,7 not enabled.
        writel(rd_val, priv->ioaddr + INTMCUMASK1);

        rd_val = readl(priv->ioaddr + INTMCUMASK2);
        rd_val = 0xFFC17A00;
        writel(rd_val , priv->ioaddr + INTMCUMASK2);

        writel(0xfffffffe, priv->ioaddr + 0xf008); // MSI_MASK_SET: mask all vectors other than vector 0
        writel(0x00000001, priv->ioaddr + 0xf00c); // MSI_MASK_CLR: unmask vector 0
    }
#else
	/* Clear MSI mask for vector 0 */
	DBGPR_TEST("Clear MSI mask\n");
	writel( 0x1, priv->ioaddr + 0xf00c); /* MSI_MSK_CLR, unmask vector 0 */

#endif
    
#else
    {
    	int rd_val;    
    	/* Enable interrupt  */
    	rd_val = readl(priv->ioaddr + INTMCUMASK1);
    	rd_val &= 0xc0000000;
#ifndef UNIFIED_DRIVER
    	rd_val |= 0xFF0000FF;//RX DMA interrupts for channel 5,6,7 not enabled.
#else
		rd_val |= 0xFF7074FF; /* LPI, PMT, Tx - 0,4,5, Rx - 0,4 are enabled */
#endif
    	writel(rd_val, priv->ioaddr + INTMCUMASK1);

    	rd_val = readl(priv->ioaddr + INTMCUMASK2);
    	rd_val = 0xFFC17A00;
    	writel(rd_val , priv->ioaddr + INTMCUMASK2);
    }
#endif
	DBGPR_FUNC("<--tc9562mac_interrupt\n");
	return IRQ_HANDLED;
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
#ifdef CONFIG_NET_POLL_CONTROLLER
/* Polling receive - used by NETCONSOLE and other diagnostic tools
 * to allow network I/O with interrupts disabled.
 */
static void tc9562mac_poll_controller(struct net_device *dev)
{
	disable_irq(dev->irq);
	tc9562mac_interrupt(dev->irq, dev);
	enable_irq(dev->irq);
}
#endif
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

static int tc9562mac_ioctl_get_qmode(struct tc9562mac_priv *priv, void __user *data)
{
	u32 tx_qcount = priv->plat->tx_queues_to_use;
	struct tc9562mac_ioctl_qmode_cfg mode;
	u8 qmode;

	DBGPR_FUNC("-->tc9562mac_ioctl_get_qmode\n");

	if (copy_from_user(&mode, data, sizeof(mode)))
		return -EFAULT;
	if (mode.queue_idx >= tx_qcount)
		return -EINVAL;

	qmode = priv->plat->tx_queues_cfg[mode.queue_idx].mode_to_use;
	switch (qmode) {
	case MTL_QUEUE_DCB:
		mode.queue_mode = TC9562MAC_IOCTL_QMODE_DCB;
		break;
	case MTL_QUEUE_AVB:
		mode.queue_mode = TC9562MAC_IOCTL_QMODE_AVB;
		break;
	default:
		return -EINVAL;
	}

	if (copy_to_user(data, &mode, sizeof(mode)))
		return -EFAULT;

	DBGPR_FUNC("<--tc9562mac_ioctl_get_qmode\n");
	return 0;
}

static int tc9562mac_ioctl_set_qmode(struct tc9562mac_priv *priv, void __user *data)
{
	u32 tx_qcount = priv->plat->tx_queues_to_use;
	int txfifosz = priv->plat->tx_fifo_size;
	struct tc9562mac_ioctl_qmode_cfg mode;
	u32 txmode = 0;
	u8 qmode;

	DBGPR_FUNC("-->tc9562mac_ioctl_set_qmode\n");

	if (txfifosz == 0)
		txfifosz = priv->dma_cap.tx_fifo_size;

	/* Adjust for real per queue fifo size */
	txfifosz /= tx_qcount;

	if (copy_from_user(&mode, data, sizeof(mode)))
		return -EFAULT;
    
	if (mode.queue_idx >= tx_qcount)
		return -EINVAL;
    /* Configuration AVB for Q0 not allowed */
	if ((0 == mode.queue_idx) && (TC9562MAC_IOCTL_QMODE_AVB == mode.queue_mode))
		return -EINVAL;
	if (!priv->hw->dma->dma_tx_mode)
		return -EINVAL;
	if (!(priv->synopsys_id >= DWMAC_CORE_4_00))
		return -EINVAL;
	if (priv->plat->force_thresh_dma_mode) {
		txmode = tc;
	} else if (priv->plat->force_sf_dma_mode || priv->plat->tx_coe) {
		txmode = SF_DMA_MODE;
	} else {
		txmode = tc;
	}

	switch (mode.queue_mode) {
	case TC9562MAC_IOCTL_QMODE_DCB:
		qmode = MTL_QUEUE_DCB;
		break;
	case TC9562MAC_IOCTL_QMODE_AVB:
		qmode = MTL_QUEUE_AVB;
		break;
	default:
		return -EINVAL;
	}

#ifdef TC9562_DEFINED
			switch (mode.queue_idx) {
			case 0: /* 1536 *2 = 3072  B*/
				txfifosz = 3072;
				break;
			case 1: /* 512 *2 = 1024  B*/
				txfifosz = 1024;
				break;
			case 2: /* 512 *2 = 1024  B*/
				txfifosz = 1024;
				break;
			case 3: /* 1536 *2 = 3072  B*/
				txfifosz = 3072;
				break;
			case 4: /* 1536 *2 = 3072  B*/
				txfifosz = 3072;
				break;
			case 5: /* 1536 *2 = 3072  B*/
				txfifosz = 3072;
				break;
			}
#endif
	priv->plat->tx_queues_cfg[mode.queue_idx].mode_to_use = qmode;
	priv->hw->dma->dma_tx_mode(priv->ioaddr, txmode, mode.queue_idx,
			txfifosz, qmode);

	DBGPR_FUNC("<--tc9562mac_ioctl_set_qmode\n");
	
	return 0;
}

static int tc9562mac_ioctl_get_cbs(struct tc9562mac_priv *priv, void __user *data)
{
	u32 tx_qcount = priv->plat->tx_queues_to_use;
	struct tc9562mac_ioctl_cbs_cfg cbs;

	DBGPR_FUNC("-->tc9562mac_ioctl_get_cbs\n");

	if (copy_from_user(&cbs, data, sizeof(cbs)))
		return -EFAULT;
	if (cbs.queue_idx >= tx_qcount)
		return -EINVAL;

    cbs.speed100cfg.send_slope = priv->cbs_speed100_cfg[cbs.queue_idx].send_slope;
    cbs.speed100cfg.idle_slope = priv->cbs_speed100_cfg[cbs.queue_idx].idle_slope;
    cbs.speed100cfg.high_credit = priv->cbs_speed100_cfg[cbs.queue_idx].high_credit;
    cbs.speed100cfg.low_credit = priv->cbs_speed100_cfg[cbs.queue_idx].low_credit;
    cbs.speed100cfg.percentage = priv->cbs_speed100_cfg[cbs.queue_idx].percentage;
	
    cbs.speed1000cfg.send_slope = priv->cbs_speed1000_cfg[cbs.queue_idx].send_slope;   
    cbs.speed1000cfg.idle_slope = priv->cbs_speed1000_cfg[cbs.queue_idx].idle_slope;
    cbs.speed1000cfg.high_credit = priv->cbs_speed1000_cfg[cbs.queue_idx].high_credit;
    cbs.speed1000cfg.low_credit = priv->cbs_speed1000_cfg[cbs.queue_idx].low_credit;
    cbs.speed1000cfg.percentage = priv->cbs_speed1000_cfg[cbs.queue_idx].percentage;

	if (copy_to_user(data, &cbs, sizeof(cbs)))
		return -EFAULT;

	DBGPR_FUNC("<--tc9562mac_ioctl_get_cbs\n");
	
	return 0;
}

static int tc9562mac_ioctl_set_cbs(struct tc9562mac_priv *priv, void __user *data)
{
	u32 tx_qcount = priv->plat->tx_queues_to_use;
	struct tc9562mac_ioctl_cbs_cfg cbs;
	u8 qmode;

	DBGPR_FUNC("-->tc9562mac_ioctl_set_cbs\n");

	if (copy_from_user(&cbs, data, sizeof(cbs)))
	{
		return -EFAULT;
	}
    
    /* queue 0 is reserved for legacy traffic; cbs configuration not allowed (registers also not available for Q0)*/    
	if ((cbs.queue_idx >= tx_qcount) || (0 == cbs.queue_idx))
	{
		return -EINVAL;
	}
	if (!priv->hw->mac->config_cbs)
	{
		return -EINVAL;
	}

	qmode = priv->plat->tx_queues_cfg[cbs.queue_idx].mode_to_use;
	if (qmode != MTL_QUEUE_AVB)
	{
		return -EINVAL;
	}
    

    priv->cbs_speed100_cfg[cbs.queue_idx].send_slope = cbs.speed100cfg.send_slope;
    priv->cbs_speed100_cfg[cbs.queue_idx].idle_slope = cbs.speed100cfg.idle_slope;
    priv->cbs_speed100_cfg[cbs.queue_idx].high_credit = cbs.speed100cfg.high_credit;
    priv->cbs_speed100_cfg[cbs.queue_idx].low_credit = cbs.speed100cfg.low_credit;
    priv->cbs_speed100_cfg[cbs.queue_idx].percentage = cbs.speed100cfg.percentage;
	
    priv->cbs_speed1000_cfg[cbs.queue_idx].send_slope = cbs.speed1000cfg.send_slope;   
    priv->cbs_speed1000_cfg[cbs.queue_idx].idle_slope = cbs.speed1000cfg.idle_slope;
    priv->cbs_speed1000_cfg[cbs.queue_idx].high_credit = cbs.speed1000cfg.high_credit;
    priv->cbs_speed1000_cfg[cbs.queue_idx].low_credit = cbs.speed1000cfg.low_credit;
    priv->cbs_speed1000_cfg[cbs.queue_idx].percentage = cbs.speed1000cfg.percentage;

    priv->cbs_cfg_status[cbs.queue_idx] = 1; /* Flag as configured */
    
    if(priv->speed == SPEED_100) {
        priv->plat->tx_queues_cfg[cbs.queue_idx].send_slope = priv->cbs_speed100_cfg[cbs.queue_idx].send_slope;
	    priv->plat->tx_queues_cfg[cbs.queue_idx].idle_slope = priv->cbs_speed100_cfg[cbs.queue_idx].idle_slope;
	    priv->plat->tx_queues_cfg[cbs.queue_idx].high_credit = priv->cbs_speed100_cfg[cbs.queue_idx].high_credit;
	    priv->plat->tx_queues_cfg[cbs.queue_idx].low_credit = priv->cbs_speed100_cfg[cbs.queue_idx].low_credit;
	    priv->plat->tx_queues_cfg[cbs.queue_idx].percentage = priv->cbs_speed100_cfg[cbs.queue_idx].percentage;
    }
    else if(priv->speed == SPEED_1000) {
        priv->plat->tx_queues_cfg[cbs.queue_idx].send_slope = priv->cbs_speed1000_cfg[cbs.queue_idx].send_slope;
	    priv->plat->tx_queues_cfg[cbs.queue_idx].idle_slope = priv->cbs_speed1000_cfg[cbs.queue_idx].idle_slope;
	    priv->plat->tx_queues_cfg[cbs.queue_idx].high_credit = priv->cbs_speed1000_cfg[cbs.queue_idx].high_credit;
	    priv->plat->tx_queues_cfg[cbs.queue_idx].low_credit = priv->cbs_speed1000_cfg[cbs.queue_idx].low_credit;
	    priv->plat->tx_queues_cfg[cbs.queue_idx].percentage = priv->cbs_speed1000_cfg[cbs.queue_idx].percentage;
    }

	priv->hw->mac->config_cbs(priv->hw, priv->plat->tx_queues_cfg[cbs.queue_idx].send_slope, priv->plat->tx_queues_cfg[cbs.queue_idx].idle_slope,
			priv->plat->tx_queues_cfg[cbs.queue_idx].high_credit, priv->plat->tx_queues_cfg[cbs.queue_idx].low_credit, cbs.queue_idx);

	DBGPR_FUNC("<--tc9562mac_ioctl_set_cbs\n");
	
	return 0;
}

static int tc9562mac_ioctl_get_est(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562mac_ioctl_est_cfg *est;
	int ret = 0;

	DBGPR_FUNC("-->tc9562mac_ioctl_get_est\n");

	est = kzalloc(sizeof(*est), GFP_KERNEL);
	if (!est)
		return -ENOMEM;

	est->enabled = priv->plat->est_cfg.enable;
	est->estwid = priv->dma_cap.estwid;
	est->estdep = priv->dma_cap.estdep;
	est->btr_offset[0] = priv->plat->est_cfg.btr_offset[0];
	est->btr_offset[1] = priv->plat->est_cfg.btr_offset[1];
	est->ctr[0] = priv->plat->est_cfg.ctr[0];
	est->ctr[1] = priv->plat->est_cfg.ctr[1];
	est->ter = priv->plat->est_cfg.ter;
	est->gcl_size = priv->plat->est_cfg.gcl_size;
	memcpy(est->gcl, priv->plat->est_cfg.gcl,
			est->gcl_size * sizeof(*est->gcl));
	if (copy_to_user(data, est, sizeof(*est))) {
		ret = -EFAULT;
		goto out_free;
	}

	DBGPR_FUNC("<--tc9562mac_ioctl_get_est\n");

out_free:
	kfree(est);
	return ret;
}

static int tc9562mac_ioctl_set_est(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562mac_est_cfg *cfg = &priv->plat->est_cfg;
	struct tc9562mac_ioctl_est_cfg *est;
	int ret = 0;

	DBGPR_FUNC("-->tc9562mac_ioctl_set_est\n");

	est = kzalloc(sizeof(*est), GFP_KERNEL);
	if (!est)
		return -ENOMEM;

	if (copy_from_user(est, data, sizeof(*est))) {
		ret = -EFAULT;
		goto out_free;
	}
	if (est->gcl_size > TC9562MAC_EST_GCL_MAX_ENTRIES) {
		ret = -EINVAL;
		goto out_free;
	}

	if (est->enabled) {
		cfg->btr_offset[0] = est->btr_offset[0];
		cfg->btr_offset[1] = est->btr_offset[1];
		cfg->ctr[0] = est->ctr[0];
		cfg->ctr[1] = est->ctr[1];
		cfg->ter = est->ter;
		cfg->gcl_size = est->gcl_size;
		memcpy(cfg->gcl, est->gcl, cfg->gcl_size * sizeof(*cfg->gcl));
	} else {
		cfg->btr_offset[0] = 0;
		cfg->btr_offset[1] = 0;
		cfg->ctr[0] = 0;
		cfg->ctr[1] = 0;
		cfg->ter = 0;
		cfg->gcl_size = 0;
		memset(cfg->gcl, 0, sizeof(cfg->gcl));
	}

	cfg->enable = est->enabled;
	//ret = tc9562mac_est_configuration(priv);
	ret = tc9562mac_est_configuration_ioctl(priv);   
	if (!est->enabled)
		ret = 0;

	DBGPR_FUNC("<--tc9562mac_ioctl_set_est\n");
	
out_free:
	kfree(est);
	return ret;
}
static int tc9562mac_ioctl_get_fpe(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562mac_ioctl_fpe_cfg *fpe;
	int ret = 0;
	unsigned int control = 0;
	
	fpe = kzalloc(sizeof(*fpe), GFP_KERNEL);
	
	if(!fpe)
		return -ENOMEM;
	
	control = readl(priv->ioaddr + GMAC_FPE_CTRL_STS);
	
	if(control & GMAC_FPE_EFPE)
	{
		fpe->enabled = 1;
		control = readl(priv->ioaddr + MTL_FPE_CTRL_STS);
		fpe->pec = (control & MTL_FPE_PEC_MASK) >> MTL_FPE_PEC_SHIFT;
		fpe->afsz = control & MTL_FPE_AFSZ_MASK;
		control = readl(priv->ioaddr + MTL_FPE_ADVANCE);
		fpe->HA_time =  (control & MTL_FPE_HOLD_ADVANCE_MASK);
		fpe->RA_time =  (control & MTL_FPE_RELEASE_ADVANCE_MASK) >> MTL_FPE_ADVANCE_SHIFT;
	}
	else
	{
		fpe->enabled = 0;
		fpe->pec = 0;
		fpe->afsz = 0;
		fpe->HA_time = 0;
		fpe->RA_time = 0;
	}
	if(copy_to_user(data, fpe, sizeof(*fpe)))
	{
		ret = -EFAULT;
		goto out_free;
	}
out_free:
		kfree(fpe);
		return ret;
}
static int tc9562mac_ioctl_set_fpe(struct tc9562mac_priv *priv, void __user *data)
{
  struct tc9562mac_fpe_cfg *cfg = &priv->plat->fpe_cfg;
  struct tc9562mac_ioctl_fpe_cfg *fpe;
  int ret = 0;
  unsigned int control = 0;

  fpe = kzalloc(sizeof(*fpe), GFP_KERNEL);
  if(!fpe)
    return -ENOMEM;
  if(copy_from_user(fpe, data, sizeof(*fpe)))
  {
    ret = -EFAULT;
    goto out_free;
  }

  if(fpe->enabled)
  {
    cfg->enable = fpe->enabled;

    control = readl(priv->ioaddr + MTL_FPE_CTRL_STS);
    control &= ~(MTL_FPE_PEC_MASK | MTL_FPE_AFSZ_MASK);
    //control |= ((fpe->pec << MTL_FPE_PEC_SHIFT) & 0xff00) | fpe->afsz;
    control |= ((fpe->pec << MTL_FPE_PEC_SHIFT) & MTL_FPE_PEC_MASK) | (fpe->afsz & MTL_FPE_AFSZ_MASK);
    writel(control, priv->ioaddr + MTL_FPE_CTRL_STS);

    control = readl(priv->ioaddr + MTL_FPE_ADVANCE);
    control &= ~(MTL_FPE_HOLD_ADVANCE_MASK | MTL_FPE_RELEASE_ADVANCE_MASK);
    //control |= (fpe->RA_time << MTL_FPE_ADVANCE_SHIFT) | fpe->HA_time;
    control |= ((fpe->RA_time << MTL_FPE_ADVANCE_SHIFT) & MTL_FPE_RELEASE_ADVANCE_MASK) | (fpe->HA_time & MTL_FPE_HOLD_ADVANCE_MASK);
    writel(control, priv->ioaddr + MTL_FPE_ADVANCE);

    cfg->pec_cfg = fpe->pec;
    cfg->afsz_cfg = fpe->afsz;
    cfg->HA_time = fpe->HA_time;
    cfg->RA_time = fpe->RA_time;

    control = readl(priv->ioaddr + GMAC_FPE_CTRL_STS);                                                    
    control |= GMAC_FPE_EFPE;
    writel(control, priv->ioaddr + GMAC_FPE_CTRL_STS);
  }
  else 
  {
    control = readl(priv->ioaddr + GMAC_FPE_CTRL_STS);                                                    
    control &= ~GMAC_FPE_EFPE;
    writel(control, priv->ioaddr + GMAC_FPE_CTRL_STS);

    control = readl(priv->ioaddr + MTL_FPE_ADVANCE);
    control &= ~(MTL_FPE_HOLD_ADVANCE_MASK | MTL_FPE_RELEASE_ADVANCE_MASK);
    writel(control, priv->ioaddr + MTL_FPE_ADVANCE);

    control = readl(priv->ioaddr + MTL_FPE_CTRL_STS);
    control &= ~(MTL_FPE_PEC_MASK | MTL_FPE_AFSZ_MASK);
    writel(control, priv->ioaddr + MTL_FPE_CTRL_STS);
  }

out_free:
  kfree(fpe);
  return ret;
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static int tc9562mac_ioctl_get_ecc(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562mac_ioctl_ecc_cfg ecc;

	DBGPR_FUNC("-->tc9562mac_ioctl_get_ecc\n");

	memset(&ecc, 0, sizeof(ecc));
	ecc.supported = priv->dma_cap.asp > 0;
	ecc.enabled = priv->ecc_enabled;
	if (ecc.enabled) {
		ecc.err_inject_supported = true;
		ecc.err_inject = priv->ecc_err_inject;
		ecc.err_where = priv->ecc_err_where;
		ecc.err_correctable = priv->ecc_err_correctable;
	}

	if (copy_to_user(data, &ecc, sizeof(ecc)))
		return -EFAULT;

	DBGPR_FUNC("<--tc9562mac_ioctl_get_ecc\n");

	return 0;
}

static int tc9562mac_ioctl_set_ecc(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562mac_ioctl_ecc_cfg ecc;

	DBGPR_FUNC("-->tc9562mac_ioctl_set_ecc\n");

	if (copy_from_user(&ecc, data, sizeof(ecc)))
		return -EFAULT;
	if (!priv->dma_cap.asp)
		return -EOPNOTSUPP;

	priv->ecc_enabled = ecc.enabled;
	priv->ecc_err_inject = ecc.err_inject;
	priv->ecc_err_where = ecc.err_where;
	priv->ecc_err_correctable = ecc.err_correctable;

	tc9562mac_update_safety_feat(priv);

	DBGPR_FUNC("<--tc9562mac_ioctl_set_ecc\n");
	
	return 0;
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

static int tc9562mac_ioctl_get_rxp(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562mac_rx_parser_cfg *cfg = &priv->plat->rxp_cfg;
	struct tc9562mac_ioctl_rxp_cfg *rxp;
	int ret = 0;

	DBGPR_FUNC("-->tc9562mac_ioctl_get_rxp\n");

	rxp = kzalloc(sizeof(*rxp), GFP_KERNEL);
	if (!rxp)
		return -ENOMEM;

	rxp->enabled = priv->rxp_enabled;
	rxp->frpes = priv->dma_cap.frpes;
	rxp->nve = cfg->nve;
	rxp->npe = cfg->npe;
	memcpy(rxp->entries, cfg->entries, rxp->nve * sizeof(*cfg->entries));
	if (copy_to_user(data, rxp, sizeof(*rxp))) {
		ret = -EFAULT;
		goto out_free;
	}

	DBGPR_FUNC("<--tc9562mac_ioctl_get_rxp\n");

out_free:
	kfree(rxp);
	return ret;
}

static int tc9562mac_ioctl_set_rxp(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562mac_rx_parser_cfg *cfg = &priv->plat->rxp_cfg;
	struct tc9562mac_ioctl_rxp_cfg *rxp;
	int ret = 0;

	DBGPR_FUNC("-->tc9562mac_ioctl_set_rxp\n");

	rxp = kzalloc(sizeof(*rxp), GFP_KERNEL);
	if (!rxp)
		return -ENOMEM;

	if (copy_from_user(rxp, data, sizeof(*rxp))) {
		ret = -EFAULT;
		goto out_free;
	}
	if (rxp->nve > TC9562MAC_RX_PARSER_MAX_ENTRIES) {
		ret = -EINVAL;
		goto out_free;
	}

	if (rxp->enabled) {
		cfg->nve = rxp->nve;
		cfg->npe = rxp->npe;
		memcpy(cfg->entries, rxp->entries,
				cfg->nve * sizeof(*cfg->entries));
	} else {
		cfg->nve = 0;
		cfg->npe = 0;
		memset(cfg->entries, 0, sizeof(cfg->entries));
	}

	ret = tc9562mac_rx_parser_configuration(priv);
	if (!rxp->enabled)
		ret = 0;

	DBGPR_FUNC("<--tc9562mac_ioctl_set_rxp\n");
	
out_free:
	kfree(rxp);
	return ret;
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static int tc9562mac_ioctl_get_mcgr(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562mac_ioctl_mcgr_cfg mcgr;
	struct tc9562mac_mcgr_cfg *cfg;

	DBGPR_FUNC("-->tc9562mac_ioctl_get_mcgr\n");

	memset(&mcgr, 0, sizeof(mcgr));

	if (copy_from_user(&mcgr, data, sizeof(mcgr)))
		return -EFAULT;
	if (mcgr.index >= TC9562MAC_PPS_MAX_NUM)
		return -EINVAL;
	if (mcgr.index >= priv->dma_cap.ppsoutnum)
		return -EINVAL;

	cfg = &priv->plat->mcgr_cfg[mcgr.index];

	mcgr.enabled = cfg->enable;
	mcgr.ctrl = cfg->ctrl;
	mcgr.debug_route = cfg->debug_route;

	if (copy_to_user(data, &mcgr, sizeof(mcgr)))
		return -EFAULT;

	DBGPR_FUNC("<--tc9562mac_ioctl_get_mcgr\n");
	
	return 0;
}

static int tc9562mac_ioctl_set_mcgr(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562mac_ioctl_mcgr_cfg mcgr;
	struct tc9562mac_mcgr_cfg *cfg;

	DBGPR_FUNC("-->tc9562mac_ioctl_set_mcgr\n");

	memset(&mcgr, 0, sizeof(mcgr));

	if (copy_from_user(&mcgr, data, sizeof(mcgr)))
		return -EFAULT;
	if (mcgr.index >= TC9562MAC_PPS_MAX_NUM)
		return -EINVAL;
	if (mcgr.index >= priv->dma_cap.ppsoutnum)
		return -EINVAL;

	cfg = &priv->plat->mcgr_cfg[mcgr.index];

	cfg->enable = mcgr.enabled;
	cfg->ctrl = mcgr.ctrl;
	cfg->debug_route = mcgr.debug_route;

	DBGPR_FUNC("<--tc9562mac_ioctl_set_mcgr\n");

	return tc9562mac_mcgr_configuration(priv);
}

static int tc9562mac_ioctl_get_pps(struct tc9562mac_priv *priv, void __user *data)
{
	u32 ptp_period = priv->hw->ptp->get_ptp_period(priv->ptpaddr,
			priv->plat->clk_ptp_rate);
	struct tc9562mac_ioctl_pps_cfg pps;
	struct tc9562mac_pps_cfg *cfg;
	bool dig;

	DBGPR_FUNC("-->tc9562mac_ioctl_get_pps\n");

	memset(&pps, 0, sizeof(pps));

	if (copy_from_user(&pps, data, sizeof(pps)))
		return -EFAULT;
	if (pps.index >= TC9562MAC_PPS_MAX_NUM)
		return -EINVAL;
	if (pps.index >= priv->dma_cap.ppsoutnum)
		return -EINVAL;

	cfg = &priv->plat->pps_cfg[pps.index];
	dig = priv->hw->ptp->get_hw_tstamping(priv->ptpaddr) & PTP_TCR_TSCTRLSSR;

	pps.enabled = cfg->enable;
	pps.ctrl_cmd = cfg->ctrl_cmd;
	if (pps.enabled) {
		pps.trgtmodsel = cfg->trgtmodsel;
		pps.target_time[0] = cfg->target_time[0];
		pps.target_time[1] = cfg->target_time[1];
		pps.interval = cfg->interval * ptp_period;
		pps.width = cfg->width * ptp_period;
	} else {
		pps.freq = 0x1 << pps.ctrl_cmd;
		if (dig)
			pps.freq /= 2;
	}

	if (copy_to_user(data, &pps, sizeof(pps)))
		return -EFAULT;

	DBGPR_FUNC("<--tc9562mac_ioctl_get_pps\n");
	
	return 0;
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

static int tc9562_PTPCLK_Config(struct tc9562mac_priv *priv, void __user *data)
{
  struct tc9562_PPS_Config *tc9562_pps_cfg, tc9562_pps_cfg_data;
  int ret = 0;
  u32 value = 0;
  __u64 temp = 0;
  __u32 sec_inc;
  struct timespec64 now;	

	printk("tc9562_PTPCLK_Config... \n");
 	if (copy_from_user(&tc9562_pps_cfg_data, data, sizeof(struct tc9562_PPS_Config)))
        printk("copy_from_user error: ifr data structure\n");

 	tc9562_pps_cfg = (struct tc9562_PPS_Config *)(&tc9562_pps_cfg_data);

	if (tc9562_pps_cfg->ptpclk_freq > 250000000){
  	printk("PPS: PTPCLK_Config: freq=%dHz is too high. Cannot config it\n",
            tc9562_pps_cfg->ptpclk_freq );
		return -1;
	}
		

	//value = (PTP_TCR_TSENA | PTP_TCR_TSCFUPDT | PTP_TCR_TSCTRLSSR);
	//priv->hw->ptp->config_hw_tstamping(priv->ptpaddr, value);

	/* program Sub Second Increment reg */

	sec_inc = priv->hw->ptp->config_sub_second_increment(
		priv->ptpaddr, tc9562_pps_cfg->ptpclk_freq,
		priv->plat->has_gmac4);
	printk("sec_inc : %x , tc9562_pps_cfg->ptpclk_freq :%d\n", sec_inc, tc9562_pps_cfg->ptpclk_freq);
	temp = div_u64(1000000000ULL, sec_inc);
	temp = (u64)(temp << 32);
	priv->default_addend = div_u64(temp, TC9562_PTP_SYSCLOCK);

	printk("priv->default_addend : %x \n", priv->default_addend);
	priv->hw->ptp->config_addend(priv->ptpaddr,
				     priv->default_addend);
	
	value = readl(priv->ptpaddr + PTP_TCR);
	if(!(value & 0x00000001))
	{
		printk("tc9562_PTPCLK_Config : init systime \n");
		/* initialize system time */
		ktime_get_real_ts64(&now);
		priv->hw->ptp->init_systime(priv->ptpaddr, (u32)now.tv_sec, now.tv_nsec);
		value |= PTP_TCR_TSINIT | PTP_TCR_TSENA | PTP_TCR_TSCFUPDT | PTP_TCR_TSCTRLSSR;
		priv->hw->ptp->config_hw_tstamping(priv->ptpaddr, value);
	}

  		return ret;
}


int tc9562_PPSOUT_Config_real(struct tc9562mac_priv *priv, 
                           struct tc9562_PPS_Config *tc9562_pps_cfg);
/*!
 * \brief This function confiures the PPS output. 
 * \param[in] pdata : pointer to private data structure.
 * \param[in] req : pointer to ioctl structure.
 * 
 * \retval 0: Success, -1 : Failure
 * */
static int tc9562_PPSOUT_Config(struct tc9562mac_priv *priv, void __user *data)
{
  struct tc9562_PPS_Config *tc9562_pps_cfg, tc9562_pps_cfg_data;

	printk("tc9562_PPSOUT_Config... \n");

  if (copy_from_user(&tc9562_pps_cfg_data, data, sizeof(struct tc9562_PPS_Config))){
        printk("copy_from_user error: ifr data structure\n");
  }

        tc9562_pps_cfg = (struct tc9562_PPS_Config *)(&tc9562_pps_cfg_data);

	if(tc9562_pps_cfg->ppsout_duty <= 0) {
		printk("PPS: PPSOut_Config: duty cycle is invalid. Using duty=1\n"); 
		tc9562_pps_cfg->ppsout_duty = 1;
	}
	else if(tc9562_pps_cfg->ppsout_duty >= 100) {
		printk("PPS: PPSOut_Config: duty cycle is invalid. Using duty=99\n"); 
		tc9562_pps_cfg->ppsout_duty = 99;
	}

	if((tc9562_pps_cfg->ppsout_ch != 0) && (tc9562_pps_cfg->ppsout_ch != 1)) {
		printk("pps channel %d not supported by GPIO\n", tc9562_pps_cfg->ppsout_ch);		
		return -1;
	}		
	return( tc9562_PPSOUT_Config_real(priv, tc9562_pps_cfg));
}

int tc9562_PPSOUT_Config_real(struct tc9562mac_priv *priv, 
                           struct tc9562_PPS_Config *tc9562_pps_cfg)
{
  unsigned int s, ss, val, align_ns = 0;
  
#ifndef UNIFIED_DRIVER
	int value, interval, width, ppscmd, trgtmodsel = 0x3;
  value = readl(priv->ptpaddr + PTP_TCR);
  if(!(value & 0x00000001))
	{
		tc9562_ptp_configuration(priv);
		printk("tc9562_PPSOUT_Config_real : ptp configuration");	
	}
#else
	int interval, width, ppscmd, trgtmodsel = 0x3;
#endif

  interval = (tc9562_pps_cfg->ptpclk_freq + tc9562_pps_cfg->ppsout_freq/2) 
                                       / tc9562_pps_cfg->ppsout_freq;

  width = ((interval * tc9562_pps_cfg->ppsout_duty) + 50)/100 - 1;
  if (width >= interval) width = interval - 1;
  if (width < 0) width = 0;

  ppscmd = 0x3;//cancel start
  printk("interval: %d, width: %d \n", interval, width);

  if (tc9562_pps_cfg->ppsout_align == 1)
	{
 		printk("PPS: PPSOut_Config: freq=%dHz, ch=%d, duty=%d, align=%d\n",
            tc9562_pps_cfg->ppsout_freq,
            tc9562_pps_cfg->ppsout_ch,
            tc9562_pps_cfg->ppsout_duty,
            tc9562_pps_cfg->ppsout_align_ns);
	}
	else
	{
 		printk("PPS: PPSOut_Config: freq=%dHz, ch=%d, duty=%d, No alignment\n",
            tc9562_pps_cfg->ppsout_freq,
            tc9562_pps_cfg->ppsout_ch,
            tc9562_pps_cfg->ppsout_duty);
	}

  printk("   : with PTP Clock freq=%dHz\n", tc9562_pps_cfg->ptpclk_freq);

  if (tc9562_pps_cfg->ppsout_align == 1)
	{
		align_ns = tc9562_pps_cfg->ppsout_align_ns;
		printk("(1000000000/tc9562_pps_cfg->ppsout_freq) : %d, tc9562_pps_cfg->ppsout_align_ns: %d \n", (1000000000/tc9562_pps_cfg->ppsout_freq), tc9562_pps_cfg->ppsout_align_ns);
		if (align_ns < 32/*(1000000000/tc9562_pps_cfg->ppsout_freq)*/) // adjust 32ns sync
		{
			align_ns += (1000000000 - 32/*(1000000000/tc9562_pps_cfg->ppsout_freq)*/);
			printk("align_ns : %x \n", align_ns);
		}	
		else 
		{
			align_ns -= 32/*(1000000000/tc9562_pps_cfg->ppsout_freq)*/;
		}
	}

	writel((interval-1), priv->ioaddr + MAC_PPSx_INTERVAL(tc9562_pps_cfg->ppsout_ch));
	writel(width, priv->ioaddr + MAC_PPSx_WIDTH(tc9562_pps_cfg->ppsout_ch));

	if(0 == tc9562_pps_cfg->ppsout_ch)
	{
		val = readl(priv->ioaddr + 0x1528);
		val &= (~0x00000F00);//GPIO2
		val |= 0x00000100;
		writel(val, priv->ioaddr + 0x1528);
	}else if(1 == tc9562_pps_cfg->ppsout_ch)
	{
		val = readl(priv->ioaddr + 0x1528);
		val &= (~0x000F0000);//GPIO4
		val |= 0x00010000;
		writel(val, priv->ioaddr + 0x1528);
	}

	val = readl(priv->ioaddr + MAC_PPS_CONTROL);
	val |= (ppscmd << (tc9562_pps_cfg->ppsout_ch * 8)) & PPSCMDx(tc9562_pps_cfg->ppsout_ch);
	writel(val, priv->ioaddr + MAC_PPS_CONTROL);
	val &= ~(0x7 << (tc9562_pps_cfg->ppsout_ch * 8));
	ppscmd = 0x5;//stop pulse train immediately
	val |= (ppscmd << (tc9562_pps_cfg->ppsout_ch * 8)) ; // stop it 
	writel(val, priv->ioaddr + MAC_PPS_CONTROL);
	
        s = readl(priv->ioaddr + PTP_GMAC4_OFFSET + PTP_STSR);     			// PTP seconds
 	ss  = readl(priv->ioaddr + PTP_GMAC4_OFFSET + PTP_STNSR);     			// PTP subseconds
	if ( readl(priv->ioaddr + PTP_GMAC4_OFFSET + PTP_STSR) != s ) 		// second roll over
	{
    		s  = readl(priv->ioaddr + PTP_GMAC4_OFFSET + PTP_STSR);     		// PTP seconds
    		ss = readl(priv->ioaddr + PTP_GMAC4_OFFSET + PTP_STNSR);     		// PTP subseconds
	}
	printk("s: %x \n", s);
	printk("ss: %x \n", ss);
  	if (tc9562_pps_cfg->ppsout_align == 1)
		{
		ss += PPS_START_DELAY; 
		if (ss >= align_ns)
		{
			//s  += 1;
printk("s: %x \n", s);
		}
		s += 2;	
    	printk("align_ns: %x \n", align_ns);
		writel(s, priv->ioaddr + MAC_PPSx_TARGET_TIME_SEC(tc9562_pps_cfg->ppsout_ch));	//PPS target sec
		writel(align_ns, priv->ioaddr + MAC_PPSx_TARGET_TIME_NSEC(tc9562_pps_cfg->ppsout_ch)); //PPS target nsec
	
		}
		else 
		{
			ss += PPS_START_DELAY; 
			if (ss >= 1000000000)
			{
				ss -= 1000000000;
				s  += 1;
			}	
    						// set subsecond  
			writel(s, priv->ioaddr + MAC_PPSx_TARGET_TIME_SEC(tc9562_pps_cfg->ppsout_ch));	//PPS target sec
			writel(ss, priv->ioaddr + MAC_PPSx_TARGET_TIME_NSEC(tc9562_pps_cfg->ppsout_ch)); //PPS target nsec
		}

   	 val = readl(priv->ioaddr + MAC_PPS_CONTROL);
	 val &= ~GENMASK(((tc9562_pps_cfg->ppsout_ch + 1) * 8) - 1, tc9562_pps_cfg->ppsout_ch * 8);
	 ppscmd = 0x2;//start Pulse train
   	 val |= (ppscmd << (tc9562_pps_cfg->ppsout_ch * 8)) | 0x10;//PPSEN0
 	 val |= (trgtmodsel << ((tc9562_pps_cfg->ppsout_ch * 8) + 5)) & TRGTMODSELx(tc9562_pps_cfg->ppsout_ch);//pulse output only
    	 writel(val, priv->ioaddr + MAC_PPS_CONTROL);
	
	 val = readl(priv->ioaddr + MAC_PPS_CONTROL);
	   
#ifdef VERIFY_PPO_USING_AUX
	// Enable Aux_control (bit 4)
  val = readl(priv->ioaddr + MAC_AUX_CTRL);
  val |= (0x1 << 4);   //set bit 4
  val |= (0x1 << 0);   //set bit 0
  writel(val, priv->ioaddr + MAC_AUX_CTRL);	
  
  val = readl(priv->ioaddr + 0x1528);
  val |= (0x1 << 4);   //set bit 4 GPIO1 TRIG00
  writel(val, priv->ioaddr + 0x1528);

// enable the trig interrupt
  val = readl(priv->ioaddr + GMAC_INT_EN);
  val &= ~(0x1 << 5);   // clear bit 5 to disable LPI interrupt
  val |= (0x1 << 12);   //set bit 12req=50000000Hz
  writel(val, priv->ioaddr + GMAC_INT_EN);
#endif

  return 0;
}
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static int tc9562mac_ioctl_set_pps(struct tc9562mac_priv *priv, void __user *data)
{
	u32 ptp_period = priv->hw->ptp->get_ptp_period(priv->ptpaddr,
			priv->plat->clk_ptp_rate);
	struct tc9562mac_ioctl_pps_cfg pps;
	struct tc9562mac_pps_cfg *cfg;

	DBGPR_FUNC("-->tc9562mac_ioctl_set_pps\n");

	memset(&pps, 0, sizeof(pps));

	if (copy_from_user(&pps, data, sizeof(pps)))
		return -EFAULT;
	if (pps.index >= TC9562MAC_PPS_MAX_NUM)
		return -EINVAL;
	if (pps.index >= priv->dma_cap.ppsoutnum)
		return -EINVAL;

	cfg = &priv->plat->pps_cfg[pps.index];

	cfg->enable = pps.enabled;
	cfg->ctrl_cmd = pps.ctrl_cmd;
	cfg->trgtmodsel = pps.trgtmodsel;
	cfg->target_time[0] = pps.target_time[0];
	cfg->target_time[1] = pps.target_time[1];
	cfg->interval = pps.interval / ptp_period;
	cfg->width = pps.width / ptp_period;
	
	DBGPR_FUNC("<--tc9562mac_ioctl_set_pps\n");
	
	return tc9562mac_pps_configuration(priv, pps.index);
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
static int tc9562_ioctl_get_tx_free_desc(struct tc9562mac_priv *priv, void __user *data)
{
	u32 tx_free_desc = 0;
	struct tc9562_ioctl_free_desc ioctl_data;
		
	DBGPR_FUNC("-->tc9562_ioctl_get_tx_free_desc\n");
		
	if (copy_from_user(&ioctl_data, data, sizeof(ioctl_data)))
		return -EFAULT;
	if ((ioctl_data.queue_idx < priv->plat->tx_queues_to_use) && (ioctl_data.queue_idx != 1)) {
	
	    tx_free_desc = tc9562mac_tx_avail(priv, ioctl_data.queue_idx);
	    if (copy_to_user(ioctl_data.ptr ,&tx_free_desc, sizeof(unsigned int)))
		    return -EFAULT;
	}
	else {
	    NMSGPR_ALERT("Channel no %d is invalid\n", ioctl_data.queue_idx);
	    return -EFAULT;
	}
	DBGPR_FUNC("<--tc9562_ioctl_get_tx_free_desc\n");

	return 0;
}

static int tc9562_ioctl_get_connected_speed(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562_ioctl_speed ioctl_data;	
	memset(&ioctl_data, 0, sizeof(ioctl_data));

	DBGPR_FUNC("-->tc9562_ioctl_get_connected_speed\n");
	
	if (copy_from_user(&ioctl_data, data, sizeof(ioctl_data)))
			return -EFAULT;

	ioctl_data.connected_speed = priv->speed;

	if (copy_to_user(data, &ioctl_data, sizeof(ioctl_data)))
		return -EFAULT;

	DBGPR_FUNC("<--tc9562_ioctl_get_connected_speed\n");
	return 0;
}
static int tc9562_ioctl_set_mac_loopback(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562_ioctl_loopback ioctl_data;
	u32 value = 0;	
	
	DBGPR_FUNC("-->tc9562_ioctl_set_mac_loopback\n");
	
	if (copy_from_user(&ioctl_data, data, sizeof(ioctl_data)))
			return -EFAULT;


    if(priv->mac_loopback_mode == ioctl_data.flags) {
		NDBGPR_L1(" MAC loopback mode is already configured for the intended configuration\n");		
		return 0;
    }
	
	value = readl((void *)priv->dev->base_addr + GMAC_CONFIG);
	if(ioctl_data.flags) {
		value |= GMAC_CONFIG_LM;
	}
	else {
		value &= ~GMAC_CONFIG_LM;
	}

	priv->mac_loopback_mode = ioctl_data.flags;
     
	writel(value, (void *)priv->dev->base_addr + GMAC_CONFIG);
	
	DBGPR_FUNC("<--tc9562_ioctl_set_mac_loopback\n");
	return 0;
}

static int tc9562_ioctl_set_phy_loopback(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562_ioctl_loopback ioctl_data;
	int regval = 0;
	
	DBGPR_FUNC("-->tc9562_ioctl_set_phy_loopback\n");
	if (copy_from_user(&ioctl_data, data, sizeof(ioctl_data)))
			return -EFAULT;

    if(priv->phy_loopback_mode == ioctl_data.flags) {
		NDBGPR_L1(" PHY loopback mode is already configured for the intended configuration\n");		
		return 0;
    }

	priv->phy_loopback_mode = ioctl_data.flags;

	tc9562mac_mdio_read_direct(priv, priv->plat->phy_addr, MII_BMCR, &regval);
	regval = (regval & (~(1<<14))) | (ioctl_data.flags<<14);
	tc9562mac_mdio_write_direct(priv, priv->plat->phy_addr, MII_BMCR, regval);

	/* Shallow Loopback */
	tc9562mac_mdio_read_direct(priv, priv->plat->phy_addr, MII_AUX_CTRL, &regval);
	regval = (regval & (~(1<<5))) | (ioctl_data.flags<<5);
	tc9562mac_mdio_write_direct(priv, priv->plat->phy_addr, MII_AUX_CTRL, regval);

	DBGPR_FUNC("<--tc9562_ioctl_set_phy_loopback\n");	
	return 0;
}


static int tc9562_confing_l2_da_filter(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562_ioctl_l2_da_filter ioctl_data;
	int ret = 0;
	unsigned int reg_val =0;
	
	DBGPR_FUNC("-->tc9562_confing_l2_da_filter\n");
	if (copy_from_user(&ioctl_data, data, sizeof(ioctl_data)))
			return -EFAULT;

	if (ioctl_data.perfect_hash) {
		DBGPR_FUNC("--> HASH filternig mode \n");
		if (HASH_TABLE_SIZE > 0)
			priv->l2_filtering_mode = 1;
		else
			ret = -1 ;
	} else {
		DBGPR_FUNC("--> Perfect Filtering mode \n");
		priv->l2_filtering_mode = 0;
	}

//	priv->l2_filtering_mode = 1;


	/* configure the l2 filter matching mode */
	reg_val = readl(priv->ioaddr + GMAC_PACKET_FILTER);
	reg_val &= ~GMAC_PACKET_FILTER_DAIF;
	reg_val |= ((ioctl_data.perfect_inverse_match & 0x1) << GMAC_PACKET_FILTER_DAIF_LPOS); 
	writel(reg_val, priv->ioaddr + GMAC_PACKET_FILTER);

	DBGPR_FUNC("Successfully selected L2 %s filtering and %s DA matching\n",
	(ioctl_data.perfect_hash ? "HASH" : "PERFECT"),
	(ioctl_data.perfect_inverse_match ? "INVERSE" : "PERFECT"));

	DBGPR_FUNC("<--tc9562_confing_l2_da_filter\n");	
	return ret;
}

static int tc9562_SA_VLAN_INS_Config(struct tc9562mac_priv *priv, void __user *data)
{
    struct tc9562_ioctl_sa_ins_cfg ioctl_data;
    u32 reg_data;

    DBGPR_FUNC("-->tc9562_confing_l2_da_filter\n");
    if(copy_from_user(&ioctl_data, data, sizeof(ioctl_data)))
        return -EFAULT;
    
    if(priv->dma_cap.sa_vlan_ins) {
        switch(ioctl_data.cmd) {
            case tc9562_SA0_VLAN_INS_REP_DESC: 
                priv->sa_vlan_ins_via_desc = ioctl_data.control_flag;
			    priv->sa_vlan_ins_via_reg = TC9562_SA0_NONE;
			    if (ioctl_data.control_flag == TC9562_SA0_NONE) {
				    memcpy(priv->ins_mac_addr, priv->dev->dev_addr, ETH_ALEN);
			    } else {
				    memcpy(priv->ins_mac_addr, ioctl_data.mac_addr, ETH_ALEN);
			    }
			    priv->hw->mac->set_umac_addr(priv->hw, priv->ins_mac_addr, 0, 0);
                reg_data = readl(priv->ioaddr + GMAC_CONFIG);
                reg_data = (reg_data & (~GMAC_CONFIG_SARC)) | (priv->sa_vlan_ins_via_reg << GMAC_CONFIG_SARC_LPOS);
                writel(reg_data, priv->ioaddr + GMAC_CONFIG);
			    NMSGPR_ALERT("SA will use MAC0 with descriptor for configuration %d\n", priv->sa_vlan_ins_via_desc);
                break;  
            case tc9562_SA1_VLAN_INS_REP_DESC: 
                priv->sa_vlan_ins_via_desc = ioctl_data.control_flag;
			    priv->sa_vlan_ins_via_reg = TC9562_SA1_NONE;
			    if (ioctl_data.control_flag == TC9562_SA1_NONE) {
				    memcpy(priv->ins_mac_addr, priv->dev->dev_addr, ETH_ALEN);
			    } else {
				    memcpy(priv->ins_mac_addr, ioctl_data.mac_addr, ETH_ALEN);
			    }
			    priv->hw->mac->set_umac_addr(priv->hw, priv->ins_mac_addr, 1, 0);
                reg_data = readl(priv->ioaddr + GMAC_CONFIG);
                reg_data = (reg_data & (~GMAC_CONFIG_SARC)) | (priv->sa_vlan_ins_via_reg << GMAC_CONFIG_SARC_LPOS);
                writel(reg_data, priv->ioaddr + GMAC_CONFIG);
			    NMSGPR_ALERT("SA will use MAC1 with descriptor for configuration %d\n", priv->sa_vlan_ins_via_desc);
                break;  
            case tc9562_SA0_VLAN_INS_REP_REG:
                priv->sa_vlan_ins_via_reg = ioctl_data.control_flag;
			    priv->sa_vlan_ins_via_desc = TC9562_SA0_NONE;
			    if (ioctl_data.control_flag == TC9562_SA0_NONE) {
				    memcpy(priv->ins_mac_addr, priv->dev->dev_addr, ETH_ALEN);
			    } else {
				    memcpy(priv->ins_mac_addr, ioctl_data.mac_addr, ETH_ALEN);
			    }
			    priv->hw->mac->set_umac_addr(priv->hw, priv->ins_mac_addr, 0, 0);
			    reg_data = readl(priv->ioaddr + GMAC_CONFIG);
                reg_data = (reg_data & (~GMAC_CONFIG_SARC)) | (priv->sa_vlan_ins_via_reg << GMAC_CONFIG_SARC_LPOS);
                writel(reg_data, priv->ioaddr + GMAC_CONFIG);
			    NMSGPR_ALERT("SA will use MAC0 with register for configuration %d\n", priv->sa_vlan_ins_via_reg);
                break;
            case tc9562_SA1_VLAN_INS_REP_REG:
                priv->sa_vlan_ins_via_reg = ioctl_data.control_flag;
			    priv->sa_vlan_ins_via_desc = TC9562_SA1_NONE;
			    if (ioctl_data.control_flag == TC9562_SA1_NONE) {
				    memcpy(priv->ins_mac_addr, priv->dev->dev_addr, ETH_ALEN);
			    } else {
				    memcpy(priv->ins_mac_addr, ioctl_data.mac_addr, ETH_ALEN);
			    }
			    priv->hw->mac->set_umac_addr(priv->hw, priv->ins_mac_addr, 1, 0);
			    reg_data = readl(priv->ioaddr + GMAC_CONFIG);
                reg_data = (reg_data & (~GMAC_CONFIG_SARC)) | (priv->sa_vlan_ins_via_reg << GMAC_CONFIG_SARC_LPOS);
                writel(reg_data, priv->ioaddr + GMAC_CONFIG);
			    NMSGPR_ALERT("SA will use MAC1 with register for configuration %d\n", priv->sa_vlan_ins_via_reg);
                break;
            default: 
                return -EINVAL;
        }   
    }
    else {
        NMSGPR_ALERT("Device doesn't supports SA Insertion/Replacement\n");
        return -EINVAL;
    }

    DBGPR_FUNC("<--tc9562_confing_l2_da_filter\n");
    return 0;
}

static int tc9562_get_tx_qcnt(struct tc9562mac_priv *priv, void __user *data)
{
    u32 tx_qcnt = 0;
	struct tc9562_ioctl_tx_qcnt ioctl_data;
		
	DBGPR_FUNC("-->tc9562_get_tx_qcnt\n");
		
	if (copy_from_user(&ioctl_data, data, sizeof(ioctl_data)))
		return -EFAULT;
	
    tx_qcnt = priv->plat->tx_queues_to_use;
    if (copy_to_user(ioctl_data.ptr ,&tx_qcnt, sizeof(unsigned int)))
	    return -EFAULT;

	DBGPR_FUNC("<--tc9562_get_tx_qcnt\n");
	
	return 0;
}

static int tc9562_get_rx_qcnt(struct tc9562mac_priv *priv, void __user *data)
{
    u32 rx_qcnt = 0;
	struct tc9562_ioctl_rx_qcnt ioctl_data;
		
	DBGPR_FUNC("-->tc9562_get_rx_qcnt\n");
		
	if (copy_from_user(&ioctl_data, data, sizeof(ioctl_data)))
		return -EFAULT;
	
    rx_qcnt = priv->plat->rx_queues_to_use;
    if (copy_to_user(ioctl_data.ptr ,&rx_qcnt, sizeof(unsigned int)))
	    return -EFAULT;

	DBGPR_FUNC("<--tc9562_get_rx_qcnt\n");
	
	return 0;
}


static int tc9562_pcie_config_reg_rd(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562_ioctl_pcie_reg_rd_wr ioctl_data;
	u32 val;

	DBGPR_FUNC("-->tc9562_pcie_config_reg_rd\n");
	
	if (copy_from_user(&ioctl_data, data, sizeof(ioctl_data)))
			return -EFAULT;

   	pci_read_config_dword(priv->plat->pdev, ioctl_data.addr, &val);

	if (copy_to_user(ioctl_data.ptr, &val, sizeof(unsigned int)))
		return -EFAULT;
		
	DBGPR_FUNC("<--tc9562_pcie_config_reg_rd\n");
	
	return 0;
}

static int tc9562_pcie_config_reg_wr(struct tc9562mac_priv *priv, void __user *data)
{
	struct tc9562_ioctl_pcie_reg_rd_wr ioctl_data;
	u32 val;	
	DBGPR_FUNC("-->tc9562_pcie_config_reg_wr\n");
	
	if (copy_from_user(&ioctl_data, data, sizeof(ioctl_data)))
			return -EFAULT;

    if (copy_from_user(&val, ioctl_data.ptr, sizeof(unsigned int)))
            return -EFAULT;

	//pci_write_config_dword(priv->plat->pdev, ioctl_data.addr, *ioctl_data.ptr);
	pci_write_config_dword(priv->plat->pdev, ioctl_data.addr, val);

	DBGPR_FUNC("<--tc9562_pcie_config_reg_wr\n");
	
	return 0;
}

static int tc9562_config_vlan_filter(struct tc9562mac_priv *priv, void __user *data)
{
    struct tc9562_ioctl_vlan_filter ioctl_data;
    u32 reg_val;
    
    DBGPR_FUNC("-->tc9562_config_vlan_filter\n");

    if (copy_from_user(&ioctl_data, data, sizeof(ioctl_data)))
			return -EFAULT;
			
	if ((ioctl_data.perfect_hash) && (priv->dma_cap.hash_filter == 0)) {
		NMSGPR_ALERT("VLAN HASH filtering is not supported\n");
		return -EFAULT;
	}

	/* configure the vlan filter */
	reg_val = readl(priv->ioaddr + GMAC_PACKET_FILTER);
	reg_val = (reg_val & (~GMAC_MPFR_VTFE)) | (ioctl_data.filter_enb_dis << GMAC_MPFR_VTFE_LPOS);
	writel(reg_val, priv->ioaddr + GMAC_PACKET_FILTER);

    reg_val = readl(priv->ioaddr + GMAC_VLAN_TAG);
    reg_val = (reg_val & (~GMAC_VLANTR_VTIM)) | (ioctl_data.perfect_inverse_match << GMAC_VLANTR_VTIM_LPOS);
    reg_val = (reg_val & (~GMAC_VLANTR_VTHM)) | (ioctl_data.perfect_hash << GMAC_VLANTR_VTHM_LPOS);

    
    /* When VLAN filtering is enabled, then VL/VID
    * should be > zero, if VLAN is not configured. Otherwise all VLAN packets will be accepted. 
    * Hence we are writting 1 into VL. It also means that MAC will always receive VLAN pkt with
    * VID = 1 if inverse march is not set.
    */
    /* If hash filtering is enabled, 
    * By default enable MAC to calculate vlan hash on only 12-bits of received VLAN tag (ie only on
    * VLAN id and ignore priority and other fields)
    * */
    
    reg_val = (reg_val & (~GMAC_VLANTR_VL)) | ( 1 << GMAC_VLANTR_VL_LPOS);
    if (ioctl_data.perfect_hash) {
        reg_val = (reg_val & (~GMAC_VLANTR_ETV)) | (1 << GMAC_VLANTR_ETV_LPOS);
    } else {
        reg_val = (reg_val & (~GMAC_VLANTR_ETV));
    }

    writel(reg_val, priv->ioaddr + GMAC_VLAN_TAG);
	priv->vlan_hash_filtering = ioctl_data.perfect_hash;

	NDBGPR_L2("Successfully %s VLAN %s filtering and %s matching\n",
		(ioctl_data.filter_enb_dis ? "ENABLED" : "DISABLED"),
		(ioctl_data.perfect_hash ? "HASH" : "PERFECT"),
		(ioctl_data.perfect_inverse_match ? "INVERSE" : "PERFECT"));
    
    DBGPR_FUNC("<--tc9562_config_vlan_filter\n");

    return 0;
}

static int tc9562_config_ptpoffload(struct tc9562mac_priv *priv, void __user *data)
{
	u32 pto_cntrl;
	u32 varMAC_TCR;
	struct tc9562_config_ptpoffloading ioctl_data;

    DBGPR_FUNC("-->tc9562_config_ptpoffload\n");
    
	if (copy_from_user(&ioctl_data, data, sizeof(ioctl_data)))
			return -EFAULT;

	pto_cntrl = GMAC_PTOCTRL_PTOEN; /* enable ptp offloading */
	
	varMAC_TCR = readl(priv->ptpaddr + PTP_TCR);
	varMAC_TCR &= ~PTP_TCR_MODE_MASK;
	
	if (ioctl_data.mode == TC9562_PTP_ORDINARY_SLAVE) {

		varMAC_TCR |= PTP_TCR_TSEVNTENA;
		priv->ptp_offloading_mode = TC9562_PTP_ORDINARY_SLAVE;

	} else if (ioctl_data.mode == TC9562_PTP_TRASPARENT_SLAVE) {

		pto_cntrl |= GMAC_PTOCTRL_APDREQEN;
		varMAC_TCR |= PTP_TCR_TSEVNTENA;
		varMAC_TCR = varMAC_TCR & (~PTP_GMAC4_TCR_SNAPTYPSEL_1);
		varMAC_TCR |= PTP_TCR_SNAPTYPSEL_1;
		priv->ptp_offloading_mode = TC9562_PTP_TRASPARENT_SLAVE;

	} else if (ioctl_data.mode == TC9562_PTP_ORDINARY_MASTER) {

		pto_cntrl |= GMAC_PTOCTRL_ASYNCEN;
		varMAC_TCR |= PTP_TCR_TSEVNTENA;
		varMAC_TCR |= PTP_TCR_TSMSTRENA;
		priv->ptp_offloading_mode = TC9562_PTP_ORDINARY_MASTER;

	} else if (ioctl_data.mode == TC9562_PTP_TRASPARENT_MASTER) {

		pto_cntrl |= GMAC_PTOCTRL_ASYNCEN | GMAC_PTOCTRL_APDREQEN;
		varMAC_TCR = varMAC_TCR & (~PTP_GMAC4_TCR_SNAPTYPSEL_1);
		varMAC_TCR |= PTP_TCR_SNAPTYPSEL_1;
		varMAC_TCR |= PTP_TCR_TSEVNTENA;
		varMAC_TCR |= PTP_TCR_TSMSTRENA;
		priv->ptp_offloading_mode = TC9562_PTP_TRASPARENT_MASTER;

	} else if (ioctl_data.mode == TC9562_PTP_PEER_TO_PEER_TRANSPARENT) {

		pto_cntrl |= GMAC_PTOCTRL_APDREQEN;
		varMAC_TCR |= (3 << PTP_TCR_SNAPTYPSEL_1_LPOS);
		priv->ptp_offloading_mode = TC9562_PTP_PEER_TO_PEER_TRANSPARENT;
	}

	priv->ptp_offload = 1;
	if (ioctl_data.en_dis == TC9562_PTP_OFFLOADING_DISABLE) {
		pto_cntrl = 0;
		varMAC_TCR = readl(priv->ptpaddr + PTP_TCR);;
		priv->ptp_offload = 0;
	}

	pto_cntrl |= (ioctl_data.domain_num << 8);
	writel(varMAC_TCR, priv->ptpaddr + PTP_TCR);
    /* Since time registers are already initialized by default, no need to initialize time. */

    if(ioctl_data.mc_uc == 1) {
          priv->hw->mac->set_umac_addr(priv->hw, ioctl_data.mc_uc_addr, 0, 2);
    }    
    
    writel(pto_cntrl, priv->ioaddr + GMAC_PTO_CTRL);
    varMAC_TCR = readl(priv->ptpaddr + PTP_TCR);
    varMAC_TCR &= (~PTP_TCR_TSENMACADDR);
    varMAC_TCR |= ((ioctl_data.mc_uc & 0x1) << PTP_TCR_MC_UC_LPOS);
    writel(varMAC_TCR, priv->ptpaddr + PTP_TCR);
    
	DBGPR_FUNC("<--tc9562_config_ptpoffload\n");

	return 0;
}

static int tc9562mac_extension_ioctl(struct tc9562mac_priv *priv, void __user *data)
{
	u32 cmd;

	DBGPR_FUNC("-->tc9562mac_extension_ioctl\n");

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	if (copy_from_user(&cmd, data, sizeof(cmd)))
		return -EFAULT;

	switch (cmd) {
	case TC9562MAC_GET_QMODE:
		return tc9562mac_ioctl_get_qmode(priv, data);
	case TC9562MAC_SET_QMODE:
		return tc9562mac_ioctl_set_qmode(priv, data);
	case TC9562MAC_GET_CBS:
		return tc9562mac_ioctl_get_cbs(priv, data);
	case TC9562MAC_SET_CBS:
		return tc9562mac_ioctl_set_cbs(priv, data);
	case TC9562MAC_GET_EST:
		return tc9562mac_ioctl_get_est(priv, data);
	case TC9562MAC_SET_EST:
		return tc9562mac_ioctl_set_est(priv, data);
	case TC9562MAC_GET_FPE: //Function to Get FPE related configurations
		return tc9562mac_ioctl_get_fpe(priv, data);
	case TC9562MAC_SET_FPE: // Function to Set FPE related configurations
		return tc9562mac_ioctl_set_fpe(priv, data);
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	case TC9562MAC_GET_ECC:
		return tc9562mac_ioctl_get_ecc(priv, data);
	case TC9562MAC_SET_ECC:
		return tc9562mac_ioctl_set_ecc(priv, data);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
	case TC9562MAC_GET_RXP:
		return tc9562mac_ioctl_get_rxp(priv, data);
	case TC9562MAC_SET_RXP:
		return tc9562mac_ioctl_set_rxp(priv, data);
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE		
	case TC9562MAC_GET_MCGR:
		return tc9562mac_ioctl_get_mcgr(priv, data);
	case TC9562MAC_SET_MCGR:
		return tc9562mac_ioctl_set_mcgr(priv, data);
	case TC9562MAC_GET_PPS:
		return tc9562mac_ioctl_get_pps(priv, data);
	case TC9562MAC_SET_PPS:
		return tc9562mac_ioctl_set_pps(priv, data);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */		
	case tc9562_GET_SPEED:
		return tc9562_ioctl_get_connected_speed(priv, data);
	case tc9562_GET_TX_FREE_DESC:		
		return tc9562_ioctl_get_tx_free_desc(priv, data);
	case tc9562_REG_RD:
		return tc9562_reg_rd(priv, data);
	case tc9562_REG_WR:
		return tc9562_reg_wr(priv, data);
	case tc9562_SET_MAC_LOOPBACK:
		return tc9562_ioctl_set_mac_loopback(priv, data);
	case tc9562_SET_PHY_LOOPBACK:
		return tc9562_ioctl_set_phy_loopback(priv, data);
	case tc9562_L2_DA_FILTERING_CMD:
		return tc9562_confing_l2_da_filter(priv, data);
	case tc9562_SET_PPS_OUT:
		return tc9562_PPSOUT_Config(priv, data);
	case tc9562_PTPCLK_CONFIG:
		return tc9562_PTPCLK_Config(priv, data);
	case tc9562_SA0_VLAN_INS_REP_DESC:
	    return tc9562_SA_VLAN_INS_Config(priv,data);
	case tc9562_SA1_VLAN_INS_REP_DESC:
	    return tc9562_SA_VLAN_INS_Config(priv,data);
	case tc9562_SA0_VLAN_INS_REP_REG:
	    return tc9562_SA_VLAN_INS_Config(priv,data);
	case tc9562_SA1_VLAN_INS_REP_REG:
	    return tc9562_SA_VLAN_INS_Config(priv,data);
	case tc9562_GET_TX_QCNT:
	    return tc9562_get_tx_qcnt(priv,data);
	case tc9562_GET_RX_QCNT:
	    return tc9562_get_rx_qcnt(priv,data);
    case tc9562_PCIE_CONFIG_REG_RD:
        return tc9562_pcie_config_reg_rd(priv, data);
	case tc9562_PCIE_CONFIG_REG_WR:
	    return tc9562_pcie_config_reg_wr(priv, data);
    case tc9562_VLAN_FILTERING:
        return tc9562_config_vlan_filter(priv, data);
    case tc9562_PTPOFFLOADING:
        return tc9562_config_ptpoffload(priv, data);
#ifdef UNIFIED_DRIVER
#ifdef UNIFIED_DRIVER
    case TC9562_TDM_INIT:   
    {            
        return tc9562_ioctl_tdm_start(priv, data);
    }
    case TC9562_TDM_UNINIT:
    {           
        return tc9562_ioctl_tdm_stop(priv, data);
    }    
#endif
#endif
	default:
		return -EINVAL;
	}

	DBGPR_FUNC("<--tc9562mac_extension_ioctl\n");
	
	return 0;
}

/**
 *  tc9562mac_ioctl - Entry point for the Ioctl
 *  @dev: Device pointer.
 *  @rq: An IOCTL specefic structure, that can contain a pointer to
 *  a proprietary structure used to pass information to the driver.
 *  @cmd: IOCTL command
 *  Description:
 *  Currently it supports the phy_mii_ioctl(...) and HW time stamping.
 */
static int tc9562mac_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	int ret = -EOPNOTSUPP;
	unsigned int reg_val = 0;
	int reg_num = 0;
	struct mii_ioctl_data *data = if_mii(rq);

	DBGPR_FUNC("-->tc9562mac_ioctl\n");
	
	if (!netif_running(dev))
		return -EINVAL;

	if (!dev->phydev)
		return -EINVAL;
	switch (cmd) {
	case SIOCGMIIPHY:
		data->phy_id = priv->plat->phy_addr;
		NMSGPR_ALERT("PHY ID: SIOCGMIIPHY\n");
		ret = 0; 
		break;
	case SIOCGMIIREG:
		reg_num = data->reg_num;
		if (data->phy_id & MDIO_PHY_ID_C45) {
			phyaddr = (data->phy_id & MDIO_PHY_ID_PRTAD) >> 5;
			reg_num |= MII_ADDR_C45 | (((data->phy_id) & MDIO_PHY_ID_DEVAD) << 16);
		} else {
			phyaddr = data->phy_id;
		}
		ret = tc9562mac_mdio_read_direct(priv, phyaddr, reg_num, &reg_val);
		if (ret)
			ret = -EIO;

		data->val_out = reg_val;
		NMSGPR_ALERT("PHY ID: SIOCGMIIREG reg:%#x reg_val:%#x\n", reg_num, reg_val);
		break;
	case SIOCSMIIREG:
        reg_num = data->reg_num;
		if (data->phy_id & MDIO_PHY_ID_C45) {
			phyaddr = (data->phy_id & MDIO_PHY_ID_PRTAD) >> 5;
			reg_num |= MII_ADDR_C45 | (((data->phy_id) & MDIO_PHY_ID_DEVAD) << 16);
		}else {
			phyaddr = data->phy_id;
		}
		
		 reg_val = data->val_in;
       	 ret = tc9562mac_mdio_write_direct(priv, phyaddr, reg_num, reg_val);
		if (ret)
			ret = -EIO;

		NMSGPR_ALERT("PHY ID: SIOCSMIIPHY reg:%#x reg_val:%#x\n",
		       reg_num, reg_val);
		break;
	case SIOCSHWTSTAMP:
		ret = tc9562mac_hwtstamp_ioctl(dev, rq);
		break;
	case SIOCSTIOCTL:
		if (!priv || !rq)
			return -EINVAL;
		ret = tc9562mac_extension_ioctl(priv, rq->ifr_data);
		break;
	default:
		break;
	}
	
	DBGPR_FUNC("<--tc9562mac_ioctl\n");
	
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,15))
static int tc9562mac_setup_tc_block_cb(enum tc_setup_type type, void *type_data,
				    void *cb_priv)
{
	struct tc9562mac_priv *priv = cb_priv;
	int ret = -EOPNOTSUPP;
	
	tc9562mac_disable_all_queues(priv);

	switch (type) {
	case TC_SETUP_CLSU32:
		if (tc_cls_can_offload_and_chain0(priv->dev, type_data)){
			ret = priv->hw->tc->setup_cls_u32(priv->dev, type_data);
			}
		break;
	default:
		break;
	}

	tc9562mac_enable_all_queues(priv);
	return ret;
}
static int tc9562mac_setup_tc_block(struct tc9562mac_priv *priv,
				 struct tc_block_offload *f)
{
	if (f->binder_type != TCF_BLOCK_BINDER_TYPE_CLSACT_INGRESS)
		return -EOPNOTSUPP;

	switch (f->command) {
	case TC_BLOCK_BIND:
		return tcf_block_cb_register(f->block, tc9562mac_setup_tc_block_cb,
				priv, priv, f->extack);
	case TC_BLOCK_UNBIND:
		tcf_block_cb_unregister(f->block, tc9562mac_setup_tc_block_cb, priv);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}
static int tc9562mac_setup_tc(struct net_device *ndev, enum tc_setup_type type,
			   void *type_data)
{
	struct tc9562mac_priv *priv = netdev_priv(ndev);

	switch (type) {
	case TC_SETUP_BLOCK:
		return tc9562mac_setup_tc_block(priv, type_data);
	case TC_SETUP_QDISC_CBS:
		return priv->hw->tc->setup_cbs(ndev, type_data);
	case TC_SETUP_QDISC_ETF:
		return priv->hw->tc->launch_time(ndev, type_data);
		
	default:
		return -EOPNOTSUPP;
	}
}
#endif

static int tc9562_set_mac_address(struct net_device *dev, void *addr)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);
	int ret = 0;

	ret = eth_mac_addr(dev, addr);
	if (ret)
		return ret;
#ifndef UNIFIED_DRIVER
	priv->hw->mac->set_umac_addr(priv->hw, dev->dev_addr, 2, 0);
#else
    priv->hw->mac->set_umac_addr(priv->hw, dev->dev_addr, HOST_MAC_ADDR_OFFSET, 0);
#endif

	return ret;
}
#ifdef CONFIG_DEBUG_FS
static struct dentry *tc9562mac_fs_dir;

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static void sysfs_display_ring(void *head, int size, int extend_desc,
			       struct seq_file *seq)
{
	int i;
	struct dma_extended_desc *ep = (struct dma_extended_desc *)head;
	struct dma_desc *p = (struct dma_desc *)head;

	for (i = 0; i < size; i++) {
		if (extend_desc) {
			seq_printf(seq, "%d [0x%x]: 0x%x 0x%x 0x%x 0x%x\n",
				   i, (unsigned int)virt_to_phys(ep),
				   le32_to_cpu(ep->basic.des0),
				   le32_to_cpu(ep->basic.des1),
				   le32_to_cpu(ep->basic.des2),
				   le32_to_cpu(ep->basic.des3));
			ep++;
		} else {
			seq_printf(seq, "%d [0x%x]: 0x%x 0x%x 0x%x 0x%x\n",
				   i, (unsigned int)virt_to_phys(p),
				   le32_to_cpu(p->des0), le32_to_cpu(p->des1),
				   le32_to_cpu(p->des2), le32_to_cpu(p->des3));
			p++;
		}
		seq_printf(seq, "\n");
	}
}

static int tc9562mac_sysfs_ring_read(struct seq_file *seq, void *v)
{
	struct net_device *dev = seq->private;
	struct tc9562mac_priv *priv = netdev_priv(dev);
	u32 rx_count = priv->plat->rx_queues_to_use + 1;
	u32 tx_count = priv->plat->tx_queues_to_use;
	u32 queue;
	struct tc9562mac_rx_queue *rx_q;
	struct tc9562mac_tx_queue *tx_q;

	DBGPR_FUNC("-->tc9562mac_sysfs_ring_read\n");
	
	for (queue = 0; queue < rx_count; queue++) {
#ifdef UNIFIED_DRIVER
	if(priv->plat->rx_dma_ch_for_host[queue] == 0)
		continue;
#endif
		rx_q = &priv->rx_queue[queue];

		seq_printf(seq, "RX Queue %d:\n", queue);

		if (priv->extend_desc) {
			seq_printf(seq, "Extended descriptor ring:\n");
			sysfs_display_ring((void *)rx_q->dma_erx,
					   DMA_RX_SIZE, 1, seq);
		} else {
			seq_printf(seq, "Descriptor ring:\n");
			sysfs_display_ring((void *)rx_q->dma_rx,
					   DMA_RX_SIZE, 0, seq);
		}
	}

	for (queue = 0; queue < tx_count; queue++) {
#ifdef UNIFIED_DRIVER
		if(priv->plat->tx_dma_ch_for_host[queue] == 0)
			continue;
#endif
		tx_q = &priv->tx_queue[queue];

		seq_printf(seq, "TX Queue %d:\n", queue);

		if (priv->extend_desc) {
			seq_printf(seq, "Extended descriptor ring:\n");
			sysfs_display_ring((void *)tx_q->dma_etx,
					   DMA_TX_SIZE, 1, seq);
		} else {
			seq_printf(seq, "Descriptor ring:\n");
			sysfs_display_ring((void *)tx_q->dma_tx,
					   DMA_TX_SIZE, 0, seq);
		}
	}
	DBGPR_FUNC("<--tc9562mac_sysfs_ring_read\n");
	return 0;
}

static int tc9562mac_sysfs_ring_open(struct inode *inode, struct file *file)
{
	return single_open(file, tc9562mac_sysfs_ring_read, inode->i_private);
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

/* Debugfs files, should appear in /sys/kernel/debug/tc9562maceth/eth0 */

static const struct file_operations tc9562mac_rings_status_fops = {
	.owner = THIS_MODULE,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE		
	.open = tc9562mac_sysfs_ring_open,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */	
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static int tc9562mac_sysfs_dma_cap_read(struct seq_file *seq, void *v)
{
	struct net_device *dev = seq->private;
	struct tc9562mac_priv *priv = netdev_priv(dev);

	DBGPR_FUNC("-->tc9562mac_sysfs_dma_cap_read\n");
	if (!priv->hw_cap_support) {
		seq_printf(seq, "DMA HW features not supported\n");
		return 0;
	}

	seq_printf(seq, "==============================\n");
	seq_printf(seq, "\tDMA HW features\n");
	seq_printf(seq, "==============================\n");

	seq_printf(seq, "\t10/100 Mbps: %s\n",
		   (priv->dma_cap.mbps_10_100) ? "Y" : "N");
	seq_printf(seq, "\t1000 Mbps: %s\n",
		   (priv->dma_cap.mbps_1000) ? "Y" : "N");
	seq_printf(seq, "\tHalf duplex: %s\n",
		   (priv->dma_cap.half_duplex) ? "Y" : "N");
	seq_printf(seq, "\tHash Filter: %s\n",
		   (priv->dma_cap.hash_filter) ? "Y" : "N");
	seq_printf(seq, "\tMultiple MAC address registers: %s\n",
		   (priv->dma_cap.multi_addr) ? "Y" : "N");
	seq_printf(seq, "\tPCS (TBI/SGMII/RTBI PHY interfaces): %s\n",
		   (priv->dma_cap.pcs) ? "Y" : "N");
	seq_printf(seq, "\tSMA (MDIO) Interface: %s\n",
		   (priv->dma_cap.sma_mdio) ? "Y" : "N");
	seq_printf(seq, "\tPMT Remote wake up: %s\n",
		   (priv->dma_cap.pmt_remote_wake_up) ? "Y" : "N");
	seq_printf(seq, "\tPMT Magic Frame: %s\n",
		   (priv->dma_cap.pmt_magic_frame) ? "Y" : "N");
	seq_printf(seq, "\tRMON module: %s\n",
		   (priv->dma_cap.rmon) ? "Y" : "N");
	seq_printf(seq, "\tIEEE 1588-2002 Time Stamp: %s\n",
		   (priv->dma_cap.time_stamp) ? "Y" : "N");
	seq_printf(seq, "\tIEEE 1588-2008 Advanced Time Stamp: %s\n",
		   (priv->dma_cap.atime_stamp) ? "Y" : "N");
	seq_printf(seq, "\t802.3az - Energy-Efficient Ethernet (EEE): %s\n",
		   (priv->dma_cap.eee) ? "Y" : "N");
	seq_printf(seq, "\tAV features: %s\n", (priv->dma_cap.av) ? "Y" : "N");
	if (priv->synopsys_id >= DWMAC_CORE_5_00) {
		seq_printf(seq, "\tTSN supported: %s\n",
			   (priv->dma_cap.tsn) ? "Y" : "N");
	}
	seq_printf(seq, "\tChecksum Offload in TX: %s\n",
		   (priv->dma_cap.tx_coe) ? "Y" : "N");
	if (priv->synopsys_id >= DWMAC_CORE_4_00) {
		seq_printf(seq, "\tIP Checksum Offload in RX: %s\n",
			   (priv->dma_cap.rx_coe) ? "Y" : "N");
	} else {
		seq_printf(seq, "\tIP Checksum Offload (type1) in RX: %s\n",
			   (priv->dma_cap.rx_coe_type1) ? "Y" : "N");
		seq_printf(seq, "\tIP Checksum Offload (type2) in RX: %s\n",
			   (priv->dma_cap.rx_coe_type2) ? "Y" : "N");
	}
	seq_printf(seq, "\tRXFIFO > 2048bytes: %s\n",
		   (priv->dma_cap.rxfifo_over_2048) ? "Y" : "N");
	seq_printf(seq, "\tNumber of Additional RX channel: %d\n",
		   priv->dma_cap.number_rx_channel);
	seq_printf(seq, "\tNumber of Additional TX channel: %d\n",
		   priv->dma_cap.number_tx_channel);
	seq_printf(seq, "\tEnhanced descriptors: %s\n",
		   (priv->dma_cap.enh_desc) ? "Y" : "N");
	DBGPR_FUNC("<--tc9562mac_sysfs_dma_cap_read\n");

	return 0;
}

static int tc9562mac_sysfs_dma_cap_open(struct inode *inode, struct file *file)
{
	return single_open(file, tc9562mac_sysfs_dma_cap_read, inode->i_private);
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

static const struct file_operations tc9562mac_dma_cap_fops = {
	.owner = THIS_MODULE,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE		
	.open = tc9562mac_sysfs_dma_cap_open,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */		
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int tc9562mac_init_fs(struct net_device *dev)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);

	DBGPR_FUNC("-->tc9562mac_init_fs\n");

	/* Create per netdev entries */
	priv->dbgfs_dir = debugfs_create_dir(dev->name, tc9562mac_fs_dir);

	if (!priv->dbgfs_dir || IS_ERR(priv->dbgfs_dir)) {
		netdev_err(priv->dev, "ERROR failed to create debugfs directory\n");

		return -ENOMEM;
	}

	/* Entry to report DMA RX/TX rings */
	priv->dbgfs_rings_status =
		debugfs_create_file("descriptors_status", S_IRUGO,
				    priv->dbgfs_dir, dev,
				    &tc9562mac_rings_status_fops);

	if (!priv->dbgfs_rings_status || IS_ERR(priv->dbgfs_rings_status)) {
		netdev_err(priv->dev, "ERROR creating tc9562mac ring debugfs file\n");
		debugfs_remove_recursive(priv->dbgfs_dir);

		return -ENOMEM;
	}

	/* Entry to report the DMA HW features */
	priv->dbgfs_dma_cap = debugfs_create_file("dma_cap", S_IRUGO,
					    priv->dbgfs_dir,
					    dev, &tc9562mac_dma_cap_fops);

	if (!priv->dbgfs_dma_cap || IS_ERR(priv->dbgfs_dma_cap)) {
		netdev_err(priv->dev, "ERROR creating tc9562mac MMC debugfs file\n");
		debugfs_remove_recursive(priv->dbgfs_dir);

		return -ENOMEM;
	}

	DBGPR_FUNC("<--tc9562mac_init_fs\n");

	return 0;
}

static void tc9562mac_exit_fs(struct net_device *dev)
{
	struct tc9562mac_priv *priv = netdev_priv(dev);

	DBGPR_FUNC("-->tc9562mac_exit_fs\n");
	
	debugfs_remove_recursive(priv->dbgfs_dir);

	DBGPR_FUNC("<--tc9562mac_exit_fs\n");
}
#endif /* CONFIG_DEBUG_FS */

static const struct net_device_ops tc9562mac_netdev_ops = {
	.ndo_open = tc9562mac_open,
	.ndo_start_xmit = tc9562mac_xmit,
	.ndo_stop = tc9562mac_release,
	.ndo_change_mtu = tc9562mac_change_mtu,
	.ndo_fix_features = tc9562mac_fix_features,
	.ndo_set_features = tc9562mac_set_features,
	.ndo_set_rx_mode = tc9562mac_set_rx_mode,
	.ndo_tx_timeout = tc9562mac_tx_timeout,
	.ndo_do_ioctl = tc9562mac_ioctl,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,15))	
	.ndo_setup_tc = tc9562mac_setup_tc,
#endif
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = tc9562mac_poll_controller,
#endif
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
#ifndef TC9562_DEFINED
	/* Callbacks are not avaiable with Kernel 4.14 */
	.ndo_tsn_capable	= tc9562mac_tsn_capable,
	.ndo_tsn_link_configure = tc9562mac_tsn_link_configure,
#endif
	.ndo_select_queue	= tc9562mac_tsn_select_queue,
	.ndo_set_mac_address = tc9562_set_mac_address, 
	.ndo_vlan_rx_add_vid = tc9562_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid = tc9562_vlan_rx_kill_vid,
};

/**
 *  tc9562mac_hw_init - Init the MAC device
 *  @priv: driver private structure
 *  Description: this function is to configure the MAC device according to
 *  some platform parameters or the HW capability register. It prepares the
 *  driver to use either ring or chain modes and to setup either enhanced or
 *  normal descriptors.
 */
static int tc9562mac_hw_init(struct tc9562mac_priv *priv)
{
	struct mac_device_info *mac;

	DBGPR_FUNC("-->tc9562mac_hw_init \n");
	/* Identify the MAC HW device */
	if (priv->plat->setup) {
		mac = priv->plat->setup(priv);
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE		
	} else if (priv->plat->has_gmac) {
		priv->dev->priv_flags |= IFF_UNICAST_FLT;
		mac = dwmac1000_setup(priv->ioaddr,
				      priv->plat->multicast_filter_bins,
				      priv->plat->unicast_filter_entries,
				      &priv->synopsys_id);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */		
	} else if (priv->plat->has_gmac4) {
		priv->dev->priv_flags |= IFF_UNICAST_FLT;
		mac = dwmac4_setup(priv->ioaddr,
				   priv->plat->multicast_filter_bins,
				   priv->plat->unicast_filter_entries,
				   &priv->synopsys_id);
	} else {
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE 
		mac = dwmac100_setup(priv->ioaddr, &priv->synopsys_id);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
	}
	if (!mac){
		NMSGPR_ALERT(" Mac device allocation error \n");
		return -ENOMEM;
	}
	priv->hw = mac;
#ifndef TC9562_DEFINED
	/* dwmac-sun8i only work in chain mode */
	if (priv->plat->has_sun8i)
		chain_mode = 1;
#endif
	/* To use the chained or ring mode */
	if (priv->synopsys_id >= DWMAC_CORE_4_00) {
		priv->hw->mode = &dwmac4_ring_mode_ops;
	} 
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	else {

		if (chain_mode) {
			priv->hw->mode = &chain_mode_ops;
			dev_info(priv->device, "Chain mode enabled\n");
			priv->mode = TC9562MAC_CHAIN_MODE;
		} else {
			priv->hw->mode = &ring_mode_ops;
			dev_info(priv->device, "Ring mode enabled\n");
			priv->mode = TC9562MAC_RING_MODE;
		}
	}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

	/* Get the HW capability (new GMAC newer than 3.50a) */
	priv->hw_cap_support = tc9562mac_get_hw_features(priv);
	if (priv->hw_cap_support) {
		dev_info(priv->device, "DMA HW capability register supported\n");

		/* We can override some gmac/dma configuration fields: e.g.
		 * enh_desc, tx_coe (e.g. that are passed through the
		 * platform) with the values from the HW capability
		 * register (if supported).
		 */
		priv->plat->enh_desc = priv->dma_cap.enh_desc;
		priv->plat->pmt = priv->dma_cap.pmt_remote_wake_up;
		priv->hw->pmt = priv->plat->pmt;

		/* TXCOE doesn't work in thresh DMA mode */
		if (priv->plat->force_no_tx_coe ||
		    priv->plat->force_thresh_dma_mode)
			priv->plat->tx_coe = 0;
		else
			priv->plat->tx_coe = priv->dma_cap.tx_coe;

		/* In case of GMAC4 rx_coe is from HW cap register. */
		if (!priv->plat->force_no_rx_coe) {
			priv->plat->rx_coe = priv->dma_cap.rx_coe;

			if (priv->dma_cap.rx_coe_type2)
				priv->plat->rx_coe = TC9562MAC_RX_COE_TYPE2;
			else if (priv->dma_cap.rx_coe_type1)
				priv->plat->rx_coe = TC9562MAC_RX_COE_TYPE1;
		} else {
			priv->plat->rx_coe = TC9562MAC_RX_COE_NONE;
		}

	} else {
		dev_info(priv->device, "No HW DMA feature register supported\n");
	}

	/* To use alternate (extended), normal or GMAC4 descriptor structures */
	if (priv->synopsys_id >= DWMAC_CORE_4_00)
		priv->hw->desc = &dwmac4_edesc_ops;
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE	
	else
		tc9562mac_selec_desc_mode(priv);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */ 

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,15))
	priv->hw->tc = &dwmac510_tc_ops;
#endif
	if (priv->plat->rx_coe) {
		priv->hw->rx_csum = priv->plat->rx_coe;
		dev_info(priv->device, "RX Checksum Offload Engine supported\n");
		if (priv->synopsys_id < DWMAC_CORE_4_00)
			dev_info(priv->device, "COE Type %d\n", priv->hw->rx_csum);
	}
	if (priv->plat->tx_coe)
		dev_info(priv->device, "TX Checksum insertion supported\n");

	if (priv->plat->pmt) {
		dev_info(priv->device, "Wake-Up On Lan supported\n");
		device_set_wakeup_capable(priv->device, 1);
		dev_info(priv->device, "Wake-Up On Lan enabled\n");
		device_set_wakeup_enable(priv->device, 1);
		priv->wolopts = WAKE_MAGIC;
	}

	if (priv->dma_cap.tsoen)
		dev_info(priv->device, "TSO supported\n");
	DBGPR_FUNC("<--tc9562mac_hw_init \n");

	return 0;
}

#ifdef TC9562_DEFINED
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
/**
 * tc9562mac_reset_subtask
 * @priv: driver private structure
 * Description: Performs driver reset, when needed
 */
static void tc9562mac_reset_subtask(struct tc9562mac_priv *priv)
{
	if (!test_and_clear_bit(TC9562MAC_RESET_REQUESTED, &priv->state))
		return;
	if (test_bit(TC9562MAC_DOWN, &priv->state))
		return;

	DBGPR_FUNC("-->tc9562mac_reset_subtask\n");

	netdev_err(priv->dev, "Reset adapter\n");

	rtnl_lock();
	
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(4, 6, 7))
	priv->dev->trans_start = jiffies;	
#else	
    netif_trans_update(priv->dev);
#endif	
	while (test_and_set_bit(TC9562MAC_RESETTING, &priv->state))
		usleep_range(1000, 2000);

	set_bit(TC9562MAC_DOWN, &priv->state);
	dev_close(priv->dev);

	smp_mb__before_atomic();
	clear_bit(TC9562MAC_DOWN, &priv->state);
	clear_bit(TC9562MAC_RESETTING, &priv->state);
	dev_open(priv->dev);
	rtnl_unlock();

	DBGPR_FUNC("<--tc9562mac_reset_subtask\n");
	
}

/**
 * tc9562mac_service_task
 * @work: work to be performed
 * Description: Performs deferred work.
 */
static void tc9562mac_service_task(struct work_struct *work)
{
	struct tc9562mac_priv *priv = container_of(work, struct tc9562mac_priv,
			service_task);

	DBGPR_FUNC("-->tc9562mac_service_task\n");
	
	tc9562mac_reset_subtask(priv);
	smp_mb__before_atomic();
	clear_bit(TC9562MAC_SERVICE_SCHED, &priv->state);
	DBGPR_FUNC("<--tc9562mac_service_task\n");
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
#endif


/*!
 * \brief API to kernel read from file
 *
 * \param[in] file   - pointer to file descriptor
 * \param[in] offset - Offset of file to start read
 * \param[in] size   - Size of the buffer to be read
 *
 * \param[out] data   - File data buffer
 *
 *
 * \return integer
 *
 * \retval 0 on success & -ve number on failure.
 */
static int file_read(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size)
{
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
	    ret = vfs_read(file, data, size, &offset);
#else //4.14.0
		ret = kernel_read(file, data, size, &offset);
#endif //4.14.0

    set_fs(oldfs);
    return ret;
}

/*!
 * \brief API to validate MAC ID
 *
 * \param[in] char *s - pointer to MAC ID string
 *
 * \return boolean
 *
 * \retval true on success and false on failure.
 */
bool isMAC(char *s)
{
    int i = 0;
    if (s == NULL)
	return false;

    for (i = 0; i < 17; i++) {
        if (i % 3 != 2 && !isxdigit(s[i]))
            return false;
        if (i % 3 == 2 && s[i] != ':')
            return false;
    }
    return true;
}

/*!
 * \brief API to extract MAC ID from given string
 *
 * \param[in] char *string - pointer to MAC ID string
 *
 * \return None
 */
void extract_macid(char *string)
{
	char *token_m = NULL;
	int j = 0;
	int mac_id = 0;

	/* Extract MAC ID byte by byte */
	token_m = strsep(&string, ":");
	while (token_m != NULL) {
		sscanf(token_m, "%x", &mac_id);
		dev_addr[j++] = mac_id;
		token_m = strsep(&string, ":");
	}
}

/*!
 * \brief API to parse and extract the user configured MAC ID
 *
 * \param[in] file_buf - Pointer to file data buffer
 *
 * \return boolean
 *
 * \return - True on Success and False in failure
 */
static bool lookfor_macid(char *file_buf)
{
	char *string = NULL, *token_n = NULL, *token_s = NULL, *token_m = NULL;
	bool status = false;
	int tc9562_device_no = 0;

	string = file_buf;
	/* Parse Line-0 */
	token_n = strsep(&string, "\n");
	while (token_n != NULL) {

		/* Check if line is enabled */
		if (token_n[0] != '#') {
			/* Extract the token based space character */
			token_s = strsep(&token_n, " ");
			if (token_s != NULL) {
			if (strncmp(token_s, config_param_list[0].mdio_key, 9) == 0) {
					token_s = strsep(&token_n, " ");
					token_m = strsep(&token_s, ":");
					sscanf(token_m, "%d", &tc9562_device_no);
					if (tc9562_device_no != mdio_bus_id) {
						token_n = strsep(&string, "\n");
						if (token_n == NULL)
							break;
						continue;
					}
				}
			}

			/* Extract the token based space character */
			token_s = strsep(&token_n, " ");
			if (token_s != NULL) {
				/* Compare if parsed string matches with key listed in configuration table */
				if (strncmp(token_s, config_param_list[0].mac_key, 6) == 0) {

					NDBGPR_L1("MAC_ID Key is found\n");
					/* Read next word */
					token_s = strsep(&token_n, " \n");
					if (token_s != NULL) {

						/* Check if MAC ID length  and MAC ID is valid */
						if ((isMAC(token_s) == true) && (strlen(token_s) ==  config_param_list[0].mac_str_len)) {
							/* If user configured MAC ID is valid,  assign default MAC ID */
							extract_macid(token_s);
							status = true;
						} else {
							NMSGPR_ALERT("Valid Mac ID not found\n");
						}
					}
				}
			}
		}
		/* Read next lile */
		token_n = strsep(&string, "\n");
        if (token_n == NULL)
			break;

	}
	return status;
}

/*!
 * \brief Parse the user configuration file for various config
 *
 * \param[in] None
 *
 * \return None
 *
 */
static void parse_config_file(void)
{
	struct file *filep = NULL;
	char *data = kmalloc(1000, GFP_KERNEL);
	mm_segment_t oldfs;
	int ret, flags = O_RDONLY, i = 0;

	DBGPR_FUNC("-->parse_config_file\n");
	oldfs = get_fs();
	set_fs(get_ds());
	filep = filp_open("config.ini", flags, 0600);
	set_fs(oldfs);
	if (IS_ERR(filep)) {
		NMSGPR_ALERT("Mac configuration file not found\n");
		NMSGPR_ALERT("Using Default MAC Address\n");
		return;
	} else {
		/* Parse the file */
		ret = file_read(filep, 0, data, 1000);
		for (i = 0; i < CONFIG_PARAM_NUM; i++) {
			if (strstr((const char *)data, config_param_list[i].mdio_key)) {
				NDBGPR_L1("Pattern Match\n");
				if (strncmp(config_param_list[i].mdio_key, "MDIOBUSID", 9) == 0) {
					/* MAC ID Configuration */
					NDBGPR_L1("MAC_ID Configuration\n");
					if (lookfor_macid(data) == false) {
						//extract_macid ((char *)config_param_list[i].str_def);
					}
				}
			}
		}
	}

	kfree(data);
	filp_close(filep, NULL);
	DBGPR_FUNC("<--parse_config_file\n");
	return;
}


/*!
* \brief This sequence is used to get Tx queue count
* \param[in] count
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

unsigned char tc9562_get_tx_channel_count(void __iomem *reg_pci_base_addr)
{
	unsigned char count;
	unsigned long varMAC_HFR2;
	DBGPR_FUNC("-->tc9562_get_tx_channel_count\n");
	varMAC_HFR2 = readl(reg_pci_base_addr + GMAC_HW_FEATURE2);
	count = ((varMAC_HFR2 & GMAC_HW_FEAT_TXCHCNT) >> 18) + 1;
	DBGPR_FUNC("<--tc9562_get_tx_channel_count\n");
	return (count);
}

/*!
* \brief This sequence is used to get Rx queue count
* \param[in] count
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

unsigned char tc9562_get_rx_channel_count(void __iomem *reg_pci_base_addr)
{
	unsigned char count;
	unsigned long varMAC_HFR2;

	DBGPR_FUNC("-->tc9562_get_rx_channel_count\n");
	varMAC_HFR2 = readl(reg_pci_base_addr + GMAC_HW_FEATURE2);
	count = ((varMAC_HFR2 & GMAC_HW_FEAT_RXCHCNT) >> 12) + 1;
	DBGPR_FUNC("<--tc9562_get_rx_channel_count\n");
	return (count);
}


static phy_interface_t tc9562_get_phy_interface(struct tc9562mac_resources *res)
{
    phy_interface_t ret = PHY_INTERFACE_MODE_MII;
    u32 hw_feat0 = readl(res->addr + GMAC_HW_FEATURE0);
    
    DBGPR_FUNC("--> tc9562_get_phy_interface \n");
    
    if (((hw_feat0 & GMAC_HW_FEAT_ACTPHYSEL) >> GMAC_HW_FEAT_ACTPHYSEL_SHIFT) == PHY_INTF_GMII_MII) {
		if (hw_feat0 & GMAC_HW_FEAT_GMIISEL) {
			NMSGPR_INFO("Phy is GMII\n");
			ret = PHY_INTERFACE_MODE_GMII;
		} else if (hw_feat0 & GMAC_HW_FEAT_MIISEL) {
			NMSGPR_INFO("Phy is MII\n");
			ret = PHY_INTERFACE_MODE_MII;
		}
	} else if (((hw_feat0 & GMAC_HW_FEAT_ACTPHYSEL) >> GMAC_HW_FEAT_ACTPHYSEL_SHIFT) == PHY_INTF_RGMII) {
		NMSGPR_INFO("Phy is RGMII\n");
		ret = PHY_INTERFACE_MODE_RGMII;
	} else if (((hw_feat0 & GMAC_HW_FEAT_ACTPHYSEL) >> GMAC_HW_FEAT_ACTPHYSEL_SHIFT) == PHY_INTF_SGMII) {
		NMSGPR_INFO("Phy is SGMII\n");
		ret = PHY_INTERFACE_MODE_SGMII;
	} else if (((hw_feat0 & GMAC_HW_FEAT_ACTPHYSEL) >> GMAC_HW_FEAT_ACTPHYSEL_SHIFT) == PHY_INTF_TBI) {
		NMSGPR_INFO("Phy is TBI\n");
		ret = PHY_INTERFACE_MODE_TBI;
	} else if (((hw_feat0 & GMAC_HW_FEAT_ACTPHYSEL) >> GMAC_HW_FEAT_ACTPHYSEL_SHIFT) == PHY_INTF_RMII) {
		NMSGPR_INFO("Phy is RMII\n");
		ret = PHY_INTERFACE_MODE_RMII;
	} else if (((hw_feat0 & GMAC_HW_FEAT_ACTPHYSEL) >> GMAC_HW_FEAT_ACTPHYSEL_SHIFT) == PHY_INTF_RTBI) {
		NMSGPR_INFO("Phy is RTBI\n");
		ret = PHY_INTERFACE_MODE_RTBI;
	} else if (((hw_feat0 & GMAC_HW_FEAT_ACTPHYSEL) >> GMAC_HW_FEAT_ACTPHYSEL_SHIFT) == PHY_INTF_SMII) {
		NMSGPR_INFO("Phy is SMII\n");
		ret = PHY_INTERFACE_MODE_SMII;
	} else if (((hw_feat0 & GMAC_HW_FEAT_ACTPHYSEL) >> GMAC_HW_FEAT_ACTPHYSEL_SHIFT) == PHY_INTF_RevMII) {
		NMSGPR_INFO("Phy is RevMII: Not supported\n");
		ret = PHY_INTERFACE_MODE_REVMII;
	} else {
		NMSGPR_ALERT("Missing interface support between PHY and MAC\n");
		ret = PHY_INTERFACE_MODE_NA;
	}

    DBGPR_FUNC("<-- tc9562_get_phy_interface \n");
    return ret;    
}

/**
 * tc9562mac_dvr_probe
 * @device: device pointer
 * @plat_dat: platform data pointer
 * @res: tc9562mac resource pointer
 * Description: this is the main probe function used to
 * call the alloc_etherdev, allocate the priv structure.
 * Return:
 * returns 0 on success, otherwise errno.
 */
int tc9562mac_dvr_probe(struct device *device,
		     struct plat_tc9562macenet_data *plat_dat,
		     struct tc9562mac_resources *res)
{
	struct net_device *ndev = NULL;
	struct tc9562mac_priv *priv;
	int ret = 0;
	u32 queue;
    u32 reg;
	unsigned char tx_ch_count = 0, rx_ch_count = 0;

	DBGPR_FUNC("-->tc9562mac_dvr_probe\n");
	
    /* Update PHY interface */
    plat_dat->interface = tc9562_get_phy_interface(res);
    
	NDBGPR_L1("HFR0 Val = 0x%08x \n", readl(res->addr + GMAC_HW_FEATURE0));
	NDBGPR_L1("HFR1 Val = 0x%08x \n", readl(res->addr + GMAC_HW_FEATURE1));
	NDBGPR_L1("HFR2 Val = 0x%08x \n", readl(res->addr + GMAC_HW_FEATURE2));
	NDBGPR_L1("HFR3 Val = 0x%08x \n", readl(res->addr + GMAC_HW_FEATURE3));

	 /* Channel count */
	tx_ch_count = tc9562_get_tx_channel_count(res->addr);
	rx_ch_count = tc9562_get_rx_channel_count(res->addr);
	
	NDBGPR_L1("No of TX Channels = %d\n", tx_ch_count);
	NDBGPR_L1("No of RX Channels = %d\n", rx_ch_count);

	ndev = alloc_etherdev_mqs(sizeof(struct tc9562mac_priv),
				  tx_ch_count, rx_ch_count);
	if (!ndev) {
		NMSGPR_ALERT("%s:Unable to alloc new net device\n",TC9562_RESOURCE_NAME);
		return -ENOMEM;
	}

	SET_NETDEV_DEV(ndev, device);

	priv = netdev_priv(ndev);
	priv->device = device;
	priv->dev = ndev;


    priv->mac_loopback_mode = 0; //Disable MAC loopback by default
    priv->phy_loopback_mode = 0; //Disable PHY loopback by default

    reg = readl(res->addr + NMODESTS_OFFSET);
    printk("0x%x\n", reg);
    if((reg & TC9562_NMODESTS_HOST_BOOT_MASK) == TC9562_NMODESTS_HOST_BOOT_MASK) {
        ret = tc9562_load_firmware(device, res);
        if(ret < 0) {
            NMSGPR_ERR("Firmware load failed\n");
            goto error_firmware_load_failed;
        }
    }
	tc9562mac_set_ethtool_ops(ndev);
	priv->pause = pause;
	priv->plat = plat_dat;
	priv->ioaddr = res->addr;
	priv->dev->base_addr = (unsigned long)res->addr;
	priv->tc9562_SRAM_pci_base_addr = res->tc9562_SRAM_pci_base_addr;
	priv->tc9562_FLASH_pci_base_addr = res->tc9562_FLASH_pci_base_addr;

	NDBGPR_L2(" Base address priv->ioaddr: %p", priv->ioaddr);
	priv->dev->irq = res->irq;
	priv->wol_irq = res->irq;
	priv->lpi_irq = res->irq;

#ifdef UNIFIED_DRIVER
	priv->tdm_start = 0;
#endif
	/* Read mac address from config.ini file */
	++mdio_bus_id;
	parse_config_file();
	res->mac = dev_addr;
	NMSGPR_INFO("MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
	dev_addr[0], dev_addr[1], dev_addr[2], dev_addr[3], dev_addr[4], dev_addr[5]);
	
	priv->sa_vlan_ins_via_desc = TC9562_SA0_NONE;
	priv->sa_vlan_ins_via_reg = TC9562_SA0_NONE;
	priv->plat->bus_id = mdio_bus_id;
	
	if (res->mac)
		memcpy(priv->dev->dev_addr, res->mac, ETH_ALEN);

	dev_set_drvdata(device, priv->dev);

	/* Verify driver arguments */
	tc9562mac_verify_args();

#ifdef TC9562_DEFINED
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	/* Allocate workqueue */
	priv->wq = create_singlethread_workqueue("tc9562mac_wq");
	if (!priv->wq) {
		dev_err(priv->device, "failed to create workqueue\n");
		goto error_wq;
	}

	INIT_WORK(&priv->service_task, tc9562mac_service_task);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */	
#endif

	/* Override with kernel parameters if supplied XXX CRS XXX
	 * this needs to have multiple instances
	 */
	if ((phyaddr >= 0) && (phyaddr <= 31))
		priv->plat->phy_addr = phyaddr;

	if (priv->plat->tc9562mac_rst) {
		ret = reset_control_assert(priv->plat->tc9562mac_rst);
		reset_control_deassert(priv->plat->tc9562mac_rst);
		/* Some reset controllers have only reset callback instead of
		 * assert + deassert callbacks pair.
		 */
		if (ret == -ENOTSUPP)
			reset_control_reset(priv->plat->tc9562mac_rst);
	}

	/* Init MAC and get the capabilities */
	ret = tc9562mac_hw_init(priv);
	if (ret){
		NMSGPR_ALERT("Hardware init failure \n");
		goto error_hw_init;
	}

	/* Configure real RX and TX queues */
	netif_set_real_num_rx_queues(ndev, priv->plat->rx_queues_to_use);
	netif_set_real_num_tx_queues(ndev, priv->plat->tx_queues_to_use);

	ndev->netdev_ops = &tc9562mac_netdev_ops;

	ndev->hw_features = NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
			    NETIF_F_RXCSUM;
#ifdef TEMP_SG_4TSO
	ndev->hw_features |= NETIF_F_SG; 
#endif

	if ((priv->plat->tso_en) && (priv->dma_cap.tsoen)) {
		ndev->hw_features |= NETIF_F_TSO | NETIF_F_TSO6;
		priv->tso = true;
		dev_info(priv->device, "TSO feature enabled\n");
	}else{
		priv->tso = false;
	}
	printk("priv->tso : %d\n", priv->tso);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,15))
	ret = priv->hw->tc->init(ndev);
	if (!ret) {
		ndev->hw_features |= NETIF_F_HW_TC;
	}
#endif
	ndev->features |= ndev->hw_features | NETIF_F_HIGHDMA;
	NDBGPR_L2("ndev->hw_features : %llx", ndev->hw_features);
	ndev->watchdog_timeo = msecs_to_jiffies(watchdog);
#ifdef TC9562MAC_VLAN_TAG_USED
	/* Both mac100 and gmac support receive VLAN tag detection */
	ndev->features |= NETIF_F_HW_VLAN_CTAG_RX;
	ndev->hw_features |= NETIF_F_HW_VLAN_CTAG_RX;
	if(priv->dma_cap.sa_vlan_ins) 
	{ 
		ndev->hw_features |= NETIF_F_HW_VLAN_CTAG_TX;
		ndev->features |= NETIF_F_HW_VLAN_CTAG_TX;
	}
  
  if(priv->dma_cap.hash_filter) {
        ndev->hw_features |= NETIF_F_HW_VLAN_CTAG_FILTER;
        ndev->features |= NETIF_F_HW_VLAN_CTAG_FILTER;
  }
#endif
	priv->msg_enable = netif_msg_init(debug, default_msg_level);

//for 3.19, max_mtu and min_mtu check in change mtu.
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0))

	/* MTU range: 46 - hw-specific max */
	ndev->min_mtu = MIN_SUPPORTED_MTU;
	if ((priv->plat->enh_desc) || (priv->synopsys_id >= DWMAC_CORE_4_00))
		ndev->max_mtu = JUMBO_LEN;
	else
		ndev->max_mtu = SKB_MAX_HEAD(NET_SKB_PAD + NET_IP_ALIGN);
	/* Will not overwrite ndev->max_mtu if plat->maxmtu > ndev->max_mtu
	 * as well as plat->maxmtu < ndev->min_mtu which is a invalid range.
	 */
	if ((priv->plat->maxmtu < ndev->max_mtu) &&
	    (priv->plat->maxmtu >= ndev->min_mtu))
		ndev->max_mtu = priv->plat->maxmtu;
	else if (priv->plat->maxmtu < ndev->min_mtu)
		dev_warn(priv->device,
			 "%s: warning: maxmtu having invalid value (%d)\n",
			 __func__, priv->plat->maxmtu);
#endif

	if (flow_ctrl)
		priv->flow_ctrl = FLOW_AUTO;	/* RX/TX pause on */

	/* Rx Watchdog is available in the COREs newer than the 3.40.
	 * In some case, for example on bugged HW this feature
	 * has to be disable and this can be done by passing the
	 * riwt_off field from the platform.
	 */
	if ((priv->synopsys_id >= DWMAC_CORE_3_50) && (!priv->plat->riwt_off)) {
		priv->use_riwt = 1;
		dev_info(priv->device,
			 "Enable RX Mitigation via HW Watchdog Timer\n");
	}

    /* Tx interrupts are also handled by Rx napi. Create napi for 6 channels */
	for (queue = 0; queue < (priv->plat->tx_queues_to_use ); queue++) {
		struct tc9562mac_rx_queue *rx_q = &priv->rx_queue[queue];

		netif_napi_add(ndev, &rx_q->napi, tc9562mac_poll,
			       (8 * (priv->plat->tx_queues_to_use)));
	}

	spin_lock_init(&priv->lock);
    spin_lock_init(&priv->mmc_lock);
    
	/* If a specific clk_csr value is passed from the platform
	 * this means that the CSR Clock Range selection cannot be
	 * changed at run-time and it is fixed. Viceversa the driver'll try to
	 * set the MDC clock dynamically according to the csr actual
	 * clock input.
	 */
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	if (!priv->plat->clk_csr)
		tc9562mac_clk_csr_set(priv);
	else
#endif  /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
		priv->clk_csr = priv->plat->clk_csr;

	tc9562mac_check_pcs_mode(priv);

	 if (priv->hw->pcs != TC9562MAC_PCS_RGMII  &&
	    priv->hw->pcs != TC9562MAC_PCS_TBI &&
	    priv->hw->pcs != TC9562MAC_PCS_RTBI) {
		/* MDIO bus Registration */
		ret = tc9562mac_mdio_register(ndev);
		if (ret < 0) {
			dev_err(priv->device,
				"%s: MDIO bus (id: %d) registration failed",
				__func__, priv->plat->bus_id);
			goto error_mdio_register;
		}
	}

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(priv->device, "%s: ERROR %i registering the device\n",
			__func__, ret);
		goto error_netdev_register;
	}

	DBGPR_FUNC("<--tc9562mac_dvr_probe\n");

	return ret;

error_netdev_register:
	if (priv->hw->pcs != TC9562MAC_PCS_RGMII &&
	    priv->hw->pcs != TC9562MAC_PCS_TBI &&
	    priv->hw->pcs != TC9562MAC_PCS_RTBI)
		tc9562mac_mdio_unregister(ndev);
error_mdio_register:
	for (queue = 0; queue < (priv->plat->tx_queues_to_use); queue++) {
		struct tc9562mac_rx_queue *rx_q = &priv->rx_queue[queue];

		netif_napi_del(&rx_q->napi);
	}
error_hw_init:
//#if 0
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	destroy_workqueue(priv->wq);

error_wq:
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
error_firmware_load_failed:
//#endif
	free_netdev(ndev);
	return ret;
}
EXPORT_SYMBOL_GPL(tc9562mac_dvr_probe);

/**
 * tc9562mac_dvr_remove
 * @dev: device pointer
 * Description: this function resets the TX/RX processes, disables the MAC RX/TX
 * changes the link status, releases the DMA descriptor rings.
 */
int tc9562mac_dvr_remove(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct tc9562mac_priv *priv = netdev_priv(ndev);

	DBGPR_FUNC("-->tc9562mac_dvr_remove\n");
	
	netdev_info(priv->dev, "%s: removing driver", __func__);

	tc9562mac_stop_all_dma(priv);

	priv->hw->mac->set_mac(priv->ioaddr, false);
	netif_carrier_off(ndev);
	unregister_netdev(ndev);
	if (priv->plat->tc9562mac_rst)
		reset_control_assert(priv->plat->tc9562mac_rst);
	clk_disable_unprepare(priv->plat->pclk);
	clk_disable_unprepare(priv->plat->tc9562mac_clk);
	if (priv->hw->pcs != TC9562MAC_PCS_RGMII &&
	    priv->hw->pcs != TC9562MAC_PCS_TBI &&
	    priv->hw->pcs != TC9562MAC_PCS_RTBI)
		tc9562mac_mdio_unregister(ndev);
//#if 0
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	destroy_workqueue(priv->wq);
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
//#endif
	free_netdev(ndev);

	DBGPR_FUNC("<--tc9562mac_dvr_remove\n");

	return 0;
}
EXPORT_SYMBOL_GPL(tc9562mac_dvr_remove);

/**
 * tc9562mac_suspend - suspend callback
 * @dev: device pointer
 * Description: this is the function to suspend the device and it is called
 * by the platform driver to stop the network queue, release the resources,
 * program the PMT register (for WoL), clean and release driver resources.
 */
int tc9562mac_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct tc9562mac_priv *priv = netdev_priv(ndev);
	unsigned long flags;

	DBGPR_FUNC("-->tc9562mac_suspend\n");

	if (!ndev )
		return 0;
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
    if (priv->eee_enabled)
		tc9562mac_disable_eee_mode(priv); 
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
    
    priv->hw->mac->set_umac_addr(priv->hw, ndev->dev_addr, 0, 0);
    if(netif_running(ndev)) {
        rtnl_lock();
        dev_close(ndev);
        rtnl_unlock();
    }
	if (ndev->phydev)
		phy_stop(ndev->phydev);

	spin_lock_irqsave(&priv->lock, flags);

	netif_device_detach(ndev);
	tc9562mac_stop_all_queues(priv);

	/* Stop TX/RX DMA */
	tc9562mac_stop_all_dma(priv);

    /* Enable Power down mode by programming the PMT regs */
    priv->hw->mac->pmt(priv->hw, priv->wolopts);
	priv->irq_wake = 1;
	#if 0	
	/* Enable Power down mode by programming the PMT regs */
	if (device_may_wakeup(priv->device)) {
		priv->hw->mac->pmt(priv->hw, priv->wolopts);
		priv->irq_wake = 1;
	} else {
		priv->hw->mac->set_mac(priv->ioaddr, false);
		pinctrl_pm_select_sleep_state(priv->device);
		/* Disable clock in case of PWM is off */
		clk_disable(priv->plat->pclk);
		clk_disable(priv->plat->tc9562mac_clk);
	}
	#endif
	spin_unlock_irqrestore(&priv->lock, flags);

	priv->oldlink = false;
	priv->speed = SPEED_UNKNOWN;
	priv->oldduplex = DUPLEX_UNKNOWN;

	DBGPR_FUNC("<--tc9562mac_suspend\n");
	
	return 0;
}
EXPORT_SYMBOL_GPL(tc9562mac_suspend);

/**
 * tc9562mac_resume - resume callback
 * @dev: device pointer
 * Description: when resume this function is invoked to setup the DMA and CORE
 * in a usable state.
 */
int tc9562mac_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct tc9562mac_priv *priv = netdev_priv(ndev);
	unsigned long flags;
    int reg_data = 0,ret = 0;
    struct tc9562mac_resources res;
	uint8_t SgmSigPol = 1; /* To handle SGM_SIG_DET */

	DBGPR_FUNC("-->tc9562mac_resume\n");

    reg_data = readl(priv->ioaddr + 0x1008);
	reg_data |= (0x1<<7);
	writel(reg_data, priv->ioaddr + 0x1008);

	if (ENABLE_SGMII_INTERFACE == INTERFACE_SELECTED){
        /* SGMII interface */
		if(1 == ENABLE_SGM_SIG_DET){
            reg_data = readl(priv->ioaddr + 0x1524);
            reg_data = (reg_data & ~0x00f00000) | 0x00200000;
            writel(reg_data, priv->ioaddr + 0x1524);	
			SgmSigPol = 0; /* Active High */
		}else if(2 == ENABLE_SGM_SIG_DET){
			reg_data = readl(priv->ioaddr + 0x1524);
			reg_data = (reg_data & ~0x00f00000) | 0x00200000;
			writel(reg_data, priv->ioaddr + 0x1524);
			
			SgmSigPol = 1; /* Active low */
		}
	}

	/* Interface configuration */
	reg_data = readl(priv->ioaddr + 0x0010);
	reg_data &= 0xffffffc7;
	if(ENABLE_RMII_INTERFACE == INTERFACE_SELECTED)
		reg_data |= 0x20;
	else if(ENABLE_RGMII_INTERFACE == INTERFACE_SELECTED)
		reg_data |= 0x8;		
 	else if(ENABLE_SGMII_INTERFACE == INTERFACE_SELECTED)
		reg_data |= 0x10;

	reg_data &= ~(0x00000800); /* Mask Polarity */
	if(1 == SgmSigPol){
		reg_data |= 0x00000800; /* Set Active low */
	}
    writel(reg_data, priv->ioaddr + 0x0010);

	/* De-assertion of EMAC software Reset*/
	reg_data = readl(priv->ioaddr + 0x1008);
	reg_data &= 0xFFFFFF7F;
	writel(reg_data, priv->ioaddr + 0x1008);	

    res.addr = priv->ioaddr;
    res.tc9562_SRAM_pci_base_addr = priv->tc9562_SRAM_pci_base_addr;
	res.tc9562_FLASH_pci_base_addr = priv->tc9562_FLASH_pci_base_addr;
	res.irq = priv->dev->irq ;
    reg_data = readl(priv->ioaddr + NMODESTS_OFFSET);
    printk("0x%x\n", reg_data);
    if((reg_data & TC9562_NMODESTS_HOST_BOOT_MASK) == TC9562_NMODESTS_HOST_BOOT_MASK) {
        ret = tc9562_load_firmware(dev, &res);
        if(ret < 0) {
            NMSGPR_ERR("Firmware load failed\n");
            return -EINVAL;
        }
    }

	/* Power Down bit, into the PM register, is cleared
	 * automatically as soon as a magic packet or a Wake-up frame
	 * is received. Anyway, it's better to manually clear
	 * this bit because it can generate problems while resuming
	 * from another devices (e.g. serial console).
	 */
    spin_lock_irqsave(&priv->lock, flags);
	priv->hw->mac->pmt(priv->hw, 0);
	spin_unlock_irqrestore(&priv->lock, flags);
	priv->irq_wake = 0;
	#if 0	
	if (device_may_wakeup(priv->device)) {
		spin_lock_irqsave(&priv->lock, flags);
		priv->hw->mac->pmt(priv->hw, 0);
		spin_unlock_irqrestore(&priv->lock, flags);
		priv->irq_wake = 0;
	} else {
		pinctrl_pm_select_default_state(priv->device);
		/* enable the clk previously disabled */
		clk_enable(priv->plat->tc9562mac_clk);
		clk_enable(priv->plat->pclk);
		/* reset the phy so that it's ready */
		if (priv->mii)
			tc9562mac_mdio_reset(priv->mii);
	}
    #endif 
	netif_device_attach(ndev);

	spin_lock_irqsave(&priv->lock, flags);

	tc9562mac_reset_queues_param(priv);

	/* reset private mss value to force mss context settings at
	 * next tso xmit (only used for gmac4).
	 */
	priv->mss = 0;

	tc9562mac_clear_descriptors(priv);

	//tc9562mac_hw_setup(ndev, false);
	//tc9562mac_init_tx_coalesce(priv);
	//tc9562mac_set_rx_mode(ndev);

	tc9562mac_start_all_queues(priv);

	spin_unlock_irqrestore(&priv->lock, flags);

	if (ndev->phydev)
		phy_start(ndev->phydev);

    if(!netif_running(ndev)) {
        rtnl_lock();
        dev_open(ndev);
        rtnl_unlock();
    }
	DBGPR_FUNC("<--tc9562mac_resume\n");

	return 0;
}
EXPORT_SYMBOL_GPL(tc9562mac_resume);

#ifndef MODULE
static int __init tc9562mac_cmdline_opt(char *str)
{
	char *opt;

	DBGPR_FUNC("-->tc9562mac_cmdline_opt\n");

	if (!str || !*str)
		return -EINVAL;
	while ((opt = strsep(&str, ",")) != NULL) {
		if (!strncmp(opt, "debug:", 6)) {
			if (kstrtoint(opt + 6, 0, &debug))
				goto err;
		} else if (!strncmp(opt, "phyaddr:", 8)) {
			if (kstrtoint(opt + 8, 0, &phyaddr))
				goto err;
		} else if (!strncmp(opt, "buf_sz:", 7)) {
			if (kstrtoint(opt + 7, 0, &buf_sz))
				goto err;
		} else if (!strncmp(opt, "tc:", 3)) {
			if (kstrtoint(opt + 3, 0, &tc))
				goto err;
		} else if (!strncmp(opt, "watchdog:", 9)) {
			if (kstrtoint(opt + 9, 0, &watchdog))
				goto err;
		} else if (!strncmp(opt, "flow_ctrl:", 10)) {
			if (kstrtoint(opt + 10, 0, &flow_ctrl))
				goto err;
		} else if (!strncmp(opt, "pause:", 6)) {
			if (kstrtoint(opt + 6, 0, &pause))
				goto err;
		} else if (!strncmp(opt, "eee_timer:", 10)) {
			if (kstrtoint(opt + 10, 0, &eee_timer))
				goto err;
		} else if (!strncmp(opt, "chain_mode:", 11)) {
			if (kstrtoint(opt + 11, 0, &chain_mode))
				goto err;
		}
	}

	DBGPR_FUNC("<--tc9562mac_cmdline_opt\n");
	
	return 0;

err:
	pr_err("%s: ERROR broken module parameter conversion", __func__);
	return -EINVAL;
}

__setup("tc9562maceth=", tc9562mac_cmdline_opt);
#endif /* MODULE */

#ifndef TC9562_DEFINED
static int __init tc9562mac_init(void)
{
#else
int tc9562mac_init(void)
{
	DBGPR_FUNC("-->tc9562mac_init\n");
#endif
#ifdef CONFIG_DEBUG_FS
	/* Create debugfs main directory if it doesn't exist yet */
	if (!tc9562mac_fs_dir) {
		tc9562mac_fs_dir = debugfs_create_dir(TC9562_RESOURCE_NAME, NULL);

		if (!tc9562mac_fs_dir || IS_ERR(tc9562mac_fs_dir)) {
			pr_err("ERROR %s, debugfs create directory failed\n",
			       TC9562_RESOURCE_NAME);

			return -ENOMEM;
		}
	}
#endif

	return 0;
}

#ifndef TC9562_DEFINED
static void __exit tc9562mac_exit(void)
{
#else
void tc9562mac_exit(void)
{
	DBGPR_FUNC("-->tc9562mac_exit\n");
#endif
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(tc9562mac_fs_dir);
#endif
}

#ifndef TC9562_DEFINED
module_init(tc9562mac_init)
module_exit(tc9562mac_exit)
#endif
