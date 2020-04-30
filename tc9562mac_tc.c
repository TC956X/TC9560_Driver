/*
 * TC9562 ethernet driver.
 *
 * tc9562mac_tc.c
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
 *  26 Feb 2020 : Added TC - CBS, FRP and Launch time feature support.
 *  VERSION     : 01-01
 */
#include <net/pkt_cls.h>
#include <net/tc_act/tc_gact.h>
#include "common.h"
#include "dwmac4.h"
#include "dwmac5.h"
#include "tc9562mac.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,15))
#include <linux/iopoll.h>

/**			
 * tc_launch_time - Enables launch time for the queue
 * @ndev: netdevice structure
 * @q: struct tc_etf_qopt_offload 
 */

int tc_launch_time(struct net_device *ndev,
			void *q)
{
	struct tc9562mac_priv *priv = netdev_priv(ndev);
	struct tc_etf_qopt_offload *qopt = (struct tc_etf_qopt_offload *)q;
	struct tc9562mac_tx_queue *tx_q;

	if((qopt->queue < 0) || (qopt->queue > priv->plat->tx_queues_to_use))
		return -EINVAL;
	
	tx_q = &priv->tx_queue[qopt->queue];

	if( (tx_q->queue_index == AVB_CLASSA_CH) || (tx_q->queue_index == AVB_CLASSB_CH) || (tx_q->queue_index == CLASS_CDT)){
		tx_q->launch_time = 1;

	}else{
		DBGPR_TEST(" Not an AVB queue: %d \n", qopt->queue);

	}

	return 0;	
	
}

static void tc_fill_all_pass_entry(struct tc9562mac_tc_entry *entry)
{
	memset(entry, 0, sizeof(*entry));
	entry->in_use = true;
	entry->is_last = true;
	entry->is_frag = false;
	entry->prio = ~0x0;
	entry->handle = 0;
	entry->val.match_data = 0x0;
	entry->val.match_en = 0x0;
	entry->val.af = 1;
	entry->val.dma_ch_no = 0x0; //DMA channel hardcoded to 0.
}

static struct tc9562mac_tc_entry *tc_find_entry(struct tc9562mac_priv *priv,
					     struct tc_cls_u32_offload *cls,
					     bool free)
{
	struct tc9562mac_tc_entry *entry, *first = NULL, *dup = NULL;
	u32 loc = cls->knode.handle;
	int i;
	
	for (i = 0; i < priv->tc_entries_max; i++) {
		entry = &priv->tc_entries[i];
		if (!entry->in_use && !first && free)
			first = entry;
		if (entry->handle == loc && !free)
			dup = entry;
	}
	
	if (dup)
		return dup;
	if (first) {
		first->handle = loc;
		first->in_use = true;

		/* Reset HW values */
		memset(&first->val, 0, sizeof(first->val));
	}

	return first;
}

static int tc_fill_actions(struct tc9562mac_tc_entry *entry,
			   struct tc9562mac_tc_entry *frag,
			   struct tc_cls_u32_offload *cls)
{
	struct tc9562mac_tc_entry *action_entry = entry;
	const struct tc_action *act;
	struct tcf_exts *exts;
	int i;	
		
	exts = cls->knode.exts;
	if (!tcf_exts_has_actions(exts))
		return -EINVAL;
	if (frag)
		action_entry = frag;

	tcf_exts_for_each_action(i, act, exts) {
		/* Accept */
		if (is_tcf_gact_ok(act)) {
			action_entry->val.af = 1;
			break;
		}
		/* Drop */
		if (is_tcf_gact_shot(act)) {
			action_entry->val.rf = 1;
			break;
		}

	}

	return 0;
}

static int tc_fill_entry(struct tc9562mac_priv *priv,
			 struct tc_cls_u32_offload *cls)
{
	struct tc9562mac_tc_entry *entry, *frag = NULL;
	struct tc_u32_sel *sel = cls->knode.sel;
	u32 off, data, mask, real_off, rem;
	u32 prio = cls->common.prio;
	int ret;

	/* Only 1 match per entry */
	if (sel->nkeys <= 0 || sel->nkeys > 1)
		return -EINVAL;
	

	off = sel->keys[0].off << sel->offshift;
	data = sel->keys[0].val;
	mask = sel->keys[0].mask;
	
	switch (ntohs(cls->common.protocol)) {
	case ETH_P_ALL:
		break;
	case ETH_P_IP:
		off += ETH_HLEN;
		break;
	default:
		return -EINVAL;
		}
	
	if (off > priv->tc_off_max)
		return -EINVAL;

	real_off = off / 4;
	rem = off % 4;

	entry = tc_find_entry(priv, cls, true);
	if (!entry)
		return -EINVAL;
	if (rem) {
		frag = tc_find_entry(priv, cls, true);
		if (!frag) {
			ret = -EINVAL;
			goto err_unuse;
		}
		entry->frag_ptr = frag;
		entry->val.match_en = (mask << (rem * 8)) &
			GENMASK(31, rem * 8);
		entry->val.match_data = (data << (rem * 8)) &
			GENMASK(31, rem * 8);
		entry->val.frame_offset = real_off;
		entry->prio = prio;

		frag->val.match_en = (mask >> (rem * 8)) &
			GENMASK(rem * 8 - 1, 0);
		frag->val.match_data = (data >> (rem * 8)) &
			GENMASK(rem * 8 - 1, 0);
		frag->val.frame_offset = real_off + 1;
		frag->prio = prio;
		frag->is_frag = true;
	} else {
		entry->frag_ptr = NULL;
		entry->val.match_en = mask;
		entry->val.match_data = data;
		entry->val.frame_offset = real_off;
		entry->prio = prio;
	}
	ret = tc_fill_actions(entry, frag, cls);
	if (ret)
		goto err_unuse;

	return 0;

err_unuse:
	if (frag)
		frag->in_use = false;
	entry->in_use = false;
	return ret;
}

static void tc_unfill_entry(struct tc9562mac_priv *priv,
			    struct tc_cls_u32_offload *cls)
{
	struct tc9562mac_tc_entry *entry;

	entry = tc_find_entry(priv, cls, false);
	if (!entry)
		return;

	entry->in_use = false;
	if (entry->frag_ptr) {
		entry = entry->frag_ptr;
		entry->is_frag = false;
		entry->in_use = false;
	}
}

/**			
 * tc_config_knode - Configures FRP entry
 * @priv: driver private structure
 * @q: struct tc_cls_u32_offload 
 */

static int tc_config_knode(struct tc9562mac_priv *priv,
			   struct tc_cls_u32_offload *cls)
{
	int ret;
	
	ret = tc_fill_entry(priv, cls);
	if (ret)
		return ret;

	ret = priv->hw->mac->rx_config(priv, priv->tc_entries,
			priv->tc_entries_max);
	if (ret)
		goto err_unfill;

	return 0;

err_unfill:
	tc_unfill_entry(priv, cls);
	return ret;
}

/**			
 * tc_delete_knode - Deletes FRP entry
 * @priv: driver private structure
 * @q: struct tc_cls_u32_offload 
 */


static int tc_delete_knode(struct tc9562mac_priv *priv,
			   struct tc_cls_u32_offload *cls)
{
	int ret;

	/* Set entry and fragments as not used */
	tc_unfill_entry(priv, cls);

		ret = priv->hw->mac->rx_config(priv, priv->tc_entries,
			priv->tc_entries_max);
	if (ret)
		return ret;

	return 0;
}

int tc_setup_cls_u32(struct net_device *ndev,
			    void *q)
{

	struct tc9562mac_priv *priv = netdev_priv(ndev);
	struct tc_cls_u32_offload *cls = (struct tc_cls_u32_offload *)q;
	
	switch (cls->command) {
		case TC_CLSU32_REPLACE_KNODE:
			tc_unfill_entry(priv, cls);
			/* Fall through */
		case TC_CLSU32_NEW_KNODE:
			return tc_config_knode(priv, cls);
		case TC_CLSU32_DELETE_KNODE:
			return tc_delete_knode(priv, cls);
		default:
			return -EOPNOTSUPP;
	}
}

/**			
 * tc_setup_cbs - Configures CBS parameter
 * @ndev: netdevice structure
 * @q: struct tc_cbs_qopt_offload 
 */

int tc_setup_cbs( struct net_device *ndev,
			void *q )
{
	struct tc9562mac_priv *priv = netdev_priv(ndev);
	struct tc_cbs_qopt_offload *qopt = (struct tc_cbs_qopt_offload *)q;
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 queue = qopt->queue;
	u32 mode_to_use;

	/* Queue 0 is not AVB capable */
	if (queue < 0 || queue >= tx_queues_count)
		return -EINVAL;

	if (priv->speed != SPEED_100 && priv->speed != SPEED_1000)
		return -EOPNOTSUPP;


	mode_to_use = priv->plat->tx_queues_cfg[queue].mode_to_use;
	if ((mode_to_use == MTL_QUEUE_AVB) && qopt->enable) {	
		if(SPEED_100 == priv->speed){
		    priv->cbs_speed100_cfg[queue].send_slope = qopt->sendslope & GENMASK(31, 0);
		    priv->cbs_speed100_cfg[queue].idle_slope = qopt->idleslope & GENMASK(31, 0);
		    priv->cbs_speed100_cfg[queue].high_credit = qopt->hicredit & GENMASK(31, 0);
		    priv->cbs_speed100_cfg[queue].low_credit = qopt->locredit & GENMASK(31, 0);
		}else{		
	    	priv->cbs_speed1000_cfg[queue].send_slope = qopt->sendslope & GENMASK(31, 0);
		    priv->cbs_speed1000_cfg[queue].idle_slope = qopt->idleslope & GENMASK(31, 0);
		    priv->cbs_speed1000_cfg[queue].high_credit = qopt->hicredit & GENMASK(31, 0);
		    priv->cbs_speed1000_cfg[queue].low_credit = qopt->locredit & GENMASK(31, 0);

		}
	    
		if(priv->speed == SPEED_100) {
	       	priv->plat->tx_queues_cfg[queue].send_slope = priv->cbs_speed100_cfg[queue].send_slope;
		    priv->plat->tx_queues_cfg[queue].idle_slope = priv->cbs_speed100_cfg[queue].idle_slope;
		    priv->plat->tx_queues_cfg[queue].high_credit = priv->cbs_speed100_cfg[queue].high_credit;
		    priv->plat->tx_queues_cfg[queue].low_credit = priv->cbs_speed100_cfg[queue].low_credit;
	   	}
		else if(priv->speed == SPEED_1000) {
		    priv->plat->tx_queues_cfg[queue].send_slope = priv->cbs_speed1000_cfg[queue].send_slope;
		    priv->plat->tx_queues_cfg[queue].idle_slope = priv->cbs_speed1000_cfg[queue].idle_slope;
		    priv->plat->tx_queues_cfg[queue].high_credit = priv->cbs_speed1000_cfg[queue].high_credit;
		    priv->plat->tx_queues_cfg[queue].low_credit = priv->cbs_speed1000_cfg[queue].low_credit;
		}

		priv->hw->mac->config_cbs(priv->hw, priv->plat->tx_queues_cfg[queue].send_slope, priv->plat->tx_queues_cfg[queue].idle_slope, priv->plat->tx_queues_cfg[queue].high_credit, priv->plat->tx_queues_cfg[queue].low_credit, queue);

	}else if ((!qopt->enable) || (MTL_QUEUE_DCB == mode_to_use)) {
		DBGPR_TEST("CBS not configured, Not an AVB Queue :%x \n", queue);
		
	}
	return 0;
}

/**			
 * tc_init - Initializes FRP related parameters
 * @ndev: netdevice structure 
 */

int tc_init(struct net_device *ndev)
{
	struct tc9562mac_priv *priv = netdev_priv(ndev);
	struct dma_features *dma_cap = &priv->dma_cap;
	unsigned int count;

	if (!dma_cap->frpsel){
		return -EINVAL;
		}

	switch (dma_cap->frpbs) {
		case 0x0:
			priv->tc_off_max = 64;
			break;
		case 0x1:
			priv->tc_off_max = 128;
			break;
		case 0x2:
			priv->tc_off_max = 256;
			break;
		default:
			return -EINVAL;
	}

	count = dma_cap->frpes;//should check this. sow
	/* Reserve one last filter which lets all pass */
	priv->tc_entries_max = count;
	priv->tc_entries = devm_kcalloc(priv->device,
			count, sizeof(*priv->tc_entries), GFP_KERNEL);
	if (!priv->tc_entries)
		return -ENOMEM;

	tc_fill_all_pass_entry(&priv->tc_entries[count - 1]);

	dev_info(priv->device, "Enabling HW TC (entries=%d, max_off=%d)\n",
			priv->tc_entries_max, priv->tc_off_max);
	
	return 0;
}

const struct tc9562_tc_ops dwmac510_tc_ops = {
	.init = tc_init,
	.setup_cls_u32 = tc_setup_cls_u32,
	.setup_cbs = tc_setup_cbs,
	.launch_time = tc_launch_time,
};
#endif
