/*
 * TC9562 ethernet driver.
 *
 * dwmac4_lib.c
 *
 * Copyright (C) 2007-2015 STMicroelectronics Ltd
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

#include <linux/io.h>
#include <linux/delay.h>
#include "common.h"
#include "dwmac4_dma.h"
#include "dwmac4.h"

int dwmac4_dma_reset(void __iomem *ioaddr)
{
	u32 value = readl(ioaddr + DMA_BUS_MODE);
	int limit;

	DBGPR_FUNC("-->dwmac4_dma_reset\n");


	/* DMA SW reset */
	value |= DMA_BUS_MODE_SFT_RESET;
	writel(value, ioaddr + DMA_BUS_MODE);
	limit = 10;
	while (limit--) {
		if (!(readl(ioaddr + DMA_BUS_MODE) & DMA_BUS_MODE_SFT_RESET))
			break;
		mdelay(10);
	}

	if (limit < 0)
		return -EBUSY;

	DBGPR_FUNC("<--dwmac4_dma_reset\n");

	return 0;
}

void dwmac4_set_rx_tail_ptr(void __iomem *ioaddr, u32 tail_ptr, u32 chan)
{
	DBGPR_FUNC("-->dwmac4_set_rx_tail_ptr\n");
	writel(tail_ptr, ioaddr + DMA_CHAN_RX_END_ADDR(chan));
	DBGPR_FUNC("<--dwmac4_set_rx_tail_ptr\n");
}

void dwmac4_set_tx_tail_ptr(void __iomem *ioaddr, u32 tail_ptr, u32 chan)
{
	DBGPR_FUNC("-->dwmac4_set_tx_tail_ptr\n");
	writel(tail_ptr, ioaddr + DMA_CHAN_TX_END_ADDR(chan));
	DBGPR_FUNC("<--dwmac4_set_tx_tail_ptr\n");
}

void dwmac4_dma_start_tx(void __iomem *ioaddr, u32 chan)
{
	u32 value = 0;

	DBGPR_FUNC("-->dwmac4_dma_start_tx\n");
	value = readl(ioaddr + DMA_CHAN_TX_CONTROL(chan));

	value |= DMA_CONTROL_ST;
	writel(value, ioaddr + DMA_CHAN_TX_CONTROL(chan));

	value = readl(ioaddr + GMAC_CONFIG);
	value |= GMAC_CONFIG_TE;
	writel(value, ioaddr + GMAC_CONFIG);
	DBGPR_FUNC("<--dwmac4_dma_start_tx\n");
}

void dwmac4_dma_stop_tx(void __iomem *ioaddr, u32 chan)
{
	u32 value = readl(ioaddr + DMA_CHAN_TX_CONTROL(chan));
	DBGPR_FUNC("-->dwmac4_dma_stop_tx\n");
	value &= ~DMA_CONTROL_ST;
	writel(value, ioaddr + DMA_CHAN_TX_CONTROL(chan));

	value = readl(ioaddr + GMAC_CONFIG);
	value &= ~GMAC_CONFIG_TE;
	writel(value, ioaddr + GMAC_CONFIG);
	DBGPR_FUNC("<--dwmac4_dma_stop_tx\n");
}

void dwmac4_dma_start_rx(void __iomem *ioaddr, u32 chan)
{
	u32 value = 0;

	DBGPR_FUNC("-->dwmac4_dma_start_rx\n");
	value = readl(ioaddr + DMA_CHAN_RX_CONTROL(chan));

	value |= DMA_CONTROL_SR;

	writel(value, ioaddr + DMA_CHAN_RX_CONTROL(chan));

	value = readl(ioaddr + GMAC_CONFIG);
	value |= GMAC_CONFIG_RE;
	writel(value, ioaddr + GMAC_CONFIG);
	DBGPR_FUNC("<--dwmac4_dma_start_rx\n");
}

void dwmac4_dma_stop_rx(void __iomem *ioaddr, u32 chan)
{
	u32 value = 0;

	DBGPR_FUNC("-->dwmac4_dma_stop_rx\n");
	value = readl(ioaddr + DMA_CHAN_RX_CONTROL(chan));

	value &= ~DMA_CONTROL_SR;
	writel(value, ioaddr + DMA_CHAN_RX_CONTROL(chan));

	value = readl(ioaddr + GMAC_CONFIG);
	value &= ~GMAC_CONFIG_RE;
	writel(value, ioaddr + GMAC_CONFIG);
	DBGPR_FUNC("<--dwmac4_dma_stop_rx\n");
}

void dwmac4_set_tx_ring_len(void __iomem *ioaddr, u32 len, u32 chan)
{
	DBGPR_FUNC("-->dwmac4_set_tx_ring_len\n");
	writel(len, ioaddr + DMA_CHAN_TX_RING_LEN(chan));
	DBGPR_FUNC("<--dwmac4_set_tx_ring_len\n");
}

void dwmac4_set_rx_ring_len(void __iomem *ioaddr, u32 len, u32 chan)
{
	DBGPR_FUNC("-->dwmac4_set_rx_ring_len\n");
	writel(len, ioaddr + DMA_CHAN_RX_RING_LEN(chan));
	DBGPR_FUNC("<--dwmac4_set_rx_ring_len\n");
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
void dwmac4_enable_dma_irq(void __iomem *ioaddr, u32 chan)
{
	DBGPR_FUNC("-->dwmac4_enable_dma_irq\n");
	writel(DMA_CHAN_INTR_DEFAULT_MASK, ioaddr +
	       DMA_CHAN_INTR_ENA(chan));
	DBGPR_FUNC("<--dwmac4_enable_dma_irq\n");
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

void dwmac410_enable_dma_irq(void __iomem *ioaddr, u32 chan)
{
	DBGPR_FUNC("-->dwmac410_enable_dma_irq\n");
	writel(DMA_CHAN_INTR_DEFAULT_MASK_4_10,
	       ioaddr + DMA_CHAN_INTR_ENA(chan));
	DBGPR_FUNC("<--dwmac410_enable_dma_irq\n");
}

void dwmac4_disable_dma_irq(void __iomem *ioaddr, u32 chan)
{
	DBGPR_FUNC("-->dwmac4_disable_dma_irq\n");
	writel(0, ioaddr + DMA_CHAN_INTR_ENA(chan));
	DBGPR_FUNC("<--dwmac4_disable_dma_irq\n");
}

int dwmac4_dma_interrupt(void __iomem *ioaddr,
			 struct tc9562mac_extra_stats *x, u32 chan)
{
	int ret = 0;
	u32 intr_status = 0;

	DBGPR_FUNC("-->dwmac4_dma_interrupt\n");
	intr_status = readl(ioaddr + DMA_CHAN_STATUS(chan));

	/* ABNORMAL interrupts */
	if (unlikely(intr_status & DMA_CHAN_STATUS_AIS)) {
		if (unlikely(intr_status & DMA_CHAN_STATUS_RBU))
			x->rx_buf_unav_irq++;
		if (unlikely(intr_status & DMA_CHAN_STATUS_RPS))
			x->rx_process_stopped_irq++;
		if (unlikely(intr_status & DMA_CHAN_STATUS_RWT))
			x->rx_watchdog_irq++;
		if (unlikely(intr_status & DMA_CHAN_STATUS_ETI))
			x->tx_early_irq++;
		if (unlikely(intr_status & DMA_CHAN_STATUS_TPS)) {
			x->tx_process_stopped_irq++;
			ret = tx_hard_error;
		}
		if (unlikely(intr_status & DMA_CHAN_STATUS_FBE)) {
			x->fatal_bus_error_irq++;
			ret = tx_hard_error;
		}
	}

		x->normal_irq_n++;
		if (likely(intr_status & DMA_CHAN_STATUS_RI)) {
			u32 value;
			
			value = readl(ioaddr + DMA_CHAN_INTR_ENA(chan));
			/* to schedule NAPI on real RIE event. */
			if (likely(value & DMA_CHAN_INTR_ENA_RIE)) {
				x->rx_normal_irq_n++;
				ret |= handle_rx;
				DBGPR_TEST("RIE %d \n",chan);
			}
		}
		if (likely(intr_status & DMA_CHAN_STATUS_TI)) {
			x->tx_normal_irq_n++;
			ret |= handle_tx;
				DBGPR_TEST("TI %d\n",chan);
		}
		if (unlikely(intr_status & DMA_CHAN_STATUS_ERI))
		{
			x->rx_early_irq++;
		}
	
	/* Clear the interrupt by writing a logic 1 to the chanX interrupt
	 * status [21-0] expect reserved bits [5-3]
	 */
	writel((intr_status & 0x3fffc7),
	       ioaddr + DMA_CHAN_STATUS(chan));

	DBGPR_FUNC("<--dwmac4_dma_interrupt\n");

	return ret;
}
#ifndef TC9562_DEFINED
void tc9562mac_dwmac4_set_mac_addr(void __iomem *ioaddr, u8 addr[6],
				unsigned int high, unsigned int low)
#else
void tc9562mac_dwmac4_set_mac_addr(void __iomem *ioaddr, u8 addr[6],
				unsigned int high, unsigned int low,unsigned int reg_n, unsigned int dcs)
#endif
{
	unsigned long data = 0;

	DBGPR_FUNC("-->tc9562mac_dwmac4_set_mac_addr\n");

	data = (addr[5] << 8) | addr[4];
	/* For MAC Addr registers se have to set the Address Enable (AE)
	 * bit that has no effect on the High Reg 0 where the bit 31 (MO)
	 * is RO.
	 */
#ifdef TC9562_DEFINED
	data |= GMAC_HI_REG_AE;

	if (dcs)
		data &= ~(GMAC_HI_REG_DCS_MASK);
	if(dcs == 1)
		data |= GMAC_HI_REG_DCS1;
	else if (dcs == 2)
		data |= GMAC_HI_REG_DCS2;
	else if (dcs == 3)
		data |= GMAC_HI_REG_DCS3;

	writel(data, ioaddr + high);
#else
	data |= (TC9562MAC_CHAN0 << GMAC_HI_DCS_SHIFT);
	writel(data | GMAC_HI_REG_AE, ioaddr + high);
#endif
	data = 0;
	data = (addr[3] << 24) | (addr[2] << 16) | (addr[1] << 8) | addr[0];
	writel(data, ioaddr + low);
	NDBGPR_L1("MAC Address val = %x %x \n", ioaddr + high,ioaddr + low);    
	NDBGPR_L1("MAC Address content = %x %x \n", readl(ioaddr + high),readl(ioaddr + low));
	DBGPR_FUNC("<--tc9562mac_dwmac4_set_mac_addr\n");
}

/* Enable disable MAC RX/TX */
void tc9562mac_dwmac4_set_mac(void __iomem *ioaddr, bool enable)
{
	u32 value = readl(ioaddr + GMAC_CONFIG);

	DBGPR_FUNC("-->tc9562mac_dwmac4_set_mac\n");

	if (enable)
		value |= GMAC_CONFIG_RE | GMAC_CONFIG_TE;
	else
		value &= ~(GMAC_CONFIG_TE | GMAC_CONFIG_RE);

	writel(value, ioaddr + GMAC_CONFIG);

	DBGPR_FUNC("<--tc9562mac_dwmac4_set_mac\n");
	
}

void tc9562mac_dwmac4_get_mac_addr(void __iomem *ioaddr, unsigned char *addr,
				unsigned int high, unsigned int low)
{
	unsigned int hi_addr, lo_addr;

	DBGPR_FUNC("-->tc9562mac_dwmac4_get_mac_addr\n");

	/* Read the MAC address from the hardware */
	hi_addr = readl(ioaddr + high);
	lo_addr = readl(ioaddr + low);

	/* Extract the MAC address from the high and low words */
	addr[0] = lo_addr & 0xff;
	addr[1] = (lo_addr >> 8) & 0xff;
	addr[2] = (lo_addr >> 16) & 0xff;
	addr[3] = (lo_addr >> 24) & 0xff;
	addr[4] = hi_addr & 0xff;
	addr[5] = (hi_addr >> 8) & 0xff;

	DBGPR_FUNC("<--tc9562mac_dwmac4_get_mac_addr\n");
}
