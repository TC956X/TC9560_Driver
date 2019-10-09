/*
 * TC9562 ethernet driver.
 *
 * dwmac100_dma.c
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

#include <asm/io.h>
#include "dwmac100.h"
#include "dwmac_dma.h"

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static void dwmac100_dma_init(void __iomem *ioaddr,
			      struct tc9562mac_dma_cfg *dma_cfg,
			      u32 dma_tx, u32 dma_rx, int atds)
{
	/* Enable Application Access by writing to DMA CSR0 */
	writel(DMA_BUS_MODE_DEFAULT | (dma_cfg->pbl << DMA_BUS_MODE_PBL_SHIFT),
	       ioaddr + DMA_BUS_MODE);

	/* Mask interrupts by writing to CSR7 */
	writel(DMA_INTR_DEFAULT_MASK, ioaddr + DMA_INTR_ENA);

	/* RX/TX descriptor base addr lists must be written into
	 * DMA CSR3 and CSR4, respectively
	 */
	writel(dma_tx, ioaddr + DMA_TX_BASE_ADDR);
	writel(dma_rx, ioaddr + DMA_RCV_BASE_ADDR);
}

/* Store and Forward capability is not used at all.
 *
 * The transmit threshold can be programmed by setting the TTC bits in the DMA
 * control register.
 */
static void dwmac100_dma_operation_mode(void __iomem *ioaddr, int txmode,
					int rxmode, int rxfifosz)
{
	u32 csr6 = readl(ioaddr + DMA_CONTROL);

	if (txmode <= 32)
		csr6 |= DMA_CONTROL_TTC_32;
	else if (txmode <= 64)
		csr6 |= DMA_CONTROL_TTC_64;
	else
		csr6 |= DMA_CONTROL_TTC_128;

	writel(csr6, ioaddr + DMA_CONTROL);
}

static void dwmac100_dump_dma_regs(void __iomem *ioaddr, u32 *reg_space)
{
	int i;

	for (i = 0; i < NUM_DWMAC100_DMA_REGS; i++)
		reg_space[DMA_BUS_MODE / 4 + i] =
			readl(ioaddr + DMA_BUS_MODE + i * 4);

	reg_space[DMA_CUR_TX_BUF_ADDR / 4] =
		readl(ioaddr + DMA_CUR_TX_BUF_ADDR);
	reg_space[DMA_CUR_RX_BUF_ADDR / 4] =
		readl(ioaddr + DMA_CUR_RX_BUF_ADDR);
}

/* DMA controller has two counters to track the number of the missed frames. */
static void dwmac100_dma_diagnostic_fr(void *data, struct tc9562mac_extra_stats *x,
				       void __iomem *ioaddr)
{
	struct net_device_stats *stats = (struct net_device_stats *)data;
	u32 csr8 = readl(ioaddr + DMA_MISSED_FRAME_CTR);

	if (unlikely(csr8)) {
		if (csr8 & DMA_MISSED_FRAME_OVE) {
			stats->rx_over_errors += 0x800;
			x->rx_overflow_cntr += 0x800;
		} else {
			unsigned int ove_cntr;
			ove_cntr = ((csr8 & DMA_MISSED_FRAME_OVE_CNTR) >> 17);
			stats->rx_over_errors += ove_cntr;
			x->rx_overflow_cntr += ove_cntr;
		}

		if (csr8 & DMA_MISSED_FRAME_OVE_M) {
			stats->rx_missed_errors += 0xffff;
			x->rx_missed_cntr += 0xffff;
		} else {
			unsigned int miss_f = (csr8 & DMA_MISSED_FRAME_M_CNTR);
			stats->rx_missed_errors += miss_f;
			x->rx_missed_cntr += miss_f;
		}
	}
}

const struct tc9562mac_dma_ops dwmac100_dma_ops = {
	.reset = dwmac_dma_reset,
	.init = dwmac100_dma_init,
	.dump_regs = dwmac100_dump_dma_regs,
	.dma_mode = dwmac100_dma_operation_mode,
	.dma_diagnostic_fr = dwmac100_dma_diagnostic_fr,
	.enable_dma_transmission = dwmac_enable_dma_transmission,
	.enable_dma_irq = dwmac_enable_dma_irq,
	.disable_dma_irq = dwmac_disable_dma_irq,
	.start_tx = dwmac_dma_start_tx,
	.stop_tx = dwmac_dma_stop_tx,
	.start_rx = dwmac_dma_start_rx,
	.stop_rx = dwmac_dma_stop_rx,
	.dma_interrupt = dwmac_dma_interrupt,
};
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
