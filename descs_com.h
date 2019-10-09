/*
 * TC9562 ethernet driver.
 *
 * descs_com.h
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

#ifndef __DESC_COM_H__
#define __DESC_COM_H__

/* Specific functions used for Ring mode */

/* Enhanced descriptors */
static inline void ehn_desc_rx_set_on_ring(struct dma_desc *p, int end)
{
	p->des1 |= cpu_to_le32(((BUF_SIZE_8KiB - 1)
			<< ERDES1_BUFFER2_SIZE_SHIFT)
		   & ERDES1_BUFFER2_SIZE_MASK);

	if (end)
		p->des1 |= cpu_to_le32(ERDES1_END_RING);
}

static inline void enh_desc_end_tx_desc_on_ring(struct dma_desc *p, int end)
{
	if (end)
		p->des0 |= cpu_to_le32(ETDES0_END_RING);
	else
		p->des0 &= cpu_to_le32(~ETDES0_END_RING);
}

static inline void enh_set_tx_desc_len_on_ring(struct dma_desc *p, int len)
{
	if (unlikely(len > BUF_SIZE_4KiB)) {
		p->des1 |= cpu_to_le32((((len - BUF_SIZE_4KiB)
					<< ETDES1_BUFFER2_SIZE_SHIFT)
			    & ETDES1_BUFFER2_SIZE_MASK) | (BUF_SIZE_4KiB
			    & ETDES1_BUFFER1_SIZE_MASK));
	} else
		p->des1 |= cpu_to_le32((len & ETDES1_BUFFER1_SIZE_MASK));
}

/* Normal descriptors */
static inline void ndesc_rx_set_on_ring(struct dma_desc *p, int end)
{
	p->des1 |= cpu_to_le32(((BUF_SIZE_2KiB - 1)
				<< RDES1_BUFFER2_SIZE_SHIFT)
		    & RDES1_BUFFER2_SIZE_MASK);

	if (end)
		p->des1 |= cpu_to_le32(RDES1_END_RING);
}

static inline void ndesc_end_tx_desc_on_ring(struct dma_desc *p, int end)
{
	if (end)
		p->des1 |= cpu_to_le32(TDES1_END_RING);
	else
		p->des1 &= cpu_to_le32(~TDES1_END_RING);
}

static inline void norm_set_tx_desc_len_on_ring(struct dma_desc *p, int len)
{
	if (unlikely(len > BUF_SIZE_2KiB)) {
		unsigned int buffer1 = (BUF_SIZE_2KiB - 1)
					& TDES1_BUFFER1_SIZE_MASK;
		p->des1 |= cpu_to_le32((((len - buffer1)
					<< TDES1_BUFFER2_SIZE_SHIFT)
				& TDES1_BUFFER2_SIZE_MASK) | buffer1);
	} else
		p->des1 |= cpu_to_le32((len & TDES1_BUFFER1_SIZE_MASK));
}

/* Specific functions used for Chain mode */

/* Enhanced descriptors */
static inline void ehn_desc_rx_set_on_chain(struct dma_desc *p)
{
	p->des1 |= cpu_to_le32(ERDES1_SECOND_ADDRESS_CHAINED);
}

static inline void enh_desc_end_tx_desc_on_chain(struct dma_desc *p)
{
	p->des0 |= cpu_to_le32(ETDES0_SECOND_ADDRESS_CHAINED);
}

static inline void enh_set_tx_desc_len_on_chain(struct dma_desc *p, int len)
{
	p->des1 |= cpu_to_le32(len & ETDES1_BUFFER1_SIZE_MASK);
}

/* Normal descriptors */
static inline void ndesc_rx_set_on_chain(struct dma_desc *p, int end)
{
	p->des1 |= cpu_to_le32(RDES1_SECOND_ADDRESS_CHAINED);
}

static inline void ndesc_tx_set_on_chain(struct dma_desc *p)
{
	p->des1 |= cpu_to_le32(TDES1_SECOND_ADDRESS_CHAINED);
}

static inline void norm_set_tx_desc_len_on_chain(struct dma_desc *p, int len)
{
	p->des1 |= cpu_to_le32(len & TDES1_BUFFER1_SIZE_MASK);
}
#endif /* __DESC_COM_H__ */
