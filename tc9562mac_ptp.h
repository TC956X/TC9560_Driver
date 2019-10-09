/*
 * TC9562 ethernet driver.
 *
 * tc9562mac_ptp.h
 *
 * Copyright (C) 2013  Vayavya Labs Pvt Ltd
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

#ifndef	__TC9562MAC_PTP_H__
#define	__TC9562MAC_PTP_H__

#include "common.h"

#ifndef TC9562_DEFINED
#define	PTP_GMAC4_OFFSET	0xb00
#else
#define	PTP_GMAC4_OFFSET	(0xb00  + MAC_OFFSET)
#endif
#define	PTP_GMAC3_X_OFFSET	0x700

/* IEEE 1588 PTP register offsets */
#define	PTP_TCR		0x00	/* Timestamp Control Reg */
#define	PTP_SSIR	0x04	/* Sub-Second Increment Reg */
#define	PTP_STSR	0x08	/* System Time – Seconds Regr */
#define	PTP_STNSR	0x0c	/* System Time – Nanoseconds Reg */
#define	PTP_STSUR	0x10	/* System Time – Seconds Update Reg */
#define	PTP_STNSUR	0x14	/* System Time – Nanoseconds Update Reg */
#define	PTP_TAR		0x18	/* Timestamp Addend Reg */
#define PTP_TS_STATUS   0x20    /* Timestamp status Reg */

#define	PTP_STNSUR_ADDSUB_SHIFT	31
#define	PTP_DIGITAL_ROLLOVER_MODE	0x3B9ACA00	/* 10e9-1 ns */
#define	PTP_BINARY_ROLLOVER_MODE	0x80000000	/* ~0.466 ns */

/* PTP Timestamp control register defines */
#define	PTP_TCR_TSENA		BIT(0)	/* Timestamp Enable */
#define	PTP_TCR_TSCFUPDT	BIT(1)	/* Timestamp Fine/Coarse Update */
#define	PTP_TCR_TSINIT		BIT(2)	/* Timestamp Initialize */
#define	PTP_TCR_TSUPDT		BIT(3)	/* Timestamp Update */
#define	PTP_TCR_TSTRIG		BIT(4)	/* Timestamp Interrupt Trigger Enable */
#define	PTP_TCR_TSADDREG	BIT(5)	/* Addend Reg Update */
#define PTP_TCR_PTGE		BIT(6)	/* Presentation Time Generation Enable */
#define	PTP_TCR_TSENALL		BIT(8)	/* Enable Timestamp for All Frames */
#define	PTP_TCR_TSCTRLSSR	BIT(9)	/* Digital or Binary Rollover Control */
/* Enable PTP packet Processing for Version 2 Format */
#define	PTP_TCR_TSVER2ENA	BIT(10)
/* Enable Processing of PTP over Ethernet Frames */
#define	PTP_TCR_TSIPENA		BIT(11)
/* Enable Processing of PTP Frames Sent over IPv6-UDP */
#define	PTP_TCR_TSIPV6ENA	BIT(12)
/* Enable Processing of PTP Frames Sent over IPv4-UDP */
#define	PTP_TCR_TSIPV4ENA	BIT(13)
/* Enable Timestamp Snapshot for Event Messages */
#define	PTP_TCR_TSEVNTENA	BIT(14)
/* Enable Snapshot for Messages Relevant to Master */
#define	PTP_TCR_TSMSTRENA	BIT(15)
/* Select PTP packets for Taking Snapshots */
#define	PTP_TCR_SNAPTYPSEL_1	BIT(16)
#define	PTP_GMAC4_TCR_SNAPTYPSEL_1	GENMASK(17, 16)
#define PTP_TCR_SNAPTYPSEL_1_LPOS   (16)
/* Enable MAC address for PTP Frame Filtering */
#define	PTP_TCR_TSENMACADDR	BIT(18)
#define	PTP_TCR_ASMEN		BIT(28)          

#define PTP_TCR_MODE_MASK   (0x3C000)
#define PTP_TCR_MC_UC_LPOS   (18)

/* SSIR defines */
#define	PTP_SSIR_SSINC_MASK		0xff
#define PTP_SSIR_SNSINC_MASK		0xff
#define	GMAC4_PTP_SSIR_SSINC_SHIFT	16
#define GMAC4_PTP_SSIR_SNSINC_SHIFT	8

#endif	/* __TC9562MAC_PTP_H__ */
