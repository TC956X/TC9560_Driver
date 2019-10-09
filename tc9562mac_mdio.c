/*
 * TC9562 ethernet driver.
 *
 * tc9562mac_mdio.c
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

#include <linux/io.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))
#include <linux/iopoll.h>
#endif
#include <linux/mii.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/slab.h>

#include "tc9562mac.h"

#define MII_BUSY 0x00000001
#define MII_WRITE 0x00000002

/* GMAC4 defines */
#define MII_GMAC4_GOC_SHIFT		2
#define MII_GMAC4_WRITE			(1 << MII_GMAC4_GOC_SHIFT)
#define MII_GMAC4_READ			(3 << MII_GMAC4_GOC_SHIFT)
#define MII_C45					(1 << 1)

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 0, 0))
int tc9562_mdio_busy_wait(void __iomem *ioaddr, u32 sleep_us, u32 timeout_us)
{
    int limit = (timeout_us / sleep_us);
    while (limit--) {
        if (!(readl(ioaddr) & MII_BUSY))
            break;
        /* The actual delay needed for read/write(C45)is 51.2usec at 2.5MHz MDIO clock */
        udelay(sleep_us);
    }

    if (limit < 0)
        return -EBUSY;

    return 0;
}
#endif
int tc9562mac_mdio_read_direct(struct tc9562mac_priv *priv, int phyaddr, int phyreg, int *data)
{
	unsigned int mii_address = priv->hw->mii.addr;
	unsigned int mii_data = priv->hw->mii.data;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))
	u32 v;
#endif
	int dev_type = 0;
	u32 value = MII_BUSY;
	
	DBGPR_FUNC_MDIO("-->tc9562mac_mdio_read_direct\n");
	
#ifdef CL45_PHY
	if (phyreg < 0x1F)
		{
			if (phyreg == 0) 
				phyreg = (MII_ADDR_C45 | (PHY_CL45_CTRL_REG_MMD_BANK << 16) | PHY_CL45_CTRL_REG_ADDR);
			else if (phyreg == 1) 
				phyreg = (MII_ADDR_C45 | (PHY_CL45_STATUS_REG_MMD_BANK << 16) | PHY_CL45_STATUS_REG_ADDR);
			else if (phyreg == 2) 
				phyreg = (MII_ADDR_C45 | (PHY_CL45_PHYID1_MMD_BANK << 16) | PHY_CL45_PHYID1_ADDR);
			else if (phyreg == 3) 
				phyreg = (MII_ADDR_C45 | (PHY_CL45_PHYID2_MMD_BANK << 16) | PHY_CL45_PHYID2_ADDR);
			else {
//			    NMSGPR_ALERT("Clause 45 register not defined for PHY register 0x%x\n", phyreg);
//			    NMSGPR_ERR("Clause 45 read failed\n");
			//    return -EBUSY;
			}
		}
#endif
	value |= (phyaddr << priv->hw->mii.addr_shift)
		& priv->hw->mii.addr_mask;
	value |= (priv->clk_csr << priv->hw->mii.clk_csr_shift)
		& priv->hw->mii.clk_csr_mask;
	if (priv->plat->has_gmac4)
		value |= MII_GMAC4_READ;
	if(phyreg & MII_ADDR_C45)
	{
		dev_type = ((phyreg >> 16) & 0x1f);
		phyreg = (phyreg & 0xffff) << 16;

		/* phy device address provided to mdio addr register */
		value |= (dev_type << priv->hw->mii.reg_shift) & priv->hw->mii.reg_mask;
		/* c45 phy enable */
		value |= MII_C45;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
	if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
			       10, 10000))
		return -EBUSY;
#else
    if (tc9562_mdio_busy_wait(priv->ioaddr + mii_address, 10, 10000))
        return -EBUSY;

#endif
		writel(phyreg ,priv->ioaddr + mii_data);
		writel(value, priv->ioaddr + mii_address);
	}else {
		/* phy register address provided to mdio addr register in c22 read */
		value |= (phyreg << priv->hw->mii.reg_shift) & priv->hw->mii.reg_mask;
		
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))
		if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
			       10, 10000))
		return -EBUSY;
#else
      if (tc9562_mdio_busy_wait(priv->ioaddr + mii_address, 10, 10000))
        return -EBUSY;	
#endif

		writel(value, priv->ioaddr + mii_address);
	}	

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))
	if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
			       10, 10000))
		return -EBUSY;
#else
    if (tc9562_mdio_busy_wait(priv->ioaddr + mii_address, 10, 10000))
        return -EBUSY;		
#endif

	/* Read the data from the MII data register */
	*data = (int)readl(priv->ioaddr + mii_data);
	*data = *data & 0xffff;

	DBGPR_FUNC_MDIO("<--tc9562mac_mdio_read_direct\n");
	return 0;
}
/**
 * tc9562mac_mdio_write
 * @bus: points to the mii_bus structure
 * @phyaddr: MII addr
 * @phyreg: MII reg
 * @phydata: phy data
 * Description: it writes the data into the MII register from within the device.
 */
int tc9562mac_mdio_write_direct(struct tc9562mac_priv *priv, int phyaddr, int phyreg,
			     u16 phydata)
{
	unsigned int mii_address = priv->hw->mii.addr;
	unsigned int mii_data = priv->hw->mii.data;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))
	u32 v;
#endif
	u32 value = MII_BUSY;
	int dev_type = 0;
	
	DBGPR_FUNC_MDIO("-->tc9562mac_mdio_write_direct\n");
	
#if 0//def CL45_PHY
	if (phyreg < 0x1F) {
	    NMSGPR_ERR("Error in Clause 45 write. PHY reg = 0x%x\n", phyreg);
	    return -EBUSY;
	}
#endif

	value |= (phyaddr << priv->hw->mii.addr_shift)
		& priv->hw->mii.addr_mask;

	value |= (priv->clk_csr << priv->hw->mii.clk_csr_shift)
		& priv->hw->mii.clk_csr_mask;
	if (priv->plat->has_gmac4)
		value |= MII_GMAC4_WRITE;
	else
		value |= MII_WRITE;
	if(phyreg & MII_ADDR_C45)
	{
		dev_type = ((phyreg >> 16) & 0x1f);
		phyreg = (phyreg & 0xffff);
		/* Device address provided in mdio address register */
		value |= (dev_type << priv->hw->mii.reg_shift) & priv->hw->mii.reg_mask;

		/* c45 phy enable */
		value |= MII_C45;
		
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))		
	/* Wait until any existing MII operation is complete */
	if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
			       10, 10000))
		return -EBUSY;
#else
     if (tc9562_mdio_busy_wait(priv->ioaddr + mii_address, 10, 10000))
        return -EBUSY;		
#endif
		
	/* Write register address & data  to mdio data register*/
		writel(((phyreg << 16 )|(phydata & 0xffff)), priv->ioaddr + mii_data);
		writel(value, priv->ioaddr + mii_address);
	}else{
		value |= (phyreg << priv->hw->mii.reg_shift) & priv->hw->mii.reg_mask;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))			
		/* Wait until any existing MII operation is complete */
		if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
				       10, 10000))
			return -EBUSY;
#else
   if (tc9562_mdio_busy_wait(priv->ioaddr + mii_address, 10, 10000))
        return -EBUSY;		
#endif

	/* Set the MII address register to write */
	writel(phydata, priv->ioaddr + mii_data);
	writel(value, priv->ioaddr + mii_address);
	}	

	DBGPR_FUNC_MDIO("<--tc9562mac_mdio_write_direct\n");
	
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))	
	/* Wait until any existing MII operation is complete */
	return readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
				  10, 10000);
#else
    return (tc9562_mdio_busy_wait(priv->ioaddr + mii_address, 10, 10000));	
#endif
}
/**
 * tc9562mac_mdio_read
 * @bus: points to the mii_bus structure
 * @phyaddr: MII addr
 * @phyreg: MII reg
 * Description: it reads data from the MII register from within the phy device.
 * For the 7111 GMAC, we must set the bit 0 in the MII address register while
 * accessing the PHY registers.
 * Fortunately, it seems this has no drawback for the 7109 MAC.
 */
static int tc9562mac_mdio_read(struct mii_bus *bus, int phyaddr, int phyreg)
{
	struct net_device *ndev = bus->priv;
	struct tc9562mac_priv *priv = netdev_priv(ndev);
	unsigned int mii_address = priv->hw->mii.addr;
	unsigned int mii_data = priv->hw->mii.data;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))
	u32 v;
#endif
	int data, dev_type = 0;
	u32 value = MII_BUSY;
	DBGPR_FUNC_MDIO("-->tc9562mac_mdio_read\n");
	
#ifdef CL45_PHY
	if (phyreg < 0x1F)
		{
			if (phyreg == 0) 
				phyreg = (MII_ADDR_C45 | (PHY_CL45_CTRL_REG_MMD_BANK << 16) | PHY_CL45_CTRL_REG_ADDR);
			else if (phyreg == 1) 
				phyreg = (MII_ADDR_C45 | (PHY_CL45_STATUS_REG_MMD_BANK << 16) | PHY_CL45_STATUS_REG_ADDR);
			else if (phyreg == 2) 
				phyreg = (MII_ADDR_C45 | (PHY_CL45_PHYID1_MMD_BANK << 16) | PHY_CL45_PHYID1_ADDR);
			else if (phyreg == 3) 
				phyreg = (MII_ADDR_C45 | (PHY_CL45_PHYID2_MMD_BANK << 16) | PHY_CL45_PHYID2_ADDR);
			else {
	//		    NMSGPR_ALERT("Clause 45 register not defined for PHY register 0x%x\n", phyreg);
	//		    NMSGPR_ERR("Clause 45 read failed\n");
	//		    return -EBUSY;
			}
		}
#endif
	value |= (phyaddr << priv->hw->mii.addr_shift)
		& priv->hw->mii.addr_mask;
	value |= (priv->clk_csr << priv->hw->mii.clk_csr_shift)
		& priv->hw->mii.clk_csr_mask;
	if (priv->plat->has_gmac4)
		value |= MII_GMAC4_READ;
	if(phyreg & MII_ADDR_C45)
	{
		dev_type = ((phyreg >> 16) & 0x1f);
		phyreg = (phyreg & 0xffff) << 16;

		NDBGPR_L2("dev type: %x, phyreg: %x \n", dev_type, phyreg);
		/* phy device address provided to mdio addr register */
		value |= (dev_type << priv->hw->mii.reg_shift) & priv->hw->mii.reg_mask;
		/* c45 phy enable */
		value |= MII_C45;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))
		if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
			       10, 10000))
			return -EBUSY;
#else
     if (tc9562_mdio_busy_wait(priv->ioaddr + mii_address, 10, 10000))
        return -EBUSY;	
#endif

		writel(phyreg ,priv->ioaddr + mii_data);
		writel(value, priv->ioaddr + mii_address);
	}else {
		/* phy register address provided to mdio addr register in c22 read */
		value |= (phyreg << priv->hw->mii.reg_shift) & priv->hw->mii.reg_mask;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))		
		if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
			       10, 10000))
			return -EBUSY;
#else
      if (tc9562_mdio_busy_wait(priv->ioaddr + mii_address, 10, 10000))
        return -EBUSY;		
#endif
		
		writel(value, priv->ioaddr + mii_address);
	}	

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))
	if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
		       10, 10000))
		return -EBUSY;
#else
     if (tc9562_mdio_busy_wait(priv->ioaddr + mii_address, 10, 10000))
        return -EBUSY;	
#endif

	/* Read the data from the MII data register */
	data = (int)readl(priv->ioaddr + mii_data);
	data = data & 0xffff;

	DBGPR_FUNC_MDIO("<--tc9562mac_mdio_read\n");
	return data;
}

/**
 * tc9562mac_mdio_write
 * @bus: points to the mii_bus structure
 * @phyaddr: MII addr
 * @phyreg: MII reg
 * @phydata: phy data
 * Description: it writes the data into the MII register from within the device.
 */
static int tc9562mac_mdio_write(struct mii_bus *bus, int phyaddr, int phyreg,
			     u16 phydata)
{
	struct net_device *ndev = bus->priv;
	struct tc9562mac_priv *priv = netdev_priv(ndev);
	unsigned int mii_address = priv->hw->mii.addr;
	unsigned int mii_data = priv->hw->mii.data;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))
	u32 v;
#endif
	u32 value = MII_BUSY;
	int dev_type = 0;

	DBGPR_FUNC_MDIO("-->tc9562mac_mdio_write\n");
	
#if 0//def CL45_PHY
	if (phyreg < 0x1F) {
	    NMSGPR_ERR("Error in Clause 45 write. PHY reg = 0x%x\n", phyreg);
	    return -EBUSY;
	}
#endif
	    
	value |= (phyaddr << priv->hw->mii.addr_shift)
		& priv->hw->mii.addr_mask;

	value |= (priv->clk_csr << priv->hw->mii.clk_csr_shift)
		& priv->hw->mii.clk_csr_mask;
	if (priv->plat->has_gmac4)
		value |= MII_GMAC4_WRITE;
	else
		value |= MII_WRITE;
	if(phyreg & MII_ADDR_C45)
	{
		dev_type = ((phyreg >> 16) & 0x1f);
		phyreg = (phyreg & 0xffff);
		NDBGPR_L2("dev type: %x, phyreg: %x \n", dev_type, phyreg);
		/* Device address provided in mdio address register */
		value |= (dev_type << priv->hw->mii.reg_shift) & priv->hw->mii.reg_mask;

		/* c45 phy enable */
		value |= MII_C45;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))		
		/* Wait until any existing MII operation is complete */
		if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
			       10, 10000))
			return -EBUSY;
#else
      if (tc9562_mdio_busy_wait(priv->ioaddr + mii_address, 10, 10000))
        return -EBUSY;	
#endif

		/* Write register address & data  to mdio data register*/
		writel(((phyreg << 16 )|(phydata & 0xffff)), priv->ioaddr + mii_data);
		writel(value, priv->ioaddr + mii_address);
	}else{
		value |= (phyreg << priv->hw->mii.reg_shift) & priv->hw->mii.reg_mask;
		
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))		
		/* Wait until any existing MII operation is complete */
		if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
				       10, 10000))
			return -EBUSY;
#else
         if (tc9562_mdio_busy_wait(priv->ioaddr + mii_address, 10, 10000))
        return -EBUSY;	
#endif			
			
		/* Set the MII address register to write */
		writel(phydata, priv->ioaddr + mii_data);
		writel(value, priv->ioaddr + mii_address);
	}	
	DBGPR_FUNC_MDIO("<--tc9562mac_mdio_write\n");
	
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))	
	/* Wait until any existing MII operation is complete */
	return readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
				  10, 10000);
#else
    return (tc9562_mdio_busy_wait(priv->ioaddr + mii_address, 10, 10000));
#endif	
}

/**
 * tc9562mac_mdio_reset
 * @bus: points to the mii_bus structure
 * Description: reset the MII bus
 */
int tc9562mac_mdio_reset(struct mii_bus *bus)
{
#if defined(CONFIG_TC9562MAC_PLATFORM)
	struct net_device *ndev = bus->priv;
	struct tc9562mac_priv *priv = netdev_priv(ndev);
	unsigned int mii_address = priv->hw->mii.addr;
	struct tc9562mac_mdio_bus_data *data = priv->plat->mdio_bus_data;

	DBGPR_FUNC_MDIO("-->tc9562mac_mdio_reset\n");

#ifdef CONFIG_OF
	if (priv->device->of_node) {
		if (data->reset_gpio < 0) {
			struct device_node *np = priv->device->of_node;

			if (!np)
				return 0;

			data->reset_gpio = of_get_named_gpio(np,
						"snps,reset-gpio", 0);
			if (data->reset_gpio < 0)
				return 0;

			data->active_low = of_property_read_bool(np,
						"snps,reset-active-low");
			of_property_read_u32_array(np,
				"snps,reset-delays-us", data->delays, 3);

			if (gpio_request(data->reset_gpio, "mdio-reset"))
				return 0;
		}

		gpio_direction_output(data->reset_gpio,
				      data->active_low ? 1 : 0);
		if (data->delays[0])
			msleep(DIV_ROUND_UP(data->delays[0], 1000));

		gpio_set_value(data->reset_gpio, data->active_low ? 0 : 1);
		if (data->delays[1])
			msleep(DIV_ROUND_UP(data->delays[1], 1000));

		gpio_set_value(data->reset_gpio, data->active_low ? 1 : 0);
		if (data->delays[2])
			msleep(DIV_ROUND_UP(data->delays[2], 1000));
	}
#endif

	if (data->phy_reset) {
		netdev_dbg(ndev, "tc9562mac_mdio_reset: calling phy_reset\n");
		data->phy_reset(priv->plat->bsp_priv);
	}

	/* This is a workaround for problems with the STE101P PHY.
	 * It doesn't complete its reset until at least one clock cycle
	 * on MDC, so perform a dummy mdio read. To be updated for GMAC4
	 * if needed.
	 */
	if (!priv->plat->has_gmac4)
		writel(0, priv->ioaddr + mii_address);
#endif

	DBGPR_FUNC_MDIO("<--tc9562mac_mdio_reset\n");

	return 0;
}

/**
 * tc9562mac_mdio_register
 * @ndev: net device structure
 * Description: it registers the MII bus
 */
int tc9562mac_mdio_register(struct net_device *ndev)
{
	int err = 0;
	struct mii_bus *new_bus;
	struct tc9562mac_priv *priv = netdev_priv(ndev);
	struct tc9562mac_mdio_bus_data *mdio_bus_data = priv->plat->mdio_bus_data;
	struct device_node *mdio_node = priv->plat->mdio_node;
	struct device *dev = ndev->dev.parent;
	int addr, found;

	DBGPR_FUNC_MDIO("-->tc9562mac_mdio_register\n");
	
	if (!mdio_bus_data)
		return 0;

	new_bus = mdiobus_alloc();
	if (!new_bus){
		NMSGPR_ALERT("MDIO BUS allocation failed \n");
		return -ENOMEM;
	}
	else {
	    NMSGPR_ALERT("MDIO BUS allocation success \n");
	}

	if (mdio_bus_data->irqs)
		memcpy(new_bus->irq, mdio_bus_data->irqs, sizeof(new_bus->irq));

#ifdef CONFIG_OF
	if (priv->device->of_node)
		mdio_bus_data->reset_gpio = -1;
#endif

	new_bus->name = "tc9562";
	new_bus->read = &tc9562mac_mdio_read;
	new_bus->write = &tc9562mac_mdio_write;

	new_bus->reset = &tc9562mac_mdio_reset;
	snprintf(new_bus->id, MII_BUS_ID_SIZE, "%s-%x",
		 new_bus->name, priv->plat->bus_id);
	new_bus->priv = ndev;
	new_bus->phy_mask = mdio_bus_data->phy_mask;
	new_bus->parent = priv->device;

	if (mdio_node)
		err = of_mdiobus_register(new_bus, mdio_node);
	else
		err = mdiobus_register(new_bus);
	if (err != 0) {
		dev_err(dev, "Cannot register the MDIO bus\n");
		goto bus_register_fail;
	}
	else {
	    NMSGPR_ALERT("MDIO BUS registered \n");
	}

	if (priv->plat->phy_node || mdio_node)
		goto bus_register_done;

	found = 0;
    for(addr = 0; addr < PHY_MAX_ADDR; addr++) {
        int phy_reg_read_status, mii_status;
        
        /* For C22 based PHYs, check for Status to detect PHY */
        phy_reg_read_status = tc9562mac_mdio_read_direct(priv, addr, MII_BMSR, &mii_status);
        
        if (phy_reg_read_status == 0) {
			if (mii_status != 0x0000 && mii_status != 0xffff) {
				NMSGPR_ALERT("TC9562: Phy detected C22 at ID/ADDR %d\n", addr);
			   /*
	             * If an IRQ was provided to be assigned after
	             * the bus probe, do it here.
	             */
	            if (!mdio_bus_data->irqs &&
	                (mdio_bus_data->probed_phy_irq > 0)) {
	                NMSGPR_ALERT("probed_phy_irq\n");
		            new_bus->irq[addr] = mdio_bus_data->probed_phy_irq;
	            }
				        
		        if (priv->plat->phy_addr == -1) {
        			priv->plat->phy_addr = addr;
			    }
			    
		        found = 1;
				break;
			}
		} else  {
			NMSGPR_ALERT("TC9562: Error reading the phy register"\
			    " MII_BMSR for phy ID/ADDR %d\n", addr);
		}
    }

    if(!found) {
        for(addr = 0; addr < PHY_MAX_ADDR; addr++) {
            int phy_reg_read_status;
            int phyreg1, phyreg2, phy_id;
            /* For C45 based PHYs, check for Vendor ID */
            phy_reg_read_status =  tc9562mac_mdio_read_direct(priv, addr, (PHY_CL45_PHYID1_REG | MII_ADDR_C45), &phyreg1);

			if(0 == phy_reg_read_status){
				phy_reg_read_status =  tc9562mac_mdio_read_direct(priv, addr, (PHY_CL45_PHYID2_REG | MII_ADDR_C45), &phyreg2);
				if(0 == phy_reg_read_status){
					phy_id = ((phyreg1 << 16 ) | phyreg2); 
				}
			}
			
			if (phy_reg_read_status == 0) {
			        NMSGPR_ALERT("TC9562: Phy detected c45 at ID ADDR %d\n", addr);
			    if (phy_id != 0x00000000 && phy_id != 0xffffffff) {
		            /*
		             * If an IRQ was provided to be assigned after
		             * the bus probe, do it here.
		             */
		            if (!mdio_bus_data->irqs &&
		                (mdio_bus_data->probed_phy_irq > 0)) {
		                NMSGPR_ALERT("probed_phy_irq\n");
			            new_bus->irq[addr] = mdio_bus_data->probed_phy_irq;
			            
		            }
		            
		            if (priv->plat->phy_addr == -1) {
            			priv->plat->phy_addr = addr;
			        }
			        
		            found = 1;
		            
		            break;
			    }
		    } else {
		        NMSGPR_ALERT("TC9562: Error reading the PHY ID for phy ID/ADDR %d\n", addr);
		    }
        }
    }
    
	if (!found && !mdio_node) {
		dev_warn(dev, "No PHY found\n");
		mdiobus_unregister(new_bus);
		mdiobus_free(new_bus);
		return -ENODEV;
	}

	DBGPR_FUNC_MDIO("<--tc9562mac_mdio_register\n");

bus_register_done:
	priv->mii = new_bus;

	return 0;

bus_register_fail:
	mdiobus_free(new_bus);
	return err;
}

/**
 * tc9562mac_mdio_unregister
 * @ndev: net device structure
 * Description: it unregisters the MII bus
 */
int tc9562mac_mdio_unregister(struct net_device *ndev)
{
	struct tc9562mac_priv *priv = netdev_priv(ndev);

	DBGPR_FUNC_MDIO("-->tc9562mac_mdio_unregister\n");
	
	if (!priv->mii)
		return 0;

	mdiobus_unregister(priv->mii);
	priv->mii->priv = NULL;
	mdiobus_free(priv->mii);
	priv->mii = NULL;

	DBGPR_FUNC_MDIO("<--tc9562mac_mdio_unregister\n");

	return 0;
}
