ifeq ($(RELEASE_PACKAGE),1)
EXTRA_CFLAGS+=-DRELEASE_PACKAGE
endif
LBITS := $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
CCFLAGS += -m64
EXTRA_CFLAGS+=-DSYSTEM_IS_64
else
CCFLAGS += -m32
endif

obj-m := tc9562_pcie_eth.o
tc9562_pcie_eth-y := tc9562mac_main.o tc9562mac_ethtool.o tc9562mac_mdio.o ring_mode.o	\
	      chain_mode.o dwmac_lib.o dwmac1000_core.o dwmac1000_dma.o	\
	      dwmac100_core.o dwmac100_dma.o enh_desc.o norm_desc.o	\
	      mmc_core.o tc9562mac_hwtstamp.o tc9562mac_ptp.o dwmac4_descs.o	\
	      dwmac4_dma.o dwmac4_lib.o dwmac4_core.o tc9562mac_tsn.o \
	      dwmac5.o tc9562_pci.o dwmac4_enh_desc.o
all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
