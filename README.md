# Toshiba Electronic Devices & Storage Corporation TC9562 PCIe Ethernet Host Driver
Release Date: September 30 2019
Release Version: V_01-00

===============================================================================
TC9562 PCIe EMAC driver is based on "Fedora 26, kernel-4.14.16" and 
"Fedora 20, kernel-3.18.11".

Note:
	MAC ID can be to be configured in "config.ini" file. The current MAC ID is
	for test purpose only. Default MAC ID can be changed in "dev_addr" array
	in file "tc9562mac_main.c".

Compilation & Run: Need to be root user to execute the following steps.
===============================================================================
1.  Execute following commands:
    #make clean
    #make
2. 	Modify "pfifo_fast" as the default qdisc
	sysctl -w net.core.default_qdisc=pfifo_fast
3.	Load PTP module
	#modprobe ptp
4.  Load the driver
	#insmod tc9562_pcie_eth.ko
5.  Remove the driver
	#rmmod tc9562_pcie_eth

Release Versions:
===============================================================================
TC9562_Host_Driver_20190930_V_01-00:
===============================================================================
1. Initial Version

