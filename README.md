# pg021-bluespec
A Bluespec implementation of a Xilinx-compatible AXI to AXI Stream DMA module.  
The specification for this can be found [here](https://www.xilinx.com/support/documentation/ip_documentation/axi_dma/v7_1/pg021_axi_dma.pdf).
This version is designed to be compatible with [these drivers](https://github.com/CTSRD-CHERI/FreeRTOS-Plus-TCP/tree/hmka2/portable/NetworkInterface/RISC-V),
and is designed to be used in conjunction with a [Xilinx AXI Ethernet subsystem](https://www.xilinx.com/support/documentation/ip_documentation/axi_ethernet/v7_1/pg138-axi-ethernet.pdf)
There are some features that are not implemented. A non-exhaustive list includes:
* Multi-channel mode
* Micro mode
* Direct DMA mode
* Interrupt Treshold/Delay
* Error reporting in the registers
* SOF/EOF bits (this module assumes each DMA Buffer Descriptor will have both SOF and EOF set)

This version also has the restriction that it can only transfer data in one direction at a time, either Memory-Mapped to Stream or Stream to Memory-Mapped.


## CHERIfication
This branch contains the CHERIfied version of this DMA engine.
The buffer descriptor layout has changed, and the fields that previously
contained addresses now must contain valid capabilities.  
The DMA register layout is also different. There is a new capability-wide
register used to fetch the first Buffer Descriptor. In addition, new
CHERI-related errors are reported in the status registers.
