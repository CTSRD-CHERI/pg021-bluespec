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

The current version is compatible with 64-bit fabrics and 128-bit capabilities.
Some effort has gone into making sections of the DMA engine 32-bit compatible
but there are several sections where this does not hold.


## AXI Interface
This DMA engine has 2 AXI master ports and a single AXI slave port.
It also has 2 AXI Stream master ports and 2 AXI Stream slave ports.

Currently the AXI address and data fields must be 64 bits wide.
The R and W user fields must be at least 1 bit wide in order to hold a capability tag.
For the slave interface, if the W user field is more than 1 bit long only the least significant bit
is used. If the R user field is more than 1 bit, the LSB bit is used for tag validity and the rest
are set to 0 on reads.

For the AXI Scatter-Gather master interface, the least significant user bit must be 1 in order for a
capability to tagged as valid.

The AXI Copy Unit master interface does not use the R and W user bits from the bus; for W they are
set to 0, and for R they are not used.

The AXI master ports currently generate bursts up to 8 flits long, and each flit is either
32 or 64 bits wide.
The slave port accepts 32 bit single-flit reads and writes. It also accepts 2-flit 64-bit reads and
writes to the capability register offsets.
The AXI Stream ports should have 32 bits of data.

Internally, some user fields are extended with extra information but this does not make it to the
bus.


## CHERIfication
This branch contains the CHERIfied version of this DMA engine.
The Buffer Descriptor layout has changed, and the fields that previously
contained addresses now must contain valid capabilities. This means that
changes must be made to the drivers in order to support writing capabilities to
these fields.

The DMA register layout is also different. There is a new capability-wide
register which holds the authorising capability for the current transaction.
Currently this register is used _only_ for authorisation, and not for control.
The address of this capability is not used.


### CHERIfied Register layout
The names used here match up with the names used in the hardware source code,
which match the names in the original specification.

Some names are different between the original specification and the driver
sources linked above.


|  Register name        |  Register Offset in bytes  |  Register Size + Type  |  Changes from Specification  |
| -------------------   | -------------------------- | ---------------------- | ---------------------------- |
| DMA_MM2S_DMACR        |  `0x00`                    |  4 bytes, integer       |  None  |
| DMA_MM2S_DMASR        |  `0x04`                    |  4 bytes, integer       |  Now reports CHERI errors in bits 7 and 11, which were previously reserved  |
| DMA_MM2S_CURDESC      |  `0x08`                    |  4 bytes, integer       |  None  |
| DMA_MM2S_CURDESC_MSB  |  `0x0C`                    |  4 bytes, integer       |  None  |
| DMA_MM2S_TAILDESC     |  `0x10`                    |  4 bytes, integer       |  None  |
| DMA_MM2S_TAILDESC_MSB |  `0x14`                    |  4 bytes, integer       |  None  |
| DMA_MM2S_CURDESC_CAP  |  `0x20`                    |  16 bytes, capability    |  New addition not present in original specification  |
| DMA_S2MM_DMACR        |  `0x30`                    |  4 bytes, integer       |  None  |
| DMA_S2MM_DMASR        |  `0x34`                    |  4 bytes, integer       |  Now reports CHERI errors in bits 7 and 11, which were previously reserved  |
| DMA_S2MM_CURDESC      |  `0x38`                    |  4 bytes, integer       |  None  |
| DMA_S2MM_CURDESC_MSB  |  `0x3C`                    |  4 bytes, integer       |  None  |
| DMA_S2MM_TAILDESC     |  `0x40`                    |  4 bytes, integer       |  None  |
| DMA_S2MM_TAILDESC_MSB |  `0x44`                    |  4 bytes, integer       |  None  |
| DMA_S2MM_CURDESC_CAP  |  `0x50`                    |  16 bytes, capability    |  New addition not present in the original specification  |

`DMA_MM2S_CURDESC_CAP` and `DMA_S2MM_CURDESC_CAP` are used to authorise accesses on the AXI bus,
and are updated when a new Buffer Descriptor is fetched, similarly to how `CURDESC` is updated.
The `CURDESC` and `TAILDESC` registers have been left unchanged.

### CHERIfied Buffer Descriptors
The layout of Buffer Descriptors has changed significantly in order to accommodate capabilities,
which are larger than 32-bit integers.

The layout is dependent on whether a 32-bit or 64-bit version of the DMA engine is desired.
Currently, selecting between 32-bit and 64-bit is dependent on the `RV64` macro being defined
or not defined; this is not the intended final behaviour.

As with the register space, the Buffer Descriptor field names have stayed similar to the original
specification, which means there are some differences between field names in the hardware design and
the names in the drivers.

The new Buffer Descriptor layout for the 64-bit version is as follows:
|  Field name      |  Field offset in bytes  |  Field Length + Type  |  Changes from Specification  |
| ------------     | ----------------------- | --------------------- | ---------------------------- |
|  NXTDESC         |  `0x00`                 |  16 bytes, capability |  Has changed from 2 32-bit integers to one capability-wide field  |
|  BUFFER_ADDRESS  |  `0x10`                 |  16 bytes, capability |  Has changed from 2 32-bit integers to one capability-wide field  |
|  CONTROL         |  `0x20`                 |  4 bytes, integer     |  Relocated  |
|  STATUS          |  `0x24`                 |  4 bytes, integer     |  Relocated  |
|  APP0            |  `0x28`                 |  4 bytes, integer     |  Relocated  |
|  APP1            |  `0x2C`                 |  4 bytes, integer     |  Relocated  |
|  APP2            |  `0x30`                 |  4 bytes, integer     |  Relocated  |
|  APP3            |  `0x34`                 |  4 bytes, integer     |  Relocated  |
|  APP4            |  `0x38`                 |  4 bytes, integer     |  Relocated  |
|  RESERVED        |  `0x3C`                 |  4 bytes, integer     |  New addition, to make Buffer Descriptor power-of-2 sized. Unused.  |


The 32-bit Buffer Descriptor layout is as follows:
|  Field name      |  Field offset in bytes  |  Field Length + Type  |  Changes from Specification  |
| ------------     | ----------------------- | --------------------- | ---------------------------- |
|  NXTDESC         |  `0x00`                 |  8 bytes, capability |  Has changed from 2 32-bit integers to one capability-wide field  |
|  BUFFER_ADDRESS  |  `0x08`                 |  8 bytes, capability |  Has changed from 2 32-bit integers to one capability-wide field  |
|  RESERVED        |  `0x10`                 |  16 bytes, integer    |  New addition, to maintain Buffer Descriptor size and field alignment. Unused. |
|  CONTROL         |  `0x20`                 |  4 bytes, integer     |  Relocated  |
|  STATUS          |  `0x24`                 |  4 bytes, integer     |  Relocated  |
|  APP0            |  `0x28`                 |  4 bytes, integer     |  Relocated  |
|  APP1            |  `0x2C`                 |  4 bytes, integer     |  Relocated  |
|  APP2            |  `0x30`                 |  4 bytes, integer     |  Relocated  |
|  APP3            |  `0x34`                 |  4 bytes, integer     |  Relocated  |
|  APP4            |  `0x38`                 |  4 bytes, integer     |  Relocated  |
|  RESERVED        |  `0x3C`                 |  4 bytes, integer     |  New addition, to make Buffer Descriptor power-of-2 sized. Unused.  |


### Behaviour on CHERI errors
CHERI errors occur when the authorising capability of an AXI access does not fully authorise the
access. This could be because of a bounds or permission violation. When this happens, the CHERI
checker will block the memory accesses before they go into the bus, meaning the DMA engine does not
modify memory that it has not been given access to via either the capabilities written into its
registers or the capabilities in the Scatter-Gather list.

In the case of a CHERI error, the DMA engine will assert the appropriate bit in its registers and
sets the interrupt signal to high until it is reset by software.



## Instantiating this design
The top-level module for this design is mkAXI4_DMA. The interface requirements are described [above](#axi-interface).
This will create an instance of this module.
The debug output from this module does not have anything to identify which particular instantiation it is from.




