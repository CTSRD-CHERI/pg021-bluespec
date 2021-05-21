package AXI4_DMA_Types;

`define ZEROES signExtend (1'b0)
`define ONES  signExtend (1'b1)

// Direct Memory Access Buffer Descriptor struct
// This is not used anywhere except for calculating the number of words
// immediately below
typedef struct {
   DMA_BD_Word nxt_desc;
   DMA_BD_Word nxtdesc_msb;
   DMA_BD_Word buffer_address;
   DMA_BD_Word buffer_address_msb;
   DMA_BD_Word reserved0;
   DMA_BD_Word reserved1;
   DMA_BD_Word control;
   DMA_BD_Word status;
   DMA_BD_Word app0;
   DMA_BD_Word app1;
   DMA_BD_Word app2;
   DMA_BD_Word app3;
   DMA_BD_Word app4;
} DMA_BD deriving (Bits, FShow);

typedef TDiv #(SizeOf #(DMA_BD), SizeOf #(DMA_BD_Word)) DMA_Num_Words;

typedef enum {
   MM2S,
   S2MM
} DMA_Dir deriving (Bits, Eq, FShow);

typedef enum {
   DMA_NXTDESC,
   DMA_NXTDESC_MSB,
   DMA_BUFFER_ADDRESS,
   DMA_BUFFER_ADDRESS_MSB,
   DMA_RESERVED0,
   DMA_RESERVED1,
   DMA_CONTROL,
   DMA_STATUS,
   DMA_APP0,
   DMA_APP1,
   DMA_APP2,
   DMA_APP3,
   DMA_APP4
} DMA_BD_Index deriving (Eq, Bits, FShow, Bounded);

// This is defined in the Xilinx specification
typedef Bit #(32) DMA_BD_Word;

//Reg_ refers to the Register Module
typedef enum {
   DMA_MM2S_DMACR          = 'h00,      // offset 0x00
   DMA_MM2S_DMASR          = 'h01,      // offset 0x04
   DMA_MM2S_CURDESC        = 'h02,      // offset 0x08
   DMA_MM2S_CURDESC_MSB    = 'h03,      // offset 0x0c
   DMA_MM2S_TAILDESC       = 'h04,      // offset 0x10
   DMA_MM2S_TAILDESC_MSB   = 'h05,      // offset 0x14
   DMA_MM2S_CURDESC_CAP    = 'h08,      // offset 0x20, length 0x10
   DMA_S2MM_DMACR          = 'h0c,      // offset 0x30
   DMA_S2MM_DMASR          = 'h0d,      // offset 0x34
   DMA_S2MM_CURDESC        = 'h0e,      // offset 0x38
   DMA_S2MM_CURDESC_MSB    = 'h0f,      // offset 0x3c
   DMA_S2MM_TAILDESC       = 'h10,      // offset 0x40
   DMA_S2MM_TAILDESC_MSB   = 'h11,      // offset 0x44
   DMA_S2MM_CURDESC_CAP    = 'h14       // offset 0x50, length 0x10
} DMA_Reg_Index deriving (Eq, Bits, FShow, Bounded);

typedef Bit #(32) DMA_Reg_Word;

// TODO change this to be the correct size
typedef Bit #(32) DMA_Copy_Word;


typedef struct {
   Bit #(8) irqdelay;
   Bit #(8) irqthreshold;
   Bit #(1) reserved1;
   Bit #(1) err_irqen;
   Bit #(1) dly_irqen;
   Bit #(1) ioc_irqen;
   Bit #(7) reserved2;
   Bit #(1) cyclic_bd_enable;
   Bit #(1) keyhole;
   Bit #(1) reset;
   Bit #(1) reserved3;
   Bit #(1) rs;
} MM2S_DMACR deriving (Bits, Eq, FShow);

MM2S_DMACR mm2s_dmacr_default = MM2S_DMACR {
   irqdelay         : 'h00,
   irqthreshold     : 'h01,
   reserved1        : 0,
   err_irqen        : 0,
   dly_irqen        : 0,
   ioc_irqen        : 0,
   reserved2        : 0,
   cyclic_bd_enable : 0,
   keyhole          : 0,
   reset            : 0,
   reserved3        : 1,
   rs               : 0
};

MM2S_DMACR mm2s_dmacr_rw_mask = MM2S_DMACR {
   irqdelay         : `ONES,
   irqthreshold     : `ONES,
   reserved1        : `ZEROES,
   err_irqen        : `ONES,
   dly_irqen        : `ONES,
   ioc_irqen        : `ONES,
   reserved2        : `ZEROES,
   cyclic_bd_enable : `ONES,
   keyhole          : `ONES,
   reset            : `ONES,
   reserved3        : `ZEROES,
   rs               : `ONES
};

typedef struct {
   Bit #(8) irqdelaysts;
   Bit #(8) irqthresholdsts;
   Bit #(1) reserved1;
   Bit #(1) err_irq;
   Bit #(1) dly_irq;
   Bit #(1) ioc_irq;
   Bit #(1) reserved2;
   Bit #(1) sgdecerr;
   Bit #(1) sgslverr;
   Bit #(1) sginterr;
   Bit #(1) reserved3;
   Bit #(1) dmadecerr;
   Bit #(1) dmaslverr;
   Bit #(1) dmainterr;
   Bit #(1) sgincld;
   Bit #(1) reserved4;
   Bit #(1) idle;
   Bit #(1) halted;
} MM2S_DMASR deriving (Bits, Eq, FShow);

MM2S_DMASR mm2s_dmasr_default = MM2S_DMASR {
   irqdelaysts      : 'h00,
   irqthresholdsts  : 'h01,
   reserved1        : 0,
   err_irq          : 0,
   dly_irq          : 0,
   ioc_irq          : 0,
   reserved2        : 0,
   sgdecerr         : 0,
   sgslverr         : 0,
   sginterr         : 0,
   reserved3        : 0,
   dmadecerr        : 0,
   dmaslverr        : 0,
   dmainterr        : 0,
   sgincld          : 1, // SG is always included for this
   reserved4        : 0,
   idle             : 0,
   halted           : 1
};

MM2S_DMASR mm2s_dmasr_rw_mask = MM2S_DMASR {
   irqdelaysts     : `ZEROES,
   irqthresholdsts : `ZEROES,
   reserved1       : `ZEROES,
   err_irq         : `ONES,
   dly_irq         : `ONES,
   ioc_irq         : `ONES,
   reserved2       : `ZEROES,
   sgdecerr        : `ZEROES,
   sgslverr        : `ZEROES,
   sginterr        : `ZEROES,
   reserved3       : `ZEROES,
   dmadecerr       : `ZEROES,
   dmaslverr       : `ZEROES,
   dmainterr       : `ZEROES,
   sgincld         : `ZEROES,
   reserved4       : `ZEROES,
   idle            : `ZEROES,
   halted          : `ZEROES
};

typedef struct {
   Bit #(26) current_descriptor_pointer;
   Bit #(6)  reserved1;
} MM2S_CURDESC deriving (Bits, Eq, FShow);

MM2S_CURDESC mm2s_curdesc_default = MM2S_CURDESC {
   current_descriptor_pointer : 0,
   reserved1                  : 0
};

MM2S_CURDESC mm2s_curdesc_rw_mask = MM2S_CURDESC {
   current_descriptor_pointer : `ZEROES,
   reserved1                  : `ZEROES
};

MM2S_CURDESC mm2s_curdesc_rw_mask_halted = MM2S_CURDESC {
   current_descriptor_pointer : `ONES,
   reserved1                  : `ZEROES
};

typedef struct {
   Bit #(32) current_descriptor_pointer_msb;
} MM2S_CURDESC_MSB deriving (Bits, Eq, FShow);

MM2S_CURDESC_MSB mm2s_curdesc_msb_default = MM2S_CURDESC_MSB {
   current_descriptor_pointer_msb : 0
};

MM2S_CURDESC_MSB mm2s_curdesc_msb_rw_mask = MM2S_CURDESC_MSB {
   current_descriptor_pointer_msb : `ZEROES
};

MM2S_CURDESC_MSB mm2s_curdesc_msb_rw_mask_halted = MM2S_CURDESC_MSB {
   current_descriptor_pointer_msb : `ONES
};

typedef struct {
   Bit #(26) tail_descriptor_pointer;
   Bit #(6) reserved1;
} MM2S_TAILDESC deriving (Bits, Eq, FShow);

MM2S_TAILDESC mm2s_taildesc_default = MM2S_TAILDESC {
   tail_descriptor_pointer : 0,
   reserved1               : 0
};

MM2S_TAILDESC mm2s_taildesc_rw_mask = MM2S_TAILDESC {
   tail_descriptor_pointer : `ONES,
   reserved1               : `ZEROES
};

typedef struct {
   Bit #(32) tail_descriptor_pointer_msb;
} MM2S_TAILDESC_MSB deriving (Bits, Eq, FShow);

MM2S_TAILDESC_MSB mm2s_taildesc_msb_default = MM2S_TAILDESC_MSB {
   tail_descriptor_pointer_msb : 0
};

MM2S_TAILDESC_MSB mm2s_taildesc_msb_rw_mask = MM2S_TAILDESC_MSB {
   tail_descriptor_pointer_msb : `ONES
};


typedef struct {
   Bit #(8) irqdelay;
   Bit #(8) irqthreshold;
   Bit #(1) reserved1;
   Bit #(1) err_irqen;
   Bit #(1) dly_irqen;
   Bit #(1) ioc_irqen;
   Bit #(7) reserved2;
   Bit #(1) cyclic_bd_enable;
   Bit #(1) keyhole;
   Bit #(1) reset;
   Bit #(1) reserved3;
   Bit #(1) rs;
} S2MM_DMACR deriving (Bits, Eq, FShow);

S2MM_DMACR s2mm_dmacr_default = S2MM_DMACR {
   irqdelay         : 'h00,
   irqthreshold     : 'h01,
   reserved1        : 0,
   err_irqen        : 0,
   dly_irqen        : 0,
   ioc_irqen        : 0,
   reserved2        : 0,
   cyclic_bd_enable : 0,
   keyhole          : 0,
   reset            : 0,
   reserved3        : 1,
   rs               : 0
};

S2MM_DMACR s2mm_dmacr_rw_mask = S2MM_DMACR {
   irqdelay         : `ONES,
   irqthreshold     : `ONES,
   reserved1        : `ZEROES,
   err_irqen        : `ONES,
   dly_irqen        : `ONES,
   ioc_irqen        : `ONES,
   reserved2        : `ZEROES,
   cyclic_bd_enable : `ONES,
   keyhole          : `ONES,
   reset            : `ONES,
   reserved3        : `ZEROES,
   rs               : `ONES
};

typedef struct {
   Bit #(8) irqdelaysts;
   Bit #(8) irqthresholdsts;
   Bit #(1) reserved1;
   Bit #(1) err_irq;
   Bit #(1) dly_irq;
   Bit #(1) ioc_irq;
   Bit #(1) reserved2;
   Bit #(1) sgdecerr;
   Bit #(1) sgslverr;
   Bit #(1) sginterr;
   Bit #(1) reserved3;
   Bit #(1) dmadecerr;
   Bit #(1) dmaslverr;
   Bit #(1) dmainterr;
   Bit #(1) sgincld;
   Bit #(1) reserved4;
   Bit #(1) idle;
   Bit #(1) halted;
} S2MM_DMASR deriving (Bits, Eq, FShow);

S2MM_DMASR s2mm_dmasr_default = S2MM_DMASR {
   irqdelaysts     : 'h00,
   irqthresholdsts : 'h01,
   reserved1       : 0,
   err_irq         : 0,
   dly_irq         : 0,
   ioc_irq         : 0,
   reserved2       : 0,
   sgdecerr        : 0,
   sgslverr        : 0,
   sginterr        : 0,
   reserved3       : 0,
   dmadecerr       : 0,
   dmaslverr       : 0,
   dmainterr       : 0,
   sgincld         : 1, // SG is always included for this
   reserved4       : 0,
   idle            : 0,
   halted          : 1
};

S2MM_DMASR s2mm_dmasr_rw_mask = S2MM_DMASR {
   irqdelaysts       : `ZEROES,
   irqthresholdsts   : `ZEROES,
   reserved1         : `ZEROES,
   err_irq           : `ONES,
   dly_irq           : `ONES,
   ioc_irq           : `ONES,
   reserved2         : `ZEROES,
   sgdecerr          : `ZEROES,
   sgslverr          : `ZEROES,
   sginterr          : `ZEROES,
   reserved3         : `ZEROES,
   dmadecerr         : `ZEROES,
   dmaslverr         : `ZEROES,
   dmainterr         : `ZEROES,
   sgincld           : `ZEROES,
   reserved4         : `ZEROES,
   idle              : `ZEROES,
   halted            : `ZEROES
};

typedef struct {
   Bit #(26) current_descriptor_pointer;
   Bit #(6) reserved1;
} S2MM_CURDESC deriving (Bits, Eq, FShow);

S2MM_CURDESC s2mm_curdesc_default = S2MM_CURDESC {
   current_descriptor_pointer : 0,
   reserved1                  : 0
};

S2MM_CURDESC s2mm_curdesc_rw_mask = S2MM_CURDESC {
   current_descriptor_pointer : `ZEROES,
   reserved1                  : `ZEROES
};

S2MM_CURDESC s2mm_curdesc_rw_mask_halted = S2MM_CURDESC {
   current_descriptor_pointer : `ONES,
   reserved1                  : `ZEROES
};

typedef struct {
   Bit #(32) current_descriptor_pointer_msb;
} S2MM_CURDESC_MSB deriving (Bits, Eq, FShow);

S2MM_CURDESC_MSB s2mm_curdesc_msb_default = S2MM_CURDESC_MSB {
   current_descriptor_pointer_msb : 0
};

S2MM_CURDESC_MSB s2mm_curdesc_msb_rw_mask = S2MM_CURDESC_MSB {
   current_descriptor_pointer_msb : `ZEROES
};

S2MM_CURDESC_MSB s2mm_curdesc_msb_rw_mask_halted = S2MM_CURDESC_MSB {
   current_descriptor_pointer_msb : `ONES
};

typedef struct {
   Bit #(26) tail_descriptor_pointer;
   Bit #(6) reserved1;
} S2MM_TAILDESC deriving (Bits, Eq, FShow);

S2MM_TAILDESC s2mm_taildesc_default = S2MM_TAILDESC {
   tail_descriptor_pointer : 0,
   reserved1               : 0
};

S2MM_TAILDESC s2mm_taildesc_rw_mask = S2MM_TAILDESC {
   tail_descriptor_pointer : `ONES,
   reserved1               : `ZEROES
};

typedef struct {
   Bit #(32) tail_descriptor_pointer_msb;
} S2MM_TAILDESC_MSB deriving (Bits, Eq, FShow);

S2MM_TAILDESC_MSB s2mm_taildesc_msb_default = S2MM_TAILDESC_MSB {
   tail_descriptor_pointer_msb : 0
};

S2MM_TAILDESC_MSB s2mm_taildesc_msb_rw_mask = S2MM_TAILDESC_MSB {
   tail_descriptor_pointer_msb : `ONES
};








// DMA Register default values
DMA_Reg_Word dma_reg_mm2s_dmacr_default =
   { 8'h00  // IRQDelay
   , 8'h01  // IRQThreshold
   , 1'h0   // Reserved
   , 1'h0   // Err_IrqEn
   , 1'h0   // Dly_IrqEn
   , 1'h0   // IOC_IrqEn
   , 7'h0   // Reserved
   , 1'h0   // Cyclic BD Enable
   , 1'h0   // Keyhole
   , 1'h0   // Reset
   , 1'h1   // Reserved
   , 1'h0   // RS
   };

DMA_Reg_Word dma_reg_mm2s_dmacr_rw_mask =
   { 8'hff  // IRQDelay
   , 8'hff  // IRQThreshold
   , 1'h0   // Reserved
   , 1'h1   // Err_IrqEn
   , 1'h1   // Dly_IrqEn
   , 1'h1   // IOC_IrqEn
   , 7'h0   // Reserved
   , 1'h1   // Cyclic BD Enable
   , 1'h1   // Keyhole
   , 1'h1   // Reset
   , 1'h0   // Reserved
   , 1'h1   // RS
   };

DMA_Reg_Word dma_reg_mm2s_dmasr_default =
   { 8'h00  // IRQDelaySts
   , 8'h01  // IRQThresholdSts
   , 1'h0   // Reserved
   , 1'h0   // Err_Irq
   , 1'h0   // Dly_Irq
   , 1'h0   // IOC_Irq
   , 1'h0   // Reserved
   , 1'h0   // SGDecErr
   , 1'h0   // SGSlvErr
   , 1'h0   // SGIntErr
   , 1'h0   // Reserved
   , 1'h0   // DMADecErr
   , 1'h0   // DMASlvErr
   , 1'h0   // DMAIntErr
   , 1'h1   // SGIncld
   , 1'h0   // Reserved
   , 1'h0   // Idle
   , 1'h1   // Halted
   };

DMA_Reg_Word dma_reg_mm2s_dmasr_rw_mask =
   { 8'h00  // IRQDelaySts
   , 8'h00  // IRQThresholdSts
   , 1'h0   // Reserved
   , 1'h1   // Err_Irq
   , 1'h1   // Dly_Irq
   , 1'h1   // IOC_Irq
   , 1'h0   // Reserved
   , 1'h0   // SGDecErr
   , 1'h0   // SGSlvErr
   , 1'h0   // SGIntErr
   , 1'h0   // Reserved
   , 1'h0   // DMADecErr
   , 1'h0   // DMASlvErr
   , 1'h0   // DMAIntErr
   , 1'h0   // SGIncId
   , 1'h0   // Reserved
   , 1'h0   // Idle
   , 1'h0   // Halted
   };

DMA_Reg_Word dma_reg_mm2s_curdesc_default =
   { 26'h0 // Current Descriptor Pointer
   , 6'h0  // Reserved
   };

DMA_Reg_Word dma_reg_mm2s_curdesc_rw_mask_halted =
   { 26'h3ffffff // Current Descriptor Pointer
   , 6'h0        // Reserved
   };

DMA_Reg_Word dma_reg_mm2s_curdesc_rw_mask =
   { 26'h0 // Current Descriptor Pointer
   , 6'h0  // Reserved
   };

DMA_Reg_Word dma_reg_mm2s_curdesc_msb_default =
   {32'h0   // Current Descriptor Pointer MSB
   };

DMA_Reg_Word dma_reg_mm2s_curdesc_msb_rw_mask_halted =
   {32'hffffffff   // Current Descriptor Pointer MSB
   };

DMA_Reg_Word dma_reg_mm2s_curdesc_msb_rw_mask =
   {32'h0 // Current Descriptor Pointer MSB
   };

DMA_Reg_Word dma_reg_mm2s_taildesc_default =
   { 26'h0     // Tail Descriptor Pointer
   , 6'h0      // Reserved
   };

DMA_Reg_Word dma_reg_mm2s_taildesc_rw_mask =
   { 26'h3ffffff     // Tail Descriptor Pointer
   , 6'h0            // Reserved
   };

DMA_Reg_Word dma_reg_mm2s_taildesc_msb_default =
   {32'h0   // Tail Descriptor Pointer MSB
   };

DMA_Reg_Word dma_reg_mm2s_taildesc_msb_rw_mask =
   {32'hffffffff   // Tail Descriptor Pointer MSB
   };

DMA_Reg_Word dma_reg_sg_ctl_default =
   { 20'h0   // Reserved
   , 4'h0    // SG_USER
   , 4'h0    // Reserved
   , 4'h3    // SG_CACHE
   };

DMA_Reg_Word dma_reg_sg_ctl_rw_mask =
   { 20'h0   // Reserved
   , 4'h0    // SG_USER
   , 4'h0    // Reserved
   , 4'h3    // SG_CACHE
   };

DMA_Reg_Word dma_reg_s2mm_dmacr_default =
   { 8'h00  // IRQDelay
   , 8'h01  // IRQThreshold
   , 1'h0   // Reserved
   , 1'h0   // Err_IrqEn
   , 1'h0   // Dly_IrqEn
   , 1'h0   // IOC_IrqEn
   , 7'h0   // Reserved
   , 1'h0   // Cyclic BD Enable
   , 1'h0   // Keyhole
   , 1'h0   // Reset
   , 1'h1   // Reserved
   , 1'h0   // RS
   };

DMA_Reg_Word dma_reg_s2mm_dmasr_default =
   { 8'h00  // IRQDelaySts
   , 8'h01  // IRQThresholdSts
   , 1'h0   // Reserved
   , 1'h0   // Err_Irq
   , 1'h0   // Dly_Irq
   , 1'h0   // IOC_Irq
   , 1'h0   // Reserved
   , 1'h0   // SGDecErr
   , 1'h0   // SGSlvErr
   , 1'h0   // SGIntErr
   , 1'h0   // Reserved
   , 1'h0   // DMADecErr
   , 1'h0   // DMASlvErr
   , 1'h0   // DMAIntErr
   , 1'h1   // SGIncId
   , 1'h0   // Reserved
   , 1'h0   // Idle
   , 1'h1   // Halted
   };

DMA_Reg_Word dma_reg_s2mm_curdesc_default =
   { 26'h0   // Current Descriptor Pointer
   , 6'h0    // Reserved
   };

DMA_Reg_Word dma_reg_s2mm_curdesc_msb_default =
   { 32'h0  // Current Descriptor Pointer MSB
   };

DMA_Reg_Word dma_reg_s2mm_taildesc_default =
   { 26'h0  // Tail Descriptor Pointer
   , 6'h0   // Reserved
   };

DMA_Reg_Word dma_reg_s2mm_taildesc_msb_default =
   { 32'h0 // Tail Descriptor Pointer MSB
   };


endpackage
