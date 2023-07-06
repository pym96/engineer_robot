#ifndef __DRIVERS_DMA_H
#define __DRIVERS_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platform.h"
	
#ifdef __cplusplus
}
#endif

#define DMA_GetFifoMode( Config )     ((uint32_t)(Config & 0x0000000F))
#define DMA_GetDirection( Config )    ((uint32_t)(Config & 0x000000F0))
#define DMA_GetMode( Config )         ((uint32_t)(Config & 0x00000F00))
#define DMA_GetPeripheral( Config )   ((uint32_t)((Config & 0x000F0000)<<4))
#define DMA_GetMemory( Config )       ((uint32_t)(Config & 0x0FF00000))
#define DMA_GetAllInterruptBits( x )  (DMA_IT_FEIF##x|DMA_IT_DMEIF##x|DMA_IT_TEIF##x|DMA_IT_HTIF##x|DMA_IT_TCIF##x)
#define DMA_GetAllFlags( x )          (DMA_FLAG_FEIF##x|DMA_FLAG_DMEIF##x|DMA_FLAG_TEIF##x|DMA_FLAG_HTIF##x|DMA_FLAG_TCIF##x)

#define DMA_Interrupt_Enable_Default   DMA_IT_TE|DMA_IT_TC
#define DMA_PTM_Config_Default         PTM_CR_SGSG_DS
#define DMA_MTP_Config_Default         MTP_CR_SGSG_DS
#define DMA_PreemptionPriority_Default 2
#define DMA_SubPriority_Default        2

typedef enum
{
    PTM_CR_SGSG_DS   = DMA_DIR_PeripheralToMemory | DMA_Mode_Circular | DMA_MemoryBurst_Single | (DMA_PeripheralBurst_Single >> 4) | DMA_FIFOMode_Disable,
    MTP_CR_SGSG_DS   = DMA_DIR_MemoryToPeripheral | DMA_Mode_Circular | DMA_MemoryBurst_Single | (DMA_PeripheralBurst_Single >> 4) | DMA_FIFOMode_Disable,
    PTM_NM_SGSG_DS   = DMA_DIR_PeripheralToMemory | DMA_Mode_Normal   | DMA_MemoryBurst_Single | (DMA_PeripheralBurst_Single >> 4) | DMA_FIFOMode_Disable,
    MTP_NM_SGSG_DS   = DMA_DIR_MemoryToPeripheral | DMA_Mode_Normal   | DMA_MemoryBurst_Single | (DMA_PeripheralBurst_Single >> 4) | DMA_FIFOMode_Disable,	
	
    PTM_CR_SGSG_EN   = DMA_DIR_PeripheralToMemory | DMA_Mode_Circular | DMA_MemoryBurst_Single | (DMA_PeripheralBurst_Single >> 4)| DMA_FIFOMode_Enable,
	  PTM_CR_M4P4_DS   = DMA_DIR_PeripheralToMemory | DMA_Mode_Circular | DMA_MemoryBurst_INC4   | (DMA_PeripheralBurst_INC4 >> 4)  | DMA_FIFOMode_Disable,
	  PTM_CR_M4P8_DS   = DMA_DIR_PeripheralToMemory | DMA_Mode_Circular | DMA_MemoryBurst_INC4   | (DMA_PeripheralBurst_INC8 >> 4)  | DMA_FIFOMode_Disable,
	  PTM_CR_M4P16_DS  = DMA_DIR_PeripheralToMemory | DMA_Mode_Circular | DMA_MemoryBurst_INC4   | (DMA_PeripheralBurst_INC16 >> 4) | DMA_FIFOMode_Disable,
	  PTM_CR_M8P4_DS   = DMA_DIR_PeripheralToMemory | DMA_Mode_Circular | DMA_MemoryBurst_INC8   | (DMA_PeripheralBurst_INC4 >> 4)  | DMA_FIFOMode_Disable,
	  PTM_CR_M8P8_DS   = DMA_DIR_PeripheralToMemory | DMA_Mode_Circular | DMA_MemoryBurst_INC8   | (DMA_PeripheralBurst_INC8 >> 4)  | DMA_FIFOMode_Disable,
	  PTM_CR_M8P16_DS  = DMA_DIR_PeripheralToMemory | DMA_Mode_Circular | DMA_MemoryBurst_INC8   | (DMA_PeripheralBurst_INC16 >> 4) | DMA_FIFOMode_Disable,
	  PTM_CR_M16P4_DS  = DMA_DIR_PeripheralToMemory | DMA_Mode_Circular | DMA_MemoryBurst_INC16  | (DMA_PeripheralBurst_INC4 >> 4)  | DMA_FIFOMode_Disable,
	  PTM_CR_M16P8_DS  = DMA_DIR_PeripheralToMemory | DMA_Mode_Circular | DMA_MemoryBurst_INC16  | (DMA_PeripheralBurst_INC8 >> 4)  | DMA_FIFOMode_Disable,
	  PTM_CR_M16P16_DS = DMA_DIR_PeripheralToMemory | DMA_Mode_Circular | DMA_MemoryBurst_INC16  | (DMA_PeripheralBurst_INC16 >> 4) | DMA_FIFOMode_Disable,
	
	  PTM_NM_SGSG_EN   = DMA_DIR_PeripheralToMemory | DMA_Mode_Normal | DMA_MemoryBurst_Single | (DMA_PeripheralBurst_Single >> 4) | DMA_FIFOMode_Enable,
	  PTM_NM_M4P4_DS   = DMA_DIR_PeripheralToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC4   | (DMA_PeripheralBurst_INC4 >> 4)   | DMA_FIFOMode_Disable,
	  PTM_NM_M4P8_DS   = DMA_DIR_PeripheralToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC4   | (DMA_PeripheralBurst_INC8 >> 4)   | DMA_FIFOMode_Disable,
	  PTM_NM_M4P16_DS  = DMA_DIR_PeripheralToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC4   | (DMA_PeripheralBurst_INC16 >> 4)  | DMA_FIFOMode_Disable,
	  PTM_NM_M8P4_DS   = DMA_DIR_PeripheralToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC8   | (DMA_PeripheralBurst_INC4 >> 4)   | DMA_FIFOMode_Disable,
	  PTM_NM_M8P8_DS   = DMA_DIR_PeripheralToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC8   | (DMA_PeripheralBurst_INC8 >> 4)   | DMA_FIFOMode_Disable,
	  PTM_NM_M8P16_DS  = DMA_DIR_PeripheralToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC8   | (DMA_PeripheralBurst_INC16 >> 4)  | DMA_FIFOMode_Disable,
	  PTM_NM_M16P4_DS  = DMA_DIR_PeripheralToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC16  | (DMA_PeripheralBurst_INC4 >> 4)   | DMA_FIFOMode_Disable,
	  PTM_NM_M16P8_DS  = DMA_DIR_PeripheralToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC16  | (DMA_PeripheralBurst_INC8 >> 4)   | DMA_FIFOMode_Disable,
	  PTM_NM_M16P16_DS = DMA_DIR_PeripheralToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC16  | (DMA_PeripheralBurst_INC16 >> 4)  | DMA_FIFOMode_Disable,
		
	  MTP_CR_SGSG_EN   = DMA_DIR_MemoryToPeripheral | DMA_Mode_Circular | DMA_MemoryBurst_Single | (DMA_PeripheralBurst_Single >> 4) | DMA_FIFOMode_Enable,
	  MTP_CR_M4P4_DS   = DMA_DIR_MemoryToPeripheral | DMA_Mode_Circular | DMA_MemoryBurst_INC4   | (DMA_PeripheralBurst_INC4 >> 4)   | DMA_FIFOMode_Disable,
	  MTP_CR_M4P8_DS   = DMA_DIR_MemoryToPeripheral | DMA_Mode_Circular | DMA_MemoryBurst_INC4   | (DMA_PeripheralBurst_INC8 >> 4)   | DMA_FIFOMode_Disable,
	  MTP_CR_M4P16_DS  = DMA_DIR_MemoryToPeripheral | DMA_Mode_Circular | DMA_MemoryBurst_INC4   | (DMA_PeripheralBurst_INC16 >> 4)  | DMA_FIFOMode_Disable,
	  MTP_CR_M8P4_DS   = DMA_DIR_MemoryToPeripheral | DMA_Mode_Circular | DMA_MemoryBurst_INC8   | (DMA_PeripheralBurst_INC4 >> 4)   | DMA_FIFOMode_Disable,
	  MTP_CR_M8P8_DS   = DMA_DIR_MemoryToPeripheral | DMA_Mode_Circular | DMA_MemoryBurst_INC8   | (DMA_PeripheralBurst_INC8 >> 4)   | DMA_FIFOMode_Disable,
	  MTP_CR_M8P16_DS  = DMA_DIR_MemoryToPeripheral | DMA_Mode_Circular | DMA_MemoryBurst_INC8   | (DMA_PeripheralBurst_INC16 >> 4)  | DMA_FIFOMode_Disable,
	  MTP_CR_M16P4_DS  = DMA_DIR_MemoryToPeripheral | DMA_Mode_Circular | DMA_MemoryBurst_INC16  | (DMA_PeripheralBurst_INC4 >> 4)   | DMA_FIFOMode_Disable,
	  MTP_CR_M16P8_DS  = DMA_DIR_MemoryToPeripheral | DMA_Mode_Circular | DMA_MemoryBurst_INC16  | (DMA_PeripheralBurst_INC8 >> 4)   | DMA_FIFOMode_Disable,
	  MTP_CR_M16P16_DS = DMA_DIR_MemoryToPeripheral | DMA_Mode_Circular | DMA_MemoryBurst_INC16  | (DMA_PeripheralBurst_INC16 >> 4)  | DMA_FIFOMode_Disable,
		
	  MTP_NM_SGSG_EN   = DMA_DIR_MemoryToPeripheral | DMA_Mode_Normal | DMA_MemoryBurst_Single | (DMA_PeripheralBurst_Single >> 4) | DMA_FIFOMode_Enable,
	  MTP_NM_M4P4_DS   = DMA_DIR_MemoryToPeripheral | DMA_Mode_Normal | DMA_MemoryBurst_INC4   | (DMA_PeripheralBurst_INC4 >> 4)   | DMA_FIFOMode_Disable,
	  MTP_NM_M4P8_DS   = DMA_DIR_MemoryToPeripheral | DMA_Mode_Normal | DMA_MemoryBurst_INC4   | (DMA_PeripheralBurst_INC8 >> 4)   | DMA_FIFOMode_Disable,
	  MTP_NM_M4P16_DS  = DMA_DIR_MemoryToPeripheral | DMA_Mode_Normal | DMA_MemoryBurst_INC4   | (DMA_PeripheralBurst_INC16 >> 4)  | DMA_FIFOMode_Disable,
	  MTP_NM_M8P4_DS   = DMA_DIR_MemoryToPeripheral | DMA_Mode_Normal | DMA_MemoryBurst_INC8   | (DMA_PeripheralBurst_INC4 >> 4)   | DMA_FIFOMode_Disable,
	  MTP_NM_M8P8_DS   = DMA_DIR_MemoryToPeripheral | DMA_Mode_Normal | DMA_MemoryBurst_INC8   | (DMA_PeripheralBurst_INC8 >> 4)   | DMA_FIFOMode_Disable,
	  MTP_NM_M8P16_DS  = DMA_DIR_MemoryToPeripheral | DMA_Mode_Normal | DMA_MemoryBurst_INC8   | (DMA_PeripheralBurst_INC16 >> 4)  | DMA_FIFOMode_Disable,
	  MTP_NM_M16P4_DS  = DMA_DIR_MemoryToPeripheral | DMA_Mode_Normal | DMA_MemoryBurst_INC16  | (DMA_PeripheralBurst_INC4 >> 4)   | DMA_FIFOMode_Disable,
	  MTP_NM_M16P8_DS  = DMA_DIR_MemoryToPeripheral | DMA_Mode_Normal | DMA_MemoryBurst_INC16  | (DMA_PeripheralBurst_INC8 >> 4)   | DMA_FIFOMode_Disable,
	  MTP_NM_M16P16_DS = DMA_DIR_MemoryToPeripheral | DMA_Mode_Normal | DMA_MemoryBurst_INC16  | (DMA_PeripheralBurst_INC16 >> 4)  | DMA_FIFOMode_Disable,
		
	  MTM_NM_SGSG_DS   = DMA_DIR_MemoryToMemory | DMA_Mode_Normal | DMA_MemoryBurst_Single | (DMA_PeripheralBurst_Single >> 4) | DMA_FIFOMode_Disable,
	  MTM_NM_M4P4_DS   = DMA_DIR_MemoryToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC4   | (DMA_PeripheralBurst_INC4 >> 4)   | DMA_FIFOMode_Disable,
	  MTM_NM_M4P8_DS   = DMA_DIR_MemoryToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC4   | (DMA_PeripheralBurst_INC8 >> 4)   | DMA_FIFOMode_Disable,
	  MTM_NM_M4P16_DS  = DMA_DIR_MemoryToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC4   | (DMA_PeripheralBurst_INC16 >> 4)  | DMA_FIFOMode_Disable,
	  MTM_NM_M8P4_DS   = DMA_DIR_MemoryToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC8   | (DMA_PeripheralBurst_INC4 >> 4)   | DMA_FIFOMode_Disable,
	  MTM_NM_M8P8_DS   = DMA_DIR_MemoryToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC8   | (DMA_PeripheralBurst_INC8 >> 4)   | DMA_FIFOMode_Disable,
	  MTM_NM_M8P16_DS  = DMA_DIR_MemoryToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC8   | (DMA_PeripheralBurst_INC16 >> 4)  | DMA_FIFOMode_Disable,
	  MTM_NM_M16P4_DS  = DMA_DIR_MemoryToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC16  | (DMA_PeripheralBurst_INC4 >> 4)   | DMA_FIFOMode_Disable,
	  MTM_NM_M16P8_DS  = DMA_DIR_MemoryToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC16  | (DMA_PeripheralBurst_INC8 >> 4)   | DMA_FIFOMode_Disable,
	  MTM_NM_M16P16_DS = DMA_DIR_MemoryToMemory | DMA_Mode_Normal | DMA_MemoryBurst_INC16  | (DMA_PeripheralBurst_INC16 >> 4)  | DMA_FIFOMode_Disable
	
}DMA_Config;

class DMAdev
{
	typedef void(*DMA_CallbackFunction_t)(void);
	
public:
    DMAdev( DMA_Stream_TypeDef *DMAx_Streamx, uint32_t DMA_Channel_x );
    void dmaInit( DMA_Config Config, uint32_t *PeripheralAddr, uint32_t *MemoryAddr, uint32_t BufferSize,
                  uint32_t PInc, uint32_t MInc, uint32_t PDataSize, uint32_t MDataSize, uint32_t Priority, 
                  uint32_t Threshold );
		void InterruptConfig( uint8_t PreemptionPriority, 
			                    uint8_t SubPriority, 
			                    uint32_t Interrupt_Enable_Bits, 
		                      DMA_CallbackFunction_t DMA_CallbackFunction );
		void InterruptConfig( DMA_CallbackFunction_t DMA_CallbackFunction );
		void IRQHandler( void );
		
		void end( void );
private:
	  DMA_Stream_TypeDef *DMAx_Streamx;
	  uint32_t DMA_Channel_x;
	  uint32_t DMA_Receive_Size;

    uint32_t DMA_All_Flags;
    uint32_t DMA_All_Interrupt_Bits;

		DMA_CallbackFunction_t DMA_Function;
};

#endif /* __DRIVERS_DMA_H */
