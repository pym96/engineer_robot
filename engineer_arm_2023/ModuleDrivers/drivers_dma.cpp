#include "drivers_dma.h"

//DMAdev usart1_TxDMA( DMA1_Stream2, DMA_Channel_7 );
//DMAdev usart1_RxDMA( DMA2_Stream2, DMA_Channel_4 );

//DMAdev usart3_TxDMA( DMA1_Stream3, DMA_Channel_4 );
//DMAdev usart3_RxDMA( DMA1_Stream1, DMA_Channel_4 );
//DMAdev usart6_TxDMA( DMA2_Stream6, DMA_Channel_5 );
DMAdev usart6_RxDMA( DMA2_Stream1, DMA_Channel_5 );
////DMAdev usart7_TxDMA( DMA1_Stream1, DMA_Channel_5 );
////DMAdev usart7_RxDMA( DMA1_Stream3, DMA_Channel_5 );
//DMAdev usart8_TxDMA( DMA1_Stream0, DMA_Channel_5 );
//DMAdev usart8_RxDMA( DMA1_Stream6, DMA_Channel_5 );

DMAdev::DMAdev( DMA_Stream_TypeDef *DMAx_Streamx, uint32_t DMA_Channel_x )
{
    this->DMAx_Streamx = DMAx_Streamx;
	  this->DMA_Channel_x = DMA_Channel_x;
}

void DMAdev::dmaInit( DMA_Config Config, uint32_t *PeripheralAddr, uint32_t *MemoryAddr, uint32_t BufferSize,
                         uint32_t PInc, uint32_t MInc, uint32_t PDataSize, uint32_t MDataSize, uint32_t Priority, 
                         uint32_t Threshold )
{
    uint32_t RCC_AHB1Periph_DMAx;
	
	  if((IS_DMA_ALL_PERIPH( DMAx_Streamx ) || IS_DMA_CHANNEL( DMA_Channel_x )) == 0 )return;
	
    if(( (uint32_t)DMAx_Streamx & DMA2_BASE ) == DMA2_BASE )  RCC_AHB1Periph_DMAx = RCC_AHB1Periph_DMA2;
	  else if(( (uint32_t)DMAx_Streamx & DMA1_BASE ) == DMA1_BASE )  RCC_AHB1Periph_DMAx = RCC_AHB1Periph_DMA1;
	  else return;
	
	  if(( (uint32_t)DMAx_Streamx & 0x0B8 ) == 0x0B8 )DMA_All_Flags = DMA_GetAllFlags( 7 );
	  else if(( (uint32_t)DMAx_Streamx & 0x0A0 ) == 0x0A0 )DMA_All_Flags = DMA_GetAllFlags( 6 );
	  else if(( (uint32_t)DMAx_Streamx & 0x088 ) == 0x088 )DMA_All_Flags = DMA_GetAllFlags( 5 );
	  else if(( (uint32_t)DMAx_Streamx & 0x070 ) == 0x070 )DMA_All_Flags = DMA_GetAllFlags( 4 );
	  else if(( (uint32_t)DMAx_Streamx & 0x058 ) == 0x058 )DMA_All_Flags = DMA_GetAllFlags( 3 );
	  else if(( (uint32_t)DMAx_Streamx & 0x040 ) == 0x040 )DMA_All_Flags = DMA_GetAllFlags( 2 );
	  else if(( (uint32_t)DMAx_Streamx & 0x028 ) == 0x028 )DMA_All_Flags = DMA_GetAllFlags( 1 );
	  else if(( (uint32_t)DMAx_Streamx & 0x010 ) == 0x010 )DMA_All_Flags = DMA_GetAllFlags( 0 );

    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMAx_Streamx);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMAx, ENABLE); /*DMA2的时钟使能*/
	
    while(DMA_GetCmdStatus(DMAx_Streamx) != DISABLE); /*等待DMA可以配置*/
	
    DMA_InitStructure.DMA_Channel = DMA_Channel_x;/*DMA通道0*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)PeripheralAddr; /*外设地址*/
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)MemoryAddr;/*存取器地址*/
    DMA_InitStructure.DMA_DIR = DMA_GetDirection( Config );/*方向从外设到内存*/
    DMA_InitStructure.DMA_BufferSize = BufferSize;/*数据传输的数量为3*/
    DMA_InitStructure.DMA_PeripheralInc = PInc;/*地址不增加*/
    DMA_InitStructure.DMA_MemoryInc = MInc;/*地址不增加*/
    DMA_InitStructure.DMA_PeripheralDataSize = PDataSize;/*数据长度半字*/
    DMA_InitStructure.DMA_MemoryDataSize = MDataSize;/*数据长度半字*/
    DMA_InitStructure.DMA_Priority = Priority;/*高优先级*/
    DMA_InitStructure.DMA_Mode = DMA_GetMode( Config );/*循环模式*/
    DMA_InitStructure.DMA_FIFOMode = DMA_GetFifoMode( Config );/*禁止FIFO*/
    DMA_InitStructure.DMA_FIFOThreshold = Threshold;/*FIFO的值*/
    DMA_InitStructure.DMA_MemoryBurst = DMA_GetMemory( Config );/*单次传输*/
    DMA_InitStructure.DMA_PeripheralBurst = DMA_GetPeripheral( Config );/*单次传输*/
    DMA_Init(DMAx_Streamx, &DMA_InitStructure); /**/
    DMA_Cmd(DMAx_Streamx, ENABLE); //开启DMA传输
}

void DMAdev::InterruptConfig( uint8_t PreemptionPriority, 
			                        uint8_t SubPriority, 
			                        uint32_t Interrupt_Enable_Bits, 
		                          DMA_CallbackFunction_t DMA_CallbackFunction )
{
	  uint16_t ItChannel;
	  NVIC_InitTypeDef NVIC_InitStructure;
	
	  this->DMA_Function = DMA_CallbackFunction;
	
	  if( DMAx_Streamx == DMA1_Stream0 )ItChannel = DMA1_Stream0_IRQn;
	  else if( DMAx_Streamx == DMA1_Stream1 )ItChannel = DMA1_Stream1_IRQn;
	  else if( DMAx_Streamx == DMA1_Stream2 )ItChannel = DMA1_Stream2_IRQn;
	  else if( DMAx_Streamx == DMA1_Stream3 )ItChannel = DMA1_Stream3_IRQn;
	  else if( DMAx_Streamx == DMA1_Stream4 )ItChannel = DMA1_Stream4_IRQn;
	  else if( DMAx_Streamx == DMA1_Stream5 )ItChannel = DMA1_Stream5_IRQn;
	  else if( DMAx_Streamx == DMA1_Stream6 )ItChannel = DMA1_Stream6_IRQn;
	  else if( DMAx_Streamx == DMA1_Stream7 )ItChannel = DMA1_Stream7_IRQn;
	  else if( DMAx_Streamx == DMA2_Stream0 )ItChannel = DMA2_Stream0_IRQn;
	  else if( DMAx_Streamx == DMA2_Stream1 )ItChannel = DMA2_Stream1_IRQn;
	  else if( DMAx_Streamx == DMA2_Stream2 )ItChannel = DMA2_Stream2_IRQn;
	  else if( DMAx_Streamx == DMA2_Stream3 )ItChannel = DMA2_Stream3_IRQn;
	  else if( DMAx_Streamx == DMA2_Stream4 )ItChannel = DMA2_Stream4_IRQn;
	  else if( DMAx_Streamx == DMA2_Stream5 )ItChannel = DMA2_Stream5_IRQn;
	  else if( DMAx_Streamx == DMA2_Stream6 )ItChannel = DMA2_Stream6_IRQn;
	  else if( DMAx_Streamx == DMA2_Stream7 )ItChannel = DMA2_Stream7_IRQn;
		
		if(( (uint32_t)DMAx_Streamx & 0x0B8 ) == 0x0B8 )DMA_All_Interrupt_Bits = DMA_GetAllInterruptBits( 7 );
	  else if(( (uint32_t)DMAx_Streamx & 0x0A0 ) == 0x0A0 )DMA_All_Interrupt_Bits = DMA_GetAllInterruptBits( 6 );
	  else if(( (uint32_t)DMAx_Streamx & 0x088 ) == 0x088 )DMA_All_Interrupt_Bits = DMA_GetAllInterruptBits( 5 );
	  else if(( (uint32_t)DMAx_Streamx & 0x070 ) == 0x070 )DMA_All_Interrupt_Bits = DMA_GetAllInterruptBits( 4 );
	  else if(( (uint32_t)DMAx_Streamx & 0x058 ) == 0x058 )DMA_All_Interrupt_Bits = DMA_GetAllInterruptBits( 3 );
	  else if(( (uint32_t)DMAx_Streamx & 0x040 ) == 0x040 )DMA_All_Interrupt_Bits = DMA_GetAllInterruptBits( 2 );
	  else if(( (uint32_t)DMAx_Streamx & 0x028 ) == 0x028 )DMA_All_Interrupt_Bits = DMA_GetAllInterruptBits( 1 );
	  else if(( (uint32_t)DMAx_Streamx & 0x010 ) == 0x010 )DMA_All_Interrupt_Bits = DMA_GetAllInterruptBits( 0 );
	
	  NVIC_InitStructure.NVIC_IRQChannel = ItChannel;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority ;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	  DMA_ITConfig(DMAx_Streamx, Interrupt_Enable_Bits, ENABLE);
	  
}

void DMAdev::InterruptConfig( DMA_CallbackFunction_t DMA_CallbackFunction )
{
    InterruptConfig( DMA_PreemptionPriority_Default, DMA_SubPriority_Default, 
			               DMA_Interrupt_Enable_Default, DMA_CallbackFunction );
}

void DMAdev::IRQHandler( void )
{
    DMA_Cmd( DMAx_Streamx, DISABLE );
	
	  if(DMA_Function)
    {
      DMA_Function();
    }
		
	  DMA_ClearITPendingBit( DMAx_Streamx, DMA_All_Interrupt_Bits );
		while (DMA_GetCmdStatus(DMAx_Streamx) != DISABLE){};
    DMA_Cmd(DMAx_Streamx, ENABLE);
}

void DMAdev::end( void )
{
    DMA_Cmd( DMAx_Streamx, DISABLE );
}

extern "C"{
//    void DMA1_Stream0_IRQHandler(void)
//		{			
//        usart8_TxDMA.IRQHandler();
//		}
////		void DMA1_Stream1_IRQHandler(void)
////		{			
////				usart7_TxDMA.IRQHandler();
////			  usart3_RxDMA.IRQHandler();
////		}
//	  void DMA1_Stream2_IRQHandler(void)
//		{			
//				
//		}
//		void DMA1_Stream3_IRQHandler(void)
//		{			
//				usart7_RxDMA.IRQHandler();
////			  usart3_TxDMA.IRQHandler();
//		}
//    void DMA1_Stream4_IRQHandler(void)
//		{
//				
//		}
//    void DMA1_Stream5_IRQHandler(void)
//		{

//		}
//    void DMA1_Stream6_IRQHandler(void)
//		{
//				usart8_RxDMA.IRQHandler();
//		}
//    void DMA1_Stream7_IRQHandler(void)
//		{
//				
//		}
//		
//		void DMA2_Stream0_IRQHandler(void)
//		{			
//				
//		}
//		void DMA2_Stream1_IRQHandler(void)
//		{			
//				usart6_RxDMA.IRQHandler();
//		}
//	  void DMA2_Stream2_IRQHandler(void)
//		{			
//				
//		}
//		void DMA2_Stream3_IRQHandler(void)
//		{			
//				
//		}
//    void DMA2_Stream4_IRQHandler(void)
//		{
//				
//		}
//    void DMA2_Stream5_IRQHandler(void)
//		{

//		}
//    void DMA2_Stream6_IRQHandler(void)
//		{
//				usart6_TxDMA.IRQHandler();
//		}
//    void DMA2_Stream7_IRQHandler(void)
//		{
//			
//		}
}
