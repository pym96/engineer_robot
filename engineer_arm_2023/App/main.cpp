#include "dev_system.h"

#include "start_task.h"

#include "app_power_ctrl.h"

#include "protocol_dbus.h"

#include "app_usart7.h"


void SoftWareInit(void)
{
	//调整时钟树 使得apb1支持最高8Mhz的串口
//	RCC_HSICmd(ENABLE);
//	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
	
//	RCC_PLLConfig(RCC_PLLSource_HSI, 16, 360, 4, 4);
//	RCC_PLLCmd(ENABLE);
//	
//	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
//	
//	RCC_HCLKConfig(RCC_SYSCLK_Div1);
//	RCC_PCLK1Config(RCC_HCLK_Div1);	
//	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	delay_init((uint32_t)1000);
	power_ctrl_configuration();

//	CAN_ALL_Init();
	
	for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
	{
		power_ctrl_on(i);
		Delay_us(709);//此延时为宏定义，非阻塞延时统一使用Delay_us，Delay_ms，操作系统延时使用osDelay
	}

	  Serial7.Serial_Init( 115200, SERIAL_8N1, 6, 0 ); //哪种初始化方式都可以
//	  Serial7.attachInterrupt(CopeSerial7Data);
	
		Serial6.Serial_Init( 115200, SERIAL_8N1, 6, 0 ); //哪种初始化方式都可以
		
		Serial8.Serial_Init( 115200, SERIAL_8N1, 6, 0 );
	
		remote_control_init();//遥控器初始
		
		
		{//GPIOI 气泵引脚
			GPIO_InitTypeDef GPIO_InitStructure;
				
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
	
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_Init(GPIOI, &GPIO_InitStructure);
		}
		
		{//GPIO 485控制引脚
			GPIO_InitTypeDef GPIO_InitStructure;
				
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
	
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_6;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_Init(GPIOI, &GPIO_InitStructure);
		}

		{//GPIO 限位开关
			GPIO_InitTypeDef GPIO_InitStructure;
				
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
	
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_Init(GPIOH, &GPIO_InitStructure);
		}

		{//GPIO 限位开关
			GPIO_InitTypeDef GPIO_InitStructure;
				
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_12;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_Init(GPIOD, &GPIO_InitStructure);
		}
}

int main(void)
{
	SoftWareInit();

	startTast();
	vTaskStartScheduler();
	
	while (1)
	{
		//delay_ms();
	}
}









//		{	
//			  USART_InitTypeDef USART_InitStructure;
//				GPIO_InitTypeDef GPIO_InitStructure;
//				NVIC_InitTypeDef NVIC_InitStructure;

//			  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
//			  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);

//				GPIO_PinAFConfig(GPIOE,GPIO_PinSource8, GPIO_AF_UART7);
//				GPIO_PinAFConfig(GPIOE,GPIO_PinSource7, GPIO_AF_UART7);
//			
//				GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_7;
//				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//				GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//				GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//				GPIO_Init(GPIOE, &GPIO_InitStructure);
//			
//				NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;
//				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
//				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//				NVIC_Init(&NVIC_InitStructure);
//			
//				//USART2 初始化设置
//				USART_InitStructure.USART_BaudRate = 115200;//波特率设置
//				USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//				USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//				USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
//				USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//				USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
//				USART_Init(UART7, &USART_InitStructure); //初始化串口2
//			
//				USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);
//				USART_Cmd(UART7, ENABLE);
//		}

