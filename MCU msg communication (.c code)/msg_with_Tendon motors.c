#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "USART3.h"
#include "control.h"



void Initial_UART3(unsigned long bound)
{         
		NVIC_InitTypeDef NVIC_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

		USART_DeInit(USART3);  
		//USART3_TX   PB.10
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
		GPIO_Init(GPIOB, &GPIO_InitStructure);  

		//USART3_RX    PB.11  
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
		GPIO_Init(GPIOB, &GPIO_InitStructure);  

		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 2 ; 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;    
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;      
		NVIC_Init(&NVIC_InitStructure);  

		USART_InitStructure.USART_BaudRate = bound; 
		USART_InitStructure.USART_WordLength = USART_WordLength_8b; 
		USART_InitStructure.USART_StopBits = USART_StopBits_1; 
		USART_InitStructure.USART_Parity = USART_Parity_No; 
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  

		USART_Init(USART3, &USART_InitStructure);  
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); 
		USART_Cmd(USART3, ENABLE);                   

}

struct Actutation_Torque act_Torque;
struct Motor_Encoder     mot_Encoder;
struct Actutation_Length act_Length;

unsigned char last_data = 0x00;
unsigned char act_rxbuf[6] = {0x00};// 接收数据的缓冲区

int act_data_receive_flag = 0;
int act_buf_count = 0;
int effect_data_length = 5;

int Torque_limit_max = 1024;

extern int act_timer_reset_flag ;
extern int manipulator_initial_flag  ;
extern int read_info_switch ;

void torque_reshape(void);
void encoder_distance(void);

void USART3_IRQHandler(void)
{
		unsigned char data = 0x00;
	
		if(USART_GetFlagStatus(USART3 , USART_IT_RXNE) == SET)
		{
			data = USART_ReceiveData(USART3);
 
			// 开始接收数据
			if(act_data_receive_flag)
			{
				// 计算校验和
				unsigned char temp_checksum = 0x00;
				for(int i = 0; i < act_buf_count ; i++)
				{
					temp_checksum = temp_checksum + act_rxbuf[i];
				}
				unsigned char checksum = ~temp_checksum;
	
				// 这里的判断很重要 ， 如果数组索引值超出数组本身的长度6 ，就会导致卡死程序		
				if(act_buf_count < 6)
				{
					// 给数组赋值
					act_rxbuf[act_buf_count] = data;
					act_buf_count++;

					// 则校验和匹配，收到一个完整数据包
					if(checksum == data) 
					{
						// 解析数据包：
						if(act_buf_count - 1 == effect_data_length)
						{
							// 高位在后，低位在前，合并两个字节成一个数
							// 得到电机扭矩torque   5-8
							if(act_timer_reset_flag >= 5 && act_timer_reset_flag <= 8)
							{
								switch(act_rxbuf[0])
								{
									case 0x01:          
										act_Torque.Torque_1 = (act_rxbuf[4] << 8) | act_rxbuf[3];
									break;
									
									case 0x02:
										act_Torque.Torque_2 = (act_rxbuf[4] << 8) | act_rxbuf[3];
									break;

									case 0x03:
										act_Torque.Torque_3 = (act_rxbuf[4] << 8) | act_rxbuf[3];
									break;

									case 0x04:
										act_Torque.Torque_4 = (act_rxbuf[4] << 8) | act_rxbuf[3];
									break;							
								}
							}							
							// 超出最大扭矩值，需要加个负号
							torque_reshape();
							
							
							// 得到电机编码器数值： 9-12
							if(act_timer_reset_flag >= 9 && act_timer_reset_flag <= 12)
							{
								switch(act_rxbuf[0])
								{
									case 0x01:          
										mot_Encoder.Encoder_1 = (act_rxbuf[4] << 8) | act_rxbuf[3];
									break;
									
									case 0x02:
										mot_Encoder.Encoder_2 = (act_rxbuf[4] << 8) | act_rxbuf[3];
									break;

									case 0x03:
										mot_Encoder.Encoder_3 = (act_rxbuf[4] << 8) | act_rxbuf[3];
									break;

									case 0x04:
										mot_Encoder.Encoder_4 = (act_rxbuf[4] << 8) | act_rxbuf[3];
									break;							
								}
							}
							
							
						}
						
					
						///////////////////////////////////////////////////////////////////////
						// 复位标志位
						act_data_receive_flag = 0;
						act_buf_count = 0;
						
						//数组也要全部清零！  
						for(int i = 0; i < 6; i++)
						{
							act_rxbuf[i] = 0x00;
						}
					}
				}
				else
				{
					// 复位标志位
					act_data_receive_flag = 0;
					act_buf_count = 0;
				}
			
			}
			
			// 判断包头到来：0xFF 0xFF
			if(data == 0xFF && last_data == 0xFF)
			{	
				act_data_receive_flag = 1;
				act_buf_count = 0;
			} 
			last_data = data;
			 
		}
		
		USART_ClearFlag(USART3, USART_IT_RXNE);	
}
 

void torque_reshape(void)
{
	if(act_Torque.Torque_1 > Torque_limit_max) 
	{
		act_Torque.Torque_1 = -(act_Torque.Torque_1 - Torque_limit_max);
	}
	if(act_Torque.Torque_2 > Torque_limit_max) 
	{
		act_Torque.Torque_2 = -(act_Torque.Torque_2 - Torque_limit_max);
	}
	if(act_Torque.Torque_3 > Torque_limit_max) 
	{
		act_Torque.Torque_3 = -(act_Torque.Torque_3 - Torque_limit_max);
	}
	if(act_Torque.Torque_4 > Torque_limit_max) 
	{
		act_Torque.Torque_4 = -(act_Torque.Torque_4 - Torque_limit_max);
	}
}

float pi        = 3.141592653;
int   rolly_dia = 11;
int   cir_limit = 4096;
float Encoder_1_initial = 0, Encoder_2_initial = 0, Encoder_3_initial = 0, Encoder_4_initial = 0;
float Encoder_1_length  = 0, Encoder_2_length  = 0, Encoder_3_length  = 0, Encoder_4_length  = 0;
float Encoder_1_cnt     = 0, Encoder_2_cnt     = 0, Encoder_3_cnt     = 0, Encoder_4_cnt     = 0;
float Encoder_1_last    = 0, Encoder_2_last    = 0, Encoder_3_last    = 0, Encoder_4_last    = 0;
float Encoder_1         = 0, Encoder_2         = 0, Encoder_3         = 0, Encoder_4         = 0;

//根据电机编码器求解电机转动的总行程：
void encoder_distance(void)
{
	if(manipulator_initial_flag == 0 )
	{
		Encoder_1_initial =  mot_Encoder.Encoder_1;
		Encoder_2_initial =  mot_Encoder.Encoder_2;
		Encoder_3_initial =  mot_Encoder.Encoder_3;
		Encoder_4_initial =  mot_Encoder.Encoder_4;							
	}
	if(manipulator_initial_flag == 1)
	{
		Encoder_1_length = Encoder_1_cnt * cir_limit + mot_Encoder.Encoder_1 - Encoder_1_initial;
		Encoder_2_length = Encoder_2_cnt * cir_limit + mot_Encoder.Encoder_2 - Encoder_2_initial;
		Encoder_3_length = Encoder_3_cnt * cir_limit + mot_Encoder.Encoder_3 - Encoder_3_initial;
		Encoder_4_length = Encoder_4_cnt * cir_limit + mot_Encoder.Encoder_4 - Encoder_4_initial;																																	
	}
	
	if(mot_Encoder.Encoder_1 > 3500 && Encoder_1_last < 500)
	{
		Encoder_1_cnt = Encoder_1_cnt - 1;
	}
	if(mot_Encoder.Encoder_1 < 500 && Encoder_1_last > 3500)
	{
		Encoder_1_cnt = Encoder_1_cnt + 1;
	}		
	
	if(mot_Encoder.Encoder_2 > 3500 && Encoder_2_last < 500)
	{
		Encoder_2_cnt = Encoder_2_cnt - 1;
	}
	if(mot_Encoder.Encoder_2 < 500 && Encoder_2_last > 3500)
	{
		Encoder_2_cnt = Encoder_2_cnt + 1;
	}		
	
	if(mot_Encoder.Encoder_3 > 3500 && Encoder_3_last < 500)
	{
		Encoder_3_cnt = Encoder_3_cnt - 1;
	}
	if(mot_Encoder.Encoder_3 < 500 && Encoder_3_last > 3500)
	{
		Encoder_3_cnt = Encoder_3_cnt + 1;
	}		
	
	if(mot_Encoder.Encoder_4 > 3500 && Encoder_4_last < 500)
	{
		Encoder_4_cnt = Encoder_4_cnt - 1;
	}
	if(mot_Encoder.Encoder_4 < 500 && Encoder_4_last > 3500)
	{
		Encoder_4_cnt = Encoder_4_cnt + 1;
	}				
	
	Encoder_1_last = mot_Encoder.Encoder_1;
	Encoder_2_last = mot_Encoder.Encoder_2;
	Encoder_3_last = mot_Encoder.Encoder_3;
	Encoder_4_last = mot_Encoder.Encoder_4;
	
	//根据总行程来求解线长的变化：
	act_Length.Length_1 = 1.0 * Encoder_1_length / cir_limit * rolly_dia * pi;
	act_Length.Length_2 = 1.0 * Encoder_2_length / cir_limit * rolly_dia * pi;
	act_Length.Length_3 = 1.0 * Encoder_3_length / cir_limit * rolly_dia * pi;
	act_Length.Length_4 = 1.0 * Encoder_4_length / cir_limit * rolly_dia * pi;

}





int act_timer_reset_flag = 1;
int timer_reset_cnt  = 0;
int act_bag_num      = 12;
int read_info_switch = 0;

void SendCmd_to_Actuation(struct Actuation_speed act_speed)
{
	switch(act_timer_reset_flag)
	{
		// 发送速度控制指令
	  case 1:
			Actuation_Cmd_generation(0x01, act_speed.motor_1 );
		break;
		
		case 2:
			Actuation_Cmd_generation(0x02, act_speed.motor_2 );
		break;

		case 3:
			Actuation_Cmd_generation(0x03, act_speed.motor_3 );
		break;

		case 4:
			Actuation_Cmd_generation(0x04, act_speed.motor_4 );
		break;			
 	
		// 求电机扭矩数值   0x3C
		// 求电机编码器数值 0x38
	
		case 5:    
			Read_info_Actuation( 0x01, 0x3C );
		break;
		
		case 6:
			Read_info_Actuation( 0x02, 0x3C );
		break;

		case 7:
			Read_info_Actuation( 0x03, 0x3C );
		break;

		case 8:
			Read_info_Actuation( 0x04, 0x3C );
		break;		

//		case 5:    
//			if(read_info_switch == 0)
//				Read_info_Actuation( 0x01, 0x3C );
//			else
////				Read_info_Actuation( 0x01, 0x38 );
//		break;
//		
//		case 6:
//			if(read_info_switch == 0)
//				Read_info_Actuation( 0x02, 0x3C );
//			else
////				Read_info_Actuation( 0x02, 0x38 );
//		break;

//		case 7:
//			if(read_info_switch == 0)
//				Read_info_Actuation( 0x03, 0x3C );
//			else
////				Read_info_Actuation( 0x03, 0x38 );
//		break;

//		case 8:
//			if(read_info_switch == 0)
//				Read_info_Actuation( 0x04, 0x3C );
//			else
////				Read_info_Actuation( 0x04, 0x38 );
//		break;	

			

		case 9:    
			Read_info_Actuation( 0x01, 0x38 );
		break;
		
		case 10:
			Read_info_Actuation( 0x02, 0x38 );
		break;

		case 11:
			Read_info_Actuation( 0x03, 0x38 );
		break;

		case 12:
			Read_info_Actuation( 0x04, 0x38 );
		break;		
		


	}
}


unsigned char tx_buf_toActuation[10];

void Actuation_Cmd_generation(unsigned char id, int speed)
{
	unsigned char pack_1     = 0xFF;
	unsigned char pack_2     = 0xFF;

	unsigned char	length     = 0x05;
	unsigned char	instrution = 0x03;
	unsigned char	address    = 0x2E;
	unsigned char	low        = (int16_t)( speed )  ; 
	unsigned char	high       = (int16_t)( speed ) >> 8; 
	unsigned char	chechsum = ~(id + length + instrution + address + low + high);

	tx_buf_toActuation[0] = pack_1;
	tx_buf_toActuation[1] = pack_2;
	tx_buf_toActuation[2] = id;
	tx_buf_toActuation[3] = length;
	tx_buf_toActuation[4] = instrution;
	tx_buf_toActuation[5] = address;
	tx_buf_toActuation[6] = low;
	tx_buf_toActuation[7] = high;
	tx_buf_toActuation[8] = chechsum;
	
	for(int i = 0; i < 9 ; i++)
	{
		USART_SendData(USART3 , tx_buf_toActuation[i]);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC ) != SET);
	}
}

/////////////////////////////////////

unsigned char rx_buf_fromActuation[10];

void Read_info_Actuation(unsigned char id, unsigned char addr)
{
	unsigned char pack_1     = 0xFF;
	unsigned char pack_2     = 0xFF;

	unsigned char	length     = 0x04;
	unsigned char	instrution = 0x02;
	unsigned char	address    = addr; 
	unsigned char	bytes      = 0x02 ; 
	unsigned char	chechsum = ~(id + length + instrution + address + bytes);

	rx_buf_fromActuation[0] = pack_1;
	rx_buf_fromActuation[1] = pack_2;
	rx_buf_fromActuation[2] = id;
	rx_buf_fromActuation[3] = length;
	rx_buf_fromActuation[4] = instrution;
	rx_buf_fromActuation[5] = address;
	rx_buf_fromActuation[6] = bytes;
	rx_buf_fromActuation[7] = chechsum;
	
	for(int i = 0; i < 8 ; i++)
	{
		USART_SendData(USART3 , rx_buf_fromActuation[i]);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC ) != SET);
	}
	
}
