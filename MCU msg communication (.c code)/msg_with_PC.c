#include "stm32f10x.h"
#include <stdio.h>
#include "USART1.h"
#include "USART3.h"
 

static unsigned char TxBuffer[256];
static unsigned char TxCounter=0;
static unsigned char count=0; 
//extern void CopeSerial1Data(unsigned char ucData);

struct cMotor Control_Motor;

void Initial_UART1(unsigned long baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); 
	
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);  
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);        
	USART_ClearFlag(USART1,USART_FLAG_TC);
	USART_Cmd(USART1, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void UART1_Put_Char(unsigned char DataToSend)
{
	TxBuffer[count++] = DataToSend;  
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  
}

void UART1_Put_String(unsigned char *Str)
{
	while(*Str)
	{
		if(*Str=='\r')UART1_Put_Char(0x0d);
			else if(*Str=='\n')UART1_Put_Char(0x0a);
				else UART1_Put_Char(*Str);
		Str++;
	}
}



//////////////////////////////////////////////////////////

extern struct Euler_angle_compute euler_ang_compute;
extern struct Circle_move circle_move;

unsigned char rxbuf[30] = {0x00};// receive data buff
int data_start = 0;
int buf_count = 0;
int rec_data_length = 10;
int rxbuf_rec_flag = 0;

int landing_ctrl = 0;
int gripper_ctrl = 0;


void USART1_IRQHandler(void)
{
		unsigned char data = 0x00 ;
	 
		if(USART_GetFlagStatus(USART1 , USART_IT_RXNE) == SET)
		{
			data = USART_ReceiveData(USART1);
			
			// detect the head of data package from PC
			if(data == 0x7E && data_start == 0)  
			{	
				data_start = 1;
				buf_count = 0;
				
				// clear all the rxbuf ！  
				for(int i = 0; i < 30; i++)
				{
					rxbuf[i] = 0x00;
				}
		
			} 

			// start to decode the data package
			if(data_start)
			{
				// storage data
				rxbuf[buf_count] = data;

				if(buf_count < rec_data_length)
				{
					// detect the end byte of data package 
					if(rxbuf[rec_data_length - 1] == 0x7F)
					{
						rxbuf_rec_flag = 1;
						
						euler_ang_compute.Eroll  			= (rxbuf[2] << 8) | rxbuf[1];
						euler_ang_compute.Epitch 			= (rxbuf[4] << 8) | rxbuf[3];
						gripper_ctrl				    		  = (rxbuf[6] << 8) | rxbuf[5];					
						landing_ctrl					 		    = (rxbuf[8] << 8) | rxbuf[7];
						
						if(euler_ang_compute.Eroll  > 30000)
							euler_ang_compute.Eroll  = euler_ang_compute.Eroll  - 65536;
						
						if(euler_ang_compute.Epitch > 30000)
							euler_ang_compute.Epitch = euler_ang_compute.Epitch - 65536;

						// clear the flag
						data_start = 0;
						buf_count = 0;

					}
					buf_count++;
				}
				else
				{
				 	// clear the flag
					data_start = 0;
					buf_count = 0;
				}
			}
			
	  }
		USART_ClearFlag(USART1, USART_IT_RXNE);	
}

extern struct Actutation_Torque act_Torque;
extern struct SGyro   stcGyro;
extern struct EndEffector EE_pos;
extern int manipulator_initial_flag ;
extern struct Actutation_Length act_Length;
extern struct Motor_Encoder    mot_Encoder;

unsigned char tx_buf_toROS[40] = {0x00};
int toROS_timer_flag = 0;

 
void SendMessage_to_Ros(void)
{
 		
	int roll_send 		 = euler_ang_compute.Croll  * 100;
	int pitch_send 		 = euler_ang_compute.Cpitch * 100;
	int yaw_send   		 = euler_ang_compute.Cyaw   * 100;

	int EE_X_send 	   = EE_pos.EE_X   * 100;
	int EE_Y_send  		 = EE_pos.EE_Y   * 100;
	int EE_Z_send      = EE_pos.EE_Z   * 100;

	int Torque_1_send  = act_Torque.Torque_1  ;
	int Torque_2_send  = act_Torque.Torque_2  ;
	int Torque_3_send  = act_Torque.Torque_3  ;
	int Torque_4_send  = act_Torque.Torque_4  ;

	int gyro_roll      = stcGyro.w[0];
	int gyro_pitch     = stcGyro.w[1];
	
	int Length_1_send  = mot_Encoder.Encoder_1  ;
	int Length_2_send  = mot_Encoder.Encoder_2  ;
	int Length_3_send  = mot_Encoder.Encoder_3  ;
	int Length_4_send  = mot_Encoder.Encoder_4  ;

	//////////////////////////////////////////////////////

	tx_buf_toROS[0]  = 0x7E & 0xff; 

	//imu euler angle：
	tx_buf_toROS[1] =  roll_send & 0xff ; 
	tx_buf_toROS[2] = (roll_send >> 8) & 0xff; 
	tx_buf_toROS[3] =  pitch_send & 0xff ; 
	tx_buf_toROS[4] = (pitch_send >> 8) & 0xff; 
	tx_buf_toROS[5] =  yaw_send & 0xff; 
	tx_buf_toROS[6] = (yaw_send >> 8) & 0xff; 

	//EE POS":
	tx_buf_toROS[7]  =  EE_X_send & 0xff ; 
	tx_buf_toROS[8]  = (EE_X_send >> 8) & 0xff; 
	tx_buf_toROS[9]  =  EE_Y_send & 0xff ; 
	tx_buf_toROS[10] = (EE_Y_send >> 8) & 0xff; 
	tx_buf_toROS[11] =  EE_Z_send & 0xff; 
	tx_buf_toROS[12] = (EE_Z_send >> 8) & 0xff; 
	
	//Acutator Torque:
	tx_buf_toROS[13] =  Torque_1_send & 0xff; 
	tx_buf_toROS[14] = (Torque_1_send >> 8) & 0xff; 
	tx_buf_toROS[15] =  Torque_2_send & 0xff; 
	tx_buf_toROS[16] = (Torque_2_send >> 8) & 0xff; 
	tx_buf_toROS[17] =  Torque_3_send & 0xff; 
	tx_buf_toROS[18] = (Torque_3_send >> 8) & 0xff; 
	tx_buf_toROS[19] =  Torque_4_send & 0xff; 
	tx_buf_toROS[20] = (Torque_4_send >> 8) & 0xff; 

	// imu gyro rate:
	tx_buf_toROS[21] =  gyro_roll & 0xff; 
	tx_buf_toROS[22] = (gyro_roll >> 8) & 0xff; 
	tx_buf_toROS[23] =  gyro_pitch & 0xff; 
	tx_buf_toROS[24] = (gyro_pitch >> 8) & 0xff; 

	// robot arm working:
	tx_buf_toROS[25] =  manipulator_initial_flag & 0xff; 
	tx_buf_toROS[26] = (manipulator_initial_flag >> 8) & 0xff; 

	// Acutator Length:
	tx_buf_toROS[27] =  Length_1_send & 0xff; 
	tx_buf_toROS[28] = (Length_1_send >> 8) & 0xff; 
	tx_buf_toROS[29] =  Length_2_send & 0xff; 
	tx_buf_toROS[30] = (Length_2_send >> 8) & 0xff; 
	tx_buf_toROS[31] =  Length_3_send & 0xff; 
	tx_buf_toROS[32] = (Length_3_send >> 8) & 0xff; 
	tx_buf_toROS[33] =  Length_4_send & 0xff; 
	tx_buf_toROS[34] = (Length_4_send >> 8) & 0xff; 
	
	// check sum
	unsigned char checksum = 0x00;
	for(int i = 1; i <= 34 ; i++)   /
	{
		checksum += tx_buf_toROS[i];
	}
	tx_buf_toROS[35] = ( ~checksum ) & 0xff;  // +1
												
	tx_buf_toROS[36] = 0x7F & 0xff;           // more +1

	for(int i = 0; i <= 36 ; i++)   // 
	{
		USART_SendData(USART1 , tx_buf_toROS[i]);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC ) != SET);
	}

}



