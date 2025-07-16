#include <main.h>
#include <pid.h>
#include <encoder.h>
#include <tim.h>
#include <stdio.h>
#include <string.h>
#include "math.h"
#include <motor.h>
#include <stdlib.h>
#include <AUNO.h> 
#include <usart.h> 
//��ȡ����������ֵ
int16_t    Motor1Speed;
short	Encoder_cnt ;
char sendBuffer[100];
int t=1;
 float target_velocity ;//����100
//������λ���ۼ�
long now_position1=0;
 float position1_output =0;
 float speed_output=0;
 
 long Target_Position=3120*5,Reality_Position=0;   /* Ŀ��λ�ã�ʵ��λ�� */
// �����ٶȱ仯���ĺ���
float limitSpeedChange(float currentSpeed, float targetSpeed, float maxChange) {
    float speedChange = targetSpeed - currentSpeed;
    if (speedChange > maxChange) {
        speedChange = maxChange;
    } else if (speedChange < -maxChange) {
        speedChange = -maxChange;
    }
    return currentSpeed + speedChange;
}
int Xianfu(int data,int max)
{	
    if(data<-max) data=-max;	
    if(data> max) data= max;	
    return data;
}

/**************************************************************************
 * @brief  ����ת����Ӧ������������
 * @param  num��ת��(float����)
 * @retval �����������long����
 * @attention ������λ�ÿ���
 * @describe ������ = ת��Ȧ�� * ���ٱ�30 * 500�� * �ı�Ƶ  (һȦ60000����)
 * @matters
**************************************************************************/
long Pulse_Encoder_Cnt(float num)
{
	long retuen_Value;
	retuen_Value = (num*3120);
    return retuen_Value;     /* 4��Ƶ */       
}
//�������룺Ŀ��ת�� rpm���������
long Rpm_Encoder_Cnt(float rpm)
{
    return (rpm*3120)/(60*1000/100);            /* 4��Ƶ */       
}


void UART_SendString(UART_HandleTypeDef *huart,  char*str) {
    uint16_t len = strlen(str);
    if (HAL_UART_Transmit(huart, (uint8_t *)str, len, HAL_MAX_DELAY) != HAL_OK) {
        
        Error_Handler();
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim1) { 
		

		  static uint8_t need_reset = 0; // new�����ñ�־
        
        
        //new: ����Ƿ���Ҫ����
        if(need_reset) {
            // ����λ�û��ڲ�״̬
            Position_PID_Left(0, 0, 1); 
            
            // �����ٶȻ��ڲ�״̬
            Incremental_PID_Left(0, 0, 1); 
            
            // ����Ӳ��������
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            
        
            // ����ȫ�ֻ�����
            Integral_bias_Left = 0;
            
            // ������ñ�־
            need_reset = 0;
        }
		   
        // 1. ��ȡ������
        Motor1Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
			 
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        Encoder_cnt = -(short)Motor1Speed;
        
        // 2. ����ʵ��λ��
        now_position1 += Encoder_cnt;
        
//new*���ж��Ƿ񵽴�Ŀ�꣬���������ֹͣ���������ñ�־���������������PID����������
			
       
        // 3. ֹͣ�����ж�
     if(labs(now_position1 - Target_Position) < 100) 
        {
            Motor1_SetSpeed(0);
            
           // �������ñ�־���´��ж�ʱִ�У�
            need_reset = 1;
           
			
        }
        else {
			// 4. λ�û�����Ŀ���ٶ�
        float target_velocity = Position_PID_Left(now_position1, Target_Position,0);
        target_velocity = Xianfu(target_velocity, Rpm_Encoder_Cnt(Rpm_Max));
			
            // 5. �ٶȻ�����PWM
            speed_output = Incremental_PID_Left(Encoder_cnt, target_velocity,0);
            speed_output = Xianfu(speed_output, PWM_MAX);
            Motor1_SetSpeed(speed_output);
        }
				
    }
		 
Data_send(
  now_position1,
            Target_Position ,
            Encoder_cnt, 
		target_velocity 
   );
      
	
 
	 }
    

	
