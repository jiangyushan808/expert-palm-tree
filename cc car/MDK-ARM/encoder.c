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
int16_t    Motor2Speed;
int16_t    Motor3Speed;
int16_t    Motor4Speed;
short	Encoder1_cnt ,Encoder2_cnt,Encoder3_cnt,Encoder4_cnt,Encoder_cnt;
char sendBuffer[100];
int t=1;

int num = 250;		//num��Ȧ������˼��Ŀ��Ȧ��
int rpm = 0;		//RPMĿ��ת��

float real_num = 0; //״̬��
 
/* ϵͳͳһת��Ϊ���������д����������ֱ��ʸߣ����ƾ��ȸ� */
long Target_Velocity=0,Reality_Velocity=0;   /* Ŀ���ٶȣ�ʵ���ٶ� */
long Target_Position=0,Reality_Position=0;   /* Ŀ��λ�ã�ʵ��λ�� */

//������λ���ۼ�
long now_position1=0;
 float position1_output =0;
  //PID���������Ŀ���ٶ�
 float target_velocity_L;
 float target_velocity_L2;
 float target_velocity_R;
 float target_velocity_R2;
 //PID���������PWM
 float speed_output_L=0;
 float speed_output_L2=0;
 float speed_output_R=0;
 float speed_output_R2=0;
 
 
 float Rpm_Num = 0;//�������ת��
 
 
 //float target_velocity ;//����test����100 
// long Target_Position=3120*2,Reality_Position=0;   /* test��Ŀ��λ�ã�ʵ��λ�� */
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

/**************************************************************************
 * @brief  ����ת�ٶ�Ӧ������������
 * @param  rpm��ת�٣�ת/min
 * @retval ���������
 * @attention 
 * @describe 	rpm/(60*1000)*SAMPLE_RATE( �������� ����ms��ȡһ�α�����) =  ��λ  תÿ��������
 * @matters
**************************************************************************/
//�������룺Ŀ��ת�� rpm���������
long Rpm_Encoder_Cnt(float rpm)
{
	long Ret_Value;
    Ret_Value = (rpm*3120)/(60*1000/10);            /* 4��Ƶ */  
	return Ret_Value;
}
//*�����ܱ���������ӦȦ��/ת��
float Num_AllEncoder(long encoder_num)
{
	float quanshu;
	quanshu = encoder_num /3120;
	
	return quanshu;
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
           Integral_bias_Left = 0;
		   Position_PID_Right(0 ,0 , 1);
		   Integral_bias_Right = 0;
			
            // �����ٶȻ��ڲ�״̬
             Incremental_PID_Left(0, 0, 1);
			 Incremental_PID_Right(0, 0, 1);		
            
            // ������ñ�־
            need_reset = 0;
        }
		   
        // 1. ��ȡ������
        Motor1Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
		Motor2Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        Motor3Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
		Motor4Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);				
		
	      __HAL_TIM_SET_COUNTER(&htim2,0);
	      __HAL_TIM_SET_COUNTER(&htim3,0);
	      __HAL_TIM_SET_COUNTER(&htim4,0);
	      __HAL_TIM_SET_COUNTER(&htim5,0);//clear num ���������
				
        Encoder1_cnt =-(short)Motor1Speed;
        Encoder2_cnt = (short)Motor2Speed;
        Encoder3_cnt = -(short)Motor3Speed;
        Encoder4_cnt = (short)Motor4Speed;
				
		Encoder_cnt=(Encoder1_cnt+Encoder2_cnt+Encoder3_cnt+Encoder4_cnt)/4;
        // 2. ����ʵ��λ��
        now_position1 += Encoder_cnt;
       
	   //���㵱ǰת��
	   Rpm_Num = Num_AllEncoder(now_position1);
       
	   
	   
        // 3. ֹͣ�����ж�
     if(labs(now_position1 - Target_Position) <100) //new*���ж��Ƿ񵽴�Ŀ�꣬���������ֹͣ���������ñ�־���������������PID����������
	
        {
            Motor1_SetSpeed(0);
            Motor2_SetSpeed(0);
			Motor3_SetSpeed(0);
 		    Motor4_SetSpeed(0);
           // �������ñ�־���´��ж�ʱִ�У�
            need_reset = 1;
           
			
        }
        else {
			// ��������λ�á������ٶȣ�״̬���и����������޷���
			Target_Velocity = Rpm_Encoder_Cnt(rpm);		/* ��ת��ת��Ϊ10ms����������Ŀ���ٶ� */
			Target_Position = Pulse_Encoder_Cnt(num);	/* ��Ȧ��ת��ΪĿ����������Ŀ��λ�� */
			
			// 4. λ�û�����Ŀ���ٶ�
           target_velocity_L = Position_PID_Left(now_position1, Target_Position,0);
           target_velocity_L2 = Position_PID_Left(now_position1, Target_Position,0);
		   target_velocity_R = Position_PID_Right(now_position1, Target_Position,0);
		   target_velocity_R2 = Position_PID_Right(now_position1, Target_Position,0);
		
         		
           target_velocity_L = Xianfu(target_velocity_L, Rpm_Encoder_Cnt(rpm));  /*λ�û�����޷����޷��������ٶ���*/
           target_velocity_L2 = Xianfu(target_velocity_L2, Rpm_Encoder_Cnt(rpm));
           target_velocity_R= Xianfu(target_velocity_R, Rpm_Encoder_Cnt(rpm));
		   target_velocity_R2= Xianfu(target_velocity_R2, Rpm_Encoder_Cnt(rpm));
			
		   target_velocity_L = Xianfu(target_velocity_L, Rpm_Encoder_Cnt(Rpm_Max));  //�޷������ת����
           target_velocity_L2 = Xianfu(target_velocity_L2, Rpm_Encoder_Cnt(Rpm_Max));
           target_velocity_R= Xianfu(target_velocity_R, Rpm_Encoder_Cnt(Rpm_Max));
	       target_velocity_R2= Xianfu(target_velocity_R2, Rpm_Encoder_Cnt(Rpm_Max));


			
            // 5. �ٶȻ�����PWM
            speed_output_L = Incremental_PID_Left(Encoder_cnt, target_velocity_L,0);
			speed_output_L2 = Incremental_PID_Left(Encoder_cnt, target_velocity_L2,0);
			speed_output_R = Incremental_PID_Right(Encoder_cnt, target_velocity_R,0);
			speed_output_R2 = Incremental_PID_Right(Encoder_cnt, target_velocity_R2,0);
			
            speed_output_L = Xianfu(speed_output_L, PWM_MAX);
            speed_output_L2 = Xianfu(speed_output_L2, PWM_MAX);
			speed_output_R = Xianfu(speed_output_R, PWM_MAX);
            speed_output_R2 = Xianfu(speed_output_R2, PWM_MAX);
			
			
			
             Motor1_SetSpeed(speed_output_L);
			 Motor2_SetSpeed(speed_output_R);
			 Motor3_SetSpeed(speed_output_L2);
			 Motor4_SetSpeed(speed_output_R2);
        }
				
    }
		 
Data_send(
  now_position1,
          Motor2Speed,
            labs(now_position1 - Target_Position), 
		Target_Position
   );
      
	
 
	 }
    
