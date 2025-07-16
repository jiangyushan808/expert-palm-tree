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
//读取编码器计数值
int16_t    Motor1Speed;
short	Encoder_cnt ;
char sendBuffer[100];
int t=1;
 float target_velocity ;//满速100
//编码器位置累加
long now_position1=0;
 float position1_output =0;
 float speed_output=0;
 
 long Target_Position=3120*5,Reality_Position=0;   /* 目标位置，实际位置 */
// 限制速度变化量的函数
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
 * @brief  计算转数对应编码器脉冲数
 * @param  num：转数(float类型)
 * @retval 电机脉冲数，long类型
 * @attention 用来做位置控制
 * @describe 脉冲数 = 转的圈数 * 减速比30 * 500线 * 四倍频  (一圈60000脉冲)
 * @matters
**************************************************************************/
long Pulse_Encoder_Cnt(float num)
{
	long retuen_Value;
	retuen_Value = (num*3120);
    return retuen_Value;     /* 4倍频 */       
}
//计算输入：目标转速 rpm输出脉冲数
long Rpm_Encoder_Cnt(float rpm)
{
    return (rpm*3120)/(60*1000/100);            /* 4倍频 */       
}


void UART_SendString(UART_HandleTypeDef *huart,  char*str) {
    uint16_t len = strlen(str);
    if (HAL_UART_Transmit(huart, (uint8_t *)str, len, HAL_MAX_DELAY) != HAL_OK) {
        
        Error_Handler();
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim1) { 
		

		  static uint8_t need_reset = 0; // new：重置标志
        
        
        //new: 检查是否需要重置
        if(need_reset) {
            // 重置位置环内部状态
            Position_PID_Left(0, 0, 1); 
            
            // 重置速度环内部状态
            Incremental_PID_Left(0, 0, 1); 
            
            // 重置硬件编码器
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            
        
            // 重置全局积分项
            Integral_bias_Left = 0;
            
            // 清除重置标志
            need_reset = 0;
        }
		   
        // 1. 读取编码器
        Motor1Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
			 
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        Encoder_cnt = -(short)Motor1Speed;
        
        // 2. 更新实际位置
        now_position1 += Encoder_cnt;
        
//new*先判断是否到达目标，如果到达则停止并设置重置标志，否则进行正常的PID计算和输出。
			
       
        // 3. 停止条件判断
     if(labs(now_position1 - Target_Position) < 100) 
        {
            Motor1_SetSpeed(0);
            
           // 设置重置标志（下次中断时执行）
            need_reset = 1;
           
			
        }
        else {
			// 4. 位置环计算目标速度
        float target_velocity = Position_PID_Left(now_position1, Target_Position,0);
        target_velocity = Xianfu(target_velocity, Rpm_Encoder_Cnt(Rpm_Max));
			
            // 5. 速度环计算PWM
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
    

	
