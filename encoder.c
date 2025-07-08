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
long target_velocity =80;
long Target_Position;
 
//编码器位置累加
short now_position1=0;
 float position1_output =0;
 float speed_output=0;
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
     return (rpm*3120)/(60*1000/10);            /* 4倍频 */       
}


void UART_SendString(UART_HandleTypeDef *huart,  char*str) {
    uint16_t len = strlen(str);
    if (HAL_UART_Transmit(huart, (uint8_t *)str, len, HAL_MAX_DELAY) != HAL_OK) {
        
        Error_Handler();
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim1) { 
//			Target_Position = Pulse_Encoder_Cnt(num);
//			 target_velocity =  Rpm_Encoder_Cnt(rpm);
//        // 1. 读取编码器
        Motor1Speed = -(int16_t)__HAL_TIM_GET_COUNTER(&htim2);
			 
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        Encoder_cnt = (short)Motor1Speed;
//        
//        // 2. 更新实际位置
//        now_position1 += Encoder_cnt;
//        

//			// 3. 位置环计算目标速度
//        float target_velocity = Position_PID_Left(now_position1, Target_Position);
//        target_velocity = Xianfu(target_velocity, Rpm_Encoder_Cnt(Rpm_Max));
//       
//        // 4. 停止条件判断
//        if(labs(now_position1 - Target_Position) < 100 ) 
//        {
//            Motor1_SetSpeed(0);
//					Bias=0;
//            // 重置积分项
//            Integral_bias_Left = 0;
//            Bias_Left = 0;
//            Last_bias_Left = 0;
//            Prev_bias_Left = 0; 
//        }
//        else {
            // 5. 速度环计算PWM
            speed_output = Incremental_PID_Left(Encoder_cnt, target_velocity);
           speed_output = Xianfu(speed_output, PWM_MAX);
            Motor1_SetSpeed(speed_output);
        
				//}
    }
		 //Rpm_Encoder_Cnt
Data_send(
   Last_Bias,
		Motor1Speed,
  speed_output,      
    Integral_bias_Left    
   );
      
	
 
	 }
    

	
