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
#include <JY61.h>


//读取编码器计数值
int16_t    Motor1Speed;
int16_t    Motor2Speed;
int16_t    Motor3Speed;
int16_t    Motor4Speed;
//编码器每采样周期的脉冲数,单位 脉冲每采样周期
short	Encoder1_cnt ,Encoder2_cnt,Encoder3_cnt,Encoder4_cnt,Encoder_cnt,EncoderL_cnt,EncoderR_cnt; /*pwm给1000 测得最大值90*/
int t=1;
float i;

int num = 250;		//num，圈数的意思，目标圈数
int rpm = 30;		//RPM目标转速  从最大脉冲每采样周期转化过来最大转速173

float real_num = 0; //状态机
float angle_real = -1.2;
float Rpm_Num = 0;//计算出的转数

uint8_t L_flag = 1;  //（当L_flag == 1时：用PID
uint8_t R_flag = 1;


/* 系统统一转换为脉冲数进行处理，脉冲数分辨率高，控制精度高 */
long Target_Velocity=0,Reality_Velocity=0;   /* 目标速度，实际速度 */
long Target_Position=0,Reality_Position=0;   /* 目标位置，实际位置 */

//编码器位置累加
long now_position1=0;
 float position1_output =0;
  //PID计算出来的目标速度 单位脉冲每采样周期 最大值90
 float target_velocity_L=400;
 float target_velocity_L2=600;
 float target_velocity_R=400;
 float target_velocity_R2=600;
 //PID计算出来的PWM
 float speed_output_L;
 float speed_output_L2;
 float speed_output_R;
 float speed_output_R2;
 

 
 //float target_velocity ;//用于test满速100 
// long Target_Position=3120*2,Reality_Position=0;   /* test用目标位置，实际位置 */

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


/**************************************************************************
 * @brief  计算转速对应编码器脉冲数
 * @param  rpm：转速；转/min
 * @retval 电机脉冲数
 * @attention  3120是总脉冲数/转
 * @describe 	rpm/(60*1000)*SAMPLE_RATE( 采样周期 多少ms读取一次编码器) =  单位  转每采样周期
 * @matters
**************************************************************************/
//计算输入：目标转速 rpm输出脉冲数
long Rpm_Encoder_Cnt(float rpm)
{
	long Ret_Value;
    Ret_Value = (rpm*3120)/(60*1000/10);            /* 4倍频 */  
	return Ret_Value;
}
/**************************************************************************
 * @brief    将编码器每采样周期的脉冲数转换为电机转速（RPM）
 * @param    pulse：编码器在一个采样周期内的脉冲数（单位：脉冲/采样周期）
 * @retval   转速（单位：RPM，转/分钟）
 * @attention 
 *           适用于定时周期为10ms的速度采样场景
 *           其中3120为编码器每圈总脉冲数（含减速比和倍频计算）
 * @describe 
 *           公式推导：
 *               每10ms采样一次 -> 每秒100次采样
 *               每秒脉冲数 = pulse × 100
 *               每分钟脉冲数 = pulse × 100 × 60
 *               RPM = 每分钟脉冲数 / 编码器每圈脉冲数
 *               即 RPM = pulse × 100 × 60 / 3120
 *           简化后变为：pulse × 1000 × 60 / 3120 / 10
 * @matters  
 *           请确保 pulse 是在10ms内采集的编码器增量脉冲数
**************************************************************************/

float EncoderCnt_Rpm(int pulse)
{
    return (float)pulse * (60.0f * 1000 / 10) / 3120;
}

//·*计算总编码器数对应圈数/转数
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

///******主控*****/
//void Use_Pid_Tocontrol()
//{
//	static int a = 0;
//	if(a<=300 && real_num == 0) //前 300ms 电机停止
//	{
//		 a++;
//		 L_flag=0;		R_flag=0;  //初始化时设为0，停止左右电机PID控制
//         Motor1_SetSpeed(0);   
//		 Motor2_SetSpeed(0);
//		 Motor3_SetSpeed(0);
//		 Motor4_SetSpeed(0);
//		return;
//	}
//    else if(real_num == 0 && a>300)
//	{
//		rpm=86;                  //设置起始速度
//		L_flag=1;		R_flag=1;//启用 PID
//		real_num = 1;  // 第一段直线
//	}
////   if(JY61_GetPitch <=-10)

//}	
/*串级PID*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
	{
    if (htim == &htim1) { 
		

		static unsigned int timecnt;
      if (++timecnt >= 100)	
	  {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);	//500ms翻转PC13电平,测试500ms，为了测试10ms外部中断，用示波器观测也是10ms一个下降沿
		timecnt = 0;
	  }
	
		 static uint8_t need_reset = 0; // new：重置标志
          
        
        //new: 检查是否需要重置
//        if(need_reset) {
//            // 重置位置环内部状态
//           Position_PID_Left(0, 0, 1);
//           Integral_bias_Left = 0;
//		   Position_PID_Right(0 ,0 , 1);
//		   Integral_bias_Right = 0;
//			
//            // 重置速度环内部状态
//             Incremental_PID_Left(0, 0, 1);
//			 Incremental_PID_Right(0, 0, 1);		
//            
//            // 清除重置标志

////            need_reset = 0;
////        }
////		 
//		
        // 1. 读取编码器
        Motor1Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
		Motor2Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        Motor3Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
		Motor4Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);				
		
	      __HAL_TIM_SET_COUNTER(&htim2,0);
	      __HAL_TIM_SET_COUNTER(&htim3,0);
	      __HAL_TIM_SET_COUNTER(&htim4,0);
	      __HAL_TIM_SET_COUNTER(&htim5,0);//clear num 清零编码器
				
        Encoder1_cnt =-(short)Motor1Speed;
        Encoder2_cnt = (short)Motor2Speed;
        Encoder3_cnt = -(short)Motor3Speed;
        Encoder4_cnt = (short)Motor4Speed;
				
		    EncoderR_cnt=(Encoder1_cnt+Encoder3_cnt)/2;
			  EncoderL_cnt=(Encoder2_cnt+Encoder4_cnt)/2;	
        // 2. 更新实际位置
     //   now_position1 += Encoder_cnt;
//      now_position2 += Encoder2_cnt;
//      now_position3 += Encoder3_cnt;
//      now_position4 += Encoder4_cnt;
	   //计算当前转数
//	   Rpm_Num = Num_AllEncoder(now_position1);
       
	   
	   
//        // 3. 停止条件判断
//     if(labs(now_position1 - Target_Position) <100) //new*先判断是否到达目标，如果到达则停止并设置重置标志，否则进行正常的PID计算和输出。
//	
//        {
//            Motor1_SetSpeed(0);
//            Motor2_SetSpeed(0);
//			Motor3_SetSpeed(0);
// 		    Motor4_SetSpeed(0);

//           // 设置重置标志（下次中断时执行）
//            need_reset = 1;
//		
//        }
//        else {

//			// 计算期望位置、期望速度（状态机中给定，用于限幅）
//			Target_Velocity = Rpm_Encoder_Cnt(rpm);		/* 将转速转化为10ms的脉冲数，目标速度 */
//			Target_Position = Pulse_Encoder_Cnt(num);	/* 将圈数转化为目标脉冲数，目标位置 */
//			
			//new;测速度环用的
			  target_velocity_L=Rpm_Encoder_Cnt(rpm);
        target_velocity_L2=Rpm_Encoder_Cnt(rpm);
        target_velocity_R=Rpm_Encoder_Cnt(rpm);
        target_velocity_R2=Rpm_Encoder_Cnt(rpm);
			
//			// 4. 位置环计算目标速度
//           target_velocity_L = Position_PID_Left(now_position1, Target_Position,0);
//           target_velocity_L2 = Position_PID_Left(now_position1, Target_Position,0);
//		   target_velocity_R = Position_PID_Right(now_position1, Target_Position,0);
//		   target_velocity_R2 = Position_PID_Right(now_position1, Target_Position,0);
//		
//         		
//           target_velocity_L = Xianfu(target_velocity_L, Rpm_Encoder_Cnt(rpm));  /*位置环输出限幅；限幅在期望速度内*/
//           target_velocity_L2 = Xianfu(target_velocity_L2, Rpm_Encoder_Cnt(rpm));
//           target_velocity_R= Xianfu(target_velocity_R, Rpm_Encoder_Cnt(rpm));
//		   target_velocity_R2= Xianfu(target_velocity_R2, Rpm_Encoder_Cnt(rpm));
//			
//		   target_velocity_L = Xianfu(target_velocity_L, Rpm_Encoder_Cnt(Rpm_Max));  //限幅在最大转速内
//           target_velocity_L2 = Xianfu(target_velocity_L2, Rpm_Encoder_Cnt(Rpm_Max));
//           target_velocity_R= Xianfu(target_velocity_R, Rpm_Encoder_Cnt(Rpm_Max));
//	       target_velocity_R2= Xianfu(target_velocity_R2, Rpm_Encoder_Cnt(Rpm_Max));


//			
//            // 5. 速度环计算PWM
//            speed_output_L = Incremental_PID_Left(EncoderL_cnt, target_velocity_L,0);
////			speed_output_L2 = Incremental_PID_Left(Encoder4_cnt, target_velocity_L2,0);
//			speed_output_R = Incremental_PID_Right(EncoderR_cnt, target_velocity_R,0);
////			speed_output_R2 = Incremental_PID_Right(Encoder_cnt, target_velocity_R2,0);
//			
//           speed_output_L = Xianfu(speed_output_L, PWM_MAX);
//////						 speed_output_L = Xianfu(speed_output_L, target_velocity_L);
//////            speed_output_L2 = Xianfu(speed_output_L2, PWM_MAX);
//			speed_output_R = Xianfu(speed_output_R, PWM_MAX);
////            speed_output_R2 = Xianfu(speed_output_R2, PWM_MAX);
//			

             Motor1_SetSpeed(speed_output_R);
			 Motor2_SetSpeed(speed_output_L);
			Motor3_SetSpeed(speed_output_R);
			 Motor4_SetSpeed(speed_output_L);
//       i = EncoderCnt_Rpm(Encoder2_cnt); //测最大速用的
        }
				
    
		 
Data_send
	(target_velocity_L , 
	speed_output_L, 
	Encoder2_cnt, 
	i
	);     	
 
	 

}

 