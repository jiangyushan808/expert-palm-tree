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

#include "stdio.h"
#include "string.h"
#include <JY61.h>

volatile struct SAcc 		stcAcc;
volatile struct SGyro 		stcGyro;
volatile struct SAngle 	stcAngle;
volatile uint8_t stcAngle_seq = 0; // sequence counter for atomic snapshot

float yaw_correction;  // 单位：° 计算出来的角度环
float current_yaw = 0.0f;    // 定义并初始化//当前角度，目标角度

//读取编码器计数值
int16_t    Motor1Speed;
int16_t    Motor2Speed;
int16_t    Motor3Speed;
int16_t    Motor4Speed;
//编码器每采样周期的脉冲数,单位 脉冲每采样周期
short	Encoder1_cnt ,Encoder2_cnt,Encoder3_cnt,Encoder4_cnt,Encoder_cnt,EncoderL_cnt,EncoderR_cnt; /*pwm给100 测得最大值89*/


float num = 1;		//num，圈数的意思，目标圈数
float rpm = 173.0f / 3.0f;		//RPM目标转速  从最大脉冲每采样周期转化过来最大转速173



float task_step = 0; //状态机
float angle_real = -1.2;
float Rpm_Num_L = 0,Rpm_Num_R = 0;//计算出的转数

uint8_t L_flag = 1;  //（当L_flag == 1时：用PID
uint8_t R_flag = 1;


/* 系统统一转换为脉冲数进行处理，脉冲数分辨率高，控制精度高 */
long Target_Velocity,Reality_Velocity;   /* 目标速度，实际速度 */
long Target_Position,Reality_Position;   /* 目标位置，实际位置 */

//编码器位置累加
long now_position3,now_position2,now_position1,now_position4,now_position_L,now_position_R,now_position;
 float position1_output =0;
  //PID计算出来的目标速度 单位脉冲每采样周期 最大值90
 float target_velocity_L1=0;
 float target_velocity_L2=0;
 float target_velocity_R1=0;
 float target_velocity_R2=0;
 //PID计算出来的PWM
 float speed_output_L1; 
 float speed_output_L2;
 float speed_output_R1;
 float speed_output_R2;
 



// 限制速度变化量的函数
/**
 * @brief 限制目标速度变化率（软启动用）
 * 
 * @param currentSpeed 当前速度值（上一周期控制器使用的目标速度）
 * @param targetSpeed  希望到达的最终目标速度（例如Rpm_Encoder_Cnt(rpm)）
 * @param maxChange    每次控制周期允许最大变化量（单位：脉冲/采样周期）
 * 
 * @return float 返回更新后的速度值（相对于currentSpeed逐步逼近targetSpeed）
 * 
 * @note
 * - 这个函数的作用是**让目标速度变化更平滑，防止系统瞬间跳变导致超调**；
 * - 如果目标速度变化太大，系统响应会过猛；加入这个函数可以避免这种情况；
 * - 应用于速度环时，对 `target_velocity` 做逐步限制提升系统稳定性。
 */
float limitSpeedChange(float currentSpeed, float targetSpeed, float maxChange) {
	
    float speedChange = targetSpeed - currentSpeed; // 计算当前需要变化的速度差值
    if (speedChange > maxChange) {
        speedChange = maxChange;   // 如果变化量超过正向最大值，则限制为 maxChange
    } else if (speedChange < -maxChange) {
        speedChange = -maxChange;    // 如果变化量超过负向最大值（即目标大幅降低），也限制
    }
    return currentSpeed + speedChange;  // 返回当前速度 + 限幅后的变化量（逐步逼近目标速度）
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

/**
 * @brief 判断当前 yaw 是否到达目标角度（处理环绕）
 * @param current 当前角度（0~360）
 * @param target 目标角度（0~360）
 * @param tolerance 容差角度（建议 2.0f）
 * @return 1 表示已到达目标角度，0 表示未到达
 */
int YawisReached(float current, float target, float tolerance)
{
    float diff = current - target;

    // 处理角度环绕（差值范围限制在 -180~180 之间）
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    if (fabs(diff) < tolerance)
        return 1;   // 已到达
    else
        return 0;   // 未到达
}


void UART_SendString(UART_HandleTypeDef *huart,  char*str) {
    uint16_t len = strlen(str);
    if (HAL_UART_Transmit(huart, (uint8_t *)str, len, HAL_MAX_DELAY) != HAL_OK) {
        
        Error_Handler();
    }
	

}

//void LED_REVERSE(void)
//{
//    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // 翻转 LED 电平
//}


//转码jy61p

void CopeSerial1Data(uint8_t ucData)
{
    static uint8_t buf[11];
    static int idx = 0;

    if(idx==0 && ucData!=0x55) return; // 帧头
    buf[idx++] = ucData;

    if(idx < 11) return; // 等待完整帧

    // 校验
    uint8_t sum = 0;
    for(int i=0;i<10;i++) sum+=buf[i];
    if(sum == buf[10] && buf[1]==0x53)
    {
        // write with seq guard: inc before and after write (odd during write)
        stcAngle_seq++;
        stcAngle.Angle[0] = buf[2] | (buf[3]<<8);
        stcAngle.Angle[1] = buf[4] | (buf[5]<<8);
        stcAngle.Angle[2] = buf[6] | (buf[7]<<8);
        stcAngle.T = buf[8] | (buf[9]<<8);
        stcAngle_seq++;
    }

    idx = 0;
}

/////******主控*****/

			
	//*串级PID*/		
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
	{
		 if (htim->Instance == TIM10)
  {
    HAL_IncTick();
  }
    if (htim == &htim1) { 
			
      //500ms翻转PC13电平,测试500ms，为了测试10ms外部中断，用示波器观测也是10ms一个下降沿
    static unsigned int timecnt;
   if(++timecnt >100)
    {
     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // 翻转 LED 电平
      timecnt=0;
    }
		
			// 计算期望位置、期望速度（状态机中给定，用于限幅）
			Target_Velocity = Rpm_Encoder_Cnt(rpm);		/* 将转速转化为10ms的脉冲数，目标速度 */
			Target_Position = Pulse_Encoder_Cnt(num);	/* 将圈数转化为目标脉冲数，目标位置 */
        
		
	
		
//	 
//	  
//	
		 static uint8_t need_reset = 0; // new：重置标志
     
        //new: 检查是否需要重置
        if(need_reset) {
            // 重置位置环内部状态
            Position_PID_Left(0, 0, 1);
            
		        Position_PID_Right(0 ,0 , 1);
		        
			
            // 重置速度环内部状态
             Incremental_PID_Left(0, 0, 1);
			       Incremental_PID_Right(0, 0, 1);		
            
					
					//角度环
					JY61P_Yaw_correct(0,0,1);
					yaw_correction =0;
					now_position_L =0;
					now_position_R =0;
            // 清除重置标志

            need_reset = 0;
        }
		 
				//0.更正角度
				 current_yaw=(float)stcAngle.Angle[2]/32768*180.0f;
//				float Target_yaw=JY61_GetYawCorrected(NO1_yaw);
//				yaw_correction =JY61P_Yaw_correct(cunrent_yaw,Target_yaw);
//				 current_yaw=30;
yaw_correction = JY61P_Yaw_correct(current_yaw, target_yaw, 0);

//				);
				yaw_correction =Xianfu (yaw_correction ,40);

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
        Encoder4_cnt = (short)Motor4Speed;//只有电机1.3是负的
				
				

			  
//        // 2. 更新实际位置

        now_position1 += Encoder1_cnt;
        now_position2 += Encoder2_cnt;
				now_position3 += Encoder3_cnt;
        now_position4 += Encoder4_cnt;
//        now_position = (now_position1+now_position2+now_position3+now_position4)//效果不好
          now_position_L = (now_position2+now_position4)/2;
					now_position_R = (now_position1+now_position3)/2;





	   //计算当前转数
	   Rpm_Num_L = Num_AllEncoder(now_position2);
     Rpm_Num_R = Num_AllEncoder(now_position1); 
	   
	   
        // 3. 停止条件判断
     if((labs(now_position_L - Target_Position) <20)&&(labs(now_position_R - Target_Position) <20)) //new*先判断是否到达目标，如果到达则停止并设置重置标志，否则进行正常的PID计算和输出。
	
        {
            Motor1_SetSpeed(0);
            Motor2_SetSpeed(0);
			      Motor3_SetSpeed(0);
 		        Motor4_SetSpeed(0);

           // 设置重置标志（下次中断时执行）
           need_reset = 1;
		
        }
        else {


//			//new;测只有速度环用的
//			  target_velocity_L1=Rpm_Encoder_Cnt(rpm);
//        target_velocity_L2=Rpm_Encoder_Cnt(rpm);
//        target_velocity_R1=Rpm_Encoder_Cnt(rpm);
//        target_velocity_R2=Rpm_Encoder_Cnt(rpm);
			
			// 4. 位置环计算目标速度
					
           target_velocity_L1 = Position_PID_Left(now_position_L, Target_Position,0);
           target_velocity_L2 = Position_PID_Left(now_position_L, Target_Position,0);
		       target_velocity_R1 = Position_PID_Right(now_position_R, Target_Position,0);
		       target_velocity_R2 = Position_PID_Right(now_position_R, Target_Position,0);

         		
					
           target_velocity_L1 = Xianfu(target_velocity_L1, Rpm_Encoder_Cnt(rpm));  /*位置环输出限幅；限幅在期望速度内*/
           target_velocity_L2 = Xianfu(target_velocity_L2, Rpm_Encoder_Cnt(rpm));
           target_velocity_R1= Xianfu(target_velocity_R1, Rpm_Encoder_Cnt(rpm));
		       target_velocity_R2= Xianfu(target_velocity_R2, Rpm_Encoder_Cnt(rpm));
			
		       target_velocity_L1 = Xianfu(target_velocity_L1, Rpm_Encoder_Cnt(Rpm_Max));  //限幅在最大转速内
           target_velocity_L2 = Xianfu(target_velocity_L2, Rpm_Encoder_Cnt(Rpm_Max));
           target_velocity_R1= Xianfu(target_velocity_R1, Rpm_Encoder_Cnt(Rpm_Max));
	         target_velocity_R2= Xianfu(target_velocity_R2, Rpm_Encoder_Cnt(Rpm_Max));



     static float prev_target_velocity_L1 = 0,prev_target_velocity_L2 = 0, prev_target_velocity_R1 = 0,prev_target_velocity_R2 = 0;

   
     target_velocity_L1 = limitSpeedChange(prev_target_velocity_L1, target_velocity_L1, 3);
		 target_velocity_L2 = limitSpeedChange(prev_target_velocity_L2, target_velocity_L1, 3);
     target_velocity_R1 = limitSpeedChange(prev_target_velocity_R1, target_velocity_R1, 3);
		 target_velocity_R2 = limitSpeedChange(prev_target_velocity_R2, target_velocity_R1, 3);
     prev_target_velocity_L1 = target_velocity_L1;
		 prev_target_velocity_L2 = target_velocity_L2;
     prev_target_velocity_R1 = target_velocity_R1;
		 prev_target_velocity_R2 = target_velocity_R2;
			
            // 5. 只有速度环计算PWM
//												      // test  PID调用
//        if (pid_initialized == 0) {
//            // 触发PID清零
//            speed_output_L1 = Incremental_PID_Left(Encoder2_cnt, target_velocity_L, 1);
//            speed_output_L2 = Incremental_PID_Left(Encoder4_cnt, target_velocity_L, 1);
//            speed_output_R1 = Incremental_PID_Right(Encoder1_cnt, target_velocity_R, 1);
//            speed_output_R2 = Incremental_PID_Right(Encoder3_cnt, target_velocity_R, 1);
//       

//            pid_initialized = 1; // 标记已初始化
//        } else {
//            // 正常PID计算
//            speed_output_L1 = Incremental_PID_Left(Encoder2_cnt, target_velocity_L, 0);
//            speed_output_L2 = Incremental_PID_Left(Encoder4_cnt, target_velocity_L, 0);
//            speed_output_R1 = Incremental_PID_Right(Encoder1_cnt, target_velocity_R, 0);
//            speed_output_R2 = Incremental_PID_Right(Encoder3_cnt, target_velocity_R, 0);


//					
//     }
//		
          speed_output_L1 = Incremental_PID_Left(Encoder2_cnt, target_velocity_L1,0);
          speed_output_L2 = Incremental_PID_Left(Encoder4_cnt, target_velocity_L2,0);
	    		speed_output_R1 = Incremental_PID_Right(Encoder1_cnt, target_velocity_R1,0);
					speed_output_R2 = Incremental_PID_Right(Encoder3_cnt, target_velocity_R2,0);

			    
          speed_output_L1 = Xianfu(speed_output_L1, PWM_MAX);  
		// 	  speed_output_L1 = Xianfu(speed_output_L1, Target_Velocity);
					speed_output_L2 = Xianfu(speed_output_L2, PWM_MAX);
		//	  speed_output_L2 = Xianfu(speed_output_L2, Target_Velocity);

			    speed_output_R1 = Xianfu(speed_output_R1, PWM_MAX);
      //   speed_output_R1 = Xianfu(speed_output_R1,Target_Velocity );
			    speed_output_R2 = Xianfu(speed_output_R2, PWM_MAX);
       //   speed_output_R2 = Xianfu(speed_output_R2,Target_Velocity );

             Motor1_SetSpeed(speed_output_R1+yaw_correction);
			       Motor2_SetSpeed(speed_output_L1-yaw_correction);
			       Motor3_SetSpeed(speed_output_R2+yaw_correction);
			       Motor4_SetSpeed(speed_output_L2-yaw_correction);

//           CAR_RouteControlTask();
}
				

//	 
//   Data_send
//	(Target_Position, 
//	now_position_L ,
//	now_position_R,
//	Motor1Speed		
//	);  

// Data_send
//	(
// Encoder1_cnt,
//  Encoder2_cnt, 
// Encoder3_cnt,
// target_velocity_L1
//	);
	 
  }
		
}