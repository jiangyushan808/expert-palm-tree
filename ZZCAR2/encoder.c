#include "encoder.h"
#include "stm32f4xx_hal.h"    
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "pid.h"
#include "motor.h"
#include "math.h"
#include <stdlib.h>
#include "niming.h"

//#include"../oled/oled.h"


extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
extern tPid pidMotor3Speed;
extern tPid pidMotor4Speed;
  
extern tPid pidPosition1;
extern tPid pidPosition2;
extern tPid pidPosition3;
extern tPid pidPosition4;
  short now_position1;
	  short now_position11;
	  short now_position22;
		  short now_position33;
			  short now_position44;
   float A1,A2,A3,A4;
	float C1,C2,C3,C4;
 
   float p_term;
//	 float maxSpeed=8;
//	 float minSpeed=-8;
	 // 新增：允许的最大速度变化量
float maxSpeedChange = 100.0; 
// 新增：存储上一次的电机速度
float lastMotor1Speed = 0.0; 

  char sendBuffer[100];
///**
// * @function: void GET_NUM(void)
// * @description: ??STM32?????,???????????
// * @param {*} 
// * @return {*}
// */


/**
 * @function:void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 * @description: ?????????,0.1S?????????,???????????????????OLED??
 * @param {TIM_HandleTypeDef *htim} 
 * @return {*}
// */



uint16_t TimerCount =0;
short Encoder1Count = 0;//encoder shu de0 num
short Encoder2Count = 0;
short Encoder3Count = 0;
short Encoder4Count = 0;

float Motor1Speed = 0.00;
float Motor2Speed = 0.00;
float Motor3Speed = 0.00;
float Motor4Speed = 0.00;

float  Motor1pwm;
float  Motor2pwm;
float  Motor3pwm;
float  Motor4pwm;
//int Motor1pwm;

float A;
float limitSpeed(float speed, float maxSpeed, float  minSpeed) {
    if (speed > maxSpeed) {
        return maxSpeed;
    }
    if (speed < minSpeed) {
        return minSpeed;
    }
    return speed;
}
void updateusart(void)
{
	 Encoder1Count =(short)__HAL_TIM_GET_COUNTER(&htim2);
	Encoder2Count =(short)__HAL_TIM_GET_COUNTER(&htim3);
	Encoder3Count =(short)__HAL_TIM_GET_COUNTER(&htim4);
	Encoder4Count =(short)__HAL_TIM_GET_COUNTER(&htim5);//save num speed
	
	
	__HAL_TIM_SET_COUNTER(&htim2,0);
	__HAL_TIM_SET_COUNTER(&htim3,0);
	__HAL_TIM_SET_COUNTER(&htim4,0);
	__HAL_TIM_SET_COUNTER(&htim5,0);//clear num
//	Motor1Speed = (float)Encoder1Count*100/3120;//  60/13/4
//	Motor2Speed = (float)Encoder2Count*100/3120;//speed=quan num/tim   zui hou shi 1s turn ji quan
//	Motor3Speed = (float)Encoder3Count*100/3120;
//	Motor4Speed = -(float)Encoder4Count*100/3120;
		Motor1Speed = (float)Encoder1Count;//  60/13/4
	Motor2Speed = (float)Encoder2Count;//speed=quan num/tim   zui hou shi 1s turn ji quan
	Motor3Speed = (float)Encoder3Count;
	Motor4Speed = -(float)Encoder4Count;
	now_position11+=Encoder1Count;
	now_position22+=Encoder2Count;
	now_position33+=Encoder3Count;
	now_position44+=Encoder4Count;
	 // now_position1=(now_position11+now_position22+now_position33+now_position44)/4;
//	Motor1Speed = limitSpeed(pidMotor1Speed.target_val, maxSpeed,  minSpeed);
  
//    Motor3Speed = limitSpeed(pidMotor1Speed.target_val, maxSpeed,  minSpeed);
//    Motor4Speed = limitSpeed(pidMotor1Speed.target_val, maxSpeed,  minSpeed);
	
}

void UART_SendString(UART_HandleTypeDef *huart,  char*str) {
    uint16_t len = strlen(str);
    if (HAL_UART_Transmit(huart, (uint8_t *)str, len, HAL_MAX_DELAY) != HAL_OK) {
        // ??????
        Error_Handler();
    }
}
// 封装串级PID控制逻辑为函数
//void ccPID(int target_position, short now_position1,short Encoder1Count ) {
//    

//        // 位置环计算
//        int Motor1pwm = PID_realize(&pidPosition1, now_position1);
//    		Motor2pwm=PID_realize(&pidPosition2, now_position1);
//    		Motor3pwm=PID_realize(&pidPosition3, now_position1);
//    		Motor4pwm=PID_realize(&pidPosition4, now_position1);
//       // int C = Motor1pwm * 100 / 3120;
//			C1=Motor1pwm*100/3120;
//		C2=Motor2pwm*100/3120;
//		C3=Motor3pwm*100/3120;
//		C4=Motor4pwm*100/3120;
//        
////      		C1= limitSpeed(Motor1pwm*100/3120, maxSpeed, minSpeed);
////				C2= limitSpeed(Motor2pwm*100/3120, maxSpeed, minSpeed);
////				C3= limitSpeed(Motor3pwm*100/3120, maxSpeed, minSpeed);
////				C4= limitSpeed(Motor4pwm*100/3120, maxSpeed, minSpeed);
//		if (pidPosition1.target_val-now_position1<50)
//		{			

//				pidMotor1Speed.target_val=0;
//			 Motor1_SetSpeed(PID_realize(&pidPosition1, Motor1Speed));
//			pidMotor2Speed.target_val=0;
//			 Motor2_SetSpeed(PID_realize(&pidPosition2, Motor2Speed));
//			pidMotor3Speed.target_val=0;
//			 Motor3_SetSpeed(PID_realize(&pidPosition3, Motor3Speed));
//			pidMotor4Speed.target_val=0;
//			 Motor4_SetSpeed(PID_realize(&pidPosition4, Motor4Speed));
//		}
//			else
//		{
//			pidMotor1Speed.target_val=	C1;
//			pidMotor2Speed.target_val=	C2;
//			pidMotor3Speed.target_val=	C3;
//			pidMotor4Speed.target_val=	C4;
//			A1=PID_realize(&pidMotor1Speed, Motor1Speed);//内环输入速度
//			A2=PID_realize(&pidMotor2Speed, Motor2Speed);
//			A3=PID_realize(&pidMotor3Speed, Motor3Speed);
//			A4=PID_realize(&pidMotor4Speed, Motor4Speed);
//            // 限制速度变化量
//          //  A = limitSpeedChange(lastMotor1Speed, A, maxSpeedChange); 
//			 Motor1_SetSpeed(A1);
//			 Motor2_SetSpeed(A2);
//			 Motor3_SetSpeed(A3);
//			 Motor4_SetSpeed(A4);
//		}
//		
//		
//}
//		
//typedef enum {
//	  ready,
//	  ready1,
//	ready2,
//	ready3,
//	ready4,
//	ready5,
//	ready6,
//    FORWARD_10,
//    TURN_LEFT_90_DEGREES_1,
//    FORWARD_5_CIRCLES,
//    TURN_LEFT_90_DEGREES_2,
//    FORWARD_15_CIRCLES,
//    RIGHT_90_DEGREES_1,
//    FORWARD_6_CIRCLES,
//    TURN_RIGHT_90_DEGREES_2,
//    FORWARD_20_CIRCLES,
////TURN_RIGHT_90_DEGREES_2
//    COMPLETE,
//} current_step;
// 
//	current_step current_state = FORWARD_10;

//void movePath() 
//	{
//    switch (current_state) {
//        case FORWARD_10:
//            // 前进 10 圈
//        			 ccPID(10 * 3120, 0,0);
//				
//                 if (fabs(pidPosition1.target_val - now_position1) < 50) {
//        current_state = TURN_LEFT_90_DEGREES_1;
//    }
//            break;
//        
//        case TURN_LEFT_90_DEGREES_1:
//            // 左转 90 度
//           pidPosition1.target_val += 3120; // 假设左转 90 度需要的脉冲数
//				 now_position1+=Encoder1Count;
//				if(  pidPosition1.target_val <	 now_position1)
//				{
//					//外侧轮子  = (L / 2D) * 3120
//				 pidMotor1Speed.target_val=-5;
//				 pidMotor2Speed.target_val=5;
//				 pidMotor3Speed.target_val=-5;
//				 pidMotor4Speed.target_val=5;
//				 Motor1_SetSpeed(PID_realize(&pidMotor1Speed, Motor1Speed));
//         Motor2_SetSpeed(PID_realize(&pidMotor2Speed, Motor2Speed));
//				 Motor3_SetSpeed(PID_realize(&pidMotor3Speed, Motor3Speed));
//         Motor4_SetSpeed(PID_realize(&pidMotor4Speed, Motor4Speed));
//									current_state = ready;
//     	}

//            break;
//        case ready:
//            // 检查是否完成左转
//            if (fabs(pidPosition1.target_val - now_position1) < 50) {
//                pidPosition1.target_val = now_position1; // 停止转弯
//               	current_state =  FORWARD_5_CIRCLES;
//            }
//					
//            break;
//        case   FORWARD_5_CIRCLES:
//            // 前进 5 圈
//            ccPID(5 * 3120, 0,0);// 5 圈的脉冲数
//            current_state = TURN_LEFT_90_DEGREES_2;
//              break;
//        case TURN_LEFT_90_DEGREES_2:
//            // 左转 90 度
//              pidPosition1.target_val += 3120; // 假设左转 90 度需要的脉冲数
//				 now_position1+=Encoder1Count;
//				if(  pidPosition1.target_val < now_position1)
//				{
//				 pidMotor1Speed.target_val=-5;
//				 pidMotor2Speed.target_val=5;
//				 pidMotor3Speed.target_val=-5;
//				 pidMotor4Speed.target_val=5;
//				 Motor1_SetSpeed(PID_realize(&pidMotor1Speed, Motor1Speed));
//         Motor2_SetSpeed(PID_realize(&pidMotor2Speed, Motor2Speed));
//				 Motor3_SetSpeed(PID_realize(&pidMotor3Speed, Motor3Speed));
//         Motor4_SetSpeed(PID_realize(&pidMotor4Speed, Motor4Speed));
//     	}
//				current_state = ready1;
//            break;
//          case ready1:
//            // 检查是否完成左转
//            if (fabs(pidPosition1.target_val - now_position1) < 50) {
//                pidPosition1.target_val = now_position1; // 停止转弯
//               current_state =  FORWARD_15_CIRCLES;
//            }
//						
//            break;
//        case FORWARD_15_CIRCLES:
//            // 前进 15 圈
//           ccPID(15 * 3120, 0,0);
//                current_state =ready2;
//            break;
//        case ready2:
//            // 检查是否到达目标位置
//            if (fabs(pidPosition1.target_val - now_position1) < 50) {
//                pidPosition1.target_val = now_position1; // 停止前进
//                current_state =  RIGHT_90_DEGREES_1;
//            }
//            break;
//        case RIGHT_90_DEGREES_1:
//            // 右转 90 度
//                pidPosition1.target_val += 3120; // 假设左转 90 度需要的脉冲数
//				 now_position1+=Encoder1Count;
//				if(  pidPosition1.target_val <	 now_position1)
//				{
//				 pidMotor1Speed.target_val=5;
//				 pidMotor2Speed.target_val=-5;
//				 pidMotor3Speed.target_val=5;
//				 pidMotor4Speed.target_val=-5;
//				 Motor1_SetSpeed(PID_realize(&pidMotor1Speed, Motor1Speed));
//         Motor2_SetSpeed(PID_realize(&pidMotor2Speed, Motor2Speed));
//				 Motor3_SetSpeed(PID_realize(&pidMotor3Speed, Motor3Speed));
//         Motor4_SetSpeed(PID_realize(&pidMotor4Speed, Motor4Speed));
//									current_state = ready3;
//     	}
//            break;
//        case ready3:
//            // 检查是否完成右转
//            if (fabs(pidPosition1.target_val - now_position1) < 50) {
//                pidPosition1.target_val = now_position1; // 停止转弯
//                	current_state = FORWARD_6_CIRCLES;
//            }
//            break;
//        case  FORWARD_6_CIRCLES:
//            // 前进 6 圈
//            ccPID(6 * 3120, 0,0);
//                current_state =ready4;
//            break;
//        case ready4:
//            // 检查是否到达目标位置
//            if (fabs(pidPosition1.target_val - now_position1) < 50) {
//                pidPosition1.target_val = now_position1; // 停止前进
//                 current_state =TURN_RIGHT_90_DEGREES_2;
//            }
//            break;
//        case TURN_RIGHT_90_DEGREES_2:
//            // 右转 90 度
//             pidPosition1.target_val += 3120; // 假设左转 90 度需要的脉冲数
//				 now_position1+=Encoder1Count;
//				if(  pidPosition1.target_val <	 now_position1)
//				{
//				 pidMotor1Speed.target_val=5;
//				 pidMotor2Speed.target_val=-5;
//				 pidMotor3Speed.target_val=5;
//				 pidMotor4Speed.target_val=-5;
//				 Motor1_SetSpeed(PID_realize(&pidMotor1Speed, Motor1Speed));
//         Motor2_SetSpeed(PID_realize(&pidMotor2Speed, Motor2Speed));
//				 Motor3_SetSpeed(PID_realize(&pidMotor3Speed, Motor3Speed));
//         Motor4_SetSpeed(PID_realize(&pidMotor4Speed, Motor4Speed));
//								current_state =ready5;
//     	}
//            break;
//        case ready5:
//            // 检查是否完成右转
//            if (fabs(pidPosition1.target_val - now_position1) < 50) {
//                pidPosition1.target_val = now_position1; // 停止转弯
//               current_state = FORWARD_20_CIRCLES;
//            }
//            break;
//        case  FORWARD_20_CIRCLES:
//            // 前进 20 圈
//            ccPID(20 * 3120, 0,0);
//				 current_state =ready6;
//            break;
//        case ready6:
//            // 检查是否到达目标位置
//            if (fabs(pidPosition1.target_val - now_position1) < 50) {
//                pidPosition1.target_val = now_position1; // 停止前进
//               ccPID(0, 0,0);
//            }
//            break;
//        default:
//            // 所有步骤完成
//            ccPID(5* 3120, 0,0);
//            break;
//    }

//	}
//	

//int limit(float Motor1pwm,float targetspeed1)
//{
//	if( Motor1pwm>targetspeed1)
//	{
//		return targetspeed1;
//	}
//	return Motor1pwm;
//}
// 
//float rechange(tPid *pidPosition1, short now_position1) {
//    if (pidPosition1->target_val - now_position1 > 0) {
//        return pidPosition1->target_val - now_position1;
//    } else {
//        return now_position1 - pidPosition1->target_val;
//    }
//}
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim1) { 
			
			// 假设htim1是控制周期的定时器（如10ms）
		  //PID_init();
 

       // TimerCount++;
        updateusart(); // 先更新速度
			
		
			// movePath() ;
	Motor1pwm=PID_realize(&pidPosition1, now_position11);
	//		C=Motor1pwm*100/3120;
		// Motor1_SetSpeed(Motor1pwm);
	//Motor1pwm=limit (Motor1pwm,pidMotor1Speed.target_val);
			//Motor1pwm= limitSpeed(Motor1pwm, maxSpeed, minSpeed);
		
		if (fabs(pidPosition1.target_val-now_position11)<3)
		{	
			pidMotor1Speed.target_val=0;
			 Motor1_SetSpeed(PID_realize(&pidMotor1Speed, Motor1Speed));
		}
		else
		{
			pidMotor1Speed.target_val=	Motor1pwm;
			A=PID_realize(&pidMotor1Speed, Motor1Speed);//内环输入速度
            // 限制速度变化量
           // A = limitSpeedChange(lastMotor1Speed, A, maxSpeedChange);
			 Motor1_SetSpeed(A/3120);
		}
	//	 lastMotor1Speed = A; 
//     // 计算每个电机的位置环 PID 输出
//        float positionOutput1 = PID_Position1_realize(&pidPosition1, (float)Encoder1Count);  // 电机 1 的位置环PID输出
//        float positionOutput2 = PID_Position2_realize(&pidPosition2, (float)Encoder2Count);  // 电机 2 的位置环PID输出
//        float positionOutput3 = PID_Position3_realize(&pidPosition3, (float)Encoder3Count);  // 电机 3 的位置环PID输出
//        float positionOutput4 = PID_Position4_realize(&pidPosition4, (float)Encoder4Count);  // 电机 4 的位置环PID输出

//        // 将每个电机的位置环 PID 输出传递给对应的速度环 PID 控制器
//        pidMotor1Speed.target_val = positionOutput1;
//        pidMotor2Speed.target_val = positionOutput2;
//        pidMotor3Speed.target_val = positionOutput3;
//        pidMotor4Speed.target_val = positionOutput4;
//			
//        // 执行PID计算并设置电机速度
//     Motor1_SetSpeed(PID_realize(&pidMotor1Speed, Motor1Speed)); // 转换为整数PWM
//        Motor2_SetSpeed(PID_realize(&pidMotor2Speed, Motor2Speed));
//        Motor3_SetSpeed(PID_realize(&pidMotor3Speed, Motor3Speed));
//        Motor4_SetSpeed(PID_realize(&pidMotor4Speed, Motor4Speed));
//        
        // 发送数据（可选）
      
       sprintf(sendBuffer, "fabs( pidPosition1.target_val-now_position1): %.2f\r\n ",
              fabs( pidPosition1.target_val-now_position1));
			ANO_DT_Send_F2(pidPosition1.target_val, A/3120, pidMotor1Speed.target_val/3120  ,pidPosition1.actual_val);//a是内环输入速度
       //pidspeed目标值，内环用pid算出速度,外环用pid算出脉冲/3120，位置误差
			 UART_SendString(&huart1, sendBuffer);
		
//		
//       sprintf(sendBuffer, "pidMotor1Speed.err: %.2f\r\n Motor1Speed: %.2f\r\n ", 
//               pidMotor1Speed.err,Motor1Speed);
			//ANO_DT_Send_F2(pidMotor1Speed.err*100, Motor1Speed*100,pidMotor1Speed.err*100,pidMotor4Speed.err*100);
      //  UART_SendString(&huart1, sendBuffer);
    }
}


