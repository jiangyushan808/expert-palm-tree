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
	 // ���������������ٶȱ仯��
float maxSpeedChange = 100.0; 
// �������洢��һ�εĵ���ٶ�
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
// ��װ����PID�����߼�Ϊ����
//void ccPID(int target_position, short now_position1,short Encoder1Count ) {
//    

//        // λ�û�����
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
//			A1=PID_realize(&pidMotor1Speed, Motor1Speed);//�ڻ������ٶ�
//			A2=PID_realize(&pidMotor2Speed, Motor2Speed);
//			A3=PID_realize(&pidMotor3Speed, Motor3Speed);
//			A4=PID_realize(&pidMotor4Speed, Motor4Speed);
//            // �����ٶȱ仯��
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
//            // ǰ�� 10 Ȧ
//        			 ccPID(10 * 3120, 0,0);
//				
//                 if (fabs(pidPosition1.target_val - now_position1) < 50) {
//        current_state = TURN_LEFT_90_DEGREES_1;
//    }
//            break;
//        
//        case TURN_LEFT_90_DEGREES_1:
//            // ��ת 90 ��
//           pidPosition1.target_val += 3120; // ������ת 90 ����Ҫ��������
//				 now_position1+=Encoder1Count;
//				if(  pidPosition1.target_val <	 now_position1)
//				{
//					//�������  = (L / 2D) * 3120
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
//            // ����Ƿ������ת
//            if (fabs(pidPosition1.target_val - now_position1) < 50) {
//                pidPosition1.target_val = now_position1; // ֹͣת��
//               	current_state =  FORWARD_5_CIRCLES;
//            }
//					
//            break;
//        case   FORWARD_5_CIRCLES:
//            // ǰ�� 5 Ȧ
//            ccPID(5 * 3120, 0,0);// 5 Ȧ��������
//            current_state = TURN_LEFT_90_DEGREES_2;
//              break;
//        case TURN_LEFT_90_DEGREES_2:
//            // ��ת 90 ��
//              pidPosition1.target_val += 3120; // ������ת 90 ����Ҫ��������
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
//            // ����Ƿ������ת
//            if (fabs(pidPosition1.target_val - now_position1) < 50) {
//                pidPosition1.target_val = now_position1; // ֹͣת��
//               current_state =  FORWARD_15_CIRCLES;
//            }
//						
//            break;
//        case FORWARD_15_CIRCLES:
//            // ǰ�� 15 Ȧ
//           ccPID(15 * 3120, 0,0);
//                current_state =ready2;
//            break;
//        case ready2:
//            // ����Ƿ񵽴�Ŀ��λ��
//            if (fabs(pidPosition1.target_val - now_position1) < 50) {
//                pidPosition1.target_val = now_position1; // ֹͣǰ��
//                current_state =  RIGHT_90_DEGREES_1;
//            }
//            break;
//        case RIGHT_90_DEGREES_1:
//            // ��ת 90 ��
//                pidPosition1.target_val += 3120; // ������ת 90 ����Ҫ��������
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
//            // ����Ƿ������ת
//            if (fabs(pidPosition1.target_val - now_position1) < 50) {
//                pidPosition1.target_val = now_position1; // ֹͣת��
//                	current_state = FORWARD_6_CIRCLES;
//            }
//            break;
//        case  FORWARD_6_CIRCLES:
//            // ǰ�� 6 Ȧ
//            ccPID(6 * 3120, 0,0);
//                current_state =ready4;
//            break;
//        case ready4:
//            // ����Ƿ񵽴�Ŀ��λ��
//            if (fabs(pidPosition1.target_val - now_position1) < 50) {
//                pidPosition1.target_val = now_position1; // ֹͣǰ��
//                 current_state =TURN_RIGHT_90_DEGREES_2;
//            }
//            break;
//        case TURN_RIGHT_90_DEGREES_2:
//            // ��ת 90 ��
//             pidPosition1.target_val += 3120; // ������ת 90 ����Ҫ��������
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
//            // ����Ƿ������ת
//            if (fabs(pidPosition1.target_val - now_position1) < 50) {
//                pidPosition1.target_val = now_position1; // ֹͣת��
//               current_state = FORWARD_20_CIRCLES;
//            }
//            break;
//        case  FORWARD_20_CIRCLES:
//            // ǰ�� 20 Ȧ
//            ccPID(20 * 3120, 0,0);
//				 current_state =ready6;
//            break;
//        case ready6:
//            // ����Ƿ񵽴�Ŀ��λ��
//            if (fabs(pidPosition1.target_val - now_position1) < 50) {
//                pidPosition1.target_val = now_position1; // ֹͣǰ��
//               ccPID(0, 0,0);
//            }
//            break;
//        default:
//            // ���в������
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim1) { 
			
			// ����htim1�ǿ������ڵĶ�ʱ������10ms��
		  //PID_init();
 

       // TimerCount++;
        updateusart(); // �ȸ����ٶ�
			
		
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
			A=PID_realize(&pidMotor1Speed, Motor1Speed);//�ڻ������ٶ�
            // �����ٶȱ仯��
           // A = limitSpeedChange(lastMotor1Speed, A, maxSpeedChange);
			 Motor1_SetSpeed(A/3120);
		}
	//	 lastMotor1Speed = A; 
//     // ����ÿ�������λ�û� PID ���
//        float positionOutput1 = PID_Position1_realize(&pidPosition1, (float)Encoder1Count);  // ��� 1 ��λ�û�PID���
//        float positionOutput2 = PID_Position2_realize(&pidPosition2, (float)Encoder2Count);  // ��� 2 ��λ�û�PID���
//        float positionOutput3 = PID_Position3_realize(&pidPosition3, (float)Encoder3Count);  // ��� 3 ��λ�û�PID���
//        float positionOutput4 = PID_Position4_realize(&pidPosition4, (float)Encoder4Count);  // ��� 4 ��λ�û�PID���

//        // ��ÿ�������λ�û� PID ������ݸ���Ӧ���ٶȻ� PID ������
//        pidMotor1Speed.target_val = positionOutput1;
//        pidMotor2Speed.target_val = positionOutput2;
//        pidMotor3Speed.target_val = positionOutput3;
//        pidMotor4Speed.target_val = positionOutput4;
//			
//        // ִ��PID���㲢���õ���ٶ�
//     Motor1_SetSpeed(PID_realize(&pidMotor1Speed, Motor1Speed)); // ת��Ϊ����PWM
//        Motor2_SetSpeed(PID_realize(&pidMotor2Speed, Motor2Speed));
//        Motor3_SetSpeed(PID_realize(&pidMotor3Speed, Motor3Speed));
//        Motor4_SetSpeed(PID_realize(&pidMotor4Speed, Motor4Speed));
//        
        // �������ݣ���ѡ��
      
       sprintf(sendBuffer, "fabs( pidPosition1.target_val-now_position1): %.2f\r\n ",
              fabs( pidPosition1.target_val-now_position1));
			ANO_DT_Send_F2(pidPosition1.target_val, A/3120, pidMotor1Speed.target_val/3120  ,pidPosition1.actual_val);//a���ڻ������ٶ�
       //pidspeedĿ��ֵ���ڻ���pid����ٶ�,�⻷��pid�������/3120��λ�����
			 UART_SendString(&huart1, sendBuffer);
		
//		
//       sprintf(sendBuffer, "pidMotor1Speed.err: %.2f\r\n Motor1Speed: %.2f\r\n ", 
//               pidMotor1Speed.err,Motor1Speed);
			//ANO_DT_Send_F2(pidMotor1Speed.err*100, Motor1Speed*100,pidMotor1Speed.err*100,pidMotor4Speed.err*100);
      //  UART_SendString(&huart1, sendBuffer);
    }
}


