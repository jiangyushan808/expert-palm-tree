
#ifndef MOTOR_H_
#define MOTOR_H_

#include "stm32f4xx_hal.h" //HAL?????
#include <main.h>
//PWM�޷� �궨��
#define PWM_MAX  600  // TIM1��Period=999
#define PWM_MIN -999
#define Dead_Voltage  100	//���������ѹ PMW = 150
#define Rpm_Max       146     /* ���ת��  rpm */


//#define MOTOR_GO HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_SET)
//void Motor_Init(void);
void Motor1_SetSpeed(int Speed);
void Motor2_SetSpeed(int Speed);
void Motor3_SetSpeed(int Speed);
void Motor4_SetSpeed(int Speed);

#endif /* MOTOR_H_ */

