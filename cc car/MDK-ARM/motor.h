
#ifndef MOTOR_H_
#define MOTOR_H_

#include "stm32f4xx_hal.h" //HAL?????
#include <main.h>
//PWM限幅 宏定义
#define PWM_MAX  99  // TIM1的Period=99
#define PWM_MIN -99
#define Dead_Voltage  100	//电机死区电压 PMW = 150
#define Rpm_Max       173     /* 最大转速  rpm */
#define DELTA_PWM_MAX  3       //增量式PID中对单次增量限幅（delta_pwm限幅） pwm的10%


//#define MOTOR_GO HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_SET)
//void Motor_Init(void);
void Motor1_SetSpeed(int Speed);
void Motor2_SetSpeed(int Speed);
void Motor3_SetSpeed(int Speed);
void Motor4_SetSpeed(int Speed);

#endif /* MOTOR_H_ */

