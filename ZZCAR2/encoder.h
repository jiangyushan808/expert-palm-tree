#ifndef ENCODER_H_
#define ENCODER_H_

#include "stm32f4xx_hal.h" //HAL?????
#include <main.h>

extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
 extern uint16_t TimerCount ;

//extern float A;
//extern float C;
extern float Motor1Speed ;
extern float Motor2Speed ;
extern float Motor3Speed ;
extern float Motor4Speed ;
float limitSpeedChange(float currentSpeed, float targetSpeed, float maxChange) ;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void UART_SendString(UART_HandleTypeDef *huart,  char*str) ;
void movePath();
#endif /* ENCODER_ENCODER_H_ */


