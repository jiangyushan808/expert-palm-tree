#ifndef ENCODER_H_
#define ENCODER_H_
#include <main.h> 
#include <pid.h>
#include <encoder.h>
float limitSpeedChange(float currentSpeed, float targetSpeed, float maxChange);
int Xianfu(int data,int max);
void UART_SendString(UART_HandleTypeDef *huart,  char*str) ;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
 extern int num;	//num��Ȧ������˼��Ŀ��Ȧ��
extern int rpm ;

extern  long Target_Velocity;
extern  long Reality_Velocity;   /* Ŀ���ٶȣ�ʵ���ٶ� */
extern long Target_Position;
extern long Reality_Position;   /* Ŀ��λ�ã�ʵ��λ�� */
extern long now_position1;
//��ȡ����������ֵ
extern int16_t    Motor4Speed;
extern  short	Encoder_cnt ;


#endif 


