#ifndef ENCODER_H_
#define ENCODER_H_
#include <main.h> 
#include <pid.h>
<<<<<<< HEAD
#include <encoder.h>
=======

>>>>>>> master
float limitSpeedChange(float currentSpeed, float targetSpeed, float maxChange);
int Xianfu(int data,int max);
void UART_SendString(UART_HandleTypeDef *huart,  char*str) ;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
<<<<<<< HEAD
 extern int num;	//num��Ȧ������˼��Ŀ��Ȧ��
extern int rpm ;

extern  float Target_Velocity;
=======
extern int num;	//num��Ȧ������˼��Ŀ��Ȧ��
extern int rpm ; //RPMĿ��ת��

extern  long Target_Velocity;
>>>>>>> master
extern  long Reality_Velocity;   /* Ŀ���ٶȣ�ʵ���ٶ� */
extern long Target_Position;
extern long Reality_Position;   /* Ŀ��λ�ã�ʵ��λ�� */
extern long now_position1;
//��ȡ����������ֵ
extern int16_t    Motor4Speed;
extern  short	Encoder_cnt ;

<<<<<<< HEAD
=======
long Pulse_Encoder_Cnt(float num);
long Rpm_Encoder_Cnt(float rpm);
float Num_AllEncoder(long encoder_num);

>>>>>>> master

#endif 


