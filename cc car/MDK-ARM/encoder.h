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
 extern int num;	//num，圈数的意思，目标圈数
extern int rpm ;

extern  float Target_Velocity;
=======
extern int num;	//num，圈数的意思，目标圈数
extern int rpm ; //RPM目标转速

extern  long Target_Velocity;
>>>>>>> master
extern  long Reality_Velocity;   /* 目标速度，实际速度 */
extern long Target_Position;
extern long Reality_Position;   /* 目标位置，实际位置 */
extern long now_position1;
//读取编码器计数值
extern int16_t    Motor4Speed;
extern  short	Encoder_cnt ;

<<<<<<< HEAD
=======
long Pulse_Encoder_Cnt(float num);
long Rpm_Encoder_Cnt(float rpm);
float Num_AllEncoder(long encoder_num);

>>>>>>> master

#endif 


