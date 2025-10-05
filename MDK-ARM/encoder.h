#ifndef ENCODER_H_
#define ENCODER_H_
#include <main.h> 
#include <pid.h>

float limitSpeedChange(float currentSpeed, float targetSpeed, float maxChange);
int Xianfu(int data,int max);
void UART_SendString(UART_HandleTypeDef *huart,  char*str) ;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
int isYawReached(float current, float target, float tolerance);

extern float num;	//num，圈数的意思，目标圈数
extern float rpm ; //RPM目标转速

extern float task_step; //状态机

extern  long Target_Velocity;
extern  long Reality_Velocity;   /* 目标速度，实际速度 */
extern long Target_Position;
extern long Reality_Position;   /* 目标位置，实际位置 */

extern long now_position1;
extern long now_position3;
extern long now_position2;
extern long now_position4;
extern long now_position_L;
extern long now_position_R;
//读取编码器计数值
extern int16_t    Motor4Speed;
extern  short	Encoder_cnt ;

void CopeSerial1Data(unsigned char ucData);
long Pulse_Encoder_Cnt(float num);
long Rpm_Encoder_Cnt(float rpm);
float Num_AllEncoder(long encoder_num);

extern float current_yaw  ;
extern float target_yaw;
extern volatile struct SAngle stcAngle;
extern volatile struct SAcc stcAcc;
extern volatile struct SGyro stcGyro;
extern volatile uint8_t stcAngle_seq;
#endif 


