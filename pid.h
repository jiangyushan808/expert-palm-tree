#ifndef __PID_H
#define __PID_H

//声明一个结构体类型
typedef struct 
{
	float target_val;//目标值
	float actual_val;//实际值
	float err;//当前偏差
	float err_last;//上次偏差
	float err_sum;//误差累计值
	float Kp,Ki,Kd;//比例，积分，微分系数
	float i_max;//积分限幅绝对值
	
} tPid;

//struct ppid
//{
//	float target_val;//目标值
//	float actual_val;//实际值
//	float err;//当前偏差
//	float err_last;//上次偏差
//	float err_sum;//误差累计值
//	float Kp,Ki,Kd;//比例，积分，微分系数
//	float i_max;//积分限幅绝对值
//	
//} ;
//struct PID {
//    float target_val;
//    float err;
//    float err_sum;
//    // 其他成员
//};
extern tPid pidPosition1;  // 电机 1 的位置环 PID
extern tPid pidPosition2;  // 电机 2 的位置环 PID
extern tPid pidPosition3;  // 电机 3 的位置环 PID
extern tPid pidPosition4;  // 电机 4 的位置环 PID

//声明函数
float P_realize(tPid * pid,float actual_val);
void PID_init();
float PI_realize(tPid * pid,float actual_val);
float PID_realize(tPid * pid,float actual_val);
float PID_Position1_realize(tPid *ppid, float actual_val);
//float PID_Position2_realize(tPid *pid float actual_val) ;
//float PID_Position3_realize(tPid *pid, float actual_val);
//float PID_Position4_realize(tPid *pid, float actual_val) ;
#endif

