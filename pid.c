#include "pid.h"

//定义一个结构体类型变量
tPid pidMotor1Speed;
tPid pidMotor2Speed;
tPid pidMotor3Speed;
tPid pidMotor4Speed;

tPid pidPosition1;
tPid pidPosition2;
tPid pidPosition3;
tPid pidPosition4;
//给结构体类型变量赋初值
void PID_init()
{
	pidMotor1Speed.actual_val=0.0;
	pidMotor1Speed.target_val=5;
	pidMotor1Speed.err=0.0;
	pidMotor1Speed.err_last=0.0;
	pidMotor1Speed.err_sum=0.0;
	pidMotor1Speed.Kp=2.3;
	pidMotor1Speed.Ki=0.9;
	pidMotor1Speed.Kd=0.1;
	 pidMotor1Speed.i_max = 50.0;  
	
		pidMotor2Speed.actual_val=0.0;
	pidMotor2Speed.target_val=7.00;
	pidMotor2Speed.err=0.0;
	pidMotor2Speed.err_last=0.0;
	pidMotor2Speed.err_sum=0.0;
	pidMotor2Speed.Kp=2.3;
	pidMotor2Speed.Ki=0.9;
	pidMotor2Speed.Kd=0.1;
	 pidMotor2Speed.i_max = 100.0;  
	
		pidMotor3Speed.actual_val=0.0;
	pidMotor3Speed.target_val=7.00;
	pidMotor3Speed.err=0.0;
	pidMotor3Speed.err_last=0.0;
	pidMotor3Speed.err_sum=0.0;
	pidMotor3Speed.Kp=2.31;
	pidMotor3Speed.Ki=0.9;
	pidMotor3Speed.Kd=0.52;
	 pidMotor3Speed.i_max = 77;  
	
	
			pidMotor4Speed.actual_val=0.0;
	pidMotor4Speed.target_val=7.00;
	pidMotor4Speed.err=0.0;
	pidMotor4Speed.err_last=0.0;
	pidMotor4Speed.err_sum=0.0;
	pidMotor4Speed.Kp=2.3;
	pidMotor4Speed.Ki=0.8;
	pidMotor4Speed.Kd=0;
	// pidMotor4Speed.i_max = 50.0;

// 初始化电机 1 的位置环 PID
    pidPosition1.actual_val = 0.0;
    pidPosition1.target_val = 3120*10;
    pidPosition1.err = 0.0;
    pidPosition1.err_last = 0.0;
    pidPosition1.err_sum = 0.0;
    pidPosition1.Kp = 0.8;  // 比例系数
    pidPosition1.Ki = 0; // 积分系数
    pidPosition1.Kd = 0;  // 微分系数
  pidPosition1.i_max = 60000;
    // 初始化电机 2 的位置环 PID
    pidPosition2.actual_val = 0.0;
    pidPosition2.target_val =15600;
    pidPosition2.err = 0.0;
    pidPosition2.err_last = 0.0;
    pidPosition2.err_sum = 0.0;
    pidPosition2.Kp = 0.8;
    pidPosition2.Ki = 0;
    pidPosition2.Kd = 0;
    pidPosition2.i_max = 100.0; 

    // 初始化电机 3 的位置环 PID
    pidPosition3.actual_val = 0.0;
    pidPosition3.target_val = 15600;
    pidPosition3.err = 0.0;
    pidPosition3.err_last = 0.0;
    pidPosition3.err_sum = 0.0;
    pidPosition3.Kp = 0.8;
    pidPosition3.Ki = 0;
    pidPosition3.Kd = 0;
    pidPosition3.i_max = 100.0;

    // 初始化电机 4 的位置环 PID
    pidPosition4.actual_val = 0.0;
    pidPosition4.target_val =15600;
    pidPosition4.err = 0.0;
    pidPosition4.err_last = 0.0;
    pidPosition4.err_sum = 0.0;
    pidPosition4.Kp = 0.8;
    pidPosition4.Ki = 0;
    pidPosition4.Kd = 0;
    pidPosition4.i_max = 100.0;
}
//比例p调节控制函数
float P_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;//当前误差=目标值-真实值
	//比例控制调节   输出=Kp*当前误差
	pid->actual_val = pid->Kp*pid->err;
	return pid->actual_val;
}
//比例P 积分I 控制函数
float PI_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;//当前误差=目标值-真实值
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
	//使用PI控制 输出=Kp*当前误差+Ki*误差累计值
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum;
	
	return pid->actual_val;
}
// PID控制函数
float PID_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;//当前误差=目标值-真实值
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
//	
	// 新增：积分限幅（Anti-Windup）
    if (pid->err_sum > pid->i_max) {
        pid->err_sum = pid->i_max;
    } else if (pid->err_sum < -pid->i_max) {
        pid->err_sum = -pid->i_max;
    }
	
	
//    // 仅对 pidMotor3Speed 进行积分限幅
//    if (pid == &pidMotor3Speed) {
//        if (pid->err_sum > pid->i_max) {
//            pid->err_sum = pid->i_max;
//        } else if (pid->err_sum < -pid->i_max) {
//            pid->err_sum = -pid->i_max;
//        }
//    }
	
	//使用PID控制 输出 = Kp*当前误差  +  Ki*误差累计值 + Kd*(当前误差-上次误差)
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);
	//保存上次误差: 这次误差赋值给上次误差
	pid->err_last = pid->err;
	
	return pid->actual_val;
}



//// 计算电机 1 的位置环 PID 输出
//float PID_Position1_realize(tPid *ppid, float actual_val) {
//    ppid->actual_val = actual_val;
//    ppid->err = ppid->target_val - ppid->actual_val; // 位置误差
//    ppid->err_sum += ppid->err; // 误差累加

//    // 积分限幅
//    if (ppid->err_sum > ppid->i_max) {
//        ppid->err_sum = ppid->i_max;
//    } else if (ppid->err_sum < -ppid->i_max) {
//        ppid->err_sum = -ppid->i_max;
//    }

//    // 计算PID输出
//    float p_term = ppid->Kp * ppid->err;
//    float i_term = ppid->Ki * ppid->err_sum;
//    float d_term = ppid->Kd * (ppid->err - ppid->err_last);

//    ppid->err_last = ppid->err;

//    return p_term + i_term + d_term; // 返回控制信号（目标速度）
//}

//// 计算电机 2 的位置环 PID 输出
//float PID_Position2_realize(tPid*pid, float actual_val) {
//    pid->actual_val = actual_val;
//    pid->err = pid->target_val - pid->actual_val; // 位置误差
//    pid->err_sum += pid->err; // 误差累加

//    // 积分限幅
//    if (pid->err_sum > pid->i_max) {
//        pid->err_sum = pid->i_max;
//    } else if (pid->err_sum < -pid->i_max) {
//        pid->err_sum = -pid->i_max;
//    }

//    // 计算PID输出
//    float p_term = pid->Kp * pid->err;
//    float i_term = pid->Ki * pid->err_sum;
//    float d_term = pid->Kd * (pid->err - pid->err_last);

//    pid->err_last = pid->err;

//    return p_term + i_term + d_term; // 返回控制信号（目标速度）
//}

//// 计算电机 3 的位置环 PID 输出
//float PID_Position3_realize(tPid *pid, float actual_val) {
//    pid->actual_val = actual_val;
//    pid->err = pid->target_val - pid->actual_val; // 位置误差
//    pid->err_sum += pid->err; // 误差累加

//    // 积分限幅
//    if (pid->err_sum > pid->i_max) {
//        pid->err_sum = pid->i_max;
//    } else if (pid->err_sum < -pid->i_max) {
//        pid->err_sum = -pid->i_max;
//    }

//    // 计算PID输出
//    float p_term = pid->Kp * pid->err;
//    float i_term = pid->Ki * pid->err_sum;
//    float d_term = pid->Kd * (pid->err - pid->err_last);

//    pid->err_last = pid->err;

//    return p_term + i_term + d_term; // 返回控制信号（目标速度）
//}

//// 计算电机 4 的位置环 PID 输出
//float PID_Position4_realize(tPid*pid, float actual_val) {
//    pid->actual_val = actual_val;
//    pid->err = pid->target_val - pid->actual_val; // 位置误差
//    pid->err_sum += pid->err; // 误差累加

//    // 积分限幅
//    if (pid->err_sum > pid->i_max) {
//        pid->err_sum = pid->i_max;
//    } else if (pid->err_sum < -pid->i_max) {
//        pid->err_sum = -pid->i_max;
//    }

//    // 计算PID输出
//    float p_term = pid->Kp * pid->err;
//    float i_term = pid->Ki * pid->err_sum;
//    float d_term = pid->Kd * (pid->err - pid->err_last);

//    pid->err_last = pid->err;

//    return p_term + i_term + d_term; // 返回控制信号（目标速度）
//}
