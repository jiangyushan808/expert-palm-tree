#ifndef __PID_H_
#define __PID_H_
#include <main.h>

int Incremental_PID_Left(int reality,int target,int reset);		//左轮 增量式PID
<<<<<<< HEAD
int Incremental_PID_Right(int reality,int target);	    //右轮 增量式PID

//左右轮都是单独的PID计算，不然static定义的变量会混叠，控制逻辑有问题
long Position_PID_Left(long reality,long target,int reset);		//左轮 位置式PID
long Position_PID_Right(long reality, long target);		//右轮 位置式PID
=======
int Incremental_PID_Right(int reality,int target,int reset);	    //右轮 增量式PID

//左右轮都是单独的PID计算，不然static定义的变量会混叠，控制逻辑有问题
long Position_PID_Left(long reality,long target,int reset);		//左轮 位置式PID
long Position_PID_Right(long reality, long target,int reset);		//右轮 位置式PID
>>>>>>> master


void Reset_Position_PID_Left(void);
void Reset_Incremental_PID_Left(void);

<<<<<<< HEAD
// 系统控制接口
void PID_System_Reset(void);


=======
>>>>>>> master


// 声明全局变量
extern long Integral_bias_Left;
extern long Integral_bias_Right;
extern float Bias_Left, Last_bias_Left, Prev_bias_Left;
extern float Bias_Right, Last_bias_Right, Prev_bias_Right;
static long Bias, Last_Bias; 
#endif
