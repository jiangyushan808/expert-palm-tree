#include <main.h>
#include <tim.h>
#include <encoder.h>
long Integral_bias_Left = 0 ;
long Integral_bias_Right = 0 ; /* 偏差累积 */
float Bias_Left = 0,Last_bias_Left = 0,Prev_bias_Left = 0;
float Bias_Right = 0,Last_bias_Right = 0,Prev_bias_Right = 0;
float Bias_yaw = 0,Last_bias_yaw = 0;
long Integral_bias_yaw = 0;

/* 位置式PID系数 */
<<<<<<< HEAD
float Position_KP=0.038,Position_KI=0,Position_KD=0.085;//6.382 
=======
float Position_KP=0.03,Position_KI=0,Position_KD=0;//6.382 
>>>>>>> master
//内环
float Incremental_KP = 5.50,Incremental_KI =  3.00050 , Incremental_KD = 0.40; //算是比较完美的参数了	

/*********************角度环*********************/
float Yaw_KP = 20.0, Yaw_KI = 0.1, Yaw_KD = 40.0;
int Yaw_PWM;
float Yaw_fAngle = 0.0f;


// 在文件顶部声明可重置的静态变量
static long PID_Bias = 0, PID_Last_Bias = 0;

/**************************************************************************
 * @brief  位置式PID控制器
 * @param  实际位置reality，目标位置target
 * @retval 电机PWM
 * @attention
 * @describe 根据位置式离散PID公式
				pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
				e(k)代表本次偏差
				e(k-1)代表上一次的偏差
				∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,...,k;
 * @matters pwm代表输出
**************************************************************************/
long Position_PID_Left(long reality, long target,int reset) //new: reset
{
	static long Bias, Last_Bias;   // 偏差Bias，上次偏差Last_Bias
	static long PWM_OUT;

	
	// new: 添加重置功能   
	//重置模式 - 当reset=1时执行
    if(reset == 1) {
        Bias = 0;
        Last_Bias = 0;
        PWM_OUT = 0;
        return 0; // 返回值不重要， reset 模式仅用于清零，不参与实际输出
    }
	
	
	Bias = target - reality; /* 计算偏差 */
	Integral_bias_Left += Bias;	 /* 偏差累积 */

	if (Integral_bias_Left > 5000)
		Integral_bias_Left = 5000; /* 积分限幅 */
	if (Integral_bias_Left < -5000)
		Integral_bias_Left = -5000;

	PWM_OUT = (Position_KP * Bias)					/* 比例环节 */
			  + (Position_KI * Integral_bias_Left)		/* 积分环节 */
			  + (Position_KD * (Bias - Last_Bias)); /* 微分环节 */

	Last_Bias = Bias; /* 保存上次偏差 */

	return PWM_OUT; /* 输出结果 */
}

//右轮的位置式PID --- 写两个函数，是为了编码器数据不干扰
<<<<<<< HEAD
long Position_PID_Right(long reality, long target)
=======
long Position_PID_Right(long reality, long target,int reset)
>>>>>>> master
{
	static long Bias, Last_Bias;   // 偏差Bias，上次偏差Last_Bias
	static long PWM_OUT;

<<<<<<< HEAD
=======
	// new: 添加重置功能   
	//重置模式 - 当reset=1时执行
    if(reset == 1) {
        Bias = 0;
        Last_Bias = 0;
        PWM_OUT = 0;
        return 0; // 返回值不重要， reset 模式仅用于清零，不参与实际输出
    }
	
	
>>>>>>> master
	Bias = target - reality; /* 计算偏差 */
	Integral_bias_Right += Bias;	 /* 偏差累积 */

	if (Integral_bias_Right > 5000)
		Integral_bias_Right = 5000; /* 积分限幅 */
	if (Integral_bias_Right < -5000)
		Integral_bias_Right = -5000;

	PWM_OUT = (Position_KP * Bias)					/* 比例环节 */
			  + (Position_KI * Integral_bias_Right)		/* 积分环节 */
			  + (Position_KD * (Bias - Last_Bias)); /* 微分环节 */

	Last_Bias = Bias; /* 保存上次偏差 */
	return PWM_OUT; /* 输出结果 */
}




/**************************************************************************
 * @brief  增量式PID控制器
 * @param  实际值reality，目标值target
 * @retval 电机PWM
 * @attention
 * @describe 根据增量式离散PID公式 
				pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
				e(k)代表本次偏差 
				e(k-1)代表上一次的偏差  以此类推 
 * @matters pwm代表输出
**************************************************************************/
int Incremental_PID_Left(int reality,int target, int reset)
{ 	
	 static float Pwm;
	
	// new: 添加重置功能  
	// 重置模式
    if (reset == 1) {
        Pwm = 0;
        Bias_Left = 0;
        Last_bias_Left = 0;
        Prev_bias_Left = 0;
        return 0;
    }
   
	 Bias_Left=target-reality;                                  /* 计算偏差 */
    
	 Pwm += (Incremental_KP*(Bias_Left-Last_bias_Left))               /* 比例环节 */
           +(Incremental_KI*Bias_Left)                           /* 积分环节 */
           +(Incremental_KD*(Bias_Left - 2 * Last_bias_Left + Prev_bias_Left));  /* 微分环节 */ 
    
   Prev_bias_Left=Last_bias_Left;                                   /* 保存上上次偏差 */
	 Last_bias_Left=Bias_Left;	                                    /* 保存上一次偏差 */
    
	 return Pwm;                                            /* 输出结果 */
}
//右轮 增量式PID计算
<<<<<<< HEAD
int Incremental_PID_Right(int reality,int target)
{ 	
	 static float Pwm;
=======
int Incremental_PID_Right(int reality,int target, int reset)
{ 	
	 static float Pwm;
	
	// new: 添加重置功能  
	// 重置模式
    if (reset == 1) {
        Pwm = 0;
        Bias_Left = 0;
        Last_bias_Left = 0;
        Prev_bias_Left = 0;
        return 0;
    }
	
	
>>>>>>> master

	 Bias_Right=target-reality;                                   /* 计算偏差 */
    
	 Pwm += (Incremental_KP*(Bias_Right-Last_bias_Right))               /* 比例环节 */
           +(Incremental_KI*Bias_Right)                           /* 积分环节 */
           +(Incremental_KD*(Bias_Right - 2 * Last_bias_Right + Prev_bias_Right));  /* 微分环节 */ 
    
   Prev_bias_Right=Last_bias_Right;                                   /* 保存上上次偏差 */
	 Last_bias_Right=Bias_Right;	                                    /* 保存上一次偏差 */
    
	 return Pwm;                                            /* 输出结果 */
}


<<<<<<< HEAD

// 封装 重置归0函数
void Reset_Position_PID_Left(void)
{
    // 通过重置参数调用位置环函数  
    Position_PID_Left(0, 0, 1);
    Integral_bias_Left = 0;
}

void Reset_Incremental_PID_Left(void)
{
    Incremental_PID_Left(0, 0, 1);
    
}


=======
//// 重置归0
//void Reset_Position_PID_Left(void)
//{
//    // 通过重置参数调用位置环函数  
//    Position_PID_Left(0, 0, 1);
//    Integral_bias_Left = 0;
//}

//void Reset_Incremental_PID_Left(void)
//{
//    Incremental_PID_Left(0, 0, 1);
////    
//}
>>>>>>> master

////角度环
//int Witc901_Yaw_correct(float real_yaw, float target_yaw)
//{
//	static long PWM_OUT;
//	Bias_yaw = target_yaw - real_yaw; /* 计算偏差 */
//	Integral_bias_yaw += Bias_yaw;	 /* 偏差累积 */
//	if(real_num == 2)
//		Integral_bias_yaw=0;
//	if (Integral_bias_yaw > 1500)
//		Integral_bias_yaw = 1500; /* 积分限幅 */
//	if (Integral_bias_yaw < -1500)
//		Integral_bias_yaw = -1500;

//	PWM_OUT = (Yaw_KP * Bias_yaw)					/* 比例环节 */
//			  + (Yaw_KI * Integral_bias_yaw)		/* 积分环节 */
//			  + (Yaw_KD * (Bias_yaw - Last_bias_yaw)); /* 微分环节 */

//	Last_bias_yaw = Bias_yaw; /* 保存上次偏差 */
//	
//	return PWM_OUT;
//}
//	
