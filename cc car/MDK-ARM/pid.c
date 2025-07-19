#include <main.h>
#include <tim.h>
#include <encoder.h>
long Integral_bias_Left = 0 ;
long Integral_bias_Right = 0 ; /* ƫ���ۻ� */
float Bias_Left = 0,Last_bias_Left = 0,Prev_bias_Left = 0;
float Bias_Right = 0,Last_bias_Right = 0,Prev_bias_Right = 0;
float Bias_yaw = 0,Last_bias_yaw = 0;
long Integral_bias_yaw = 0;

/* λ��ʽPIDϵ�� */
<<<<<<< HEAD
float Position_KP=0.038,Position_KI=0,Position_KD=0.085;//6.382 
=======
float Position_KP=0.03,Position_KI=0,Position_KD=0;//6.382 
>>>>>>> master
//�ڻ�
float Incremental_KP = 5.50,Incremental_KI =  3.00050 , Incremental_KD = 0.40; //���ǱȽ������Ĳ�����	

/*********************�ǶȻ�*********************/
float Yaw_KP = 20.0, Yaw_KI = 0.1, Yaw_KD = 40.0;
int Yaw_PWM;
float Yaw_fAngle = 0.0f;


// ���ļ��������������õľ�̬����
static long PID_Bias = 0, PID_Last_Bias = 0;

/**************************************************************************
 * @brief  λ��ʽPID������
 * @param  ʵ��λ��reality��Ŀ��λ��target
 * @retval ���PWM
 * @attention
 * @describe ����λ��ʽ��ɢPID��ʽ
				pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
				e(k)������ƫ��
				e(k-1)������һ�ε�ƫ��
				��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,...,k;
 * @matters pwm�������
**************************************************************************/
long Position_PID_Left(long reality, long target,int reset) //new: reset
{
	static long Bias, Last_Bias;   // ƫ��Bias���ϴ�ƫ��Last_Bias
	static long PWM_OUT;

	
	// new: ������ù���   
	//����ģʽ - ��reset=1ʱִ��
    if(reset == 1) {
        Bias = 0;
        Last_Bias = 0;
        PWM_OUT = 0;
        return 0; // ����ֵ����Ҫ�� reset ģʽ���������㣬������ʵ�����
    }
	
	
	Bias = target - reality; /* ����ƫ�� */
	Integral_bias_Left += Bias;	 /* ƫ���ۻ� */

	if (Integral_bias_Left > 5000)
		Integral_bias_Left = 5000; /* �����޷� */
	if (Integral_bias_Left < -5000)
		Integral_bias_Left = -5000;

	PWM_OUT = (Position_KP * Bias)					/* �������� */
			  + (Position_KI * Integral_bias_Left)		/* ���ֻ��� */
			  + (Position_KD * (Bias - Last_Bias)); /* ΢�ֻ��� */

	Last_Bias = Bias; /* �����ϴ�ƫ�� */

	return PWM_OUT; /* ������ */
}

//���ֵ�λ��ʽPID --- д������������Ϊ�˱��������ݲ�����
<<<<<<< HEAD
long Position_PID_Right(long reality, long target)
=======
long Position_PID_Right(long reality, long target,int reset)
>>>>>>> master
{
	static long Bias, Last_Bias;   // ƫ��Bias���ϴ�ƫ��Last_Bias
	static long PWM_OUT;

<<<<<<< HEAD
=======
	// new: ������ù���   
	//����ģʽ - ��reset=1ʱִ��
    if(reset == 1) {
        Bias = 0;
        Last_Bias = 0;
        PWM_OUT = 0;
        return 0; // ����ֵ����Ҫ�� reset ģʽ���������㣬������ʵ�����
    }
	
	
>>>>>>> master
	Bias = target - reality; /* ����ƫ�� */
	Integral_bias_Right += Bias;	 /* ƫ���ۻ� */

	if (Integral_bias_Right > 5000)
		Integral_bias_Right = 5000; /* �����޷� */
	if (Integral_bias_Right < -5000)
		Integral_bias_Right = -5000;

	PWM_OUT = (Position_KP * Bias)					/* �������� */
			  + (Position_KI * Integral_bias_Right)		/* ���ֻ��� */
			  + (Position_KD * (Bias - Last_Bias)); /* ΢�ֻ��� */

	Last_Bias = Bias; /* �����ϴ�ƫ�� */
	return PWM_OUT; /* ������ */
}




/**************************************************************************
 * @brief  ����ʽPID������
 * @param  ʵ��ֵreality��Ŀ��ֵtarget
 * @retval ���PWM
 * @attention
 * @describe ��������ʽ��ɢPID��ʽ 
				pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
				e(k)������ƫ�� 
				e(k-1)������һ�ε�ƫ��  �Դ����� 
 * @matters pwm�������
**************************************************************************/
int Incremental_PID_Left(int reality,int target, int reset)
{ 	
	 static float Pwm;
	
	// new: ������ù���  
	// ����ģʽ
    if (reset == 1) {
        Pwm = 0;
        Bias_Left = 0;
        Last_bias_Left = 0;
        Prev_bias_Left = 0;
        return 0;
    }
   
	 Bias_Left=target-reality;                                  /* ����ƫ�� */
    
	 Pwm += (Incremental_KP*(Bias_Left-Last_bias_Left))               /* �������� */
           +(Incremental_KI*Bias_Left)                           /* ���ֻ��� */
           +(Incremental_KD*(Bias_Left - 2 * Last_bias_Left + Prev_bias_Left));  /* ΢�ֻ��� */ 
    
   Prev_bias_Left=Last_bias_Left;                                   /* �������ϴ�ƫ�� */
	 Last_bias_Left=Bias_Left;	                                    /* ������һ��ƫ�� */
    
	 return Pwm;                                            /* ������ */
}
//���� ����ʽPID����
<<<<<<< HEAD
int Incremental_PID_Right(int reality,int target)
{ 	
	 static float Pwm;
=======
int Incremental_PID_Right(int reality,int target, int reset)
{ 	
	 static float Pwm;
	
	// new: ������ù���  
	// ����ģʽ
    if (reset == 1) {
        Pwm = 0;
        Bias_Left = 0;
        Last_bias_Left = 0;
        Prev_bias_Left = 0;
        return 0;
    }
	
	
>>>>>>> master

	 Bias_Right=target-reality;                                   /* ����ƫ�� */
    
	 Pwm += (Incremental_KP*(Bias_Right-Last_bias_Right))               /* �������� */
           +(Incremental_KI*Bias_Right)                           /* ���ֻ��� */
           +(Incremental_KD*(Bias_Right - 2 * Last_bias_Right + Prev_bias_Right));  /* ΢�ֻ��� */ 
    
   Prev_bias_Right=Last_bias_Right;                                   /* �������ϴ�ƫ�� */
	 Last_bias_Right=Bias_Right;	                                    /* ������һ��ƫ�� */
    
	 return Pwm;                                            /* ������ */
}


<<<<<<< HEAD

// ��װ ���ù�0����
void Reset_Position_PID_Left(void)
{
    // ͨ�����ò�������λ�û�����  
    Position_PID_Left(0, 0, 1);
    Integral_bias_Left = 0;
}

void Reset_Incremental_PID_Left(void)
{
    Incremental_PID_Left(0, 0, 1);
    
}


=======
//// ���ù�0
//void Reset_Position_PID_Left(void)
//{
//    // ͨ�����ò�������λ�û�����  
//    Position_PID_Left(0, 0, 1);
//    Integral_bias_Left = 0;
//}

//void Reset_Incremental_PID_Left(void)
//{
//    Incremental_PID_Left(0, 0, 1);
////    
//}
>>>>>>> master

////�ǶȻ�
//int Witc901_Yaw_correct(float real_yaw, float target_yaw)
//{
//	static long PWM_OUT;
//	Bias_yaw = target_yaw - real_yaw; /* ����ƫ�� */
//	Integral_bias_yaw += Bias_yaw;	 /* ƫ���ۻ� */
//	if(real_num == 2)
//		Integral_bias_yaw=0;
//	if (Integral_bias_yaw > 1500)
//		Integral_bias_yaw = 1500; /* �����޷� */
//	if (Integral_bias_yaw < -1500)
//		Integral_bias_yaw = -1500;

//	PWM_OUT = (Yaw_KP * Bias_yaw)					/* �������� */
//			  + (Yaw_KI * Integral_bias_yaw)		/* ���ֻ��� */
//			  + (Yaw_KD * (Bias_yaw - Last_bias_yaw)); /* ΢�ֻ��� */

//	Last_bias_yaw = Bias_yaw; /* �����ϴ�ƫ�� */
//	
//	return PWM_OUT;
//}
//	
