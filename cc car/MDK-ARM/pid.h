#ifndef __PID_H_
#define __PID_H_
#include <main.h>

int Incremental_PID_Left(int reality,int target,int reset);		//���� ����ʽPID
<<<<<<< HEAD
int Incremental_PID_Right(int reality,int target);	    //���� ����ʽPID

//�����ֶ��ǵ�����PID���㣬��Ȼstatic����ı��������������߼�������
long Position_PID_Left(long reality,long target,int reset);		//���� λ��ʽPID
long Position_PID_Right(long reality, long target);		//���� λ��ʽPID
=======
int Incremental_PID_Right(int reality,int target,int reset);	    //���� ����ʽPID

//�����ֶ��ǵ�����PID���㣬��Ȼstatic����ı��������������߼�������
long Position_PID_Left(long reality,long target,int reset);		//���� λ��ʽPID
long Position_PID_Right(long reality, long target,int reset);		//���� λ��ʽPID
>>>>>>> master


void Reset_Position_PID_Left(void);
void Reset_Incremental_PID_Left(void);

<<<<<<< HEAD
// ϵͳ���ƽӿ�
void PID_System_Reset(void);


=======
>>>>>>> master


// ����ȫ�ֱ���
extern long Integral_bias_Left;
extern long Integral_bias_Right;
extern float Bias_Left, Last_bias_Left, Prev_bias_Left;
extern float Bias_Right, Last_bias_Right, Prev_bias_Right;
static long Bias, Last_Bias; 
#endif
