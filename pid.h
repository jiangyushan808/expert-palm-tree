#ifndef __PID_H
#define __PID_H

//����һ���ṹ������
typedef struct 
{
	float target_val;//Ŀ��ֵ
	float actual_val;//ʵ��ֵ
	float err;//��ǰƫ��
	float err_last;//�ϴ�ƫ��
	float err_sum;//����ۼ�ֵ
	float Kp,Ki,Kd;//���������֣�΢��ϵ��
	float i_max;//�����޷�����ֵ
	
} tPid;

//struct ppid
//{
//	float target_val;//Ŀ��ֵ
//	float actual_val;//ʵ��ֵ
//	float err;//��ǰƫ��
//	float err_last;//�ϴ�ƫ��
//	float err_sum;//����ۼ�ֵ
//	float Kp,Ki,Kd;//���������֣�΢��ϵ��
//	float i_max;//�����޷�����ֵ
//	
//} ;
//struct PID {
//    float target_val;
//    float err;
//    float err_sum;
//    // ������Ա
//};
extern tPid pidPosition1;  // ��� 1 ��λ�û� PID
extern tPid pidPosition2;  // ��� 2 ��λ�û� PID
extern tPid pidPosition3;  // ��� 3 ��λ�û� PID
extern tPid pidPosition4;  // ��� 4 ��λ�û� PID

//��������
float P_realize(tPid * pid,float actual_val);
void PID_init();
float PI_realize(tPid * pid,float actual_val);
float PID_realize(tPid * pid,float actual_val);
float PID_Position1_realize(tPid *ppid, float actual_val);
//float PID_Position2_realize(tPid *pid float actual_val) ;
//float PID_Position3_realize(tPid *pid, float actual_val);
//float PID_Position4_realize(tPid *pid, float actual_val) ;
#endif

