#include "pid.h"

//����һ���ṹ�����ͱ���
tPid pidMotor1Speed;
tPid pidMotor2Speed;
tPid pidMotor3Speed;
tPid pidMotor4Speed;

tPid pidPosition1;
tPid pidPosition2;
tPid pidPosition3;
tPid pidPosition4;
//���ṹ�����ͱ�������ֵ
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

// ��ʼ����� 1 ��λ�û� PID
    pidPosition1.actual_val = 0.0;
    pidPosition1.target_val = 3120*10;
    pidPosition1.err = 0.0;
    pidPosition1.err_last = 0.0;
    pidPosition1.err_sum = 0.0;
    pidPosition1.Kp = 0.8;  // ����ϵ��
    pidPosition1.Ki = 0; // ����ϵ��
    pidPosition1.Kd = 0;  // ΢��ϵ��
  pidPosition1.i_max = 60000;
    // ��ʼ����� 2 ��λ�û� PID
    pidPosition2.actual_val = 0.0;
    pidPosition2.target_val =15600;
    pidPosition2.err = 0.0;
    pidPosition2.err_last = 0.0;
    pidPosition2.err_sum = 0.0;
    pidPosition2.Kp = 0.8;
    pidPosition2.Ki = 0;
    pidPosition2.Kd = 0;
    pidPosition2.i_max = 100.0; 

    // ��ʼ����� 3 ��λ�û� PID
    pidPosition3.actual_val = 0.0;
    pidPosition3.target_val = 15600;
    pidPosition3.err = 0.0;
    pidPosition3.err_last = 0.0;
    pidPosition3.err_sum = 0.0;
    pidPosition3.Kp = 0.8;
    pidPosition3.Ki = 0;
    pidPosition3.Kd = 0;
    pidPosition3.i_max = 100.0;

    // ��ʼ����� 4 ��λ�û� PID
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
//����p���ڿ��ƺ���
float P_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//������ʵֵ
	pid->err = pid->target_val - pid->actual_val;//��ǰ���=Ŀ��ֵ-��ʵֵ
	//�������Ƶ���   ���=Kp*��ǰ���
	pid->actual_val = pid->Kp*pid->err;
	return pid->actual_val;
}
//����P ����I ���ƺ���
float PI_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//������ʵֵ
	pid->err = pid->target_val - pid->actual_val;//��ǰ���=Ŀ��ֵ-��ʵֵ
	pid->err_sum += pid->err;//����ۼ�ֵ = ��ǰ����ۼƺ�
	//ʹ��PI���� ���=Kp*��ǰ���+Ki*����ۼ�ֵ
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum;
	
	return pid->actual_val;
}
// PID���ƺ���
float PID_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//������ʵֵ
	pid->err = pid->target_val - pid->actual_val;//��ǰ���=Ŀ��ֵ-��ʵֵ
	pid->err_sum += pid->err;//����ۼ�ֵ = ��ǰ����ۼƺ�
//	
	// �����������޷���Anti-Windup��
    if (pid->err_sum > pid->i_max) {
        pid->err_sum = pid->i_max;
    } else if (pid->err_sum < -pid->i_max) {
        pid->err_sum = -pid->i_max;
    }
	
	
//    // ���� pidMotor3Speed ���л����޷�
//    if (pid == &pidMotor3Speed) {
//        if (pid->err_sum > pid->i_max) {
//            pid->err_sum = pid->i_max;
//        } else if (pid->err_sum < -pid->i_max) {
//            pid->err_sum = -pid->i_max;
//        }
//    }
	
	//ʹ��PID���� ��� = Kp*��ǰ���  +  Ki*����ۼ�ֵ + Kd*(��ǰ���-�ϴ����)
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);
	//�����ϴ����: �����ֵ���ϴ����
	pid->err_last = pid->err;
	
	return pid->actual_val;
}



//// ������ 1 ��λ�û� PID ���
//float PID_Position1_realize(tPid *ppid, float actual_val) {
//    ppid->actual_val = actual_val;
//    ppid->err = ppid->target_val - ppid->actual_val; // λ�����
//    ppid->err_sum += ppid->err; // ����ۼ�

//    // �����޷�
//    if (ppid->err_sum > ppid->i_max) {
//        ppid->err_sum = ppid->i_max;
//    } else if (ppid->err_sum < -ppid->i_max) {
//        ppid->err_sum = -ppid->i_max;
//    }

//    // ����PID���
//    float p_term = ppid->Kp * ppid->err;
//    float i_term = ppid->Ki * ppid->err_sum;
//    float d_term = ppid->Kd * (ppid->err - ppid->err_last);

//    ppid->err_last = ppid->err;

//    return p_term + i_term + d_term; // ���ؿ����źţ�Ŀ���ٶȣ�
//}

//// ������ 2 ��λ�û� PID ���
//float PID_Position2_realize(tPid*pid, float actual_val) {
//    pid->actual_val = actual_val;
//    pid->err = pid->target_val - pid->actual_val; // λ�����
//    pid->err_sum += pid->err; // ����ۼ�

//    // �����޷�
//    if (pid->err_sum > pid->i_max) {
//        pid->err_sum = pid->i_max;
//    } else if (pid->err_sum < -pid->i_max) {
//        pid->err_sum = -pid->i_max;
//    }

//    // ����PID���
//    float p_term = pid->Kp * pid->err;
//    float i_term = pid->Ki * pid->err_sum;
//    float d_term = pid->Kd * (pid->err - pid->err_last);

//    pid->err_last = pid->err;

//    return p_term + i_term + d_term; // ���ؿ����źţ�Ŀ���ٶȣ�
//}

//// ������ 3 ��λ�û� PID ���
//float PID_Position3_realize(tPid *pid, float actual_val) {
//    pid->actual_val = actual_val;
//    pid->err = pid->target_val - pid->actual_val; // λ�����
//    pid->err_sum += pid->err; // ����ۼ�

//    // �����޷�
//    if (pid->err_sum > pid->i_max) {
//        pid->err_sum = pid->i_max;
//    } else if (pid->err_sum < -pid->i_max) {
//        pid->err_sum = -pid->i_max;
//    }

//    // ����PID���
//    float p_term = pid->Kp * pid->err;
//    float i_term = pid->Ki * pid->err_sum;
//    float d_term = pid->Kd * (pid->err - pid->err_last);

//    pid->err_last = pid->err;

//    return p_term + i_term + d_term; // ���ؿ����źţ�Ŀ���ٶȣ�
//}

//// ������ 4 ��λ�û� PID ���
//float PID_Position4_realize(tPid*pid, float actual_val) {
//    pid->actual_val = actual_val;
//    pid->err = pid->target_val - pid->actual_val; // λ�����
//    pid->err_sum += pid->err; // ����ۼ�

//    // �����޷�
//    if (pid->err_sum > pid->i_max) {
//        pid->err_sum = pid->i_max;
//    } else if (pid->err_sum < -pid->i_max) {
//        pid->err_sum = -pid->i_max;
//    }

//    // ����PID���
//    float p_term = pid->Kp * pid->err;
//    float i_term = pid->Ki * pid->err_sum;
//    float d_term = pid->Kd * (pid->err - pid->err_last);

//    pid->err_last = pid->err;

//    return p_term + i_term + d_term; // ���ؿ����źţ�Ŀ���ٶȣ�
//}
