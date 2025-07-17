#include "stm32f4xx_hal.h"                // Device header
#include "main.h"
#include "tim.h"


void Motor1_SetSpeed(int Speed) {
//	if (Speed >= 0) {						//���������ת���ٶ�ֵ
//		HAL_GPIO_WritePin(GPIOB, Apower1_Pin, GPIO_PIN_SET);	//PD6�øߵ�ƽ
//		HAL_GPIO_WritePin(GPIOA, Bpower1_Pin, GPIO_PIN_SET); 	//PD7�õ͵�ƽ�����÷���Ϊ��ת
//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,	Speed);		//PWM����Ϊ�ٶ�ֵ
//	} else {								//���򣬼����÷�ת���ٶ�ֵ
//		HAL_GPIO_WritePin(GPIOB, Bpower1_Pin, GPIO_PIN_RESET);	//PD6�øߵ�ƽ
//		HAL_GPIO_WritePin(GPIOA, Apower1_Pin, GPIO_PIN_RESET); 	//PD7�õ͵�ƽ�����÷���Ϊ��ת
//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,-Speed);				//PWM����Ϊ�����ٶ�ֵ����Ϊ��ʱ�ٶ�ֵΪ��������PWMֻ�ܸ�����
//	}
	if (Speed >= 0) {						//���������ת���ٶ�ֵ
		HAL_GPIO_WritePin(AIN10_GPIO_Port, AIN10_Pin, GPIO_PIN_SET);	//PD6�øߵ�ƽ
		HAL_GPIO_WritePin(AIN20_GPIO_Port, AIN20_Pin , GPIO_PIN_RESET); 	//PD7�õ͵�ƽ�����÷���Ϊ��ת
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,	Speed);		//PWM����Ϊ�ٶ�ֵ
	} else {								//���򣬼����÷�ת���ٶ�ֵ
		HAL_GPIO_WritePin(AIN10_GPIO_Port,AIN10_Pin, GPIO_PIN_RESET);	//PD6�øߵ�ƽ
		HAL_GPIO_WritePin(AIN20_GPIO_Port, AIN20_Pin , GPIO_PIN_SET); 	//PD7�õ͵�ƽ�����÷���Ϊ��ת
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,-Speed);				//PWM����Ϊ�����ٶ�ֵ����Ϊ��ʱ�ٶ�ֵΪ��������PWMֻ�ܸ�����
	}
}

/**
  * ��    ����ֱ�����2�����ٶ�
  * ��    ����Speed Ҫ���õ��ٶȣ���Χ��-100~100
  * �� �� ֵ����
  */
void Motor2_SetSpeed(int Speed) {
	if (Speed >= 0) {						//���������ת���ٶ�ֵ
		HAL_GPIO_WritePin(BIN20_GPIO_Port, BIN20_Pin, GPIO_PIN_SET);	//PD6�øߵ�ƽ
		HAL_GPIO_WritePin(BIN10_GPIO_Port, BIN10_Pin, GPIO_PIN_RESET); 	//PD7�õ͵�ƽ�����÷���Ϊ��ת
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,	Speed);			//PWM����Ϊ�ٶ�ֵ
	} else {								//���򣬼����÷�ת���ٶ�ֵ
		HAL_GPIO_WritePin(BIN10_GPIO_Port, BIN10_Pin, GPIO_PIN_RESET);	//PD6�øߵ�ƽ
		HAL_GPIO_WritePin(BIN20_GPIO_Port,BIN20_Pin, GPIO_PIN_SET); 	//PD7�õ͵�ƽ�����÷���Ϊ��ת
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,	-Speed);				//PWM����Ϊ�����ٶ�ֵ����Ϊ��ʱ�ٶ�ֵΪ��������PWMֻ�ܸ�����
	}
}

/**
  * ��    ����ֱ�����2�����ٶ�
  * ��    ����Speed Ҫ���õ��ٶȣ���Χ��-100~100
  * �� �� ֵ����
  */
void Motor3_SetSpeed(int  Speed) {
	if (Speed >= 0) {						//���������ת���ٶ�ֵ
		HAL_GPIO_WritePin(AIN2_GPIO_Port ,AIN2_Pin, GPIO_PIN_SET);	//PD6�øߵ�ƽ
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin, GPIO_PIN_RESET); 	//PD7�õ͵�ƽ�����÷���Ϊ��ת
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,	Speed);					//PWM����Ϊ�ٶ�ֵ
	} else {								//���򣬼����÷�ת���ٶ�ֵ
		HAL_GPIO_WritePin(AIN2_GPIO_Port , AIN2_Pin, GPIO_PIN_RESET);	//PD6�øߵ�ƽ
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin, GPIO_PIN_SET); 	//PD7�õ͵�ƽ�����÷���Ϊ��ת
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,	-Speed);				//PWM����Ϊ�����ٶ�ֵ����Ϊ��ʱ�ٶ�ֵΪ��������PWMֻ�ܸ�����
	}
}

/**
  * ��    ����ֱ�����2�����ٶ�
  * ��    ����Speed Ҫ���õ��ٶȣ���Χ��-100~100
  * �� �� ֵ����
  */
void Motor4_SetSpeed(int  Speed) {
	if (Speed >= 0) {						//���������ת���ٶ�ֵ
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);	//PD6�øߵ�ƽ
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET); 	//PD7�õ͵�ƽ�����÷���Ϊ��ת
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,	Speed);				//PWM����Ϊ�ٶ�ֵ
	} else {								//���򣬼����÷�ת���ٶ�ֵ
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);	//PD6�øߵ�ƽ
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET); 	//PD7�õ͵�ƽ�����÷���Ϊ��ת
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,	-Speed);			//PWM����Ϊ�����ٶ�ֵ����Ϊ��ʱ�ٶ�ֵΪ��������PWMֻ�ܸ�����
	}
}


