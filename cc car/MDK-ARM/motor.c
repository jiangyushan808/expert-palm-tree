#include "stm32f4xx_hal.h"                // Device header
#include "main.h"
#include "tim.h"


void Motor1_SetSpeed(int Speed) {
//	if (Speed >= 0) {						//如果设置正转的速度值
//		HAL_GPIO_WritePin(GPIOB, Apower1_Pin, GPIO_PIN_SET);	//PD6置高电平
//		HAL_GPIO_WritePin(GPIOA, Bpower1_Pin, GPIO_PIN_SET); 	//PD7置低电平，设置方向为正转
//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,	Speed);		//PWM设置为速度值
//	} else {								//否则，即设置反转的速度值
//		HAL_GPIO_WritePin(GPIOB, Bpower1_Pin, GPIO_PIN_RESET);	//PD6置高电平
//		HAL_GPIO_WritePin(GPIOA, Apower1_Pin, GPIO_PIN_RESET); 	//PD7置低电平，设置方向为正转
//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,-Speed);				//PWM设置为负的速度值，因为此时速度值为负数，而PWM只能给正数
//	}
	if (Speed >= 0) {						//如果设置正转的速度值
		HAL_GPIO_WritePin(AIN10_GPIO_Port, AIN10_Pin, GPIO_PIN_SET);	//PD6置高电平
		HAL_GPIO_WritePin(AIN20_GPIO_Port, AIN20_Pin , GPIO_PIN_RESET); 	//PD7置低电平，设置方向为正转
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,	Speed);		//PWM设置为速度值
	} else {								//否则，即设置反转的速度值
		HAL_GPIO_WritePin(AIN10_GPIO_Port,AIN10_Pin, GPIO_PIN_RESET);	//PD6置高电平
		HAL_GPIO_WritePin(AIN20_GPIO_Port, AIN20_Pin , GPIO_PIN_SET); 	//PD7置低电平，设置方向为正转
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,-Speed);				//PWM设置为负的速度值，因为此时速度值为负数，而PWM只能给正数
	}
}

/**
  * 函    数：直流电机2设置速度
  * 参    数：Speed 要设置的速度，范围：-100~100
  * 返 回 值：无
  */
void Motor2_SetSpeed(int Speed) {
	if (Speed >= 0) {						//如果设置正转的速度值
		HAL_GPIO_WritePin(BIN20_GPIO_Port, BIN20_Pin, GPIO_PIN_SET);	//PD6置高电平
		HAL_GPIO_WritePin(BIN10_GPIO_Port, BIN10_Pin, GPIO_PIN_RESET); 	//PD7置低电平，设置方向为正转
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,	Speed);			//PWM设置为速度值
	} else {								//否则，即设置反转的速度值
		HAL_GPIO_WritePin(BIN10_GPIO_Port, BIN10_Pin, GPIO_PIN_RESET);	//PD6置高电平
		HAL_GPIO_WritePin(BIN20_GPIO_Port,BIN20_Pin, GPIO_PIN_SET); 	//PD7置低电平，设置方向为正转
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,	-Speed);				//PWM设置为负的速度值，因为此时速度值为负数，而PWM只能给正数
	}
}

/**
  * 函    数：直流电机2设置速度
  * 参    数：Speed 要设置的速度，范围：-100~100
  * 返 回 值：无
  */
void Motor3_SetSpeed(int  Speed) {
	if (Speed >= 0) {						//如果设置正转的速度值
		HAL_GPIO_WritePin(AIN2_GPIO_Port ,AIN2_Pin, GPIO_PIN_SET);	//PD6置高电平
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin, GPIO_PIN_RESET); 	//PD7置低电平，设置方向为正转
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,	Speed);					//PWM设置为速度值
	} else {								//否则，即设置反转的速度值
		HAL_GPIO_WritePin(AIN2_GPIO_Port , AIN2_Pin, GPIO_PIN_RESET);	//PD6置高电平
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin, GPIO_PIN_SET); 	//PD7置低电平，设置方向为正转
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,	-Speed);				//PWM设置为负的速度值，因为此时速度值为负数，而PWM只能给正数
	}
}

/**
  * 函    数：直流电机2设置速度
  * 参    数：Speed 要设置的速度，范围：-100~100
  * 返 回 值：无
  */
void Motor4_SetSpeed(int  Speed) {
	if (Speed >= 0) {						//如果设置正转的速度值
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);	//PD6置高电平
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET); 	//PD7置低电平，设置方向为正转
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,	Speed);				//PWM设置为速度值
	} else {								//否则，即设置反转的速度值
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);	//PD6置高电平
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET); 	//PD7置低电平，设置方向为正转
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,	-Speed);			//PWM设置为负的速度值，因为此时速度值为负数，而PWM只能给正数
	}
}


