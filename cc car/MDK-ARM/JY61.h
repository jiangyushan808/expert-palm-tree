#ifndef __JY61_H
#define __JY61_H

#include "main.h"

typedef struct
{
    int16_t Angle[3];  // [0]Roll, [1]Pitch, [2]Yaw (µ¥Î»£º¡À32768 = ¡À180¡ã)
    int16_t T;         // ÎÂ¶È
} __attribute__((packed)) SAngle;

void JY61_Init(UART_HandleTypeDef *huart);
void JY61_ReceiveHandler(uint8_t byte);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
float JY61_GetYaw(void);
float JY61_GetPitch(void);
float JY61_GetRoll(void);

extern SAngle stcAngle;

#endif
