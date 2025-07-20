#ifndef __JY61_H
#define __JY61_H

#include "main.h"

typedef struct
{
    int16_t Angle[3];  // Roll, Pitch, Yaw
    int16_t T;         // ÎÂ¶È
} __attribute__((packed)) SAngle;

void JY61_Init(UART_HandleTypeDef *huart);
void JY61_ReceiveHandler(uint8_t byte);
float JY61_GetYaw(void);
float JY61_GetPitch(void);
float JY61_GetRoll(void);

extern SAngle stcAngle;

#endif
