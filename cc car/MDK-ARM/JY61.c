#include "jy61.h"
#include <string.h>

UART_HandleTypeDef *jy61_huart;        // 保存串口句柄
uint8_t jy61_rx_byte;                  // 用于接收的单字节缓存
uint8_t jy61_buffer[11];               // 接收数据缓冲区（JY61P 每帧数据11字节）
uint8_t jy61_cnt = 0;                  // 缓冲区接收计数器

// 用于存储解析后的角度数据（Roll, Pitch, Yaw）
SAngle stcAngle;

/**
 * @brief 初始化 JY61 接收功能
 * @param huart 使用的串口句柄，比如 &huart1
 */
void JY61_Init(UART_HandleTypeDef *huart)
{
    jy61_huart = huart;
    HAL_UART_Receive_IT(jy61_huart, &jy61_rx_byte, 1);  // 开启串口中断，接收1字节
}

/**
 * @brief 串口中断回调中调用的接收处理函数
 * @param byte 刚收到的1字节数据
 */
void JY61_ReceiveHandler(uint8_t byte)
{
    // 如果还没接收到帧头 0x55，则丢弃前面所有数据
    if (jy61_cnt == 0 && byte != 0x55) return;

    jy61_buffer[jy61_cnt++] = byte;

    // 每帧完整数据长度为11字节
    if (jy61_cnt == 11)
    {
        // 判断是否是角度帧（0x55 0x53）
        if (jy61_buffer[0] == 0x55 && jy61_buffer[1] == 0x53)
        {
            // 从第3字节开始，连续8字节为 Roll, Pitch, Yaw, Temp（各2字节）
            memcpy(&stcAngle, &jy61_buffer[2], 8);
        }

        // 清空计数器，准备接收下一帧
        jy61_cnt = 0;
    }

    // 继续接收下一个字节（必须重新启动中断）
    HAL_UART_Receive_IT(jy61_huart, &jy61_rx_byte, 1);
}
//中断回调
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart == jy61_huart) 
//    {
//        JY61_ReceiveHandler(jy61_rx_byte);  // 处理接收到的1字节
//    }
//}

/**
 * @brief 获取 Yaw（偏航角，Z轴）单位为°
 */
float JY61_GetYaw(void)
{
    return (float)stcAngle.Angle[2] / 32768.0f * 180.0f;
}

/**
 * @brief 获取 Pitch（俯仰角，Y轴）单位为°
 */
float JY61_GetPitch(void)
{
    return (float)stcAngle.Angle[1] / 32768.0f * 180.0f;
}

/**
 * @brief 获取 Roll（横滚角，X轴）单位为°
 */
float JY61_GetRoll(void)
{
    return (float)stcAngle.Angle[0] / 32768.0f * 180.0f;
}
