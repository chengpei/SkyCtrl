#ifndef __ESP01S_H
#define __ESP01S_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "flight_control.h"

typedef enum {
    ESP01S_OK = 0,
    ESP01S_ERROR,
    ESP01S_TIMEOUT
} ESP01S_Status_t;

// 初始化 UART，复位 ESP01S
ESP01S_Status_t ESP01S_Init(UART_HandleTypeDef *huart, uint32_t timeout_ms);

// 连接 Wi-Fi
ESP01S_Status_t ESP01S_ConnectWiFi();

// 连接 TCP 服务
ESP01S_Status_t ESP01S_ConnectTCP();

// 发送数据
ESP01S_Status_t ESP01S_Send(const char *data, uint16_t len);

// 接收数据到 buffer（阻塞）
ESP01S_Status_t ESP01S_Receive(char *buf, uint16_t buf_len, uint32_t timeout_ms);

/* ===================== 中断接收接口 ===================== */
#define ESP01S_IRQ_RX_BUF_SIZE 256

// UART 中断接收每个字节调用
void ESP01S_IRQ_ProcessByte(uint8_t byte);

// 是否有新的遥控数据
bool ESP01S_IRQ_DataReady(void);

// 获取最新的遥控数据
ControlInput_t ESP01S_IRQ_GetControlInput(void);

// TCP 是否已连接
bool ESP01S_IRQ_IsConnected(void);

#endif
