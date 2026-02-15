#include "esp01s.h"
#include <string.h>

static UART_HandleTypeDef *ESP_UART;

/* ===================== 中断接收相关变量 ===================== */
static uint8_t irq_rx_buf[ESP01S_IRQ_RX_BUF_SIZE];
static uint8_t irq_rx_idx = 0;
static ControlInput_t irq_ctrl_input;
static bool irq_data_ready = false;
static bool irq_tcp_connected = false;

/* ===================== 内部工具 ===================== */
static int parse_int(const char **p)
{
    int sign = 1;
    int val = 0;

    if (**p == '-') {
        sign = -1;
        (*p)++;
    }

    if (**p < '0' || **p > '9')
        return 0x7FFFFFFF;   // 非法标记

    while (**p >= '0' && **p <= '9') {
        val = val * 10 + (**p - '0');
        (*p)++;
    }
    return sign * val;
}

static ESP01S_Status_t ESP_SendCmd(const char *cmd, uint32_t wait_ms)
{
    HAL_UART_Transmit(ESP_UART, (uint8_t*)cmd, strlen(cmd), wait_ms);
    HAL_UART_Transmit(ESP_UART, (uint8_t*)"\r\n", 2, wait_ms);
    HAL_Delay(wait_ms);
    return ESP01S_OK;
}

static void parse_packet(const char *p)
{
    if (strncmp(p, "STOP", 4) == 0) {
        // 执行急停逻辑
        irq_ctrl_input.stick_x     = 0;
        irq_ctrl_input.stick_y     = 0;
        irq_ctrl_input.btn_up      = 0;
        irq_ctrl_input.btn_down    = 0;
        irq_ctrl_input.btn_yaw_cw  = 0;
        irq_ctrl_input.btn_yaw_ccw = 0;
        FlightControl_Disarm();
        irq_data_ready = true;
        return;
    }

    // 1️⃣ 必须以 "ROLL=" 开头，否则直接忽略
    if (strncmp(p, "ROLL=", 5) != 0)
        return;

    int roll=0, pitch=0, up=0, down=0, yaw_cw=0, yaw_ccw=0;

    // ROLL=
    if (strncmp(p, "ROLL=", 5) != 0) return;
    p += 5;
    roll = parse_int(&p);
    if (roll == 0x7FFFFFFF || *p++ != ';') return;

    // PITCH=
    if (strncmp(p, "PITCH=", 6) != 0) return;
    p += 6;
    pitch = parse_int(&p);
    if (pitch == 0x7FFFFFFF || *p++ != ';') return;

    // UP=
    if (strncmp(p, "UP=", 3) != 0) return;
    p += 3;
    up = parse_int(&p);
    if (*p++ != ';') return;

    // DOWN=
    if (strncmp(p, "DOWN=", 5) != 0) return;
    p += 5;
    down = parse_int(&p);
    if (*p++ != ';') return;

    // YAW_CW=
    if (strncmp(p, "YAW_CW=", 7) != 0) return;
    p += 7;
    yaw_cw = parse_int(&p);
    if (*p++ != ';') return;

    // YAW_CCW=
    if (strncmp(p, "YAW_CCW=", 8) != 0) return;
    p += 8;
    yaw_ccw = parse_int(&p);

    // ----------- 合法性检查 -----------
    if (roll  < -1000 || roll  > 1000) return;
    if (pitch < -1000 || pitch > 1000) return;

    // ----------- 写入控制结构 -----------
    irq_ctrl_input.stick_x     = roll  * 0.001f;
    irq_ctrl_input.stick_y     = pitch * 0.001f;
    irq_ctrl_input.btn_up      = (uint8_t)up;
    irq_ctrl_input.btn_down    = (uint8_t)down;
    irq_ctrl_input.btn_yaw_cw  = (uint8_t)yaw_cw;
    irq_ctrl_input.btn_yaw_ccw = (uint8_t)yaw_ccw;

    irq_data_ready = true;
}

/* ===================== 阻塞接口实现 ===================== */
ESP01S_Status_t ESP01S_Init(UART_HandleTypeDef *huart, uint32_t timeout_ms)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
    ESP_UART = huart;
    ESP_SendCmd("AT+RST", timeout_ms);
    return ESP01S_OK;
}

ESP01S_Status_t ESP01S_ConnectWiFi()
{
    // WIFI-3/dsJChangxin.
    // iPhone13mini/chengpei
    // ax30002601/chengpei
    ESP_SendCmd("AT+CWJAP=\"ax30002601\",\"chengpei\"", 10000);
    return ESP01S_OK;
}

ESP01S_Status_t ESP01S_ConnectTCP()
{
    ESP_SendCmd("AT+CIPSTART=\"TCP\",\"home.chengpei.top\",37775", 1000);
    ESP_SendCmd("AT+CIPMODE=1", 100);
    ESP_SendCmd("AT+CIPSEND", 100);
    irq_tcp_connected = true;
    return ESP01S_OK;
}

ESP01S_Status_t ESP01S_Send(const char *data, uint16_t len)
{
    if(HAL_UART_Transmit(ESP_UART, (uint8_t*)data, len, 1000) != HAL_OK) return ESP01S_ERROR;
    return ESP01S_OK;
}

ESP01S_Status_t ESP01S_Receive(char *buf, uint16_t buf_len, uint32_t timeout_ms)
{
    uint32_t tickstart = HAL_GetTick();
    uint16_t count = 0;

    while(HAL_GetTick() - tickstart < timeout_ms)
    {
        uint8_t byte;
        if(HAL_UART_Receive(ESP_UART, &byte, 1, 10) == HAL_OK)
        {
            if(count < buf_len-1) buf[count++] = byte;
        }
        if(count > 0 && buf[count-1] == '\n') break;
    }

    if(count > 0)
    {
        buf[count] = 0;
        return ESP01S_OK;
    }
    return ESP01S_TIMEOUT;
}

/* ===================== 中断接收实现 ===================== */
void ESP01S_IRQ_ProcessByte(uint8_t byte)
{
    if(irq_rx_idx < ESP01S_IRQ_RX_BUF_SIZE - 1)
    {
        irq_rx_buf[irq_rx_idx++] = byte;
        if(byte == '\n')  // 检测到换行
        {
            // 去掉可能存在的 \r
            if(irq_rx_idx >= 2 && irq_rx_buf[irq_rx_idx-2] == '\r')
                irq_rx_buf[irq_rx_idx-2] = '\0';
            else
                irq_rx_buf[irq_rx_idx-1] = '\0';

            parse_packet((char*)irq_rx_buf);
            irq_rx_idx = 0;
        }
    }
    else
    {
        irq_rx_idx = 0; // 溢出丢弃
    }
}

bool ESP01S_IRQ_DataReady(void)
{
    return irq_data_ready;
}

ControlInput_t ESP01S_IRQ_GetControlInput(void)
{
    irq_data_ready = false;
    return irq_ctrl_input;
}

bool ESP01S_IRQ_IsConnected(void)
{
    return irq_tcp_connected;
}
