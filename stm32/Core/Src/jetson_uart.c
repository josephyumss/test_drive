#include "jetson_uart.h"

#include <string.h>

#include "board_config.h"
#include "main.h"

extern UART_HandleTypeDef huart1;

static uint8_t rx_byte;
static volatile uint16_t rx_write_index;
static volatile uint16_t rx_read_index;
static volatile uint16_t rx_overflow_count;
static uint8_t rx_buffer[AMR_UART_RX_BUFFER_SIZE];
static uint8_t tx_buffer[AMR_MAX_FRAME_SIZE];
static volatile bool tx_busy;

void JetsonUart_Init(void)
{
    rx_write_index = 0U;
    rx_read_index = 0U;
    rx_overflow_count = 0U;
    tx_busy = false;
    (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

void JetsonUart_ProcessRx(AmrApp *app, uint32_t now_ms)
{
    while (rx_read_index != rx_write_index) {
        uint8_t byte = rx_buffer[rx_read_index];
        rx_read_index = (uint16_t)(
            (rx_read_index + 1U) % AMR_UART_RX_BUFFER_SIZE
        );
        (void)AmrApp_ProcessRxByte(app, byte, now_ms);
    }
}

bool JetsonUart_Transmit(const uint8_t *data, size_t size)
{
    if ((data == NULL)
        || (size == 0U)
        || (size > sizeof(tx_buffer))
        || tx_busy) {
        return false;
    }

    memcpy(tx_buffer, data, size);
    tx_busy = true;
    if (HAL_UART_Transmit_IT(&huart1, tx_buffer, (uint16_t)size)
        != HAL_OK) {
        tx_busy = false;
        return false;
    }
    return true;
}

uint16_t JetsonUart_GetOverflowCount(void)
{
    return rx_overflow_count;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint16_t next = (uint16_t)(
            (rx_write_index + 1U) % AMR_UART_RX_BUFFER_SIZE
        );
        if (next == rx_read_index) {
            rx_overflow_count++;
        } else {
            rx_buffer[rx_write_index] = rx_byte;
            rx_write_index = next;
        }
        (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        tx_busy = false;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        (void)HAL_UART_AbortReceive_IT(&huart1);
    }
}

void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
    }
}
