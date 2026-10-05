#ifndef JETSON_UART_H
#define JETSON_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "amr_app.h"

void JetsonUart_Init(void);
void JetsonUart_ProcessRx(AmrApp *app, uint32_t now_ms);
bool JetsonUart_Transmit(const uint8_t *data, size_t size);
uint16_t JetsonUart_GetOverflowCount(void);

#endif
