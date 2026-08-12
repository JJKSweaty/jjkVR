#ifndef TFLUNA_UART_H
#define TFLUNA_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint8_t frame[9];
  uint8_t index;
} TfLunaUartParser;

bool TfLunaUart_Feed(TfLunaUartParser *parser, uint8_t byte,
                     uint16_t *distance_cm, uint16_t *amplitude);

#endif
