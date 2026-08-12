#include "tfluna_uart.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
  TfLunaUartParser parser = {0};
  uint16_t distance_cm = 0U;
  uint16_t amplitude = 0U;
  const uint8_t frame[9] = {0x59U, 0x59U, 0x7BU, 0x00U, 0xC8U,
                            0x01U, 0x00U, 0x00U, 0xF6U};

  assert(!TfLunaUart_Feed(&parser, 0x00U, &distance_cm, &amplitude));
  for (uint32_t index = 0U; index < 8U; index++)
  {
    assert(!TfLunaUart_Feed(&parser, frame[index], &distance_cm, &amplitude));
  }
  assert(TfLunaUart_Feed(&parser, frame[8], &distance_cm, &amplitude));
  assert(distance_cm == 123U);
  assert(amplitude == 456U);
  puts("tfluna_uart_test: passed");
  return 0;
}
