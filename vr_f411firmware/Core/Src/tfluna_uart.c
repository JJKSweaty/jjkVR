#include "tfluna_uart.h"

bool TfLunaUart_Feed(TfLunaUartParser *parser, uint8_t byte,
                     uint16_t *distance_cm, uint16_t *amplitude)
{
  if (parser->index < 2U && byte != 0x59U)
  {
    parser->index = 0U;
    return false;
  }

  parser->frame[parser->index++] = byte;
  if (parser->index < sizeof(parser->frame))
  {
    return false;
  }

  uint8_t checksum = 0U;
  for (uint32_t index = 0U; index < sizeof(parser->frame) - 1U; index++)
  {
    checksum = (uint8_t)(checksum + parser->frame[index]);
  }
  parser->index = 0U;
  if (checksum != parser->frame[sizeof(parser->frame) - 1U])
  {
    return false;
  }

  *distance_cm = (uint16_t)parser->frame[2] |
                 ((uint16_t)parser->frame[3] << 8);
  *amplitude = (uint16_t)parser->frame[4] |
               ((uint16_t)parser->frame[5] << 8);
  return true;
}
