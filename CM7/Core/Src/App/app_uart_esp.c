#include "app_uart_esp.h"
#include "app_hw.h"

#include <string.h>

void app_uart_esp_send(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), HAL_MAX_DELAY);
}
