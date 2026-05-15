#include "app_debug.h"
#include "app_hw.h"

#include <string.h>

void app_debug_uart(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  HAL_UART_Transmit(&huart3, (uint8_t *)text, (uint16_t)strlen(text), HAL_MAX_DELAY);
}
