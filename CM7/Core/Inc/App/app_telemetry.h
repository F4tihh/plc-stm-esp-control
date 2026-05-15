#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

#include <stdint.h>

void app_telemetry_init(void);
void app_telemetry_send_live_to_esp(void);
void app_telemetry_tick(uint32_t now_ms);

#endif
