#ifndef APP_COMMAND_H
#define APP_COMMAND_H

#include <stdint.h>

typedef enum
{
  MB_CMD_NONE = 0,
  MB_CMD_START,
  MB_CMD_STOP,
  MB_CMD_SPEED_SET,
  MB_CMD_TORQUE_SET
} ModbusCommandType_t;

void app_command_init(void);

void app_command_post(ModbusCommandType_t cmd);
void app_command_post_stop(void);

ModbusCommandType_t app_command_peek(void);
ModbusCommandType_t app_command_take(void);

void app_command_set_staged_speed_guard(uint8_t on);
uint8_t app_command_take_staged_speed_guard(void);

void app_command_release_guard(void);
void app_command_set_active(uint8_t on);
uint8_t app_command_is_active(void);
uint8_t app_command_guard_elapsed(void);
void app_command_extend_guard_ms(uint32_t ms);

void app_command_set_skip_telemetry_once(uint8_t on);
uint8_t app_command_consume_skip_telemetry_once(void);

void app_command_worker_run(void);

int app_state_get_speed(void);
void app_state_set_speed(int speed);
int app_state_get_torque(void);
void app_state_set_torque(int torque);

uint8_t app_plc_is_running(void);
void app_plc_set_running(uint8_t on);
uint8_t app_plc_is_busy(void);
void app_plc_set_busy(uint8_t on);

uint32_t app_live_setpoint_last_ms(void);
void app_live_setpoint_touch(uint32_t now_ms);

#endif
