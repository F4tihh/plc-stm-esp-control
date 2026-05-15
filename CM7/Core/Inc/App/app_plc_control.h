#ifndef APP_PLC_CONTROL_H
#define APP_PLC_CONTROL_H

#include <stdint.h>

void app_plc_start_fixed_mode_from_esp(void);
void app_plc_stop_from_esp(void);
void app_plc_apply_live_speed(void);
void app_plc_apply_live_torque(void);

void app_plc_reset_meter(void);
void app_plc_set_fixed_speed_torque_distance(uint16_t fixed_speed, uint16_t fixed_torque, uint16_t distance);

int app_plc_read_one_holding_register(uint16_t raw_reg_addr, uint16_t *out_value);
int app_plc_read_one_coil(uint16_t raw_coil_addr, uint8_t *out_state);
void app_plc_diagnostic_read_all(void);

void app_plc_motion_test_once(void);

#endif
