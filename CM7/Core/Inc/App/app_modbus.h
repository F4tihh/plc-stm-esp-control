#ifndef APP_MODBUS_H
#define APP_MODBUS_H

#include <stdint.h>

int app_modbus_eth_link_ready(void);
int app_modbus_open_socket(void);
int app_modbus_write_single_register(uint16_t reg_addr, uint16_t value);
int app_modbus_write_single_coil(uint16_t coil_addr, uint8_t state);
int app_modbus_write_single_register_on_socket(int sock, uint16_t reg_addr, uint16_t value);
int app_modbus_write_single_coil_on_socket(int sock, uint16_t coil_addr, uint8_t state);
int app_modbus_read_float_holding(uint16_t raw_reg_addr, float *out_value);
int app_modbus_read_float_holding_on_socket(int sock, uint16_t raw_reg_addr, float *out_value);

int app_modbus_read_all_registers(void);

#endif
