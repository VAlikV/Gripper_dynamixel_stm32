#ifndef __DYNAMIXEL_STM32_AX12_P1_FIXED_H
#define __DYNAMIXEL_STM32_AX12_P1_FIXED_H

#include "main.h"
#include <stdint.h>

/*
 * DYNAMIXEL AX-12 / AX-12A, Protocol 1.0
 *
 * API style is aligned with the previously implemented MX-28 Protocol 2.0 driver:
 * - public functions return uint8_t: 1 = success, 0 = failure
 * - read functions write data through output pointers
 * - UART is assumed to be configured as half-duplex
 *
 * TTL wiring:
 *   STM32 USARTx_TX in half-duplex mode -> DYNAMIXEL DATA
 *   STM32 GND                           -> DYNAMIXEL GND
 *   DYNAMIXEL V+                        -> external servo supply
 */

#define DXL_DEFAULT_TIMEOUT_MS        100
#define DXL_BROADCAST_ID             0xFE

/* Protocol 1.0 instructions */
#define DXL_INST_PING                0x01
#define DXL_INST_READ                0x02
#define DXL_INST_WRITE               0x03
#define DXL_INST_REG_WRITE           0x04
#define DXL_INST_ACTION              0x05
#define DXL_INST_FACTORY_RESET       0x06
#define DXL_INST_SYNC_WRITE          0x83

/* AX-12 Control Table: EEPROM area */
#define DXL_ADDR_MODEL_NUMBER        0
#define DXL_ADDR_FIRMWARE_VERSION    2
#define DXL_ADDR_ID                  3
#define DXL_ADDR_BAUD_RATE           4
#define DXL_ADDR_RETURN_DELAY_TIME   5
#define DXL_ADDR_CW_ANGLE_LIMIT      6
#define DXL_ADDR_CCW_ANGLE_LIMIT     8
#define DXL_ADDR_TEMP_LIMIT          11
#define DXL_ADDR_MIN_VOLTAGE_LIMIT   12
#define DXL_ADDR_MAX_VOLTAGE_LIMIT   13
#define DXL_ADDR_MAX_TORQUE          14
#define DXL_ADDR_STATUS_RETURN_LEVEL 16
#define DXL_ADDR_ALARM_LED           17
#define DXL_ADDR_SHUTDOWN            18

/* AX-12 Control Table: RAM area */
#define DXL_ADDR_TORQUE_ENABLE       24
#define DXL_ADDR_LED                 25
#define DXL_ADDR_CW_COMP_MARGIN      26
#define DXL_ADDR_CCW_COMP_MARGIN     27
#define DXL_ADDR_CW_COMP_SLOPE       28
#define DXL_ADDR_CCW_COMP_SLOPE      29
#define DXL_ADDR_GOAL_POSITION       30
#define DXL_ADDR_MOVING_SPEED        32
#define DXL_ADDR_TORQUE_LIMIT        34
#define DXL_ADDR_PRESENT_POSITION    36
#define DXL_ADDR_PRESENT_SPEED       38
#define DXL_ADDR_PRESENT_LOAD        40
#define DXL_ADDR_PRESENT_VOLTAGE     42
#define DXL_ADDR_PRESENT_TEMP        43
#define DXL_ADDR_REGISTERED          44
#define DXL_ADDR_MOVING              46
#define DXL_ADDR_LOCK                47
#define DXL_ADDR_PUNCH               48

/* AX-12 constants */
#define DXL_AX12_MAX_POSITION        1023
#define DXL_AX12_MAX_SPEED           1023
#define DXL_AX12_MAX_TORQUE          1023

#define DXL_CW_DIRECTION             0
#define DXL_CCW_DIRECTION            1

#define DXL_AX12_BAUD_1000000        1
#define DXL_AX12_BAUD_500000         3
#define DXL_AX12_BAUD_400000         4
#define DXL_AX12_BAUD_250000         7
#define DXL_AX12_BAUD_200000         9
#define DXL_AX12_BAUD_115200         16
#define DXL_AX12_BAUD_57600          34
#define DXL_AX12_BAUD_19200          103
#define DXL_AX12_BAUD_9600           207

/* Low-level functions */
HAL_StatusTypeDef dynamixel_uart_send(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t length);
uint8_t dynamixel_send_packet_v1(UART_HandleTypeDef *huart, uint8_t id, uint8_t instruction, const uint8_t *params, uint8_t params_len);
uint8_t dynamixel_read_status_packet_v1(UART_HandleTypeDef *huart, uint8_t *packet, uint8_t packet_max_len, uint8_t *packet_len);
uint8_t dynamixel_checksum_v1(const uint8_t *packet, uint8_t packet_len);

/* Common Protocol 1.0 operations */
uint8_t dynamixel_ping(UART_HandleTypeDef *huart, uint8_t id);
uint8_t dynamixel_write(UART_HandleTypeDef *huart, uint8_t id, uint8_t address, const uint8_t *data, uint8_t data_len);
uint8_t dynamixel_read(UART_HandleTypeDef *huart, uint8_t id, uint8_t address, uint8_t data_len, uint8_t *out_data, uint8_t *out_len);
uint8_t dynamixel_action(UART_HandleTypeDef *huart);
uint8_t dynamixel_factory_reset(UART_HandleTypeDef *huart, uint8_t id);

/* AX-12 helpers */
uint8_t dynamixel_set_led(UART_HandleTypeDef *huart, uint8_t id, uint8_t enable);
uint8_t dynamixel_set_torque_enable(UART_HandleTypeDef *huart, uint8_t id, uint8_t enable);
uint8_t dynamixel_set_joint_mode(UART_HandleTypeDef *huart, uint8_t id);
uint8_t dynamixel_set_wheel_mode(UART_HandleTypeDef *huart, uint8_t id);
uint8_t dynamixel_set_angle_limits(UART_HandleTypeDef *huart, uint8_t id, uint16_t cw_limit, uint16_t ccw_limit);
uint8_t dynamixel_set_goal_position(UART_HandleTypeDef *huart, uint8_t id, int32_t position);
uint8_t dynamixel_set_moving_speed(UART_HandleTypeDef *huart, uint8_t id, int32_t speed);
uint8_t dynamixel_set_position_and_velocity(UART_HandleTypeDef *huart, uint8_t id, int32_t position, int32_t velocity);
uint8_t dynamixel_set_max_torque(UART_HandleTypeDef *huart, uint8_t id, uint16_t max_torque);
uint8_t dynamixel_set_torque_limit(UART_HandleTypeDef *huart, uint8_t id, uint16_t torque_limit);
uint8_t dynamixel_set_id(UART_HandleTypeDef *huart, uint8_t id, uint8_t new_id);
uint8_t dynamixel_set_baudrate(UART_HandleTypeDef *huart, uint8_t id, uint8_t baudrate_value);
uint8_t dynamixel_set_return_delay_time(UART_HandleTypeDef *huart, uint8_t id, uint8_t delay_time);
uint8_t dynamixel_set_compliance_margin(UART_HandleTypeDef *huart, uint8_t id, uint8_t cw_margin, uint8_t ccw_margin);
uint8_t dynamixel_set_compliance_slope(UART_HandleTypeDef *huart, uint8_t id, uint8_t cw_slope, uint8_t ccw_slope);
uint8_t dynamixel_set_punch(UART_HandleTypeDef *huart, uint8_t id, uint16_t punch);

/* AX-12 read helpers */
uint8_t dynamixel_read_present_position(UART_HandleTypeDef *huart, uint8_t id, int32_t *position);
uint8_t dynamixel_read_present_moving_velocity(UART_HandleTypeDef *huart, uint8_t id, int32_t *velocity);
uint8_t dynamixel_read_present_load(UART_HandleTypeDef *huart, uint8_t id, int32_t *load);
uint8_t dynamixel_read_present_voltage(UART_HandleTypeDef *huart, uint8_t id, int32_t *voltage_x10);
uint8_t dynamixel_read_present_temperature(UART_HandleTypeDef *huart, uint8_t id, int32_t *temperature_c);
uint8_t dynamixel_read_moving(UART_HandleTypeDef *huart, uint8_t id, uint8_t *moving);
uint8_t dynamixel_read_id(UART_HandleTypeDef *huart, uint8_t id, uint8_t *read_id);
uint8_t dynamixel_read_baudrate(UART_HandleTypeDef *huart, uint8_t id, uint8_t *baudrate_value);

/* Utility */
uint8_t dynamixel_baudrate_to_value(uint32_t baudrate);
uint32_t dynamixel_value_to_baudrate(uint8_t value);
uint16_t dynamixel_pos_deg_to_value(float pos_deg);
uint16_t dynamixel_vel_deg_s_to_value(float speed_deg_s);
uint16_t dynamixel_vel_pct_to_wheel_value(float speed_pct);
int32_t dynamixel_ax12_decode_signed_speed(uint16_t raw);
int32_t dynamixel_ax12_decode_signed_load(uint16_t raw);

#endif /* __DYNAMIXEL_STM32_AX12_P1_FIXED_H */
