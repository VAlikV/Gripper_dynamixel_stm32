#ifndef __DYNAMIXEL_STM32_H
#define __DYNAMIXEL_STM32_H

#include "main.h"
#include <stdint.h>

/*
 * DYNAMIXEL MX-28, Protocol 2.0
 * MX-28T / MX-28AT: TTL half-duplex UART.
 * MX-28R / MX-28AR: RS-485, напрямую к UART STM32 не подключается.
 */

#define DXL_DEFAULT_TIMEOUT_MS        100
#define DXL_BROADCAST_ID             0xFE

/* Protocol 2.0 instructions */
#define DXL_INST_PING                0x01
#define DXL_INST_READ                0x02
#define DXL_INST_WRITE               0x03
#define DXL_INST_STATUS              0x55

/* MX-28 Protocol 2.0 Control Table */
#define DXL_ADDR_ID                  7
#define DXL_ADDR_BAUD_RATE           8
#define DXL_ADDR_RETURN_DELAY_TIME   9
#define DXL_ADDR_OPERATING_MODE      11
#define DXL_ADDR_TORQUE_ENABLE       64
#define DXL_ADDR_LED                 65
#define DXL_ADDR_STATUS_RETURN_LEVEL 68
#define DXL_ADDR_GOAL_VELOCITY       104
#define DXL_ADDR_PROFILE_ACCEL       108
#define DXL_ADDR_PROFILE_VELOCITY    112
#define DXL_ADDR_GOAL_POSITION       116
#define DXL_ADDR_PRESENT_LOAD    	 126
#define DXL_ADDR_PRESENT_VELOCITY    128
#define DXL_ADDR_PRESENT_POSITION    132
#define DXL_ADDR_PRESENT_INPUT_VOLT  144
#define DXL_ADDR_PRESENT_TEMP        146
#define DXL_ADDR_P		             84
#define DXL_ADDR_D		             80

/* Operating modes */
#define DXL_MODE_CURRENT             0
#define DXL_MODE_VELOCITY            1
#define DXL_MODE_POSITION            3
#define DXL_MODE_EXT_POSITION        4
#define DXL_MODE_CURRENT_BASED_POS   5
#define DXL_MODE_PWM                 16

#define DXL_LOBYTE(x)                ((uint8_t)(((uint16_t)(x)) & 0xFF))
#define DXL_HIBYTE(x)                ((uint8_t)((((uint16_t)(x)) >> 8) & 0xFF))

HAL_StatusTypeDef dynamixel_uart_send(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t length);

uint8_t dynamixel_send_packet_v2(UART_HandleTypeDef *huart, uint8_t id, uint8_t instruction,
                                 const uint8_t *params, uint16_t params_len);

uint8_t dynamixel_read_status_packet_v2(UART_HandleTypeDef *huart, uint8_t *packet,
                                        uint16_t packet_max_len, uint16_t *packet_len);

uint16_t dynamixel_update_crc(uint16_t crc_accum, const uint8_t *data_blk_ptr, uint16_t data_blk_size);

uint8_t dynamixel_ping(UART_HandleTypeDef *huart, uint8_t id);
uint8_t dynamixel_write(UART_HandleTypeDef *huart, uint8_t id, uint16_t address,
                        const uint8_t *data, uint16_t data_len);
uint8_t dynamixel_read(UART_HandleTypeDef *huart, uint8_t id, uint16_t address,
                       uint16_t data_len, uint8_t *out_data, uint16_t *out_len);

uint8_t dynamixel_set_led(UART_HandleTypeDef *huart, uint8_t id, uint8_t enable);
uint8_t dynamixel_set_torque_enable(UART_HandleTypeDef *huart, uint8_t id, uint8_t enable);
uint8_t dynamixel_set_operating_mode(UART_HandleTypeDef *huart, uint8_t id, uint8_t mode);
uint8_t dynamixel_set_profile_acceleration(UART_HandleTypeDef *huart, uint8_t id, uint32_t acceleration);
uint8_t dynamixel_set_profile_velocity(UART_HandleTypeDef *huart, uint8_t id, uint32_t velocity);
uint8_t dynamixel_set_goal_position(UART_HandleTypeDef *huart, uint8_t id, int32_t position);
uint8_t dynamixel_read_present_position(UART_HandleTypeDef *huart, uint8_t id, int32_t *position);
uint8_t dynamixel_read_present_moving_velocity(UART_HandleTypeDef *huart, uint8_t id, int32_t *speed);
uint8_t dynamixel_read_present_load(UART_HandleTypeDef *huart, uint8_t id, int32_t *load);

uint8_t dynamixel_set_p_value(UART_HandleTypeDef *huart, uint8_t id, uint16_t p);
uint8_t dynamixel_set_d_value(UART_HandleTypeDef *huart, uint8_t id, uint16_t d);

#endif /* __DYNAMIXEL_STM32_H */
