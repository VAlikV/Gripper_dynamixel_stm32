#include "dynamixel_stm32_ax12_p1.h"
#include <string.h>

/*
 * Set to 1 if you want WRITE commands to wait for a Status Packet.
 * For first bring-up, 0 is safer because it avoids blocking on every WRITE
 * if Status Return Level is configured to not respond to writes.
 */
#define DXL_WAIT_STATUS_AFTER_WRITE  0

static uint16_t clamp_u16(uint16_t value, uint16_t max_value)
{
    return (value > max_value) ? max_value : value;
}

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void make_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)((value >> 0) & 0xFF);
    data[1] = (uint8_t)((value >> 8) & 0xFF);
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return ((uint16_t)data[0] << 0) |
           ((uint16_t)data[1] << 8);
}

HAL_StatusTypeDef dynamixel_uart_send(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef ret;

    HAL_HalfDuplex_EnableTransmitter(huart);

    ret = HAL_UART_Transmit(huart, (uint8_t *)data, length, DXL_DEFAULT_TIMEOUT_MS);
    if (ret != HAL_OK)
    {
        HAL_HalfDuplex_EnableReceiver(huart);
        return ret;
    }

    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET)
    {
    }

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    HAL_HalfDuplex_EnableReceiver(huart);
    return HAL_OK;
}

uint8_t dynamixel_checksum_v1(const uint8_t *packet, uint8_t packet_len)
{
    /*
     * Protocol 1.0 packet:
     * FF FF ID LENGTH INSTRUCTION PARAMS... CHECKSUM
     * CHECKSUM = ~(ID + LENGTH + INSTRUCTION + PARAMS...)
     */
    uint16_t sum = 0;

    if (packet_len < 6)
        return 0;

    for (uint8_t i = 2; i < packet_len - 1; i++)
    {
        sum += packet[i];
    }

    return (uint8_t)(~sum & 0xFF);
}

uint8_t dynamixel_send_packet_v1(
    UART_HandleTypeDef *huart,
    uint8_t id,
    uint8_t instruction,
    const uint8_t *params,
    uint8_t params_len
)
{
    /*
     * FF FF ID LENGTH INSTRUCTION PARAMS... CHECKSUM
     * LENGTH = params_len + 2, where 2 = instruction + checksum
     */
    uint8_t packet_len = (uint8_t)(6 + params_len);

    if (packet_len > 128)
        return 0;

    uint8_t packet[128];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = (uint8_t)(params_len + 2);
    packet[4] = instruction;

    for (uint8_t i = 0; i < params_len; i++)
    {
        packet[5 + i] = params[i];
    }

    packet[packet_len - 1] = dynamixel_checksum_v1(packet, packet_len);

    return (dynamixel_uart_send(huart, packet, packet_len) == HAL_OK);
}

uint8_t dynamixel_read_status_packet_v1(
    UART_HandleTypeDef *huart,
    uint8_t *packet,
    uint8_t packet_max_len,
    uint8_t *packet_len
)
{
    uint8_t header[4];

    /*
     * Protocol 1.0 Status Packet:
     * FF FF ID LENGTH ERROR PARAMS... CHECKSUM
     * Total length = 4 + LENGTH
     */
    if (HAL_UART_Receive(huart, header, 4, DXL_DEFAULT_TIMEOUT_MS) != HAL_OK)
        return 0;

    if (header[0] != 0xFF || header[1] != 0xFF)
        return 0;

    uint8_t length_field = header[3];
    uint8_t total_len = (uint8_t)(4 + length_field);

    if (total_len > packet_max_len)
        return 0;

    memcpy(packet, header, 4);

    if (HAL_UART_Receive(huart, &packet[4], length_field, DXL_DEFAULT_TIMEOUT_MS) != HAL_OK)
        return 0;

    uint8_t expected = dynamixel_checksum_v1(packet, total_len);
    if (packet[total_len - 1] != expected)
        return 0;

    if (packet_len != NULL)
        *packet_len = total_len;

    return 1;
}

uint8_t dynamixel_ping(UART_HandleTypeDef *huart, uint8_t id)
{
    uint8_t packet[16];
    uint8_t packet_len = 0;

    if (!dynamixel_send_packet_v1(huart, id, DXL_INST_PING, NULL, 0))
        return 0;

    if (!dynamixel_read_status_packet_v1(huart, packet, sizeof(packet), &packet_len))
        return 0;

    if (packet[2] != id)
        return 0;

    /* packet[4] = ERROR */
    if (packet[4] != 0x00)
        return 0;

    return 1;
}

uint8_t dynamixel_write(
    UART_HandleTypeDef *huart,
    uint8_t id,
    uint8_t address,
    const uint8_t *data,
    uint8_t data_len
)
{
    /* WRITE params: START_ADDRESS DATA... */
    uint8_t params_len = (uint8_t)(1 + data_len);

    if (params_len > 64)
        return 0;

    uint8_t params[64];
    params[0] = address;

    for (uint8_t i = 0; i < data_len; i++)
    {
        params[1 + i] = data[i];
    }

    if (!dynamixel_send_packet_v1(huart, id, DXL_INST_WRITE, params, params_len))
        return 0;

#if DXL_WAIT_STATUS_AFTER_WRITE
    if (id != DXL_BROADCAST_ID)
    {
        uint8_t status[16];
        uint8_t status_len = 0;

        if (!dynamixel_read_status_packet_v1(huart, status, sizeof(status), &status_len))
            return 0;

        if (status[2] != id || status[4] != 0x00)
            return 0;
    }
#endif

    return 1;
}

uint8_t dynamixel_read(
    UART_HandleTypeDef *huart,
    uint8_t id,
    uint8_t address,
    uint8_t data_len,
    uint8_t *out_data,
    uint8_t *out_len
)
{
    uint8_t params[2];
    params[0] = address;
    params[1] = data_len;

    if (!dynamixel_send_packet_v1(huart, id, DXL_INST_READ, params, 2))
        return 0;

    uint8_t packet[64];
    uint8_t packet_len = 0;

    if (!dynamixel_read_status_packet_v1(huart, packet, sizeof(packet), &packet_len))
        return 0;

    if (packet[2] != id || packet[4] != 0x00)
        return 0;

    /* LENGTH = error + params + checksum, so params_len = LENGTH - 2 */
    uint8_t params_returned = (uint8_t)(packet[3] - 2);
    if (params_returned > data_len)
        params_returned = data_len;

    for (uint8_t i = 0; i < params_returned; i++)
    {
        out_data[i] = packet[5 + i];
    }

    if (out_len != NULL)
        *out_len = params_returned;

    return 1;
}

uint8_t dynamixel_action(UART_HandleTypeDef *huart)
{
    return dynamixel_send_packet_v1(huart, DXL_BROADCAST_ID, DXL_INST_ACTION, NULL, 0);
}

uint8_t dynamixel_factory_reset(UART_HandleTypeDef *huart, uint8_t id)
{
    return dynamixel_send_packet_v1(huart, id, DXL_INST_FACTORY_RESET, NULL, 0);
}

uint8_t dynamixel_set_led(UART_HandleTypeDef *huart, uint8_t id, uint8_t enable)
{
    uint8_t data = enable ? 1 : 0;
    return dynamixel_write(huart, id, DXL_ADDR_LED, &data, 1);
}

uint8_t dynamixel_set_torque_enable(UART_HandleTypeDef *huart, uint8_t id, uint8_t enable)
{
    uint8_t data = enable ? 1 : 0;
    return dynamixel_write(huart, id, DXL_ADDR_TORQUE_ENABLE, &data, 1);
}

uint8_t dynamixel_set_angle_limits(UART_HandleTypeDef *huart, uint8_t id, uint16_t cw_limit, uint16_t ccw_limit)
{
    uint8_t data[4];

    cw_limit = clamp_u16(cw_limit, DXL_AX12_MAX_POSITION);
    ccw_limit = clamp_u16(ccw_limit, DXL_AX12_MAX_POSITION);

    make_u16_le(&data[0], cw_limit);
    make_u16_le(&data[2], ccw_limit);

    return dynamixel_write(huart, id, DXL_ADDR_CW_ANGLE_LIMIT, data, 4);
}

uint8_t dynamixel_set_joint_mode(UART_HandleTypeDef *huart, uint8_t id)
{
    return dynamixel_set_angle_limits(huart, id, 0, DXL_AX12_MAX_POSITION);
}

uint8_t dynamixel_set_wheel_mode(UART_HandleTypeDef *huart, uint8_t id)
{
    return dynamixel_set_angle_limits(huart, id, 0, 0);
}

uint8_t dynamixel_set_goal_position(UART_HandleTypeDef *huart, uint8_t id, int32_t position)
{
    uint8_t data[2];
    uint16_t pos = (uint16_t)clamp_i32(position, 0, DXL_AX12_MAX_POSITION);

    make_u16_le(data, pos);
    return dynamixel_write(huart, id, DXL_ADDR_GOAL_POSITION, data, 2);
}

uint8_t dynamixel_set_moving_speed(UART_HandleTypeDef *huart, uint8_t id, int32_t speed)
{
    uint8_t data[2];

    /*
     * AX-12, Joint Mode:
     *   Moving Speed register is a 10-bit magnitude: 0...1023.
     *   Do not encode a sign bit here. Bit 10 is meaningful only in Wheel Mode.
     *
     * If you need signed wheel rotation, use dynamixel_set_wheel_speed().
     */
    uint16_t encoded = (uint16_t)clamp_i32(speed, 0, DXL_AX12_MAX_SPEED);

    make_u16_le(data, encoded);
    return dynamixel_write(huart, id, DXL_ADDR_MOVING_SPEED, data, 2);
}

uint8_t dynamixel_set_wheel_speed(UART_HandleTypeDef *huart, uint8_t id, int32_t speed)
{
    uint8_t data[2];
    uint16_t encoded;

    /*
     * AX-12, Wheel Mode:
     *   bits 0..9  = magnitude
     *   bit 10     = direction
     * Here positive is CCW, negative is CW.
     */
    if (speed < 0)
    {
        uint16_t magnitude = (uint16_t)clamp_i32(-speed, 0, DXL_AX12_MAX_SPEED);
        encoded = (uint16_t)(magnitude | 0x0400);
    }
    else
    {
        encoded = (uint16_t)clamp_i32(speed, 0, DXL_AX12_MAX_SPEED);
    }

    make_u16_le(data, encoded);
    return dynamixel_write(huart, id, DXL_ADDR_MOVING_SPEED, data, 2);
}

uint8_t dynamixel_set_position_and_velocity(UART_HandleTypeDef *huart, uint8_t id, int32_t position, int32_t velocity)
{
    uint8_t data[4];

    uint16_t pos = (uint16_t)clamp_i32(position, 0, DXL_AX12_MAX_POSITION);

    /*
     * This writes two adjacent AX-12 registers:
     *   30..31 = Goal Position
     *   32..33 = Moving Speed
     *
     * In Joint Mode, Moving Speed is a magnitude, not a signed value.
     * Therefore negative velocity is clamped to 0 instead of setting bit 10.
     */
    uint16_t vel = (uint16_t)clamp_i32(velocity, 0, DXL_AX12_MAX_SPEED);

    make_u16_le(&data[0], pos);
    make_u16_le(&data[2], vel);

    return dynamixel_write(huart, id, DXL_ADDR_GOAL_POSITION, data, 4);
}

uint8_t dynamixel_set_max_torque(UART_HandleTypeDef *huart, uint8_t id, uint16_t max_torque)
{
    uint8_t data[2];
    max_torque = clamp_u16(max_torque, DXL_AX12_MAX_TORQUE);
    make_u16_le(data, max_torque);
    return dynamixel_write(huart, id, DXL_ADDR_MAX_TORQUE, data, 2);
}

uint8_t dynamixel_set_torque_limit(UART_HandleTypeDef *huart, uint8_t id, uint16_t torque_limit)
{
    uint8_t data[2];
    torque_limit = clamp_u16(torque_limit, DXL_AX12_MAX_TORQUE);
    make_u16_le(data, torque_limit);
    return dynamixel_write(huart, id, DXL_ADDR_TORQUE_LIMIT, data, 2);
}

uint8_t dynamixel_set_id(UART_HandleTypeDef *huart, uint8_t id, uint8_t new_id)
{
    return dynamixel_write(huart, id, DXL_ADDR_ID, &new_id, 1);
}

uint8_t dynamixel_set_baudrate(UART_HandleTypeDef *huart, uint8_t id, uint8_t baudrate_value)
{
    return dynamixel_write(huart, id, DXL_ADDR_BAUD_RATE, &baudrate_value, 1);
}

uint8_t dynamixel_set_return_delay_time(UART_HandleTypeDef *huart, uint8_t id, uint8_t delay_time)
{
    /* AX-12 unit is 2 usec. */
    return dynamixel_write(huart, id, DXL_ADDR_RETURN_DELAY_TIME, &delay_time, 1);
}

uint8_t dynamixel_set_compliance_margin(UART_HandleTypeDef *huart, uint8_t id, uint8_t cw_margin, uint8_t ccw_margin)
{
    uint8_t data[2] = {cw_margin, ccw_margin};
    return dynamixel_write(huart, id, DXL_ADDR_CW_COMP_MARGIN, data, 2);
}

uint8_t dynamixel_set_compliance_slope(UART_HandleTypeDef *huart, uint8_t id, uint8_t cw_slope, uint8_t ccw_slope)
{
    uint8_t data[2] = {cw_slope, ccw_slope};
    return dynamixel_write(huart, id, DXL_ADDR_CW_COMP_SLOPE, data, 2);
}

uint8_t dynamixel_set_punch(UART_HandleTypeDef *huart, uint8_t id, uint16_t punch)
{
    uint8_t data[2];
    punch = clamp_u16(punch, 1023);
    make_u16_le(data, punch);
    return dynamixel_write(huart, id, DXL_ADDR_PUNCH, data, 2);
}

uint8_t dynamixel_read_present_position(UART_HandleTypeDef *huart, uint8_t id, int32_t *position)
{
    uint8_t data[2];
    uint8_t len = 0;

    if (position == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_PRESENT_POSITION, 2, data, &len))
        return 0;

    if (len != 2)
        return 0;

    *position = (int32_t)read_u16_le(data);
    return 1;
}

uint8_t dynamixel_read_present_moving_velocity(UART_HandleTypeDef *huart, uint8_t id, int32_t *velocity)
{
    uint8_t data[2];
    uint8_t len = 0;

    if (velocity == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_PRESENT_SPEED, 2, data, &len))
        return 0;

    if (len != 2)
        return 0;

    *velocity = dynamixel_ax12_decode_signed_speed(read_u16_le(data));
    return 1;
}

uint8_t dynamixel_read_present_load(UART_HandleTypeDef *huart, uint8_t id, int32_t *load)
{
    uint8_t data[2];
    uint8_t len = 0;

    if (load == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_PRESENT_LOAD, 2, data, &len))
        return 0;

    if (len != 2)
        return 0;

    *load = dynamixel_ax12_decode_signed_load(read_u16_le(data));
    return 1;
}

uint8_t dynamixel_read_present_voltage(UART_HandleTypeDef *huart, uint8_t id, int32_t *voltage_x10)
{
    uint8_t data[1];
    uint8_t len = 0;

    if (voltage_x10 == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_PRESENT_VOLTAGE, 1, data, &len))
        return 0;

    if (len != 1)
        return 0;

    /* AX-12 unit: 0.1 V. */
    *voltage_x10 = data[0];
    return 1;
}

uint8_t dynamixel_read_present_temperature(UART_HandleTypeDef *huart, uint8_t id, int32_t *temperature_c)
{
    uint8_t data[1];
    uint8_t len = 0;

    if (temperature_c == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_PRESENT_TEMP, 1, data, &len))
        return 0;

    if (len != 1)
        return 0;

    *temperature_c = data[0];
    return 1;
}

uint8_t dynamixel_read_moving(UART_HandleTypeDef *huart, uint8_t id, uint8_t *moving)
{
    uint8_t data[1];
    uint8_t len = 0;

    if (moving == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_MOVING, 1, data, &len))
        return 0;

    if (len != 1)
        return 0;

    *moving = data[0] & 0x01;
    return 1;
}

uint8_t dynamixel_read_id(UART_HandleTypeDef *huart, uint8_t id, uint8_t *read_id)
{
    uint8_t data[1];
    uint8_t len = 0;

    if (read_id == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_ID, 1, data, &len))
        return 0;

    if (len != 1)
        return 0;

    *read_id = data[0];
    return 1;
}

uint8_t dynamixel_read_baudrate(UART_HandleTypeDef *huart, uint8_t id, uint8_t *baudrate_value)
{
    uint8_t data[1];
    uint8_t len = 0;

    if (baudrate_value == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_BAUD_RATE, 1, data, &len))
        return 0;

    if (len != 1)
        return 0;

    *baudrate_value = data[0];
    return 1;
}

uint8_t dynamixel_baudrate_to_value(uint32_t baudrate)
{
    if (baudrate == 0)
        return 0;

    return (uint8_t)((2000000U / baudrate) - 1U);
}

uint32_t dynamixel_value_to_baudrate(uint8_t value)
{
    return (uint32_t)(2000000U / ((uint32_t)value + 1U));
}

uint16_t dynamixel_pos_deg_to_value(float pos_deg)
{
    if (pos_deg < 0.0f)
        pos_deg = 0.0f;

    if (pos_deg > 300.0f)
        pos_deg = 300.0f;

    return (uint16_t)(pos_deg * 1023.0f / 300.0f + 0.5f);
}

uint16_t dynamixel_vel_deg_s_to_value(float speed_deg_s)
{
    if (speed_deg_s < 0.0f)
        speed_deg_s = 0.0f;

    /*
     * AX-12 Moving Speed unit is 0.111 rpm.
     * 1 rpm = 6 deg/s, so one register unit is 0.111 * 6 = 0.666 deg/s.
     */
    uint16_t value = (uint16_t)(speed_deg_s / (0.111f * 6.0f) + 0.5f);
    return clamp_u16(value, DXL_AX12_MAX_SPEED);
}

uint16_t dynamixel_vel_pct_to_wheel_value(float speed_pct)
{
    if (speed_pct < 0.0f)
        speed_pct = 0.0f;

    if (speed_pct > 100.0f)
        speed_pct = 100.0f;

    return (uint16_t)(speed_pct * 1023.0f / 100.0f + 0.5f);
}

int32_t dynamixel_ax12_decode_signed_speed(uint16_t raw)
{
    int32_t magnitude = (int32_t)(raw & 0x03FF);

    /* Bit 10 is direction. 0 = CCW, 1 = CW. */
    if (raw & 0x0400)
        return -magnitude;

    return magnitude;
}

int32_t dynamixel_ax12_decode_signed_load(uint16_t raw)
{
    int32_t magnitude = (int32_t)(raw & 0x03FF);

    /* Bit 10 is direction. 0 = CCW, 1 = CW. */
    if (raw & 0x0400)
        return -magnitude;

    return magnitude;
}
