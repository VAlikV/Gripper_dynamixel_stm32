#include <dynamixel_stm32_mx28_p2.h>
#include <string.h>

/*
 * 0 — WRITE-команды не ждут ответа. Удобно для первого запуска.
 * 1 — после каждой WRITE-команды читается Status Packet.
 */
#define DXL_WAIT_STATUS_AFTER_WRITE  0

static void dxl_make_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)((value >> 0)  & 0xFF);
    data[1] = (uint8_t)((value >> 8)  & 0xFF);
    data[2] = (uint8_t)((value >> 16) & 0xFF);
    data[3] = (uint8_t)((value >> 24) & 0xFF);
}

static void dxl_make_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)((value >> 0)  & 0xFF);
    data[1] = (uint8_t)((value >> 8)  & 0xFF);
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

    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET) {}

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    HAL_HalfDuplex_EnableReceiver(huart);
    return HAL_OK;
}

uint8_t dynamixel_send_packet_v2(UART_HandleTypeDef *huart, uint8_t id, uint8_t instruction,
                                 const uint8_t *params, uint16_t params_len)
{
    uint16_t packet_len = (uint16_t)(10 + params_len);
    if (packet_len > 256) return 0;

    uint8_t packet[256];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;

    uint16_t length_field = (uint16_t)(params_len + 3);
    packet[5] = DXL_LOBYTE(length_field);
    packet[6] = DXL_HIBYTE(length_field);
    packet[7] = instruction;

    for (uint16_t i = 0; i < params_len; i++) packet[8 + i] = params[i];

    uint16_t crc = dynamixel_update_crc(0, packet, packet_len - 2);
    packet[packet_len - 2] = DXL_LOBYTE(crc);
    packet[packet_len - 1] = DXL_HIBYTE(crc);

    return (dynamixel_uart_send(huart, packet, packet_len) == HAL_OK);
}

uint8_t dynamixel_read_status_packet_v2(UART_HandleTypeDef *huart, uint8_t *packet,
                                        uint16_t packet_max_len, uint16_t *packet_len)
{
    uint8_t header[7];

    /* Сначала читаем 7 байт, затем ровно LEN байт. Не ждём фиксированные 64 байта. */
    if (HAL_UART_Receive(huart, header, 7, DXL_DEFAULT_TIMEOUT_MS) != HAL_OK) return 0;

    if (header[0] != 0xFF || header[1] != 0xFF || header[2] != 0xFD || header[3] != 0x00) return 0;

    uint16_t length_field = (uint16_t)header[5] | ((uint16_t)header[6] << 8);
    uint16_t total_len = (uint16_t)(7 + length_field);
    if (total_len > packet_max_len) return 0;

    memcpy(packet, header, 7);

    if (HAL_UART_Receive(huart, &packet[7], length_field, DXL_DEFAULT_TIMEOUT_MS) != HAL_OK) return 0;

    uint16_t received_crc = (uint16_t)packet[total_len - 2] | ((uint16_t)packet[total_len - 1] << 8);
    uint16_t calculated_crc = dynamixel_update_crc(0, packet, total_len - 2);

    if (received_crc != calculated_crc) return 0;
    if (packet[7] != DXL_INST_STATUS) return 0;

    if (packet_len != NULL) *packet_len = total_len;
    return 1;
}

uint8_t dynamixel_ping(UART_HandleTypeDef *huart, uint8_t id)
{
    uint8_t packet[32];
    uint16_t packet_len = 0;

    if (!dynamixel_send_packet_v2(huart, id, DXL_INST_PING, NULL, 0)) return 0;
    if (!dynamixel_read_status_packet_v2(huart, packet, sizeof(packet), &packet_len)) return 0;
    if (packet[4] != id) return 0;
    if (packet[8] != 0x00) return 0;

    return 1;
}

uint8_t dynamixel_write(UART_HandleTypeDef *huart, uint8_t id, uint16_t address,
                        const uint8_t *data, uint16_t data_len)
{
    uint16_t params_len = (uint16_t)(2 + data_len);
    if (params_len > 128) return 0;

    uint8_t params[128];
    params[0] = DXL_LOBYTE(address);
    params[1] = DXL_HIBYTE(address);

    for (uint16_t i = 0; i < data_len; i++) params[2 + i] = data[i];

    if (!dynamixel_send_packet_v2(huart, id, DXL_INST_WRITE, params, params_len)) return 0;

#if DXL_WAIT_STATUS_AFTER_WRITE
    if (id != DXL_BROADCAST_ID)
    {
        uint8_t status[32];
        uint16_t status_len = 0;
        if (!dynamixel_read_status_packet_v2(huart, status, sizeof(status), &status_len)) return 0;
        if (status[4] != id || status[8] != 0x00) return 0;
    }
#endif

    return 1;
}

uint8_t dynamixel_read(UART_HandleTypeDef *huart, uint8_t id, uint16_t address,
                       uint16_t data_len, uint8_t *out_data, uint16_t *out_len)
{
    uint8_t params[4];
    params[0] = DXL_LOBYTE(address);
    params[1] = DXL_HIBYTE(address);
    params[2] = DXL_LOBYTE(data_len);
    params[3] = DXL_HIBYTE(data_len);

    if (!dynamixel_send_packet_v2(huart, id, DXL_INST_READ, params, 4)) return 0;

    uint8_t packet[128];
    uint16_t packet_len = 0;
    if (!dynamixel_read_status_packet_v2(huart, packet, sizeof(packet), &packet_len)) return 0;
    if (packet[4] != id || packet[8] != 0x00) return 0;

    uint16_t length_field = (uint16_t)packet[5] | ((uint16_t)packet[6] << 8);
    uint16_t params_returned = (uint16_t)(length_field - 4);
    if (params_returned > data_len) params_returned = data_len;

    for (uint16_t i = 0; i < params_returned; i++) out_data[i] = packet[9 + i];
    if (out_len != NULL) *out_len = params_returned;

    return 1;
}

uint8_t dynamixel_set_led(UART_HandleTypeDef *huart, uint8_t id, uint8_t enable)
{
    uint8_t data = enable ? 1 : 0;
    return dynamixel_write(huart, id, DXL_ADDR_LED, &data, 1);
}

uint8_t dynamixel_set_p_value(UART_HandleTypeDef *huart, uint8_t id, uint16_t p)
{
	uint8_t data[2];
	dxl_make_u16(data, (uint16_t)p);
    return dynamixel_write(huart, id, DXL_ADDR_P, data, 2);
}

uint8_t dynamixel_set_d_value(UART_HandleTypeDef *huart, uint8_t id, uint16_t d)
{
	uint8_t data[2];
	dxl_make_u16(data, (uint16_t)d);
    return dynamixel_write(huart, id, DXL_ADDR_D, data, 2);
}

uint8_t dynamixel_set_torque_enable(UART_HandleTypeDef *huart, uint8_t id, uint8_t enable)
{
    uint8_t data = enable ? 1 : 0;
    return dynamixel_write(huart, id, DXL_ADDR_TORQUE_ENABLE, &data, 1);
}

uint8_t dynamixel_set_operating_mode(UART_HandleTypeDef *huart, uint8_t id, uint8_t mode)
{
    return dynamixel_write(huart, id, DXL_ADDR_OPERATING_MODE, &mode, 1);
}

uint8_t dynamixel_set_profile_acceleration(UART_HandleTypeDef *huart, uint8_t id, uint32_t acceleration)
{
    uint8_t data[4];
    dxl_make_u32(data, acceleration);
    return dynamixel_write(huart, id, DXL_ADDR_PROFILE_ACCEL, data, 4);
}

uint8_t dynamixel_set_profile_velocity(UART_HandleTypeDef *huart, uint8_t id, uint32_t velocity)
{
    uint8_t data[4];
    dxl_make_u32(data, velocity);
    return dynamixel_write(huart, id, DXL_ADDR_PROFILE_VELOCITY, data, 4);
}

uint8_t dynamixel_set_goal_position(UART_HandleTypeDef *huart, uint8_t id, int32_t position)
{
    uint8_t data[4];
    dxl_make_u32(data, (uint32_t)position);
    return dynamixel_write(huart, id, DXL_ADDR_GOAL_POSITION, data, 4);
}

uint8_t dynamixel_read_present_position(UART_HandleTypeDef *huart, uint8_t id, int32_t *position)
{
    uint8_t data[4];
    uint16_t len = 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_PRESENT_POSITION, 4, data, &len)) return 0;
    if (len != 4) return 0;

    uint32_t raw = ((uint32_t)data[0] << 0)  |
                   ((uint32_t)data[1] << 8)  |
                   ((uint32_t)data[2] << 16) |
                   ((uint32_t)data[3] << 24);
    *position = (int32_t)raw;
    return 1;
}

uint8_t dynamixel_read_present_moving_velocity(UART_HandleTypeDef *huart, uint8_t id, int32_t *speed)
{
    uint8_t data[4];
    uint16_t len = 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_PRESENT_VELOCITY, 4, data, &len)) return 0;
    if (len != 4) return 0;

    uint32_t raw = ((uint32_t)data[0] << 0)  |
                   ((uint32_t)data[1] << 8)  |
                   ((uint32_t)data[2] << 16) |
                   ((uint32_t)data[3] << 24);
    *speed = (int32_t)raw;
    return 1;
}

uint8_t dynamixel_read_present_load(UART_HandleTypeDef *huart, uint8_t id, int32_t *load)
{
    uint8_t data[2];
    uint16_t len = 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_PRESENT_LOAD, 2, data, &len))
        return 0;

    if (len != 2)
        return 0;

    uint16_t raw = ((uint16_t)data[0] << 0) |
                   ((uint16_t)data[1] << 8);

    *load = (int32_t)((int16_t)raw);

    return 1;
}

uint16_t dynamixel_update_crc(uint16_t crc_accum, const uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
    static const uint16_t crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for (uint16_t j = 0; j < data_blk_size; j++)
    {
        uint16_t i = ((crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
