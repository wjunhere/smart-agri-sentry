#include "lora_frame.h"
#include <string.h>

#define SENSOR_PAYLOAD_LEN 24
#define ERROR_PAYLOAD_LEN  1
#define FRAME_OVERHEAD     5  // header(2) + device_id(1) + msg_type(1) + len(1)

uint8_t crc8_maxim(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static int pack_frame(uint8_t device_id, uint8_t msg_type,
                      const uint8_t *payload, uint8_t payload_len,
                      uint8_t *buf, uint16_t buf_size)
{
    if (buf == NULL || payload == NULL) {
        return LORA_ERR_NULL;
    }
    if (buf_size < FRAME_OVERHEAD + payload_len + 1) {
        return LORA_ERR_NO_SPACE;
    }
    buf[0] = LORA_FRAME_HEADER_0;
    buf[1] = LORA_FRAME_HEADER_1;
    buf[2] = device_id;
    buf[3] = msg_type;
    buf[4] = payload_len;
    memcpy(&buf[5], payload, payload_len);
    buf[5 + payload_len] = crc8_maxim(buf, 5 + payload_len);
    return FRAME_OVERHEAD + payload_len + 1;
}

int lora_frame_pack_data(uint8_t device_id, const cj702_data_t *avg, uint8_t *buf, uint16_t buf_size)
{
    if (avg == NULL) {
        return LORA_ERR_NULL;
    }
    uint8_t payload[SENSOR_PAYLOAD_LEN];
    payload[0]  = (uint8_t)(avg->co2 >> 8);
    payload[1]  = (uint8_t)(avg->co2);
    payload[2]  = (uint8_t)(avg->hcho >> 8);
    payload[3]  = (uint8_t)(avg->hcho);
    payload[4]  = (uint8_t)(avg->tvoc >> 8);
    payload[5]  = (uint8_t)(avg->tvoc);
    payload[6]  = (uint8_t)(avg->pm25 >> 8);
    payload[7]  = (uint8_t)(avg->pm25);
    payload[8]  = (uint8_t)(avg->pm10 >> 8);
    payload[9]  = (uint8_t)(avg->pm10);
    payload[10] = (uint8_t)(avg->temp >> 8);
    payload[11] = (uint8_t)(avg->temp);
    payload[12] = (uint8_t)(avg->humidity >> 8);
    payload[13] = (uint8_t)(avg->humidity);
    payload[14] = (uint8_t)(avg->soil_temp >> 8);
    payload[15] = (uint8_t)(avg->soil_temp);
    payload[16] = (uint8_t)(avg->soil_humidity >> 8);
    payload[17] = (uint8_t)(avg->soil_humidity);
    payload[18] = (uint8_t)(avg->ec >> 8);
    payload[19] = (uint8_t)(avg->ec);
    payload[20] = (uint8_t)(avg->leaf_wetness >> 8);
    payload[21] = (uint8_t)(avg->leaf_wetness);
    payload[22] = (uint8_t)(avg->leaf_temp >> 8);
    payload[23] = (uint8_t)(avg->leaf_temp);
    return pack_frame(device_id, LORA_MSG_DATA, payload, SENSOR_PAYLOAD_LEN, buf, buf_size);
}

int lora_frame_pack_error(uint8_t device_id, uint8_t error_code, uint8_t *buf, uint16_t buf_size)
{
    return pack_frame(device_id, LORA_MSG_ERROR, &error_code, ERROR_PAYLOAD_LEN, buf, buf_size);
}
