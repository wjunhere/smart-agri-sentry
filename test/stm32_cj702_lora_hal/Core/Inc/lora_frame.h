#ifndef __LORA_FRAME_H
#define __LORA_FRAME_H

#include <stdint.h>
#include "cj702.h"

#define LORA_FRAME_HEADER_0 0xAA
#define LORA_FRAME_HEADER_1 0x55
#define LORA_MSG_DATA       0x01
#define LORA_MSG_ERROR      0xFF
#define LORA_ERROR_TIMEOUT  0x01
#define LORA_ERROR_INCOMPLETE 0x02

typedef enum {
    LORA_OK = 0,
    LORA_ERR_NULL = -1,
    LORA_ERR_NO_SPACE = -2,
} lora_frame_status_t;

uint8_t crc8_maxim(const uint8_t *data, uint16_t len);
int lora_frame_pack_data(uint8_t device_id, const cj702_data_t *avg, uint8_t *buf, uint16_t buf_size);
int lora_frame_pack_error(uint8_t device_id, uint8_t error_code, uint8_t *buf, uint16_t buf_size);

#endif
