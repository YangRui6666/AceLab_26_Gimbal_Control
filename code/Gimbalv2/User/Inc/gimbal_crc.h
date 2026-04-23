#ifndef GIMBAL_CRC_H
#define GIMBAL_CRC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t gimbal_crc16_modbus(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif
