#ifndef LITEWING_ESPNOW_H_
#define LITEWING_ESPNOW_H_

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LITEWING_ESPNOW_VERSION 1
#define LITEWING_COMMAND_MAGIC "LWCM"

#define LITEWING_COMMAND_FLAG_ARM        (1u << 0)
#define LITEWING_COMMAND_FLAG_DISARM     (1u << 1)
#define LITEWING_COMMAND_FLAG_ESTOP      (1u << 2)
#define LITEWING_COMMAND_FLAG_RESET_STOP (1u << 3)

typedef struct __attribute__((packed)) {
    char magic[4];
    uint8_t version;
    uint8_t flags;
    uint16_t sequence;
    int16_t roll_cdeg;
    int16_t pitch_cdeg;
    int16_t yaw_rate_cdeg_s;
    uint16_t thrust;
} litewing_command_packet_t;

esp_err_t litewingEspnowInit(void);

#ifdef __cplusplus
}
#endif

#endif
