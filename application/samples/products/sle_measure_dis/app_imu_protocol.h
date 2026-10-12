#ifndef APP_IMU_PROTOCOL_H
#define APP_IMU_PROTOCOL_H

#include <stdint.h>
#include "multi_anchor_config.h"

#define SLEM_PROFILE_MSG_IMU 0xFFFFFFEBU
#define SLEM_PROFILE_MSG_IMU_BATCH 0xFFFFFFECU
#define SLEM_PROFILE_MSG_IMU_ACK 0xFFFFFFEDU
#define SLE_IMU_BATCH_SAMPLES IMU_BATCH_SAMPLE_COUNT
#define SLE_IMU_RING_CAPACITY IMU_SAMPLE_RING_SIZE

typedef struct {
    uint64_t timestamp_us;
    uint32_t sequence;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;
} sle_imu_packet_t;

typedef struct {
    uint32_t sequence;
    uint16_t sample_count;
    uint16_t dropped_samples;
    sle_imu_packet_t samples[SLE_IMU_BATCH_SAMPLES];
} sle_imu_batch_t;

/* ACK identifies both sequence and acquisition time to reject stale notifications. */
typedef struct {
    uint64_t last_timestamp_us;
    uint32_t sequence;
    uint32_t reserved;
} sle_imu_ack_t;

#endif
