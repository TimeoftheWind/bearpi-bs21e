#ifndef APP_IMU_PROTOCOL_H
#define APP_IMU_PROTOCOL_H

#include <stdint.h>

#define SLEM_PROFILE_MSG_IMU 0xFFFFFFEBU
#define SLEM_PROFILE_MSG_IMU_BATCH 0xFFFFFFECU
#define SLE_IMU_BATCH_SAMPLES 4U
#define SLE_IMU_RING_CAPACITY 32U

typedef struct {
    uint64_t timestamp_us;
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

#endif
