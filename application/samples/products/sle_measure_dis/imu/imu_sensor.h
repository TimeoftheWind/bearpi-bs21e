#ifndef SLE_IMU_SENSOR_H
#define SLE_IMU_SENSOR_H

#include "errcode.h"
#include "../app_imu_protocol.h"

errcode_t sle_imu_sensor_init(void);
errcode_t sle_imu_sensor_read(sle_imu_packet_t *packet);

#endif
