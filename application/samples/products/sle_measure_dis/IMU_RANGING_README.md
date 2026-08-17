# SLE IMU + ranging firmware

This directory builds two firmware variants from the same source tree.

## Battery board with IMU

In HiSpark configuration select:

- `SAMPLE_SUPPORT_SLE_MEASURE_DIS_CLIENT=y`
- `SAMPLE_SUPPORT_SLE_MEASURE_DIS_SERVER=n`

The Client samples the sensors at 20 Hz, runs a 9-axis Mahony attitude filter, and stores samples in a
32-entry overwrite-oldest ring. A single SSAP transmit task sends IQ first, then up to two four-sample IMU
batches after each complete IQ report. `SLEM_PROFILE_MSG_IMU_BATCH` therefore carries four timestamped samples.

Default wiring inherited from `icm42607p_lowpower_test`:

- ICM42607P: software SPI, CLK 14, CS 13, DI 12, DO 11
- QMC5883P: I2C0, SCL 22, SDA 23, address `0x2c`

## USB board connected to the computer

In HiSpark configuration select:

- `SAMPLE_SUPPORT_SLE_MEASURE_DIS_SERVER=y`
- `SAMPLE_SUPPORT_SLE_MEASURE_DIS_CLIENT=n`

The Server gives IQ/distance processing its own higher-priority task. IMU batches are emitted by a separate
lower-priority task as binary frames on the same debug serial port; they are no longer expanded to CSV.
Each frame is: `0xA55A` (little-endian uint16), type `1` (uint8), version `1` (uint8), payload length
(little-endian uint16), batch sequence (little-endian uint32), then one raw `sle_imu_batch_t` payload.

The payload starts with `sequence` (uint32), `sample_count` (uint16), and `dropped_samples` (uint16), followed
by four raw `sle_imu_packet_t` records. Each record contains exactly these fields in target-native little-endian
layout:

```text
timestamp_us,roll,pitch,yaw,ax,ay,az,gx,gy,gz,mx,my,mz,q0,q1,q2,q3,temp
```

Distance records are separate lines:

```text
DIST,timestamp_us,distance_m,calculation_time_ms
```

`dropped_samples` reports client-side ring-buffer overwrites since the preceding transmitted batch.

## Units and first hardware checks

- timestamp: microseconds from the battery board TCXO since boot
- roll/pitch/yaw: degrees
- ax/ay/az: g
- gx/gy/gz: degrees per second
- mx/my/mz: QMC5883P raw counts pending board-specific calibration
- q0/q1/q2/q3: unit quaternion, scalar first
- temp: degrees Celsius

The constants and axis directions in `imu/imu_sensor.c` must be checked on the physical IMU carrier. Keep the
board still and level first: acceleration should be close to `(0, 0, +/-1)`, gyro close to zero, and the
quaternion norm close to one. Magnetometer hard/soft-iron calibration and gyro bias calibration are still
required for accurate yaw.
