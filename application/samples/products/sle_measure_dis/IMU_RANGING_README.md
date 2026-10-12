# SLE IMU + ranging firmware

This directory builds two firmware variants from the same source tree.

## Battery board with IMU

In HiSpark configuration select:

- `SAMPLE_SUPPORT_SLE_MEASURE_DIS_CLIENT=y`
- `SAMPLE_SUPPORT_SLE_MEASURE_DIS_SERVER=n`

The Client samples the sensors at 20 Hz and stores the acceleration, gyroscope and magnetometer data in a
32-entry overwrite-oldest ring. A single SSAP transmit task always gives pending IQ data priority, while
four-sample IMU batches are sent independently as soon as enough samples are available. Failed IMU batches
remain pending for retry instead of being discarded. `SLEM_PROFILE_MSG_IMU_BATCH` carries four timestamped samples.

Default wiring inherited from `icm42607p_lowpower_test`:

- ICM42607P: software SPI, CLK 14, CS 13, DI 12, DO 11
- QMC5883P: I2C0, SCL 22, SDA 23, address `0x2c`

## USB board connected to the computer

In HiSpark configuration select:

- `SAMPLE_SUPPORT_SLE_MEASURE_DIS_SERVER=y`
- `SAMPLE_SUPPORT_SLE_MEASURE_DIS_CLIENT=n`

The Server gives IQ/distance processing its own higher-priority task. Distance results and IMU batches enter
separate fixed-size queues and are emitted by one lower-priority telemetry task, with distance results taking
priority. This prevents concurrent log writers from corrupting CSV records. Each IMU record has these fields:

```text
IMU,batch_sequence,sample_sequence,timestamp_us,ax,ay,az,gx,gy,gz,mx,my,mz
```

Distance records contain the local IQ report arrival time and algorithm result time in the Server clock domain:

```text
DIST,distance_sequence,measurement_timestamp_us,result_timestamp_us,distance_m,calculation_time_ms
```

Client-side ring-buffer overwrites are reported as `IMU_DROP,batch_sequence,dropped_sample_count`.

## Units and first hardware checks

- IMU timestamp: sensor-read start time in microseconds from the battery board TCXO since boot
- ax/ay/az: g
- gx/gy/gz: degrees per second
- mx/my/mz: QMC5883P raw counts pending board-specific calibration

The constants and axis directions in `imu/imu_sensor.c` must be checked on the physical IMU carrier. Keep the
board still and level first: acceleration should be close to `(0, 0, +/-1)`, gyro close to zero, and the
Magnetometer hard/soft-iron calibration and gyro bias calibration are still required for calibrated data.
