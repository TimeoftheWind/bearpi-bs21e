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

The Server receives four-sample IMU batches, calculates distance locally, and expands each batch back into
individual records on its debug serial port. Existing Windows-side CSV parsing can therefore continue to use
exactly 18 comma-separated columns per IMU sample:

```text
timestamp,roll,pitch,yaw,ax,ay,az,gx,gy,gz,mx,my,mz,q0,q1,q2,q3,temp
```

Distance records are separate lines:

```text
DIST,timestamp_us,distance_m,calculation_time_ms
```

If the client ring overwrites old samples, the server additionally prints:

```text
IMU_DROP,batch_sequence,dropped_sample_count
```

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
