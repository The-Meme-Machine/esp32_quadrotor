# esp32_quadrotor

## Hardware
- [ESP32-S3 board](https://www.waveshare.com/wiki/ESP32-S3-Zero) (S3 chip is essential for real-time filtering)
- ESC with DSHOT support
- Adafruit LSM6DSOX accelerometer + gyroscope
- Adafruit LIS3MDL magnetometer or [combo unit](https://www.adafruit.com/product/4517)
- TBS ublox M-10Q GPS 

## Dependencies
- [ESP-DSP Library](https://github.com/espressif/esp-dsp) (must download manually and add to PROJECTDIR/components/.)

### Build
The project is configured to be build using Platform.io.

## Configuration
DSHOT frequency can be changed in the DSHOT.h file.
I2C addresses and registers can be changed in the IMU.h file.

## Useful Documents
- [ESP32-S3 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
- [LIS3MDL Datasheet](https://www.st.com/resource/en/datasheet/lis3mdl.pdf)
- [LSM6DSOX Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsox.pdf) 
- [DSHOT - the missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)