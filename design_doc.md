# Design Document

## Goals
- Learning C.
- Designing controller, filters, estimators from scratch.
- Optimizing controller and filters for deployment on an embedded processor.
- Learning how to structure flight code.
- Reading open source code and understanding how they approached the same problems.
- Understand multi-core programming.

## Microcontroller
Trade study between the many ESP32-S3 boards sitting around my desk and a STM H723.
| Category | ESP32-S3 | STM32-H723 | Weight | Notes |
|:---------|:--------:|:----------:|:------:|:-----:|
|Physical Size| **23mm x 18mm** | 76mm x 42mm | Very High | Space is at a premium in a sub-5in quadrotor build.
|Architecture| Dual-core Xtensa LX7 | **Dual-core ARM Cortex-M7 + Cortex-M4** | Medium |
|Frequency| 240 MHz | **550 MHz + 240 MHz** | Medium | The faster core would be very useful in speeding the control loop. |
|FPU| Single-precision | **Double-precision** | Low | 32-bits floating point is sufficient as the chosen IMU outputs in two's complement. |
|DSP Acceleration| Yes | Yes | High | The only hope of calculating the KF at per-loop rate.|

While the additional hardware features of the H723 beat out the S3, the size of the development board makes it impractical to mount. Part of the fun of the project is fitting a full-state EKF into the control loop of the slower ESP32. 

## ESC
DSHOT (digital) motor control protocol precludes the need for calibration.

## Frame
A generic, perfectly axi-symmetric frame was chosen for the ease of analyzing plant dynamics (inertia tensor particularly).

## Mistakes
- Accessing the IMU over I2C consumes a little less than half of the control loop time (even at 1000kHz bus speed). SPI turned out to be the better choice.
- Manufacturing a single 20mm x 20mm board would have simpified mounting and bus integration (Also would have enabled SPI communication with IMU). 
- A brushed motor is much easier to model than a brushless one + ESC.