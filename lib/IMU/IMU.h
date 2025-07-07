// Remember to configure CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2 true
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// #define LOG_LOCAL_LEVEL ESP_LOG_INFO // log verbosity

#define IMU_ADDR 0x6A
#define MAG_ADDR 0x1C

#define IMU_WHOAMI 0b01101100
#define MAG_WHOAMI 0b00111101

#define FIXED_POINT 1000 // 3 decimal places preserved

// Below definitions are for 1.66kHz, 4g, 500dps
// Sensor noise (for Kalman Filter)
#define RATE_NOISE 3.8 // mdps / sqrt(Hz)
#define ACCEL_NOISE 75 // ug / sqrt(Hz) @ 4g

// Zero level
#define RATE_ZERO 1000 // mdps
#define ACCEL_ZERO 20  // mg

// Sensitivity
#define RATE_SENS 17.50  // mdps / LSB
#define ACCEL_SENS 0.122 // mg / LSB

typedef struct
{
    // Gyro data
    int16_t g_x; //
    int16_t g_y;
    int16_t g_z;

    // Accelerometer data
    int16_t xl_x;
    int16_t xl_y;
    int16_t xl_z;

} IMU_packet;

typedef struct
{
    // Mag data
    int16_t m_x;
    int16_t m_y;
    int16_t m_z;

} mag_packet;

extern void setup_imu_mag(gpio_num_t sda, gpio_num_t slc);
extern void write_to_buf(i2c_master_dev_handle_t dev, uint8_t addr, uint8_t val);
extern IMU_packet *read_IMU();
extern mag_packet *read_mag();