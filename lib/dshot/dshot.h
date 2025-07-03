#include <driver/rmt_tx.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_attr.h>

#define RMT_CLK_SRC RMT_CLK_SRC_APB
#define RMT_RESOLUTION_HZ 40 * 1000 * 1000 // 40MHz resolution (25ns per tick)
#define DSHOT_TICK_TIME 25                 // 25ns tick time
#define DSHOT_THROTTLE_MIN 48              // Minimum value of throttle
#define DSHOT_THROTTLE_MAX 2047            // Max value of throttle
#define NUM_MOTORS 4
#define RMT_SYNC false      // DO NOT USE RMT SYNC MANAGER AS IT ADDS START AND END PULSE
#define DSHOT_FRAME_SIZE 16 // 16 bit frame size

// DSHOT600 Timings
// #define DSHOT_BIT_1_HIGH 1250 // 1-bit: 1250ns high
// #define DSHOT_BIT_1_LOW 425   // 1-bit: 1250ns high
// #define DSHOT_BIT_0_HIGH 625  // 0-bit: 625ns high
// #define DSHOT_BIT_0_LOW 1050  // 0-bit: 625ns high
// #define DSHOT_BIT_TIME 1675   // Total bit time

// DSHOT300 Timings
#define DSHOT_BIT_1_HIGH 2500 // 1-bit: 1250ns high
#define DSHOT_BIT_1_LOW 825   // 1-bit: 1250ns high
#define DSHOT_BIT_0_HIGH 1250 // 0-bit: 625ns high
#define DSHOT_BIT_0_LOW 2075  // 0-bit: 625ns high
#define DSHOT_BIT_TIME 3325   // Total bit time

// DSHOT150 Timings
// #define DSHOT_BIT_1_HIGH 5000 // 1-bit: 1250ns high
// #define DSHOT_BIT_1_LOW 1675  // 1-bit: 1250ns high
// #define DSHOT_BIT_0_HIGH 2500 // 0-bit: 625ns high
// #define DSHOT_BIT_0_LOW 4175  // 0-bit: 625ns high
// #define DSHOT_BIT_TIME 6675   // Total bit time

// DSHOT command structure
// typedef union
// {
//     struct
//     {
//         uint16_t throttle_value : 11;
//         uint16_t telemetry_request : 1;
//         uint16_t checksum : 4;
//     };
//     uint16_t raw;
// } dshot_packet;

typedef struct
{
    uint16_t throttle_value : 11;
    uint16_t telemetry_request : 1;
    uint16_t checksum : 4;
    uint16_t raw : 16;
} dshot_packet;

// extern motor_channels setup_rmt_channels(gpio_num_t pins[NUM_MOTORS]);
extern void setup_rmt_channels(gpio_num_t pins[NUM_MOTORS]);

// extern void send_dshot_frame(uint16_t throttle[NUM_MOTORS], bool telemetry[NUM_MOTORS], motor_channels channels);
extern void send_dshot_frame(uint16_t (*throttle)[NUM_MOTORS], bool telemetry);