#include <stdint.h>
#include <esp_timer.h>

// PID controller structure for rate control
typedef struct
{
    // Controller gains
    float kp;
    float ki;
    float kd;

    // Setpoint (desired rate)
    float setpoint;

    // Error tracking
    float integral;
    float prev_error;

    // Output limits
    float out_min;
    float out_max;

    // Timing for derivative and integral calculation
    int64_t last_compute_time; // Last computation time in microseconds

} pid_controller_t;

extern void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float out_min, float out_max);

extern float pid_compute(pid_controller_t *pid, float setpoint, float measured_value);
