#include <rate.h>

static inline float clamp(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    // Set last compute time to current time to start fresh
    pid->last_compute_time = esp_timer_get_time();
}

/**
 * @brief Computes the PID output.
 */
float pid_compute(pid_controller_t *pid, float setpoint, float measured_value)
{

    // Get current time
    int64_t current_time = esp_timer_get_time();

    // Calculate time delta in seconds
    float delta_t = (float)(current_time - pid->last_compute_time) / 1000000.0f;

    // Handle timer overflow or first run
    if (delta_t <= 0.0f)
    {
        delta_t = 1e-6; // Set to a very small positive number to avoid division by zero
    }
    pid->last_compute_time = current_time;

    // Update setpoint
    pid->setpoint = setpoint;

    // --- Proportional Term ---
    float error = pid->setpoint - measured_value;
    float p_term = pid->kp * error;

    // --- Integral Term (with anti-windup) ---
    // Accumulate integral
    pid->integral += pid->ki * error * delta_t;
    // Clamp integral to prevent windup
    pid->integral = clamp(pid->integral, pid->out_min, pid->out_max);
    float i_term = pid->integral;

    // --- Derivative Term (with basic filtering) ---
    // Note: A more robust implementation might filter the derivative
    float derivative = (error - pid->prev_error) / delta_t;
    float d_term = pid->kd * derivative;

    // --- Combine Terms & Clamp Output ---
    float output = p_term + i_term + d_term;
    output = clamp(output, pid->out_min, pid->out_max);

    // --- Save state for next iteration ---
    pid->prev_error = error;

    return output;
}