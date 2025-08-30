#include <angle.h>

// PID Loop
// Axes are uncoupled, so three separate PIDs are adequate
static float roll_integral = 0.0f, pitch_integral = 0.0f, yaw_integral = 0.0f;
static float prev_roll_error = 0.0f, prev_pitch_error = 0.0f, prev_yaw_error = 0.0f;

// Setpoints (desired angles/rates)
float roll_setpoint = 0.0f;
float pitch_setpoint = 0.0f;
float yaw_setpoint = 0.0f;

// Measured values (replace with actual sensor fusion output if available)
float roll_measured = g_x;
float pitch_measured = g_y;
float yaw_measured = g_z;

// PID gains (tune as needed)
const float Kp = 0.8f, Ki = 0.01f, Kd = 0.05f;

// Calculate errors
float roll_error = roll_setpoint - roll_measured;
float pitch_error = pitch_setpoint - pitch_measured;
float yaw_error = yaw_setpoint - yaw_measured;

// Integrate errors
roll_integral += roll_error;
pitch_integral += pitch_error;
yaw_integral += yaw_error;

// Derivative terms
float roll_derivative = roll_error - prev_roll_error;
float pitch_derivative = pitch_error - prev_pitch_error;
float yaw_derivative = yaw_error - prev_yaw_error;

// PID outputs
float roll_output = Kp * roll_error + Ki * roll_integral + Kd * roll_derivative;
float pitch_output = Kp * pitch_error + Ki * pitch_integral + Kd * pitch_derivative;
float yaw_output = Kp * yaw_error + Ki * yaw_integral + Kd * yaw_derivative;

// Save errors for next iteration
prev_roll_error = roll_error;
prev_pitch_error = pitch_error;
prev_yaw_error = yaw_error;
