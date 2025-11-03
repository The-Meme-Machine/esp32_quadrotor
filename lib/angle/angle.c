#include <angle.h>

// PID Gains
float angle_Kp_roll = 0.8f;
float angle_Ki_roll = 0.01f;
float angle_Kd_roll = 0.05f;

float angle_Kp_pitch = 0.8f;
float angle_Ki_pitch = 0.01f;
float angle_Kd_pitch = 0.05f;

float angle_Kp_yaw = 0.8f;
float angle_Ki_yaw = 0.01f;
float angle_Kd_yaw = 0.05f;

// Requires cascades PID loops with rate controllers
float angle_PID_roll_control(float roll_setpoint, float roll_measured)
{
    // // PID Loop
    // // Axes are uncoupled, so three separate PIDs are adequate
    float roll_output = 0.0f;

    // return roll output
    return roll_output;
}

float angle_PID_pitch_control(float pitch_setpoint, float pitch_measured)
{
    // Similar implementation as roll control
    float pitch_output = 0.0f;
    return pitch_output;
}

float angle_PID_yaw_control(float yaw_setpoint, float yaw_measured)
{
    // Similar implementation as roll control
    float yaw_output = 0.0f;
    return yaw_output;
}