#include <esp_dsp.h>
#include <stdint.h>
#include <math.h>

#define mdps_to_radss (0.00001745329252f) // Convert mdps to rad/s

typedef struct
{
    // Filter gain
    float beta;

    // Sample period (1.0f / sample_frequency)
    float sample_period;

    // Quaternion components
    float q0;
    float q1;
    float q2;
    float q3;

    // Euler angles (calculated on demand)
    float roll;
    float pitch;
    float yaw;

} Madgwick_est;

extern void Madgwick_init(Madgwick_est *filter, float sample_frequency, float beta);
void Madgwick_update(Madgwick_est *filter, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
extern void Madgwick_updateIMU(Madgwick_est *filter, float gx, float gy, float gz, float ax, float ay, float az);
extern void Madgwick_computeEulerAngles(Madgwick_est *filter);