#include "angle_utils.h"
#include "../app/user_config.h"

float wrap_neg_pi_to_pi(float angle_rad)
{
    while (angle_rad > (MATH_TWO_PI * 0.5f))
    {
        angle_rad -= MATH_TWO_PI;
    }

    while (angle_rad < -(MATH_TWO_PI * 0.5f))
    {
        angle_rad += MATH_TWO_PI;
    }

    return angle_rad;
}

float wrap_0_2pi(float angle_rad)
{
    while (angle_rad >= MATH_TWO_PI)
    {
        angle_rad -= MATH_TWO_PI;
    }

    while (angle_rad < 0.0f)
    {
        angle_rad += MATH_TWO_PI;
    }

    return angle_rad;
}

float angleUtils_compensate_delay(float angle_rad, float speed_rad_s, float delay_s)
{
    float compensated_angle = angle_rad + speed_rad_s * delay_s;
    return wrap_0_2pi(compensated_angle);
}
