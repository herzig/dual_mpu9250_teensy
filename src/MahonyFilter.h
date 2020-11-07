// Adapted from kriswiners version to be object oriented (to better support multiple sensors), original: https://github.com/kriswiner/MPU9250
// Ivo Herzig (2020)


// Implementation of Sebastian Madgwick's "...efficient orientation filter
// for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples & more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a
// quaternion-based estimate of absolute device orientation -- which can be
// converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional
// Kalman-based filtering algorithms but is much less computationally
// intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!


#ifndef _MAHONY_FILTER_H_
#define _MAHONY_FILTER_H_

#include <Arduino.h>

class MahonyFilter
{
private:

    // These are the free parameters in the Mahony filter and fusion scheme, Kp
    // for proportional feedback, Ki for integral
    const float Kp = 2.0f * 5.0f;
    const float Ki = 0.0f;

    // integral error for Mahony method
    float eInt[3] = {0.0f, 0.0f, 0.0f};

public:
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    MahonyFilter();

    void update(float ax, float ay, float az, float gx, float gy,
                float gz, float mx, float my, float mz,
                float deltat);

};

#endif