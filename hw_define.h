// MIT License

// Copyright (c) 2023 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __HW_DEFINE_H__
#define __HW_DEFINE_H__

#ifdef __cplusplus
extern "C" {
#endif

#define USE_MPU6050
// #define USE_MPU6500
// #define USE_AK8963


/* Step motor direction index */
#define MOTORLEFT_DIR_FORWARD       0
#define MOTORLEFT_DIR_BACKWARD      1
#define MOTORRIGHT_DIR_FORWARD      1
#define MOTORRIGHT_DIR_BACKWARD     0

/* Robot parameters */
#define WHEEL_RADIUS                0.033                                   /*!< Wheel radius in meter */
#define WHEEL_SEPARATION            0.165                                   /*!< Wheel separate distance in meter */
#define TURNING_RADIUS              0.08                                    /*!< Turning radius in degree */
#define ROBOT_RADIUS                0.1                                     /*!< Robot radius in meter    */
#define MAX_LINEAR_VELOCITY         (WHEEL_RADIUS * 2 * PI * 60 / 60)       /*!< Max linear velocity */
#define MAX_ANGULAR_VELOCITY        (MAX_LINEAR_VELOCITY / TURNING_RADIUS)  /*!< Max angular velocity */
#define MIN_LINEAR_VELOCITY         -MAX_LINEAR_VELOCITY                    /*!< Min linear velocity */
#define MIN_ANGULAR_VELOCITY        -MAX_ANGULAR_VELOCITY                   /*!< Min angular velocity */

/* Step driver parameters */
#define MICROSTEP_DIV               8           /*!< Step driver microstep divider */
#define NUM_PULSE_PER_ROUND         200         /*!< The number of pulse per round of motor */

#define PI                  3.14159265359

/* Convert constant */
#define DEG2RAD(x)      (x * PI / 180.0f)     /*!< Convert from degree to radian (PI/180) */
#define RAD2DEG(x)      (x * 180.0f / PI)     /*!< convert from radian to degree (180/PI) */

/*
 *  Convert from velocity (m/s) to frequency (Hz) for motor driver.
 *
 *                      2*pi*WHELL_RADIUS
 *  velocity (m/s) =  ------------------------------
 *                    NUM_PULSE_PER_ROUND * STEP_DIV
 *
 */
#define VEL2FREQ        ((NUM_PULSE_PER_ROUND*MICROSTEP_DIV)/(2*PI*WHEEL_RADIUS))

/* Convert motor tick to angular in radian */
#define TICK2RAD        360.0f/(NUM_PULSE_PER_ROUND*MICROSTEP_DIV)*PI/180.0f


#ifdef __cplusplus
}
#endif

#endif /* __HW_DEFINE_H__ */
