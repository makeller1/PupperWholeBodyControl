//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
// FROM: https://github.com/kriswiner/MPU9250/blob/master/MPU9250_BME280_SPIFlash_Ladybug/MadgwickFilter.ino
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"
#include <math.h>

// For Serial
#include "Arduino.h"

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	512.0f		// sample frequency in Hz (Note: I use dt instead)
#define betaDef		0.1f		// 2 * proportional gain was: .1 kris suggests .60

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

//====================================================================================================
// Functions
#define SERIAL_PORT Serial
//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
void MadgwickKrisUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // For debugging
            // Serial.print(mx,7);
            // Serial.print(", ");
            // Serial.print(my,7);
            // Serial.print(", ");
            // Serial.print(mz,7);
            // Serial.println(";");
		// 	// Convert gyroscope degrees/sec to radians/sec
			gx *= 0.0174533f;
			gy *= 0.0174533f;
			gz *= 0.0174533f;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q0;
            float _2q2 = 2.0f * q1;
            float _2q3 = 2.0f * q2;
            float _2q4 = 2.0f * q3;
            float _2q1q3 = 2.0f * q0 * q2;
            float _2q3q4 = 2.0f * q2 * q3;
            float q1q1 = q0 * q0;
            float q1q2 = q0 * q1;
            float q1q3 = q0 * q2;
            float q1q4 = q0 * q3;
            float q2q2 = q1 * q1;
            float q2q3 = q1 * q2;
            float q2q4 = q1 * q3;
            float q3q3 = q2 * q2;
            float q3q4 = q2 * q3;
            float q4q4 = q3 * q3;

            // Normalise accelerometer measurement
            norm = sqrtf(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrtf(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q0 * mx;
            _2q1my = 2.0f * q0 * my;
            _2q1mz = 2.0f * q0 * mz;
            _2q2mx = 2.0f * q1 * mx;
            hx = mx * q1q1 - _2q1my * q3 + _2q1mz * q2 + mx * q2q2 + _2q2 * my * q2 + _2q2 * mz * q3 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q3 + my * q1q1 - _2q1mz * q1 + _2q2mx * q2 - my * q2q2 + my * q3q3 + _2q3 * mz * q3 - my * q4q4;
            _2bx = sqrtf(hx * hx + hy * hy);
            _2bz = -_2q1mx * q2 + _2q1my * q1 + mz * q1q1 + _2q2mx * q3 - mz * q2q2 + _2q3 * my * q3 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q2 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q1 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q1 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s1;
            qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s2;
            qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s3;
            qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s4;

            // Integrate to yield quaternion
            q0 += qDot1 * dt;
            q1 += qDot2 * dt;
            q2 += qDot3 * dt;
            q3 += qDot4 * dt;
            norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);    // normalise quaternion
            norm = 1.0f/norm;
            q0 *= norm;
            q1 *= norm;
            q2 *= norm;
            q3 *= norm;
}



//====================================================================================================
// END OF CODE
//====================================================================================================
