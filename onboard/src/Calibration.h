// Calibration of magnetometer and gyroscope
// Performs one iteration of recursive least squares (RLS) to determine calibration parameters for IMU
// Procedure:
//  1. Start with pupper at rest. A short beep indicates the gyroscope bias is being determined. (5 seconds)
//  2. After medium beep, rotate pupper so that it's belly follows the surface of a sphere. Magnetometer additive bias will be determined now. (45 seconds)
//  3. After short beep, repeat step 2. Magnetometer multiplicative bias will be determined now. (45 seconds)
//  4. After a long beep, calibration is complete. 
//
// Note: Teensy must be restarted to run Calibration more than once 

// #pragma once
#ifndef _PUPPER_CALIBRATION_HH_
#define _PUPPER_CALIBRATION_HH_

#include <BasicLinearAlgebra.h>
#include <array>
#include <EEPROM.h>
#include <BasicLinearAlgebra.h>
#include <math.h>

#define ITER_0 5000 // Iteration to end gryroscope calibration
#define ITER_1 50000 // Iteration to end translation calibration
#define ITER_2 95000 // Iteration to end scaling calibration
#define T_LONG_BEEP 3000 // ms of long beep to indicate claibration complete

bool CalibrateIMU(std::array<float, 6> raw_data);
void StoreParams(std::array<float, 9> params);
std::array<float,9> ReadParams();
void Beep(int k, int iter_0, int iter_f); // Beep bios speaker 
void BeepLow(int k = 1, int iter_0 = 0, int iter_f = 2); // Beep bios speaker 
float CheckCalMag(std::array<float, 3> mag_data, std::array<float, 9> calib_params); // Returns the mean of the magnetometer magnitude

#endif