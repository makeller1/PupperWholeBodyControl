//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
// Modified from: 
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#pragma once
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <array>
//----------------------------------------------------------------------------------------------------
// Variable declaration

float beta;				// algorithm gain
float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

std::array<float, 4> MadgwickKrisUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, double udt);
// void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, double dt);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
