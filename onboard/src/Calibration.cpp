#include "Calibration.h"
#define BEEP_PIN 2 // 14 on pupper, 2 on single motor setup

bool CalibrateIMU(std::array<float, 6> raw_data)
{   // Inputs: raw_data - array with elements mag_x, mag_y, mag_z, gry_x, gry_y, gry_z
    //                    (units arbitrary)

    // Initial calibration parameters
    static float b_gyr_x = 0; // Gyroscope bias
    static float b_gyr_y = 0;
    static float b_gyr_z = 0; 
    static BLA::Matrix<4,1> x_hat = {0, 0, 0, 0}; // Magnetometer Translation b_mag_x, b_mag_y, b_mag_z
    static BLA::Matrix<3,1> x_hat2 = {0, 0, 0}; // m_mag_x, m_mag_y, m_mag_z Magnetometer Scaling

    static BLA::Matrix<4,4> Pk = {100,   0,   0,   0, 
                                    0, 100,   0,   0, 
                                    0,   0, 100,   0,
                                    0,   0,   0, 100};
    static BLA::Matrix<3,3> Pk2 = {100,   0,   0,
                                     0, 100,   0,
                                     0,   0, 100};
    const BLA::Matrix<4,4> I_4 = {1, 0, 0, 0, 
                                0, 1, 0, 0, 
                                0, 0, 1, 0,
                                0, 0, 0, 1};
    const BLA::Matrix<3,3> I_3 = {1, 0, 0, 
                                  0, 1, 0, 
                                  0, 0, 1};

    static int k = 1;

    if (k <= ITER_0)
    {
        Beep(k, 0, 200);
        // Perform GYRO calibration (recursive mean) to determine bias
        b_gyr_x = b_gyr_x + 1.0f/(float)k*(raw_data[3] - b_gyr_x);
        b_gyr_y = b_gyr_y + 1.0f/(float)k*(raw_data[4] - b_gyr_y);
        b_gyr_z = b_gyr_z + 1.0f/(float)k*(raw_data[5] - b_gyr_z);
    }
    
    if (k > ITER_0 && k <= ITER_1)
    {
        Beep(k, ITER_0, ITER_0 + 200);
        // Perform MAG translation calibration
        BLA::Matrix<1,4> Hk = {raw_data[0],raw_data[1], raw_data[2], 1};
        float yk = (raw_data[0]*raw_data[0]) + (raw_data[1]*raw_data[1]) + (raw_data[2]*raw_data[2]);
        BLA::Matrix<4,1> Kk = (Pk*~Hk)/((Hk*Pk*~Hk+1.0f)(0));
        Pk = (I_4 - Kk*Hk)*Pk;
        x_hat = x_hat + Kk*(yk - (Hk*x_hat)(0));
    }

    if (k > ITER_1 && k < ITER_2)
    {
        Beep(k, ITER_1, ITER_1 + 75);
        // Perform MAG scaling calibration
        BLA::Matrix<1,3> Hk2 = {(raw_data[0]-.5f*x_hat(0))*(raw_data[0]-.5f*x_hat(0)),
                                (raw_data[1]-.5f*x_hat(1))*(raw_data[1]-.5f*x_hat(1)),
                                (raw_data[2]-.5f*x_hat(2))*(raw_data[2]-.5f*x_hat(2))};
        BLA::Matrix<3,1> Kk2 = (Pk2*~Hk2)/((Hk2*Pk2*~Hk2+1)(0));
        Pk2 = (I_3 - Kk2*Hk2)*Pk2;
        x_hat2 = x_hat2 + Kk2*(1.0f - (Hk2*x_hat2)(0));
    }

    if (k == ITER_2) // Strict equality to prevent redundant EEPROM writing
    {
        // Determine translation parameters
        float b_mag_x = 0.5f*x_hat(0);
        float b_mag_y = 0.5f*x_hat(1);
        float b_mag_z = 0.5f*x_hat(2);

        // Determine scaling parameters
        float a = sqrtf(1.0f/x_hat2(0));
        float b = sqrtf(1.0f/x_hat2(1));
        float c = sqrtf(1.0f/x_hat2(2));
        float s_mag_x = b*c/(a*b*c);
        float s_mag_y = a*c/(a*b*c);
        float s_mag_z = a*b/(a*b*c);

        std::array<float, 9> params = {b_gyr_x, b_gyr_y, b_gyr_z, b_mag_x, b_mag_y, b_mag_z, s_mag_x, s_mag_y, s_mag_z};
        // Store calibration parameters in EEPROM
        StoreParams(params);
    }

    if (k >= ITER_2 && k <= ITER_2 + T_LONG_BEEP) 
    {
        // Indicate calibration complete
        Beep(k, ITER_2, ITER_2 + T_LONG_BEEP -1);
    }

    if (k > ITER_2 + T_LONG_BEEP)
    {
        // End calibration
        return true;
    }

    k++;
    return false;
}

void StoreParams(std::array<float, 9> params)
{
    int eeAddress = 0;
    // Store params in EEPROM
    for (int i = 0; i<=9; i++)
    {
        EEPROM.put(eeAddress, params[i]);
        eeAddress += sizeof(float);
    }
}

std::array<float,9> ReadParams()
{
    std::array<float,9> params;
    int eeAddress = 0;
    // Read params in EEPROM
    for (int i = 0; i<=9; i++)
    {   float f;
        EEPROM.get(eeAddress, f);
        params[i] = f;
        eeAddress += sizeof(float);
    }
    return params;
}

void Beep(int k, int iter_0, int iter_f)
{
    if (k >= iter_0 && k <= iter_f)
    {
        analogWrite(BEEP_PIN, 10); // Sound
    }
    else
    {
        analogWrite(BEEP_PIN, 0); // Silent
    }
}

void BeepLow(int k, int iter_0, int iter_f)
{
    if (k >= iter_0 && k <= iter_f)
    {
        analogWrite(BEEP_PIN, 5); // Sound
    }
    else
    {
        analogWrite(BEEP_PIN, 0); // Silent
    }
}

float CheckCalMag(std::array<float, 3> mag_data, std::array<float, 9> calib_params) // Returns the mean of the magnetometer magnitude
{
    // Tests if the calibration is still required by calculating the magnitude of the magnetic field.
    // Correct calibration should result in a value near 1.0 +/- .2 for any orientation.
    // static int k = 1;
    // static float r_mean = 0;

    float& b_mag_x = calib_params[3];
    float& b_mag_y = calib_params[4];
    float& b_mag_z = calib_params[5];
    float& s_mag_x = calib_params[6];
    float& s_mag_y = calib_params[7];
    float& s_mag_z = calib_params[8];

    float magnitude_k = pow((mag_data[0]-b_mag_x)*s_mag_x, 2) + pow((mag_data[1]-b_mag_y)*s_mag_y, 2) + pow((mag_data[2]-b_mag_z)*s_mag_z, 2);
    // r_mean = r_mean + 1.0/(float)k *(magnitude_k - r_mean);
    // k = k + 1;

    return magnitude_k;
}