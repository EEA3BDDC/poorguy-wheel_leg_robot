//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

//#include "tm_stm32_ahrs_imu.h"

#define AHRSIMU_PI              3.141592653f            /*!< PI definition */
#define AHRSIMU_RAD2DEG(x)      ((x) * 57.2957795f)     /*!< Radians to degrees converter */
#define AHRSIMU_DEG2RAD(x)      ((x) * 0.0174532925f)   /*!< Radians to degrees converter */

typedef struct _AHRSIMU_t {
    float Roll;             /*!< Roll angle value. This parameter is in units of degrees */
    float Pitch;            /*!< Pitch angle value. This parameter is in units of degrees */
    float Yaw;              /*!< Yaw angle value. This parameter is in units of degrees */
    float Inclination;      /*!< Inclination in units of degrees */
    
    float _beta;
    float _q0, _q1, _q2, _q3;
    float _sampleRate;
} AHRSIMU_t;

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(AHRSIMU_t* AHRSIMU, float gx, float gy, float gz, float ax, float ay, float az);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
