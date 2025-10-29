//=====================================================================================================
// Mahony.c
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

//---------------------------------------------------------------------------------------------------
// Header files

#include "alg_mahony.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	1000.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
//模块内使用函数
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root | 快速平方根倒数计算
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

static float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// 对外暴露接口
//====================================================================================================

/**
 * @file alg_mahony.h
 * @brief Manhony算法注册函数
 * @details 该函数用于注册一个Mahony算法实例，初始化相关参数。
 * @param config Mahony算法初始化结构体指针
 * @return MahonyInstance_s指针，注册成功返回指针，失败返回NULL
 * @date 2025-07-04 
 * @author Hu Wenxin
 */
MahonyInstance_s *Mahony_Register(MahonyInitConfig_s *config){
	if (config == NULL){
		return NULL; // 配置参数错误
	}
	MahonyInstance_s *mahony_instance = (MahonyInstance_s *)malloc(sizeof(MahonyInstance_s));
	memset(mahony_instance, 0, sizeof(MahonyInstance_s));
	mahony_instance->work_mode = config->work_mode;

	mahony_instance->twoKp = config->twoKp;
	mahony_instance->twoKi = config->twoKi;

	mahony_instance->q = (float*)config->q;
	mahony_instance->q_out = config->q_out;

	return mahony_instance;
}

// 源代码函数 void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
/**
 * @file alg_mahony.h
 * @brief Mahony九轴算法更新函数
 * @details 该函数用于更新Mahony算法的状态，计算四元数
 * @param mahony MahonyInstance_s类型的指针,Mahony算法实例指针
 * @param data MahonyData_AHRS_t类型的指针，包含陀螺仪、加速度计和磁力计数据
 * @return float* 成功返回指向四元数的指针，失败返回NULL
 * @date 2025-07-04
 * @author Hu Wenxin
 */
float* Mahony_Supdate_AHRS(MahonyInstance_s *mahony, MahonyData_AHRS_t *data){
	//检查确保 *data非空
	if (data == NULL) {
		return NULL; // 返回NULL表示错误
	}
	// 源代码的错误矫正希望用户输入错了也可以滤波 但是我们希望用户正确使用配置好的函数
	// // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	// if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
	// 	MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);
	// 	return;
	// }

	float gx = data->gyro[0];
	float gy = data->gyro[1];
	float gz = data->gyro[2];
	float ax = data->accel[0];
	float ay = data->accel[1];
	float az = data->accel[2];
	float mx = data->mag[0];
	float my = data->mag[1];
	float mz = data->mag[2];
	float *q = mahony->q;
	
	float recipNorm;
  	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
	{

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q[0] * q[0];
        q0q1 = q[0] * q[1];
        q0q2 = q[0] * q[2];
        q0q3 = q[0] * q[3];
        q1q1 = q[1] * q[1];
        q1q2 = q[1] * q[2];
        q1q3 = q[1] * q[3];
        q2q2 = q[2] * q[2];
        q2q3 = q[2] * q[3];
        q3q3 = q[3] * q[3];   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;

	mahony->q_out[0] = q[1];
	mahony->q_out[1] = q[2];
	mahony->q_out[2] = q[3];
	mahony->q_out[3] = q[0];
	return mahony->q_out;
}

// void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az)
/**
 * @file alg_mahony.h
 * @brief Mahony六轴算法更新函数
 * @details 该函数用于更新Mahony算法的状态，计算四元数
 * @param mahony MahonyInstance_s类型的指针，Mahony算法实例指针
 * @param data MahonyData_INS_t类型的指针，包含陀螺仪和加速度计数据
 * @return float* 成功返回指向四元数的指针，失败返回NULL
 * @date 2025-07-04
 * @author Hu Wenxin
 */
float* Mahony_Supdate_IMU(MahonyInstance_s *mahony, MahonyData_INS_t *data){
	//检查确保 *data非空
	if (data == NULL) {
		return NULL; // 返回NULL表示错误
	}
	
	float gx = data->gyro[0];
	float gy = data->gyro[1];
	float gz = data->gyro[2];
	float ax = data->accel[0];
	float ay = data->accel[1];
	float az = data->accel[2];
	float *q = mahony->q;

	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;                  
	q[3] *= recipNorm;	

	mahony->q_out[0] = q[1];
	mahony->q_out[1] = q[2];
	mahony->q_out[2] = q[3];
	mahony->q_out[3] = q[0];
	return mahony->q_out;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
