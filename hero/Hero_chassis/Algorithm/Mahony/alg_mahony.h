//=====================================================================================================
// Mahony.h 照搬老代码移植来的库 最多做了二次封装以适配库
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
#ifndef __MAHONY_H
#define __MAHONY_H

#include <stdbool.h>
#include "main.h"
//----------------------------------------------------------------------------------------------------
//Mahony算法的工作模式枚举
typedef enum{
    Mahony_AHRS_MODE = 0, // 全部传感器工作模式,包含磁力计的九轴融合
    Mahony_IMU_MODE,      // IMU模式,只使用陀螺仪和加速度计的六轴融合
} Mahony_Work_Mode_e;

// TODO:注释的老代码等到封完之后再删
// extern volatile float twoKp;			// 2 * proportional gain (Kp)
// extern volatile float twoKi;			// 2 * integral gain (Ki)
// extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

/**
 * @brief 采用六轴融合的mahony算法数据结构体
 * @details 该结构体包含陀螺仪和加速度计数据，用于六轴融合的mahony算法。
 */
typedef struct {
    float gyro[3];     // 陀螺仪数据,xyz
    float accel[3];    // 加速度计数据,xyz
} MahonyData_INS_t; 

/**
 * @brief 采用九轴融合的mahony算法数据结构体
 * @details 该结构体包含陀螺仪、加速度计和磁力计数据，用于九轴融合的mahony算法。
 */
typedef struct {
    float gyro[3];     // 陀螺仪数据,xyz
    float accel[3];    // 加速度计数据,xyz
    float mag[3];      // 磁力计数据,xyz
} MahonyData_AHRS_t;

typedef struct{
    Mahony_Work_Mode_e work_mode; // 工作模式,全模式或IMU模式

    float twoKp; //2 * proportional gain (Kp)
    float twoKi; //2 * integral gain (Ki)
    float *q;  // quaternion of sensor frame relative to auxiliary frame / 需要被滤波的数据的指针

    MahonyData_AHRS_t *ahrs_data; // 九轴融合数据
    MahonyData_INS_t *ins_data;   // 六轴融合数据

    float *q_out; // 输出的四元数数据,xyz和w
} MahonyInstance_s;

typedef struct{
    Mahony_Work_Mode_e work_mode; // 工作模式,全模式或IMU模式
    float twoKp; //2 * proportional gain (Kp)
    float twoKi; //2 * integral gain (Ki)
    float *q;  // quaternion of sensor frame relative to auxiliary frame / 似乎是需要被滤波的四元数数据
    float *q_out; // 输出的四元数数据,xyz和w
} MahonyInitConfig_s;


//======================================================================================================
// 对外暴露接口
//======================================================================================================

/**
 * @file alg_mahony.h
 * @brief Manhony算法注册函数
 * @details 该函数用于注册一个Mahony算法实例，初始化相关参数。
 * @param config Mahony算法初始化结构体指针
 * @return MahonyInstance_s指针，注册成功返回指针，失败返回NULL
 * @date 2025-07-04 
 * @author Hu Wenxin
 */
MahonyInstance_s *Mahony_Register(MahonyInitConfig_s *config);

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
float* Mahony_Supdate_AHRS(MahonyInstance_s *mahony, MahonyData_AHRS_t *data);

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
float* Mahony_Supdate_IMU(MahonyInstance_s *mahony, MahonyData_INS_t *data);

// TODO:注释的老代码等到封完之后再删
// void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
// void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
