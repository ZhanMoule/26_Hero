
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       InsTask.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             ��Ҫ����������bmi088��������ist8310�������̬���㣬�ó�ŷ���ǣ�
  *             �ṩͨ��bmi088��data ready �ж�����ⲿ�������������ݵȴ��ӳ�
  *             ͨ��DMA��SPI�����ԼCPUʱ��.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  //---------------------------�����ǳ��������ͷ�ڽ�����-----------------------------------//
  *2022.7.02 �����ȵ�Ҫ�� ����ܲ�����
  *1�����ڵĴ���yaw�������Ʈ��� ������ô������ܽ�������Ǵ�����������������ǰ�ڵ���
  *Ʈʮ�ֳ��󣨸ó�һ��debugֵ������Ʈ1�ȶ࣬ʮ�ֳ���
  *2�����ô����ƾ��ǳ�ʼ�������Ʋ��Ұ�ist8310_read_mag(ist8310_real_data.mag)����ȡ��ע��
  *�������� ����ȡ��ע�Ϳ���Ч�� Ȼ����������Ϊʲô�������ˣ����ţ�
  *3���Աȹٷ�����ᷢ�ֹٷ�������gyro_update_flag����IMU_NOTIFY_SHFITS
  *���������� �������ﳭ�����ó���IMU_UPDATE_SHFITS ��������Ϊʲô ���ԸĻ�ȥ���ԣ�΢Ц.jpg��
  *4���Ǹ��ٷ����ԵĽ����Ʈ�Ĵ���û������ ��ֻҪϸ������ �ͻᷢ��ֵ����û�������������ٷ��궨����
  *�������������У׼������������ ���ǲ�û�е����Ǹ�У׼���� ��֮���ǹٷ���������ûдҲ��֪����
  *�����ڸ�ʲô���Ƿ���ʹ���ˣ�
  *5���䰮���� Զ�������� ����
  */
//2023.9 �������Ǹ�Ϊ�Ϻ��汾
//2024.10 ���þ��У���Ϲ�ģ�Ҫ���������Ƿ���
#include "IMU_Task.h"
#include "main.h"
#include "cmsis_os.h"
#include "task.h"
#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "BMI088driver.h"
#include "ist8310driver.h"
//#include "arm_math.h"
//#include "usbd_cdc_if.h"
#include "math.h"
#include "MahonyAHRS.h"
//#include "my_fun.h"
#include "pid.h"
#include <stdbool.h>
//#include "gimbal_task.h"
//#include "GimbalFun.h"
//#include "IMU_can.h"
//#include "vision.h"

#define PI 3.1415926f
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm����
#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                     \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \
	
/**
  * @brief          ��ת������,���ٶȼƺʹ�����,��������Ư,��Ϊ�豸�в�ͬ��װ��ʽ
  * @param[out]     gyro: ������Ư����ת
  * @param[out]     accel: ������Ư����ת
  * @param[out]     mag: ������Ư����ת
  * @param[in]      bmi088: �����Ǻͼ��ٶȼ�����
  * @param[in]      ist8310: ����������
  * @retval         none
  */
static void imu_cali_slove(float gyro[3], float accel[3], float mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_temp_control(float temp);

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����imu_update_flag��ֵ����SPI DMA
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_cmd_spi_dma(void);

void AHRS_init(float quat[4], float accel[3], float mag[3]);
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3]);
void get_angle(float quat[4], float *yaw, float *pitch, float *roll);
void mpu_offset_clc(void);

extern SPI_HandleTypeDef hspi1;


static TaskHandle_t InsTask_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};


uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};





volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;
volatile uint8_t imu_read_flag=0;

volatile bool imu_init_finish_flag = 0;
volatile bool gimbal_return_finish_flag = 0;

bmi088_real_data_t bmi088_real_data;
bmi088_real_data_t bmi088_offset_data;

float gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
float gyro_offset[3];
float gyro_cali_offset[3];

float accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
float accel_offset[3];
float accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
float mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
float mag_offset[3];
float mag_cali_offset[3];
imu_chass IMU_Chass_MSG;
imu_chass_can IMU_CHASS_CAN;
static uint8_t first_temperate;
PID_struct_t imu_temp_pid;
static const float timing_time = 0.001f;   //tast run time , unit s.�������е�ʱ�� ��λ s

//���ٶȼƵ�ͨ�˲�
static float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

float INS_gyro[3] = {0.0f, 0.0f, 0.0f};
float INS_accel[3] = {0.0f, 0.0f, 0.0f};
float INS_mag[3] = {0.0f, 0.0f, 0.0f};
float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3];      //euler angle, unit rad.ŷ���� ��λ rad
float INS_angle_deg[3] = {0.0f, 0.0f, 0.0f};

float IMU_angle[3];

float imu_vision[3]={0.0f,0.0f,0.0f};

float *IMU_kf_result;
float yaw_add_one_tick=-0.00015f;
float yaw_offset;


/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu����, ��ʼ�� bmi088, ist8310, ����ŷ����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void InsTask(void const *pvParameters)
{
    uint16_t count = 0;
    portTickType currentTime;
    
    osDelay(InsTask_INIT_TIME);
    
    while(BMI088_init())
    {
        osDelay(100);
    }
    
//     while(ist8310_init())
//    {
//        osDelay(100);
//    }
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
    
    PID_init(&imu_temp_pid,TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD , TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    AHRS_init(INS_quat, INS_accel, INS_mag);

//   accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = bmi088_real_data.accel[0];//p
//   accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = bmi088_real_data.accel[1];//yoll
//   accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = bmi088_real_data.accel[2];//z

    //get the handle of task
    //��ȡ��ǰ�������������
    InsTask_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    //set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;
    bmi088_offset_data.gyro[0] =-0.00105551549; 
	bmi088_offset_data.gyro[1] = 0.00144814304;
	bmi088_offset_data.gyro[2] = 0.000177296082;
    while(1)
    {
//        IMU_task_add++;
        currentTime = xTaskGetTickCount(); //��ǰϵͳʱ��
        //wait spi DMA tansmit done
        //�ȴ�SPI DMA����
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }

        if(!imu_init_finish_flag)
            count++;

        if(!imu_init_finish_flag && count > 500)
        {
            imu_init_finish_flag = 1;
            count = 700;
        }

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
						bmi088_real_data.gyro[0] -= bmi088_offset_data.gyro[0];
						bmi088_real_data.gyro[1] -= bmi088_offset_data.gyro[1];
						bmi088_real_data.gyro[2] -= bmi088_offset_data.gyro[2];
        }
        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }

        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }

        imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
        //���ٶȼƵ�ͨ�˲�
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

        AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
		//����Ʈ
		static int pre_tick = 0;
		int tick = HAL_GetTick();
		if (pre_tick)
		yaw_offset += yaw_add_one_tick * (tick - pre_tick);
		IMU_angle[0] = 180.0f*INS_angle[0]/PI+yaw_offset+180.0f;   //yaw_offset; //yaw
		IMU_angle[1] = 180.0f*INS_angle[1]/PI+180.0f; //pitch
		IMU_angle[2] = 180.0f*INS_angle[2]/PI+180.0f; //roll
		pre_tick = tick;   

     vTaskDelayUntil(&currentTime, 1);
    }
}

/**
  * @brief          ��ת������,���ٶȼƺʹ�����,��������Ư,��Ϊ�豸�в�ͬ��װ��ʽ
  * @param[out]     gyro: ������Ư����ת
  * @param[out]     accel: ������Ư����ת
  * @param[out]     mag: ������Ư����ת
  * @param[in]      bmi088: �����Ǻͼ��ٶȼ�����
  * @param[in]      ist8310: ����������
  * @retval         none
  */
static void imu_cali_slove(float gyro[3], float accel[3], float mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}

void AHRS_init(float quat[4], float accel[3], float mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

/**
  * @brief          ������������Ư
  * @param[out]     gyro_offset:������Ư
  * @param[in]      gyro:���ٶ�����
  * @param[out]     offset_time_count: �Զ���1
  * @retval         none
  */
void gyro_offset_calc(float gyro_offset[3], float gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

        gyro_offset[0] = gyro_offset[0] + 0.00005f * gyro[0];
        gyro_offset[1] = gyro_offset[1] + 0.00005f * gyro[1];
        gyro_offset[2] = gyro_offset[2] + 0.00005f * gyro[2];
        (*offset_time_count)++;
}


/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc 
  * @retval         none
  */
/**
  * @brief          У׼������
  * @param[out]     �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[out]     �����ǵ���Ư���ɼ������ǵľ�ֹ�������Ϊoffset
  * @param[out]     �����ǵ�ʱ�̣�ÿ����gyro_offset���û��1,
  * @retval         none
  */
void INS_cali_gyro(float cali_scale[3], float cali_offset[3], uint16_t *time_count)
{
        if( *time_count == 0)
        {
            gyro_offset[0] = gyro_cali_offset[0];
            gyro_offset[1] = gyro_cali_offset[1];
            gyro_offset[2] = gyro_cali_offset[2];
        }
        gyro_offset_calc(gyro_offset, INS_gyro, time_count);

        cali_offset[0] = gyro_offset[0];
        cali_offset[1] = gyro_offset[1];
        cali_offset[2] = gyro_offset[2];
        cali_scale[0] = 1.0f;
        cali_scale[1] = 1.0f;
        cali_scale[2] = 1.0f;

}

void INS_set_cali_gyro(float cali_scale[3], float cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}

void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3])
{
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

//������Ԫ�ؽ������ŷ����
void get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}
/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_temp_control(float temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_Calc_Speed(&imu_temp_pid,temp,45.0f);
        if (imu_temp_pid.output < 0.0f)
        {
            imu_temp_pid.output = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.output;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //��û�дﵽ���õ��¶ȣ�һֱ����ʼ���
        //in beginning, max power
        if (temp > 45.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //�ﵽ�����¶ȣ�������������Ϊһ������ʣ���������
                //
                first_temperate = 1;
                imu_temp_pid.i_out = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == DRDY_IST8310_Pin)
    {
        mag_update_flag |= 1 << IMU_DR_SHFITS;

        if(mag_update_flag &= 1 << IMU_DR_SHFITS)
        {
            mag_update_flag &= ~(1 << IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);

//            ist8310_read_mag(ist8310_real_data.mag);
        }
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {
        //wake up the task
        //��������
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(InsTask_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else if(GPIO_Pin == GPIO_PIN_1)
    {
        gimbal_return_finish_flag = 1;
    }
}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����imu_update_flag��ֵ����SPI DMA
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_cmd_spi_dma(void)
{
    //���������ǵ�DMA����
    if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }
    //�������ٶȼƵ�DMA����
    if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }

    if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        return;
    }
}

void DMA2_Stream2_IRQHandler(void)
{
    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //�����Ƕ�ȡ���
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }

        //accel read over
        //���ٶȼƶ�ȡ���
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //�¶ȶ�ȡ���
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}

/**
  * @brief          calculate IMU Zero_drift
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IMU��Ư
  * @param[in]      none
  * @retval         none
  */
void mpu_offset_clc(void)
{
	static uint16_t i =0 ,j=0;
	while( i < 3000 || j < 3000 )
	{
			while (imu_read_flag == 0)
			{
			}
			imu_read_flag = 0;
			if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
			{
				if(i<3000)
				{
					gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
					BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
					bmi088_offset_data.gyro[0] += bmi088_real_data.gyro[0];
					bmi088_offset_data.gyro[1] += bmi088_real_data.gyro[1];
					bmi088_offset_data.gyro[2] += bmi088_real_data.gyro[2];
				}	
				else if(i==3000)
				{
					bmi088_offset_data.gyro[0] = bmi088_offset_data.gyro[0] / 3000.0f;
					bmi088_offset_data.gyro[1] = bmi088_offset_data.gyro[1] / 3000.0f;
					bmi088_offset_data.gyro[2] = bmi088_offset_data.gyro[2] / 3000.0f;
				}
				i++;		
			}

			if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
			{
				if(j<3000)
				{
					accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
					BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
					bmi088_offset_data.accel[0] += bmi088_real_data.accel[0];
					bmi088_offset_data.accel[1] += bmi088_real_data.accel[1];
					bmi088_offset_data.accel[2] += bmi088_real_data.accel[2];
				}	
				else if(j==3000)
				{
					bmi088_offset_data.accel[0] = bmi088_offset_data.accel[0] / 3000.0f;
					bmi088_offset_data.accel[1] = bmi088_offset_data.accel[1] / 3000.0f;
					bmi088_offset_data.accel[2] = bmi088_offset_data.accel[2] / 3000.0f;
				}
				j++;
			}
	}
}
