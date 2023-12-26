#include "main.h"
#include "MPU6050.h"

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;

MPU6050_Angle mpu6050_Angle;

/**
 * @brief
 * 初始化
 */
void MPU6050_Init(void)
{
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
    }
}
/**
 * @brief
 * 加速計
 */
void MPU6050_Read_Accel(void)
{
#ifdef mpu6050_Uart
    char buffer[Uart_Buffer];
#endif
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
    /*高低位數據相加*/
    Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/
    // 16bit 65536 默認量測+-2g因為要計算單位為1g所以這邊要把65536/4=16384所以這裡轉換的單位1g
    Ax = Accel_X_RAW / 16384.0;
    Ay = Accel_Y_RAW / 16384.0;
    Az = Accel_Z_RAW / 16384.0;
    // 轉換弧度值
    Ax = acos(Ax);
    Ay = acos(Ay);
    Az = acos(Az);
    // 需要換算向角
    mpu6050_Angle.X_Angle = Ax * 57.32;
    mpu6050_Angle.Y_Angle = Ay * 57.32;
    mpu6050_Angle.Z_Angle = Az * 57.32;
#ifdef mpu6050_Uart
    /*Uart 打印個個維度角度*/
    sprintf(buffer, "X_Deg= %0.2f,Y_Deg= %0.2f,Z_Deg=%0.2f", mpu6050_Angle.X_Angle, mpu6050_Angle.Y_Angle, mpu6050_Angle.Z_Angle);
    Uart_sendstring(buffer, pc_uart);
#endif
    /*偵測事件*/
    if (Enable_Function(&mpu6050_Angle.Y_Angle) == True)
        Event_Execute();
    else
        Turn_OFF
}
/**
 * @brief
 * 陀螺儀
 */
void MPU6050_Read_Gyro(void)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

    Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    Gx = Gyro_X_RAW / 131.0;
    Gy = Gyro_Y_RAW / 131.0;
    Gz = Gyro_Z_RAW / 131.0;
}
