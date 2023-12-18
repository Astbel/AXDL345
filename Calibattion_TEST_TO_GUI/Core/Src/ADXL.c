#include "main.h"
#include "ADXL.h"

/*ADXL 函市庫用於溝通 XYZ軸*/

void adxl_write(uint8_t address, uint8_t value)
{
    uint8_t data[2];
    data[0] = address | 0x40; // multibyte write enabled
    data[1] = value;
    HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_RESET); // pull the cs pin low to enable the slave
    HAL_SPI_Transmit(&hspi2, data, 2, 100);                          // transmit the address and data
    HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_SET);   // pull the cs pin high to disable the slave
}

void adxl_read(uint8_t address)
{
    address |= 0x80;                                                 // read operation
    address |= 0x40;                                                 // multibyte read
    HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_RESET); // pull the cs pin low to enable the slave
    HAL_SPI_Transmit(&hspi2, &address, 1, 100);                      // send the address from where you want to read data
    HAL_SPI_Receive(&hspi2, data_rec, 6, 100);                       // read 6 BYTES of data
    HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_SET);   // pull the cs pin high to disable the slave
}

void adxl_init(void)
{
    adxl_write(0x31, 0x01); // data_format range= +- 4g
    adxl_write(0x2d, 0x00); // reset all bits
    adxl_write(0x2d, 0x08); // power_cntl measure and wake up 8hz
}

/*顯示在PC上*/
void adxl_send_data_parsing_pc(void)
{
    /*Buffer for ringbuffer*/
    char buffer[Uart_Buffer];
    /**/
    adxl_read(0x32);
    x = ((data_rec[1] << 8) | data_rec[0]);
    y = ((data_rec[3] << 8) | data_rec[2]);
    z = ((data_rec[5] << 8) | data_rec[4]);

    // Convert into 'g'的 0.0078 g/LSB 這裡其實就是三軸向量
    xg = x * .0078;
    yg = y * .0078;
    zg = z * .0078;
    /*send Var to PC*/
    // sprintf(buffer, "X= %0.4f,Y= %0.4f,Z=%0.4f", xg, yg, zg);

    /*轉換成角度和弧度*/
    /*Pitch Angle 相對於地面水平的前後傾斜角度 */
    /*Roll Angle 相對於地面水平的左右傾斜角度 */
    float pitch = atan2(-xg, sqrt(yg * yg + zg * zg));
    float roll = atan2(yg, sqrt(xg * xg + zg * zg));

    float pitch_deg = pitch * (180.0 / M_PI);  
    float roll_deg = roll * (180.0 / M_PI);    

    // Assuming pitch and roll are in degrees 轉換為2D XY直角坐標系
    float xy_angle = atan2(tan(roll * M_PI / 180), tan(pitch * M_PI / 180)) * 180 / M_PI;
    //輸出X Y 矢量 以及角度
    sprintf(buffer, "X= %0.2f,Y= %0.2f,Ang=%0.2f", xg, yg, xy_angle);
    /*計算水平 垂直軸*/
    // sprintf(buffer, "Pitch_Deg= %0.2f,Roll_Deg= %0.2f", pitch_deg,roll_deg);

    // float x_Vector = atan2(xg, sqrt(pow(yg, 2) + pow(zg, 2)));
    // float y_Vector = atan2(yg, sqrt(pow(xg, 2) + pow(zg, 2)));
    // float z_Vector = atan2(sqrt(pow(xg, 2) + pow(yg, 2)), zg);

    // float x_Degree = x_Vector * (180.0 / M_PI);
    // float y_Degree = y_Vector * (180.0 / M_PI);
    // float z_Degree = z_Vector * (180.0 / M_PI);

    /*各向角計算*/
    // sprintf(buffer, "X_Deg= %0.2f,Y_Deg= %0.2f,Z_Deg=%0.2f", x_Degree, y_Degree, z_Degree);

    Uart_sendstring(buffer, pc_uart);
}
