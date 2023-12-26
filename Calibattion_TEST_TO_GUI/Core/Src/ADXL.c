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
    /*高低8bit合成一組*/
    adxl_read(0x32);
    x = ((data_rec[1] << 8) | data_rec[0]);
    y = ((data_rec[3] << 8) | data_rec[2]);
    z = ((data_rec[5] << 8) | data_rec[4]);

    // Convert into 'g'的 0.0078 g/LSB 這裡其實就是三軸向量
    xg = x * .0078;
    yg = y * .0078;
    zg = z * .0078;
    /*send Var to PC*/

#ifdef TwoD_Method
    Two_Dimensions_XY_Coorditiration(&xg, &yg, &zg);
#endif

#ifdef ThreeD_Method
    Three_Dimensions_XYZ_Coorditiration(&xg, &yg, &zg);
#endif
}

/**
 * @brief 3D坐標系轉換為直角坐標系
 *
 * @param x 水平
 * @param y 垂直
 * @param z 旋轉
 */
void Two_Dimensions_XY_Coorditiration(float *x, float *y, float *z)
{
#ifdef Display_XY_Coorditiration
    /* Uart Buffer */
    char buffer[Uart_Buffer];
#endif

    // Calculate pitch and roll using the values pointed by x, y, and z
    float x_Vector = atan2(-(*x), sqrt((*y) * (*y) + (*z) * (*z)));
    float y_Vector = atan2(-(*y), sqrt((*x) * (*x) + (*z) * (*z)));

    // Convert pitch and roll to degrees
    float x_Degree = x_Vector * (180.0 / M_PI);
    float y_Degree = y_Vector * (180.0 / M_PI);

    // Calculate xy_angle
    float xy_angle = atan2(tan(x_Degree * M_PI / 180), tan(y_Degree * M_PI / 180)) * 180 / M_PI;

#ifdef Display_XY_Coorditiration
    // Use *x, *y, and xy_angle in sprintf
    sprintf(buffer, "X= %0.2f, Y= %0.2f, Ang= %0.2f", *x, *y, xy_angle);
    Uart_sendstring(buffer, pc_uart);
#endif

    /*PTR Function 指向並且判斷是否執行method*/
    if (Enable_Function(&xy_angle) == True)
        Event_Execute();
    else
        Turn_OFF
}

/**
 * @brief 3D座標系下判定90度旋轉Z軸
 *
 * @param x 水平
 * @param y 垂直
 * @param z 旋轉
 */
void Three_Dimensions_XYZ_Coorditiration(float *x, float *y, float *z)
{

/* Uart Buffer */
#ifdef Display_XYZ_Coorditiration
    char buffer[Uart_Buffer];
#endif
    // char buffer[Uart_Buffer];

    /*三維矢量計算*/
    float x_Vector = atan2(-(*x), sqrt((*y) * (*y) + (*z) * (*z)));
    float y_Vector = atan2(-(*y), sqrt((*x) * (*x) + (*z) * (*z)));
    float z_Vector = atan2(sqrt(pow((*x), 2) + pow((*y), 2)), (*z));
    /*XYZ三軸向角計算*/
    float x_Degree = x_Vector * (180.0 / M_PI);
    float y_Degree = y_Vector * (180.0 / M_PI);
    float z_Degree = z_Vector * (180.0 / M_PI);

#ifdef Display_XYZ_Coorditiration
    /*Uart 打印個個維度角度*/
    sprintf(buffer, "X_Deg= %0.2f,Y_Deg= %0.2f,Z_Deg=%0.2f", x_Degree, y_Degree, z_Degree);
    Uart_sendstring(buffer, pc_uart);
#endif
    // sprintf(buffer,"Y_Degree=%0.2f",y_Degree);
    // Uart_sendstring(buffer, pc_uart);

    if (Enable_Function(&y_Degree) == True)
        Event_Execute();
    else
        Turn_OFF
}
/**
 * @brief 判斷角度是否到達目標智能功能
 *
 * @param degree
 */
uint8_t Enable_Function(float *degree)
{
    if (((*degree) > 87) && ((*degree) < 92))
        enable_flag = True;
    else
        enable_flag = False;
    return enable_flag;
}

/**
 * @brief  90度時執行事件function test
 *
 */
void Event_Execute(void)
{
    Uart_sendstring("Testing Function Angle", pc_uart);
    Turn_ON
}
