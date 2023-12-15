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

    // Convert into 'g'

    xg = x * .0078;
    yg = y * .0078;
    zg = z * .0078;
    /*send Var to PC*/
    sprintf(buffer, "X= %0.4f,Y= %0.4f,Z=%0.4f", xg, yg, zg);
    Uart_sendstring(buffer, pc_uart);
}
