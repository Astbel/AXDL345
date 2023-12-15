#include "main.h"
void UART_TEST_SEND(void);
/*ISR call Back Handler*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim10)
    {
      adxl_send_data_parsing_pc();

    //   Uart_sendstring("Test",pc_uart);
    }
}

