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

/*旋轉編碼器*/
void Rotary_Encoder(GPIO_TypeDef *GPIOxA, uint16_t GPIO_PinA, GPIO_TypeDef *GPIOxB, uint16_t GPIO_PinB)
{
  uint8_t phaseA = HAL_GPIO_ReadPin(GPIOxA, GPIO_PinA);
  uint8_t phaseB = HAL_GPIO_ReadPin(GPIOxB, GPIO_PinB);
  int result = Pin_process(phaseA, phaseB);
   static int16_t count;
   /*順時針*/
  if(result==DIR_CW)
  {
     count++;
     /*檢查角度如果角度正確則執行*/
  }
  /*逆時針*/
  else if (result==DIR_CCW)
  {
     count--;
    /*檢查角度如果角度正確則執行*/
  }
  
}
