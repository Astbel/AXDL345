#include "main.h"
void UART_TEST_SEND(void);
/*ISR call Back Handler*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim10)
  {
    // adxl_send_data_parsing_pc();
   Print_Function();
    // Rotary_Encoder(GPIOA,Rotary_CLK_Pin,GPIOA,Rotary_DT_Pin);
    //   Uart_sendstring("Test",pc_uart);
  }
}

/*旋轉編碼器*/
/*控制亮度*/
void Rotary_Encoder(GPIO_TypeDef *GPIOxA, uint16_t GPIO_PinA, GPIO_TypeDef *GPIOxB, uint16_t GPIO_PinB)
{
  char buffer[Uart_Buffer];

  phaseA = HAL_GPIO_ReadPin(GPIOxA, GPIO_PinA);
  phaseB = HAL_GPIO_ReadPin(GPIOxB, GPIO_PinB);
  obser_result = Pin_process(phaseA, phaseB);
  static int16_t count;
  /*順時針*/
  if (obser_result == DIR_CW)
    // count++;
    obser_cout++;
  /*逆時針*/
  else if (obser_result == DIR_CCW)
    // count--;
    obser_cout--;
  /*Duty計算*/
  // Vector_Space.modula_duty = count * PWM_Resloution;
  Vector_Space.modula_duty = obser_cout * PWM_Resloution;
  /*調變控制亮度*/
  Control_Lighting(&Vector_Space.modula_duty);

  sprintf(buffer, "Duty is %d", TIM1->CCR1);
	Uart_sendstring(buffer, pc_uart);
}

void Get_Vector_Degree_Init(void)
{
/*Uart buffer*/
#ifdef View_initail
  char buffer[Uart_Buffer];
#endif
  /*取得使量位置*/
  adxl_read(0x32);
  x = ((data_rec[1] << 8) | data_rec[0]);
  y = ((data_rec[3] << 8) | data_rec[2]);
  z = ((data_rec[5] << 8) | data_rec[4]);

  // Convert into 'g'的 0.0078 g/LSB 這裡其實就是三軸向量
  xg = x * .0078;
  yg = y * .0078;
  zg = z * .0078;

  /*計算矢量*/
  Vector_Space.x_Vector = atan2(-xg, sqrt(yg * yg + zg * zg));
  Vector_Space.y_Vector = atan2(-yg, sqrt(xg * xg + zg * zg));
  Vector_Space.z_Vector = atan2(sqrt(pow(xg, 2) + pow(yg, 2)), zg);

  /*XYZ三軸向角計算*/
  Vector_Space.x_Degree = Vector_Space.x_Vector * (180.0 / M_PI);
  Vector_Space.y_Degree = Vector_Space.y_Vector * (180.0 / M_PI);
  Vector_Space.z_Degree = Vector_Space.z_Vector * (180.0 / M_PI);

#ifdef View_initail
  /*Uart 打印個個維度角度*/
  sprintf(buffer, "X_Deg= %0.2f,Y_Deg= %0.2f,Z_Deg=%0.2f", x_Degree, y_Degree, z_Degree);
  Uart_sendstring(buffer, pc_uart);
#endif
  /*Status Check*/
}

/**
 * @brief
 * default配置PWM Duty為
 * @param duty_compare 調變亮度
 */
void Control_Lighting(int *duty_compare)
{
  /*初始duty未配置*/
  Pwm_out = ((Initail_Duty * TIM1->ARR) / MAX_DUTY_percentage) + *duty_compare;

  /*限制Duty*/
  if (Pwm_out > MAX_DUTY)
    Pwm_out = MAX_DUTY;
  if (Pwm_out < Min_DUTY)
    Pwm_out = Min_DUTY;
  TIM1->CCR1 = Pwm_out;
  mointer_Duty = (float)Pwm_out / MAX_DUTY;
}

/*測試打印旋轉編碼器*/
void Print_Function(void)
{
  char buffer[Uart_Buffer];
  int data;
  data=TIM2->CNT;
  Control_Lighting(&data);
  /*計算百分比*/
  sprintf(buffer, "Duty is %0.2f", mointer_Duty);
	Uart_sendstring(buffer, pc_uart);

}
