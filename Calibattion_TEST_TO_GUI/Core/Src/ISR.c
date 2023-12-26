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
    // MPU6050_Read_Accel();


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
  static int32_t diff_wise_check = 0;
  static uint16_t Last_PWM_value = 0;

  clock_status = Status_Clock_Wise(duty_compare);

  if ((Pwm_out == MAX_DUTY) && (clock_status == 1))
    return;
  else if ((Pwm_out == Min_DUTY) && ((clock_status == -1) || (clock_status == 0)))
    return;
  else
  {
    if (clock_status == 1)
      Pwm_out = CalculatePWMValue(Initail_Duty, TIM1->ARR, MAX_DUTY_percentage, *duty_compare);
    /*逆時針鎖最小值事件*/
    else if (clock_status == -1)
    {
      diff_wise_check = Last_PWM_value + *duty_compare;

      if (IsOverflow(diff_wise_check, MAX_DUTY))
        Pwm_out = Min_DUTY;
      else
        Pwm_out = CalculatePWMValue(Initail_Duty, TIM1->ARR, MAX_DUTY_percentage, *duty_compare);
    }

    LimitDutyRange();
    Last_PWM_value = Pwm_out;
  }

  TIM1->CCR1 = Pwm_out;
  mointer_Duty = (float)Pwm_out / MAX_DUTY;
}

/*測試打印旋轉編碼器*/
void Print_Function(void)
{
  char buffer[Uart_Buffer];
  int data;
  data = TIM2->CNT;
  Control_Lighting(&data);
  /*計算百分比*/
  sprintf(buffer, "Duty is %0.2f", mointer_Duty);
  // sprintf(buffer, "Duty is %d", clock_status);
  Uart_sendstring(buffer, pc_uart);
}
/**
 * @brief
 * 確認狀態使用
 * @param rotary_encoder_phase
 * @return uint8_t CW CCW
 */
int8_t Status_Clock_Wise(uint32_t *rotary_encoder_phase)
{
  char buffer[Uart_Buffer];
  static uint32_t Last_value = 0; // 静态变量用于保持上一次的值
  static int8_t result = 0;
  uint32_t Now_value = *rotary_encoder_phase;

  // 判断顺时针还是逆时针
  if (Now_value > Last_value)
    result = 1; // 顺时针
  else if (Now_value < Last_value)
    result = -1; // 逆时针
  else
    result = 0;
  // 跟新舊的值
  Last_value = Now_value;
  // 无变化
  return result;
}

/*SubFuntion*/
// Helper function to check for overflow
int IsOverflow(int32_t value, int32_t max_limit)
{
  return value > max_limit;
}

// Helper function to calculate PWM value
uint16_t CalculatePWMValue(int32_t initial_duty, int32_t arr, int32_t percentage, int duty_compare)
{
  return (initial_duty * arr) / percentage + duty_compare;
}

// Helper function to limit duty range
void LimitDutyRange(void)
{
  if (Pwm_out > MAX_DUTY)
    Pwm_out = MAX_DUTY;
}




