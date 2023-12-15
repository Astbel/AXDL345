#include "main.h"
void UART_TEST_SEND(void);
/*ISR call Back Handler*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim10)
    {
        // ADC Scan
        //  Multi_ADC_Sample();
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        // C# LOG獲得命令
       
        // Get_Command_From_C_shrap();
        WaveFrom_Event();
// PWM_Duty_Freq_Change();
#ifdef ISR_DISPLAY
        PWM_Duty_Freq_Dual_Channel();
#endif
// SPWM_Method();
#ifdef ISR_DISPLAY
        if (mointer_Enable == True)
            Display_message_on_gui();
#endif
    }
}

/**
 * @brief
 * 初始化PWM紀錄訊息
 */
void initail_PWM_info(void)
{
    /*計算初始化頻率*/
    uint32_t initail_Freq = (uint32_t)((SystemCoreClock) / ((TIM1->PSC + 1) * (TIM1->ARR)));
    mointer_Freq = initail_Freq / Freq_Gain;
    /*計算初始化DUTY*/
    mointer_Duty = (float)TIM1->CCR1 / TIM1->ARR;
}

/*Table 在initail 就建好*/

/**
 * @brief
 * 三角波問題在ISR無法啟用
 */
void DAC_Tri_Wave(void)
{
    for (int i = 0; i < Tri_Resltion; i++)
    {
        sawtooth_table[i] = (uint16_t)(((i * 4096) / Tri_Resltion) & 0xFFF);
    }

}

/*sin(wt+Amp)其中Amp 是大小 */
void get_sineval(void)
{
    for (int i = 0; i < 100; i++)
    {
        sine_table[i] = (((sin(i * 2 * PI / 100) + 1)) * (4096 / 2));
    }
}
