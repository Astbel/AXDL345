#include "main.h"
struct PFC_VARIABLES PFC_Variables;
struct Allert_Portect Dyanmic_Portect;
struct Vector_Space  Vector_Space;
// 申明變數
uint32_t Protect_12V, Protect_5V;
/*控制PWM 變數*/
uint16_t PWM_Duty;
uint16_t Str_PWM;
uint32_t Str_Freq;
uint32_t MAX_DUTY_Calculate;
uint32_t ARR_LAST_TIME_SAVE;
/*Flash 變數申明區*/
uint8_t dac_flag;
/*Flash 地址申明區*/
uint32_t Data_5V_Min_Addr;
uint32_t Data_5V_Max_Addr;
uint32_t Data_12V_Min_Addr;
uint32_t Data_12V_Max_Addr;
uint32_t Data_OTP_Addr;
uint32_t Data_OCP_Addr;
uint32_t Data_OVP_Addr;
uint16_t SPWM_OUT;
uint16_t PWM_Channel;
uint8_t mointer_Enable;
/*GUI display*/
uint16_t mointer_Freq;
float mointer_Duty;
/*Sine Wave*/
uint32_t sine_table[Sine_Resltion];
uint32_t sawtooth_table[Tri_Resltion];
/*Wave 設定值*/
uint16_t wave_Freq;
uint16_t wave_Vpp;
uint8_t wave_select;
/*AXDL inital*/
uint8_t data_rec[6];
int16_t x, y, z;
float xg, yg, zg;
uint8_t enable_flag;
uint16_t Initail_Duty;
uint16_t Pwm_out;
/*
 * 初始化變數變量
 * 請區分 所有結構體 為一組 以利於分辨
 *
 */
void Initail_Variable(void)
{
    // inital adc value of array adc
    for (int i = 0; i < 5; i++)
        PFC_Variables.adc_raw[i] = 0;
    // Inital Flash variable
    for (int i = 0; i < 6; i++)
        data_rec[i] = 0;
    x=0;
    y=0;
    z=0;
    xg=0;
    yg=0;
    zg=0;
   Initail_Duty=10;
}
