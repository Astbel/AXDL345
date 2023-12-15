#include "main.h"
struct PFC_VARIABLES PFC_Variables;

// 申明變數

/*控制PWM 變數*/

/*Flash 變數申明區*/

/*Flash 地址申明區*/

/*GUI display*/

/*Sine Wave*/

/*Wave 設定值*/

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

    

    /*Flash 測試變數使用區*/

    /*Flash 宣告變數區*/
}
