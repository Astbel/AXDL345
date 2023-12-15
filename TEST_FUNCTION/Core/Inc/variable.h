#ifndef __VARIABLE_H_
#define __VARIABLE_H_
/************************include**************/

/******************結構體***********************/

extern struct PFC_VARIABLES PFC_Variables;
extern struct Flash_Dynamic Flash_Memory;
/*********************WaveFrom*****************************/
extern uint16_t wave_Freq;
extern uint16_t wave_Vpp;
extern uint8_t wave_select;
/**********************C# command 指標函數************************/
typedef void (*CommandHandler)(void);
/***********************Flash**********************/
// 測試函數 變數宣告

// 正式使用函數 變數宣告

/********************STRUCT***************************/
struct PFC_VARIABLES
{
    uint32_t adc_raw[5];
};

// Flash 結構體
struct Flash_Dynamic
{
    /*兩點校正公式參數*/
    float block_c;
    float block_b;
    float block_a;
    /*兩點Max min上下限存取*/
    uint32_t adc_value_min;
    uint32_t adc_value_max;
    /*2點校正輸出*/
    float slope_value;
};


/*command 結構體*/
// 定义命令-处理函数映射表
typedef struct
{
    const char *commandName;
    CommandHandler handler;
} CommandEntry;

/*********************Flash***************************/


/*********************Slope method**********************/

#endif
