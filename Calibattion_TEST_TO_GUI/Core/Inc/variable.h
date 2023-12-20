#ifndef __VARIABLE_H_
#define __VARIABLE_H_
/************************include**************/

/******************結構體***********************/

extern struct PFC_VARIABLES PFC_Variables;
extern struct Allert_Portect Dyanmic_Portect;
extern struct Flash_Dynamic Flash_Memory;
extern struct Vector_Space  Vector_Space;
/*********************WaveFrom*****************************/
extern uint16_t wave_Freq;
extern uint16_t wave_Vpp;
extern uint8_t wave_select;
/**********************C# command 指標函數************************/
typedef void (*CommandHandler)(void);
/***********************Flash**********************/
// 測試函數 變數宣告
extern uint16_t PWM_Channel;
extern uint32_t sine_table[100];
extern uint32_t sawtooth_table[100];
// 正式使用函數 變數宣告
extern uint16_t mointer_Freq;
extern float mointer_Duty;
extern uint8_t mointer_Enable;
extern uint8_t dac_flag;
/*********************Boolean*************************/
extern uint8_t enable_flag;
/********************STRUCT***************************/
struct PFC_VARIABLES
{
    uint32_t adc_raw[5];
};
struct Allert_Portect
{
    uint8_t OVP;
    uint8_t OCP;
    uint8_t OTP;
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

// 保護點設置
extern uint32_t Protect_12V, Protect_5V;
/*command 結構體*/
// 定义命令-处理函数映射表
typedef struct
{
    const char *commandName;
    CommandHandler handler;
} CommandEntry;

/*********************Flash***************************/
extern int check_data;
// Addr for memory
extern uint32_t Data_5V_Min_Addr;
extern uint32_t Data_5V_Max_Addr;
extern uint32_t Data_12V_Min_Addr;
extern uint32_t Data_12V_Max_Addr;
extern uint32_t Data_OTP_Addr;
extern uint32_t Data_OCP_Addr;
extern uint32_t Data_OVP_Addr;

extern uint16_t PWM_Duty;
extern uint16_t Str_PWM;
extern uint32_t Str_Freq;
extern uint32_t MAX_DUTY_Calculate;
extern uint32_t ARR_LAST_TIME_SAVE;
/*SPWM output*/
extern uint16_t SPWM_OUT;
/*********************AXDL**********************/
extern uint8_t data_rec[6];
extern int16_t x,y,z;
extern float xg, yg, zg;

struct Vector_Space
{
    float x_Vector;
    float y_Vector;
    float z_Vector;
    float x_Degree;
    float y_Degree;
    float z_Degree;
    int degree;
};



/****************Rotary Encoder*********************/
extern float degree;
#endif
