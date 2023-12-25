#ifndef __FUNCTION_H_
#define __FUNCTION_H_


/**ADC**/
extern void Multi_ADC_Sample(void);
/**Inital Variable**/
extern void Initail_Variable(void);
/**Flash**/
extern void Flash_Erase_Sectors(uint32_t startSector, uint32_t endSector);
extern uint32_t Flash_Write_Flash_Memory(uint32_t *data,uint32_t startAddr, uint16_t numberofwords);
extern uint32_t Flash_Find_Data(uint32_t *targetData, uint32_t size, uint32_t flashAddress);
/**Uart command ptr function**/
extern void Get5VMinCommand(void);
extern void Get5VMaxCommand(void);
extern void Get12VMinCommand(void);
extern void Get12VMaxCommand(void);
extern void EraseFlashMemoryCommand(void);
extern void Check_Flash_Memory_Data(void);
extern void Serial_Slopping_Method(void);
extern void OTP_Protect_Event(void);
extern void OCP_Protect_Event(void);
extern void OVP_Protect_Event(void);
extern void Black_Box_Write_Message_Status(void);
/*PWM 調變區*/
extern void PWM_Duty_Freq_Change(void);
extern void PWM_Duty_Freq_Dual_Channel(void);
extern void Display_message_on_gui(void);

/*初始向量計算*/
extern void Get_Vector_Degree_Init(void);
extern void RemoveSubstringAndProcess(const char* target, size_t start_pos, uint16_t* wave_value);
/*旋轉編碼器*/
extern void Rotary_Encoder(GPIO_TypeDef *GPIOxA, uint16_t GPIO_PinA, GPIO_TypeDef *GPIOxB, uint16_t GPIO_PinB);
/*向角判斷執行*/
extern void Event_Execute(void);
extern void Control_Lighting(int *duty_compare);
extern void Print_Function(void);
#endif