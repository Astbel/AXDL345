#ifndef __ADXL_H_
#define __ADXL_H_

/**/

extern void adxl_write (uint8_t address, uint8_t value);
extern void adxl_read (uint8_t address);
extern void adxl_init (void);
extern void adxl_send_data_parsing_pc(void);

extern void Two_Dimition_XY_Coorditiration(float *x, float *y, float *z);
extern void Three_Dimensions_XYZ_Coorditiration(float *x, float *y, float *z);
extern uint8_t Enable_Function(float *degree);

/*Ptr function def*/







#endif