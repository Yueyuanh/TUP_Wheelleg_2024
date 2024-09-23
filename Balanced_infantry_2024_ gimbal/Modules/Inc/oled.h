#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C"{
#endif

// the I2C address of oled
#define OLED_I2C_ADDRESS    0x78

//the resolution of oled   128*64
#define MAX_COLUMN      128
#define MAX_ROW         64

#define X_WIDTH         MAX_COLUMN
#define Y_WIDTH         MAX_ROW

#define OLED_CMD        0x00
#define OLED_DATA       0x01

#define CHAR_SIZE_WIDTH 6
#define CHAR_SIZE_HIGHT 12
	
#ifdef __cplusplus
enum Pen_Typedef_t
{
    PEN_CLEAR = 0x00,
    PEN_WRITE = 0x01,
    PEN_INVERSION= 0x02,
};
	

	
	
#endif

#ifdef __cplusplus
}
#endif

#endif
