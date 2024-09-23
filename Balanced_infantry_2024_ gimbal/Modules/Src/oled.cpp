/**
 ******************************************************************************
 * @file    oled.cpp
 * @author  Xushuang
 * @version V1.0.0 �������
 * @date    2023/10/4
 * @brief		�˴�ΪOLED��ʾ���ĺ���
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "oled.h"
#include "bsp_iic.h"
#include "oledfont.h"
#include <stdarg.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c2;

static uint8_t OLED_GRAM[128][8];



/**
  * @brief          write data/command to OLED, if you use spi, please rewrite the function
  * @param[in]      dat: the data ready to write
  * @param[in]      cmd: OLED_CMD means command; OLED_DATA means data
  * @retval         none
  */
/**
  * @brief          д���ݻ���ָ�OLED�� ���ʹ�õ���SPI������д�������
  * @param[in]      dat: Ҫд����ֽ�
  * @param[in]      cmd: OLED_CMD ����д����ֽ���ָ��; OLED_DATA ����д����ֽ�������
  * @retval         none
  */
void OledWriteByte(uint8_t dat, uint8_t cmd)
{
  static uint8_t cmd_data[2];
  if(cmd == OLED_CMD){
    cmd_data[0] = 0x00;
  }else{
    cmd_data[0] = 0x40;
  }
  cmd_data[1] = dat;
	BspI2CMasterTransmit(&hi2c2, OLED_I2C_ADDRESS, cmd_data, 2, 10);
}


/**
  * @brief          initialize the oled device
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ��ʼ��OLEDģ�飬
  * @param[in]      none
  * @retval         none
  */
void OLEDInit(void)
{
  OledWriteByte(0xAE, OLED_CMD);    //display off
  OledWriteByte(0x20, OLED_CMD);    //Set Memory Addressing Mode	
  OledWriteByte(0x10, OLED_CMD);    //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
  OledWriteByte(0xb0, OLED_CMD);    //Set Page Start Address for Page Addressing Mode,0-7
  OledWriteByte(0xc8, OLED_CMD);    //Set COM Output Scan Direction
  OledWriteByte(0x00, OLED_CMD);    //---set low column address
  OledWriteByte(0x10, OLED_CMD);    //---set high column address
  OledWriteByte(0x40, OLED_CMD);    //--set start line address
  OledWriteByte(0x81, OLED_CMD);    //--set contrast control register
  OledWriteByte(0xff, OLED_CMD);    //brightness 0x00~0xff
  OledWriteByte(0xa1, OLED_CMD);    //--set segment re-map 0 to 127
  OledWriteByte(0xa6, OLED_CMD);    //--set normal display
  OledWriteByte(0xa8, OLED_CMD);    //--set multiplex ratio(1 to 64)
  OledWriteByte(0x3F, OLED_CMD);    //
  OledWriteByte(0xa4, OLED_CMD);    //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
  OledWriteByte(0xd3, OLED_CMD);    //-set display offset
  OledWriteByte(0x00, OLED_CMD);    //-not offset
  OledWriteByte(0xd5, OLED_CMD);    //--set display clock divide ratio/oscillator frequency
  OledWriteByte(0xf0, OLED_CMD);    //--set divide ratio
  OledWriteByte(0xd9, OLED_CMD);    //--set pre-charge period
  OledWriteByte(0x22, OLED_CMD);    //
  OledWriteByte(0xda, OLED_CMD);    //--set com pins hardware configuration
  OledWriteByte(0x12, OLED_CMD);
  OledWriteByte(0xdb, OLED_CMD);    //--set vcomh
  OledWriteByte(0x20, OLED_CMD);    //0x20,0.77xVcc
  OledWriteByte(0x8d, OLED_CMD);    //--set DC-DC enable
  OledWriteByte(0x14, OLED_CMD);    //
  OledWriteByte(0xaf, OLED_CMD);    //--turn on oled panel
}

/**
  * @brief          turn on OLED display
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ��OLED��ʾ
  * @param[in]      none
  * @retval         none
  */
void OLEDDisplayOn(void)
{
  OledWriteByte(0x8d, OLED_CMD);
  OledWriteByte(0x14, OLED_CMD);
  OledWriteByte(0xaf, OLED_CMD);
}

/**
  * @brief          turn off OLED display
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          �ر�OLED��ʾ
  * @param[in]      none
  * @retval         none
  */
void OLEDDisplayOff(void)
{
  OledWriteByte(0x8d, OLED_CMD);
  OledWriteByte(0x10, OLED_CMD);
  OledWriteByte(0xae, OLED_CMD);
}

/**
  * @brief          operate the graphic ram(size: 128*8 char)
  * @param[in]      pen: the type of operate.
                    PEN_CLEAR: set ram to 0x00
                    PEN_WRITE: set ram to 0xff
                    PEN_INVERSION: bit inversion 
  * @retval         none
  */
/**
  * @brief          ����GRAM�ڴ�(128*8char����)
  * @param[in]      pen: ��������.
                    PEN_CLEAR: ����Ϊ0x00
                    PEN_WRITE: ����Ϊ0xff
                    PEN_INVERSION: ��λȡ��
  * @retval         none
  */
void OLEDOperateGram(Pen_Typedef_t pen)
{
  uint8_t i, n;

  for (i = 0; i < 8; i++){
		for (n = 0; n < 128; n++){
			if (pen == PEN_WRITE){
				OLED_GRAM[n][i] = 0xff;
      }else if (pen == PEN_CLEAR){
        OLED_GRAM[n][i] = 0x00;
      }else{
        OLED_GRAM[n][i] = 0xff - OLED_GRAM[n][i];
      }
    }
  }
}

/**
  * @brief          cursor set to (x,y) point
  * @param[in]      x:X-axis, from 0 to 127
  * @param[in]      y:Y-axis, from 0 to 7
  * @retval         none
  */
/**
  * @brief          ���ù�����(x,y)
  * @param[in]      x:x��, �� 0 �� 127
  * @param[in]      y:y��, �� 0 �� 7
  * @retval         none
  */
void OLEDSetPos(uint8_t x, uint8_t y)
{
  OledWriteByte((0xb0 + y), OLED_CMD);              //set page address y
  OledWriteByte(((x&0xf0)>>4)|0x10, OLED_CMD);      //set column high address
  OledWriteByte((x&0x0f), OLED_CMD);                //set column low address
}


/**
  * @brief          draw one bit of graphic raw, operate one point of screan(128*64)
  * @param[in]      x: x-axis, [0, X_WIDTH-1]
  * @param[in]      y: y-axis, [0, Y_WIDTH-1]
  * @param[in]      pen: type of operation,
                        PEN_CLEAR: set (x,y) to 0
                        PEN_WRITE: set (x,y) to 1
                        PEN_INVERSION: (x,y) value inversion 
  * @retval         none
  */
/**
  * @brief          ����GRAM�е�һ��λ���൱�ڲ�����Ļ��һ����
  * @param[in]      x:x��,  [0,X_WIDTH-1]
  * @param[in]      y:y��,  [0,Y_WIDTH-1]
  * @param[in]      pen: ��������,
                        PEN_CLEAR: ���� (x,y) ��Ϊ 0
                        PEN_WRITE: ���� (x,y) ��Ϊ 1
                        PEN_INVERSION: (x,y) ֵ��ת
  * @retval         none
  */
void OLEDDrawPoint(int8_t x, int8_t y, Pen_Typedef_t pen)
{
  uint8_t page = 0, row = 0;

  /* check the corrdinate */
  if ((x < 0) || (x > (X_WIDTH - 1)) || (y < 0) || (y > (Y_WIDTH - 1))){
		return;
  }
  page = y / 8;
  row = y % 8;

  if (pen == PEN_WRITE){
		OLED_GRAM[x][page] |= 1 << row;
  }else if (pen == PEN_INVERSION){
    OLED_GRAM[x][page] ^= 1 << row;
	}else{
    OLED_GRAM[x][page] &= ~(1 << row);
  }
}




/**
  * @brief          draw a line from (x1, y1) to (x2, y2)
  * @param[in]      x1: the start point of line
  * @param[in]      y1: the start point of line
  * @param[in]      x2: the end point of line
  * @param[in]      y2: the end point of line
  * @param[in]      pen: type of operation,PEN_CLEAR,PEN_WRITE,PEN_INVERSION.
  * @retval         none
  */
/**
  * @brief          ��һ��ֱ�ߣ���(x1,y1)��(x2,y2)
  * @param[in]      x1: ���
  * @param[in]      y1: ���
  * @param[in]      x2: �յ�
  * @param[in]      y2: �յ�
  * @param[in]      pen: ��������,PEN_CLEAR,PEN_WRITE,PEN_INVERSION.
  * @retval         none
  */
  
void OLEDDrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, Pen_Typedef_t pen)
{
  uint8_t col = 0, row = 0;
  uint8_t x_st = 0, x_ed = 0, y_st = 0, y_ed = 0;
  float k = 0.0f, b = 0.0f;

  if (y1 == y2){
    (x1 <= x2) ? (x_st = x1):(x_st = x2);
    (x1 <= x2) ? (x_ed = x2):(x_ed = x1);

    for (col = x_st; col <= x_ed; col++){
			OLEDDrawPoint(col, y1, pen);
    }
  }else if (x1 == x2){
    (y1 <= y2) ? (y_st = y1):(y_st = y2);
    (y1 <= y2) ? (y_ed = y2):(y_ed = y1);

    for (row = y_st; row <= y_ed; row++){
			OLEDDrawPoint(x1, row, pen);
    }
  }else{
    k = ((float)(y2 - y1)) / (x2 - x1);
    b = (float)y1 - k * x1;

    (x1 <= x2) ? (x_st = x1):(x_st = x2);
    (x1 <= x2) ? (x_ed = x2):(x_ed = x2);

    for (col = x_st; col <= x_ed; col++){
			OLEDDrawPoint(col, (uint8_t)(col * k + b), pen);
    }
  }
}


/**
  * @brief          show a character
  * @param[in]      row: start row of character
  * @param[in]      col: start column of character
  * @param[in]      chr: the character ready to show
  * @retval         none
  */
/**
  * @brief          ��ʾһ���ַ�
  * @param[in]      row: �ַ��Ŀ�ʼ��
  * @param[in]      col: �ַ��Ŀ�ʼ��
  * @param[in]      chr: �ַ�
  * @retval         none
  */
void OLED_show_char(uint8_t row, uint8_t col, uint8_t chr)
{
	uint8_t x = col * 6;
  uint8_t y = row * 12;
  uint8_t temp, t, t1;
  uint8_t y0 = y;
  chr = chr - ' ';

  for (t = 0; t < 12; t++){
		temp = asc2_1206[chr][t];

    for (t1 = 0; t1 < 8; t1++){
			if (temp&0x80)
				OLEDDrawPoint(x, y, PEN_WRITE);
      else
        OLEDDrawPoint(x, y, PEN_CLEAR);

        temp <<= 1;
        y++;
        if ((y - y0) == 12){
					y = y0;
					x++;
          break;
        }
    }
	}
}


/**
  * @brief          show a character string
  * @param[in]      row: row of character string begin
  * @param[in]      col: column of character string begin
  * @param[in]      chr: the pointer to character string
  * @retval         none
  */
/**
  * @brief          ��ʾһ���ַ���
  * @param[in]      row: �ַ����Ŀ�ʼ��
  * @param[in]      col: �ַ����Ŀ�ʼ��
  * @param[in]      chr: �ַ���
  * @retval         none
  */
void OLED_show_string(uint8_t row, uint8_t col, const char *chr)
{
  uint8_t n =0;

  while (chr[n] != '\0'){
		OLED_show_char(row, col, chr[n]);
    col++;

    if (col > 20){
      col = 0;
      row += 1;
    }
    n++;
  }
}


/**
  * @brief          formatted output in oled 128*64
  * @param[in]      row: row of character string begin, 0 <= row <= 4;
  * @param[in]      col: column of character string begin, 0 <= col <= 20;
  * @param          *fmt: the pointer to format character string
  * @note           if the character length is more than one row at a time, the extra characters will be truncated
  * @retval         none
  */
/**
  * @brief          ��ʽ���
  * @param[in]      row: ��ʼ�У�0 <= row <= 4;
  * @param[in]      col: ��ʼ�У� 0 <= col <= 20;
  * @param[in]      *fmt:��ʽ������ַ���
  * @note           ����ַ������ȴ���һ�У�������ַ��ỻ��
  * @retval         none
  */
void OLED_printf(uint8_t row, uint8_t col, const char *fmt,...)
{
  static char LCD_BUF[128] = {0};
  static va_list ap;
  uint8_t remain_size = 0;

  if ((row > 4) || (col > 20) ){
		return;
  }
  va_start(ap, fmt);

  vsprintf((char *)LCD_BUF, fmt, ap);

  va_end(ap);

  remain_size = 21 - col;

  LCD_BUF[remain_size] = '\0';

  OLED_show_string(row, col, LCD_BUF);
}

/**
  * @brief          send the data of gram to oled sreen
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          �������ݵ�OLED��GRAM
  * @param[in]      none
  * @retval         none
  */
void OLED_refresh_gram(void)
{
  uint8_t i, n;

  for (i = 0; i < 8; i++){
		OLEDSetPos(0, i);
    for (n = 0; n < 128; n++){
			OledWriteByte(OLED_GRAM[n][i], OLED_DATA);
    }
  }
}



/**
  * @brief          show the logo of RoboMaster
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ��ʾRM��LOGO
  * @param[in]      none
  * @retval         none
  */
void OLED_LOGO(void)
{
  uint8_t temp_char = 0;
  uint8_t x = 0, y = 0;
  uint8_t i = 0;
  OLEDOperateGram(PEN_CLEAR);

  for(; y < 64; y += 8){
		for(x = 0; x < 128; x++){
			temp_char = LOGO_BMP[x][y/8];
      for(i = 0; i < 8; i++){
				if(temp_char & 0x80){
					OLEDDrawPoint(x, y + i,PEN_WRITE);
        }else{
          OLEDDrawPoint(x,y + i,PEN_CLEAR);
        }
        temp_char <<= 1;
      }
    }
  }
  OLED_refresh_gram();
}

/**
  * @brief          ԭ��������
  * @param[in]      none
  * @retval         none
  */
void OLED_GENSHIN_LOGO(void)
{
  uint8_t temp_char = 0;
  uint8_t x = 0, y = 0;
  uint8_t i = 0;
  OLEDOperateGram(PEN_CLEAR);


  for(; y < 64; y += 8){
		for(x = 0; x < 128; x++){
			temp_char = GENSHIN_IMPACT_LOGO[x][y/8];
      for(i = 0; i < 8; i++){
				if(temp_char & 0x80){
					OLEDDrawPoint(x, y + i,PEN_WRITE);
        }else{
          OLEDDrawPoint(x,y + i,PEN_CLEAR);
        }
        temp_char <<= 1;
      }
    }
  }
  OLED_refresh_gram();
}
