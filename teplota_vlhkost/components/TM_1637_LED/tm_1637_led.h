/*
 * tm_1637_led.h
 *
 *  Created on: 2. 2. 2022
 *      Author: marcel
 */

#ifndef COMPONENTS_TM_1637_LED_TM_1637_LED_H_
#define COMPONENTS_TM_1637_LED_TM_1637_LED_H_

#include "driver/i2c.h"
#include "driver/gpio.h"

/* pro TM_1637 displej dekuji https://github.com/AlexAlexFr  */
#define TM_1637_DIO	GPIO_NUM_14
#define TM_1637_CLK	GPIO_NUM_16


#define DIO_INP  	gpio_set_direction(TM_1637_DIO, GPIO_MODE_DEF_INPUT)
#define DIO_OUT  	gpio_set_direction(TM_1637_DIO, GPIO_MODE_DEF_OUTPUT)
#define DIO_SET  	gpio_set_level(TM_1637_DIO, 1)
#define DIO_CLR  	gpio_set_level(TM_1637_DIO, 0)
#define DIO_IS_SET  gpio_get_level(TM_1637_DIO)
#define DIO_IS_CLR  !gpio_get_level(TM_1637_DIO)
#define CLK_INP  	gpio_set_direction(TM_1637_CLK, GPIO_MODE_DEF_INPUT)
#define CLK_OUT  	gpio_set_direction(TM_1637_CLK, GPIO_MODE_DEF_OUTPUT)
#define CLK_SET  	gpio_set_level(TM_1637_CLK, 1)
#define CLK_CLR  	gpio_set_level(TM_1637_CLK, 0)
#define TM_1637_GPIO_CONFIG 1
#define TM1637_SERIAL_INIT  CLK_CLR; CLK_INP; DIO_CLR; DIO_INP;

#define CLK_H  CLK_INP
#define CLK_L  CLK_OUT

#define DIO_H  DIO_INP
#define DIO_L  DIO_OUT

#define HIGH  1
#define LOW   0

#define SERIAL_DELAY os_delay_us(10);




const uint8_t chset1[11] = {  // hgfedcba
  0b00111111,  // 0
  0b00000110,  // 1
  0b01011011,  // 2
  0b01001111,  // 3
  0b01100110,  // 4
  0b01101101,  // 5
  0b01111101,  // 6
  0b00000111,  // 7
  0b01111111,  // 8
  0b01101111,  // 9
  0b00000001   // 10  //NA
};

const uint8_t chset2[21] = {
  0b00000000,  //0   //32   //space
  0b01000000,  //1   //45   //-
  0b01110111,  //2   //65   //A
  0b00111001,  //3   //67   //C
  0b01111001,  //4   //69   //E
  0b01110001,  //5   //70   //F
  0b01110110,  //6   //72   //H
  0b00111000,  //7   //76   //L
  0b01110011,  //8   //80   //P
  0b00111110,  //9   //85   //U
  0b00001000,  //10  //95   //_
  0b01111100,  //11  //98   //b
  0b01011000,  //12  //99   //c
  0b01011110,  //13  //100  //d
  0b01110100,  //14  //104  //h
  0b01010100,  //15  //110  //n
  0b01011100,  //16  //111  //o
  0b01010000,  //17  //114  //r
  0b01111000,  //18  //116  //t
  0b00011100,  //19  //117  //u
  0b01100011   //20  //176  //"degree"
};

const uint8_t chcode[21] = {
  32,   //space
  45,   //-
  65,   //A
  67,   //C
  69,   //E
  70,   //F
  72,   //H
  76,   //L
  80,   //P
  85,   //U
  95,   //_
  98,   //b
  99,   //c
  100,  //d
  104,  //h
  110,  //n
  111,  //o
  114,  //r
  116,  //t
  117,  //u
  176   //"degree"
};

#define DATA_COMMAND     0b01000000
#define AUTO_ADDRESS     0b00000000
#define FIXED_ADDRESS    0b00000100
#define WRITE_DATA       0b00000000
#define READ_DATA        0b00000010

#define ADDRESS_COMMAND  0b11000000
#define DEFAULT_ADDRESS  0b00000000

#define DISPLAY_COMMAND  0b10000000
#define DISPLAY_ON       0b00001000
#define DISPLAY_OFF      0b00000000
#define DISPLAY_BRIGHT   0b00000010

enum {
	disp_bright0,
	disp_bright1,
	disp_bright2,
	disp_bright3,
	disp_bright4,
	disp_bright5,
	disp_bright6,
	disp_bright7,
}DISPLAY_BRIGHT_SET;

void led_dots(uint8_t on);
void led_day_set ();
void led_night_set ();
void led_print(uint8_t pos, char *str);


#endif /* COMPONENTS_TM_1637_LED_TM_1637_LED_H_ */
