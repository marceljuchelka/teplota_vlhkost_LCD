/*
 * tm_1637_led.c
 *
 *  Created on: 2. 2. 2022
 *      Author: marcel
 */

#include <stdio.h>
#include <string.h>
#include "tm_1637_led.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

uint8_t sbuff[4] = { 0x00, 0x00, 0x00, 0x00 };

//-----------------------------------------------------------------------------
void serial_begin(void)
    {
    SERIAL_DELAY;
    DIO_L;
    SERIAL_DELAY;
    }

//-----------------------------------------------------------------------------
void serial_end(void)
    {
    CLK_L;
    SERIAL_DELAY;
    DIO_L;
    SERIAL_DELAY;

    CLK_H;
    SERIAL_DELAY;
    DIO_H;
    SERIAL_DELAY;
    }

//-----------------------------------------------------------------------------
void serial_cycle(uint8_t data)
    {
    CLK_L;
    SERIAL_DELAY;

    if(data) { DIO_H; } else { DIO_L; }
    SERIAL_DELAY;

    CLK_H;
    SERIAL_DELAY;
    SERIAL_DELAY;
    }

//-----------------------------------------------------------------------------
uint8_t serial_write(uint8_t data)
    {
    uint8_t ack=HIGH;

    for(uint8_t mask=1; mask; mask<<=1)  serial_cycle(data & mask);

    serial_cycle(HIGH);

    if(DIO_IS_CLR) ack=LOW;

    return ack;
    }


//-----------------------------------------------------------------------------
void led_update(void)
    {
    serial_begin();
    serial_write(DATA_COMMAND | AUTO_ADDRESS | WRITE_DATA);
    serial_end();

    serial_begin();
    serial_write(ADDRESS_COMMAND | DEFAULT_ADDRESS);

    for(uint8_t k=0; k<4; k++) serial_write(sbuff[k]);

    serial_end();

//    serial_begin();
//    serial_write(DISPLAY_COMMAND | DISPLAY_ON | disp_bright2);
//    serial_end();
    }

void led_night_set (){
    serial_begin();
    serial_write(DISPLAY_COMMAND | DISPLAY_ON | disp_bright0);
    serial_end();
}

void led_day_set (){
    serial_begin();
    serial_write(DISPLAY_COMMAND | DISPLAY_ON | disp_bright7);
    serial_end();
}

//-----------------------------------------------------------------------------
void led_char(uint8_t pos, uint8_t code)
    {
    uint8_t tmp = chset1[10];  //for unsuppotted chars

    if(code>=48 && code<=57) tmp=chset1[code-48];

    for(uint8_t k=0; k<21; k++)
            {
            if(code==chcode[k]) tmp=chset2[k];
            }

    if(pos<4) sbuff[pos]=(tmp|(sbuff[pos]&0b10000000));
    }

//-----------------------------------------------------------------------------
void led_print(uint8_t pos, char *str)
    {
    for(;((*str) && (pos<4));) led_char(pos++,*str++);

    led_update();
    }

//-----------------------------------------------------------------------------
void led_dots(uint8_t on)
    {
//    if(on) SET_BIT(sbuff[1],7);
//    else CLR_BIT(sbuff[1],7);
	if(on) sbuff[1]|= (1<<7);
	else sbuff[1]&= ~(1<<7);

    led_update();
    }
