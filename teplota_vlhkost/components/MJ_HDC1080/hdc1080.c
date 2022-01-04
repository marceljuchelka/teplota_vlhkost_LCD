/*                  e-gadget.header
 * hdc1080.c
 *
 *  Created on: 10.06.2021
 *    Modyfied: 10.06.2021 15:38:32
 *      Author: Marcel Juchelka
 *
 * Project name: "test_HDC1080_I2C"
 *
 *
 *          MCU: ATmega328P
 *        F_CPU: 8 000 000 Hz
 *
 *    Flash: 3 588 bytes   [ 10,9 % ]
 *      RAM:  11 bytes   [ 0,5 % ]
 *   EEPROM:  0 bytes   [ 0,0 % ]
 *
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "hdc1080.h"



uint16_t swap_uint16(uint16_t swap_num){
	uint8_t byte_A;
	SWAP16 byte_swap;
	byte_swap.BYTES2 = swap_num;
	byte_A = byte_swap.MSB;
	byte_swap.MSB = byte_swap.LSB;
	byte_swap.LSB = byte_A;
	return byte_swap.BYTES2;
}

esp_err_t hdc_start_mereni(reg_map reg_){
	esp_err_t ret;
	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, hdc_1080_address <<1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, reg_, I2C_MASTER_ACK);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(ret != ESP_OK) printf("chyba start mereni\n");
	return ret;
}

/*zapis hodnot do registru programovani HDC1080*/
void hdc1080_write_register(reg_map reg_, uint16_t reg_value){
	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, hdc_1080_address <<1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, reg_, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, (reg_value>>8), I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, (reg_value), I2C_MASTER_ACK);
	i2c_master_stop(cmd);
	if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS) != ESP_OK) printf("chyba reg write");
	i2c_cmd_link_delete(cmd);
}

/* nastaveni registru pred ctenim */
void hdc1080_set_register(reg_map reg_set){
	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, hdc_1080_address <<1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, reg_set, I2C_MASTER_ACK);
	i2c_master_stop(cmd);
	if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS) != ESP_OK) printf("chyba reg set");
	i2c_cmd_link_delete(cmd);
}


uint16_t hdc1080_read_register(reg_map set_register){
	uint16_t res = 0;
	uint8_t * buf = (uint8_t*)&res;
	hdc1080_set_register(set_register);
	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, hdc_1080_address <<1| I2C_MASTER_READ, I2C_MASTER_ACK);
	i2c_master_read(cmd, buf, 2, I2C_MASTER_ACK);
	i2c_master_stop(cmd);
	if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS) != ESP_OK) printf("chyba reg read");
	i2c_cmd_link_delete(cmd);
	return swap_uint16(res);

}



float hdc1080_read_temp(){
	uint16_t temp = 0;
	HDC1080_READ_VALUE hodnoty;
	float temp_F = 0;
	/*nastaveni registru */
	hdc1080_init();
	/*start mereni od 0 */
	hdc_start_mereni(reg_Temperature);
	/*cekani na prevod*/
	vTaskDelay(10/portTICK_PERIOD_MS);
	/*nacteni 4 byte */
	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, hdc_1080_address <<1| I2C_MASTER_READ, I2C_MASTER_ACK);
	i2c_master_read(cmd, &hodnoty.bytes[0], 4, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);
	if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS) != ESP_OK) printf("chyba temp");
	i2c_cmd_link_delete(cmd);
	printf("read temp raw = %X  %X\n", hodnoty.bytes[0], hodnoty.bytes[1]);
	printf("read hum raw = %X\n", hodnoty.humid_r);
//	temp = swap_uint16(temp);
	temp_F = ((((float)temp)/(float)65536)*(float)165)-(float)40;
//	temp32 = ((uint32_t)temp * 165 /65536)-40;
//	temp = (uint16_t) temp_F;
	i2c_driver_delete(I2C_NUM_0);
	return temp_F;
}


float hdc1080_read_hum(){
	uint16_t hum = 0;
	uint8_t * buf = (uint8_t*)&hum;
//	uint32_t hum32 = 0;
	hdc1080_init();
	hdc1080_set_register(reg_Humidity);
	vTaskDelay(5/portTICK_PERIOD_MS);
	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
//	i2c_master_read(cmd, buf, 2, I2C_MASTER_ACK);
	i2c_master_read_byte(cmd, buf+1, I2C_MASTER_ACK);
	i2c_master_read_byte(cmd, buf, I2C_MASTER_NACK);
	i2c_master_stop(cmd);
	if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS) != ESP_OK) printf("chyba hum");
	i2c_cmd_link_delete(cmd);
	printf("readhumpraw = %X\n", hum);
	hum = swap_uint16(hum);
//	hum32 = ((uint32_t)hum*100l/65536l);
	float hum_F  = ((float)hum*100)/65536;
	i2c_driver_delete(I2C_NUM_0);
 	return hum_F;
}

int hdc1080_test(){
	esp_err_t ret;
	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, hdc_1080_address <<1|I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(ret != ESP_OK){
		printf("neni spojeni s hdc1080\n");
		return 0;
	}
	return 1;
}

void hdc1080_init(){
	dev_config config_register = 0;
	my_i2c_config();
//	printf("config register init = %X\n", hdc1080_read_register(reg_Configuration));
	config_register|= (HumMR_8_bit << HRES) | (TempMR_11_bit << TRES) | (1<<MODE);
//	printf("config register pred zapisem = %X\n", config_register);
	hdc1080_write_register(reg_Configuration, config_register);
	printf("config register init po zapisu = %X\n", hdc1080_read_register(reg_Configuration));
	printf("config registrer device ID = %d\n", hdc1080_read_register(reg_DeviceID));
}

	/*konfigurace i2c na ESP  vcetne driveru*/
void my_i2c_config(){
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_SDA_PIN;
    conf.sda_pullup_en = 1;
	conf.scl_io_num = I2C_SCL_PIN;
    conf.scl_pullup_en = 1;
	conf.clk_stretch_tick = 100;
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode));
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
}

