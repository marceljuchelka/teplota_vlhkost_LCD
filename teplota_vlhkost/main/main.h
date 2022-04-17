/*
 * main.h
 *
 *  Created on: 24. 12. 2021
 *      Author: marcel
 */

#ifndef MAIN_MAIN_H_
#define MAIN_MAIN_H_

#include "driver/i2c.h"

/* deklarace */
#define witty  1
#if witty == 1
#define EXAMPLE_ESP_WIFI_SSID       "TeePee"
#define EXAMPLE_ESP_WIFI_PASS		"07006400aa"
#define jas							3
#else
#define EXAMPLE_ESP_WIFI_SSID       "TeePee"
#define EXAMPLE_ESP_WIFI_PASS		"07006400aa"
#endif
#define EXAMPLE_ESP_MAXIMUM_RETRY  	10

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

enum {
	vlhkost_wifi,
	teplota_wifi,
	jas_wifi,
	ozon_wifi,
	restart_wifi,
};





#endif /* MAIN_MAIN_H_ */
