/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "sntp.h"
#include "time.h"
#include <netdb.h>
#include "../components/mylib/mylib.h"
#include "../components/MJ_AM2320B/mj_am2320b.h"
#include "../components/MJ_HDC1080/hdc1080.h"
#include "../components/MK_LCD/mk_lcd44780.h"
#include "../main/main.h"



#if prace == 1
	#define senzor_web_teplota	"&sensor[marcel_teplota_prace]="
	#define senzor_web_vlhkost	"&sensor[marcel_vlhkost_prace]="
#else
	#define senzor_web_teplota	"&sensor[marcel_teplota_doma]="
	#define senzor_web_vlhkost	"&sensor[marcel_vlhkost_doma]="
#endif
#define led_pin_mb	2

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static const char *TAG = "wifi station";
static int s_retry_num = 0;

/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "http://meter.v108b.com"
#define WEB_PORT 80
#define WEB_URL "http://meter.v108b.com/sensor/receive/?module=marcel"
//#define WEB_URL "http://meter.v108b.com/sensor/receive/?module=marcel&sensor[marcel_teplota]=22.0"

static const char *TAG1 = "example";

//static const char *REQUEST = "GET " WEB_URL " HTTP/1.0\r\n"
//    "Host: "WEB_SERVER"\r\n"
//    "User-Agent: esp-idf/1.0 esp32\r\n"
//    "\r\n";

//static const char	*http_web_url 	=	"GET " WEB_URL " HTTP/1.0\r\n"
//static const char	*http_host 		= 	"Host: "WEB_SERVER"\r\n";
//static const char	*http_user 		= 	"User-Agent: esp-idf/1.0 esp32\r\n r\n";

static void http_get_task(uint8_t velicina,float* hodnota)
{
	char buf[90];
	char request[200];
	char *REQUEST = &request[0];

	if(velicina == temperat) sprintf(buf,"%s%s%2.1f",WEB_URL,senzor_web_teplota,*hodnota);
	else sprintf(buf,"%s%s%2.1f",WEB_URL,senzor_web_vlhkost,*hodnota);
//	printf("buf = %s\n",buf);
	sprintf(request,"GET %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: esp-idf/1.0 esp32\r\n"
			"\r\n", buf,WEB_SERVER);

	printf(REQUEST);


	const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];

//    while(0) {
        int err = getaddrinfo(WEB_SERVER, "80", &hints, &res);
        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
//            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG1, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG1, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
//            continue;
        }
        ESP_LOGI(TAG1, "... allocated socket");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG1, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
//            continue;
        }

        ESP_LOGI(TAG1, "... connected");
        freeaddrinfo(res);

        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG1, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
//            continue;
        }
        ESP_LOGI(TAG1, "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG1, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
//            continue;
        }
        ESP_LOGI(TAG1, "... set socket receiving timeout success");

        /* Read HTTP response */
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
        } while(r > 0);

        ESP_LOGI(TAG1, "... done reading from socket. Last read return=%d errno=%d\r\n", r, errno);
        close(s);
//        for(int countdown = 1; countdown >= 0; countdown--) {
//            ESP_LOGI(TAG1, "%d... ", countdown);
//            vTaskDelay(1000 / portTICK_PERIOD_MS);
//        }
        ESP_LOGI(TAG1, "Starting again!");
//    }
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


uint16_t test_num(uint8_t byte_hi, uint8_t byte_lo){
	uint16_t vysledek = (byte_lo)|(byte_hi<<8);
	printf("byte_hi = %X  byle_lo = %X  vysledek = %X nebo %d \n", byte_hi, byte_lo,vysledek,vysledek);
	return vysledek;
}

esp_err_t test_i2c(uint8_t data){
	i2c_cmd_handle_t cmd;
	esp_err_t ret = 0;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, data&0xFE, I2C_MASTER_ACK);
	i2c_master_stop(cmd);
	printf("ret = %d\n", i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
	return ret;
}

void tisk_teplota(){
    char buff[16];
    float AM_teplota;
	AM_teplota = am2320_getdata(temperat);
	printf("teplota AM2320 = %2.1f\n", AM_teplota);
	sprintf(buff,"teplota %2.1f", AM_teplota);
	http_get_task(temperat, &AM_teplota);
	lcd_str_al(0, 0, buff, _left);
}

void tisk_vlhkost(){
    char buff[16];
    float AM_vlhkost;
	AM_vlhkost = am2320_getdata(humidy);
	printf("vlhkost AM2320 = %2.1f\n", AM_vlhkost);
	sprintf(buff,"vlhkost %2.1f", AM_vlhkost);
	http_get_task(humidy, &AM_vlhkost);
	lcd_str_al(1, 0, buff, _left);
}

void tisk_cas(){

}

void led_blik(uint16_t cas){
	uint8_t uroven = 0;
	gpio_set_level(led_pin_mb, 0);
	vTaskDelay(cas/portTICK_PERIOD_MS);
	gpio_set_level(led_pin_mb, 1);
}
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };

    /* Setting a password implies station will connect to all security modes including WEP/WPA.
        * However these modes are deprecated and not advisable to be used. Incase your Access point
        * doesn't support WPA2, these mode can be enabled by commenting below line */

    if (strlen((char *)wifi_config.sta.password)) {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}

void app_main()
{
	uint16_t uroven = 0;
		vTaskDelay(500/portTICK_PERIOD_MS);
	gpio_set_direction(led_pin_mb, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
	my_i2c_pcf8574_config();
	lcd_init();
		lcd_str("START PROGRAMU");
		vTaskDelay(1000/portTICK_PERIOD_MS);
		lcd_cls();

	while(1){
		uroven^= 1;
		tisk_vlhkost();
		led_blik(100);
		vTaskDelay(8000/portTICK_PERIOD_MS);
		led_blik(100);
		tisk_teplota();
		gpio_set_level(led_pin_mb, uroven);
		vTaskDelay(8000/portTICK_PERIOD_MS);


	}


}




