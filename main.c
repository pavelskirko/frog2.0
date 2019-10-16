#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/timer.h"
//#include "udp_logging.h"
#include "mssg.pb.h"
#include "tcp_server.h"
#include "spi.h"
#include "esp_heap_caps.h"
#include "esp_heap_caps_init.h"
#include "button.h"

static const char *TAG = "frog";

#define TIMER_DIVIDER         1  //  Hardware timer clock divider
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

#define BUTTON_GPIO	35

void IRAM_ATTR ok_blink_task(void *pvParameters)
{
	int level1 = 0;
	while (1)
	{
		    	gpio_set_level(27, level1);
		    	ESP_LOGI(TAG, "TICK");
		    	level1 = !level1;
		    	vTaskDelay(300 / portTICK_PERIOD_MS);
	}

}
//void IRAM_ATTR button_isr_handler(void* arg)
//{
//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//	vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);
//	xTaskToNotify = NULL;
//
//}

//void button_init()
//{
//	gpio_config_t pin_config;
//	pin_config.pin_bit_mask = GPIO_SEL_35;
//	pin_config.mode = GPIO_MODE_INPUT;
//	pin_config.pull_up_en = GPIO_PULLUP_DISABLE;
//	pin_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
//	pin_config.intr_type = GPIO_PIN_INTR_NEGEDGE;
//	gpio_config(&pin_config);
//	gpio_install_isr_service(0);
//	gpio_isr_handler_add(GPIO_NUM_35, button_isr_handler, NULL);
////	xSemaphore = xSemaphoreCreateBinary();
//}

void timer_setup()
{
	timer_config_t config;
	config.divider = TIMER_DIVIDER;
	config.counter_dir = TIMER_COUNT_UP;
	config.counter_en = TIMER_PAUSE;
	config.alarm_en = TIMER_ALARM_DIS;
	config.intr_type = TIMER_INTR_LEVEL;
	config.auto_reload = TIMER_AUTORELOAD_EN;
	timer_init(0,0,&config);

}


void app_main(void)
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
	   ESP_ERROR_CHECK(nvs_flash_erase());
	   ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
//	heap_caps_init();
	ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
	gpio_set_direction(27, GPIO_MODE_OUTPUT);
	gpio_set_direction(26, GPIO_MODE_OUTPUT);
//	button_init();
	ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
	wifi_event_group = xEventGroupCreate();
	start_dhcp_server();
	initialise_wifi_in_ap();
//	 udp_logging_init( "192.168.1.2", 1337, udp_logging_vprintf);

	timer_setup();

	xTaskCreatePinnedToCore(tcp_server,"tcp_server",4096*8,NULL,5,NULL, 0); // Core 0
//	 xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
	 xTaskCreatePinnedToCore(ok_blink_task, "ok_blink", 1024*2, NULL, 1, NULL, 0); // Core 0
	 xTaskCreatePinnedToCore(get_data, "get_data", 8096*4, NULL, 5, NULL, 1); // Core 1

	 button_event_t ev;
	 QueueHandle_t button_events = button_init(PIN_BIT(BUTTON_GPIO));
	 while (true) {
	     if (xQueueReceive(button_events, &ev, 1000/portTICK_PERIOD_MS)) {
	         if ((ev.pin == BUTTON_GPIO) && (ev.event == BUTTON_DOWN)) {
	        	 xEventGroupSetBits(SpiEventGroup, BIT5);
	         }

	     }
	 }
}
