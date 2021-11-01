/* Light Streamer
Stream light values from the TSL2591 sensor to a MQTT topic.
tim@gremalm.se
http://tim.gremalm.se
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "Main";

void testtask(void *pvParameters) {
	ESP_LOGI(TAG, "Start");

	while(1) {
		ESP_LOGI(TAG, "Loop");
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
}

void app_main(void) {
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	xTaskCreate(&testtask, "Test_task", 4096, NULL, 5, NULL);
}

