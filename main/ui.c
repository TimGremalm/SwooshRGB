#include "ui.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/adc.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

static const char *TAG = "UI";
ui_config_t ui_config;

// Pins for potentiometers
#define POT1 ADC1_CHANNEL_6  // IO34
#define POT2 ADC1_CHANNEL_7  // IO35
#define POT_MAX 4095  // 12bit ADC

// Pins for Encoder switch
#define GPIO_SW1 14
#define GPIO_SW2 12
#define GPIO_SW4 2
#define GPIO_SW8 4
#define GPIO_PIN_SEL  ((1ULL<<GPIO_SW1) | (1ULL<<GPIO_SW2) | (1ULL<<GPIO_SW4) | (1ULL<<GPIO_SW8))

typedef union {
	struct {
		unsigned char bit0 : 1;
		unsigned char bit1 : 1;
		unsigned char bit2 : 1;
		unsigned char bit3 : 1;
		unsigned char bit4 : 1;
		unsigned char bit5 : 1;
		unsigned char bit6 : 1;
		unsigned char bit7 : 1;
	} u;
	uint8_t value;
} SwCode;

void uitask(void *pvParameters) {
	ui_config = *(ui_config_t *) pvParameters;
	ESP_LOGI(TAG, "Start");

	ESP_LOGI(TAG, "Configure ADC1");
	adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
	adc1_config_channel_atten(POT1, ADC_ATTEN_DB_11);
	adc1_config_channel_atten(POT2, ADC_ATTEN_DB_11);

	ESP_LOGI(TAG, "Configure GPIO for encoder switch");
	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = GPIO_PIN_SEL;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	int pot1_raw = 0;
	int pot2_raw = 0;
	float pot1_value = 0;
	float pot2_value = 0;
	SwCode posswitch;
	while(1) {
		// Get pot values
		pot1_raw = adc1_get_raw(POT1);
		pot2_raw = adc1_get_raw(POT2);
		// Inverse pot value, then make ite range from 0-1.0
		pot1_value = ((float)(POT_MAX - pot1_raw)) / POT_MAX;
		pot2_value = ((float)(POT_MAX - pot2_raw)) / POT_MAX;
		//ESP_LOGI(TAG, "Pot 1: %d %f", pot1_raw, pot1_value);
		//ESP_LOGI(TAG, "Pot 2: %d %f", pot2_raw, pot2_value);
		// Send pot floats. Wait for 10 ticks for space to become available if necessary.
		if (xQueueGenericSend(ui_config.queue_ui_pot1, ( void * ) &pot1_value, (TickType_t) 10, queueSEND_TO_BACK ) != pdPASS) {
			//ESP_LOGI(TAG, "Failed to post pot1 message, even after 10 ticks.");
		}
		if (xQueueGenericSend(ui_config.queue_ui_pot2, ( void * ) &pot2_value, (TickType_t) 10, queueSEND_TO_BACK ) != pdPASS) {
			//ESP_LOGI(TAG, "Failed to post pot2 message, even after 10 ticks.");
		}

		// Get encoder multipositionswitch values
		posswitch.u.bit0 = gpio_get_level(GPIO_SW1);
		posswitch.u.bit1 = gpio_get_level(GPIO_SW2);
		posswitch.u.bit2 = gpio_get_level(GPIO_SW4);
		posswitch.u.bit3 = gpio_get_level(GPIO_SW8);
		posswitch.u.bit4 = 0;
		posswitch.u.bit5 = 0;
		posswitch.u.bit6 = 0;
		posswitch.u.bit7 = 0;
		//ESP_LOGI(TAG, "Multipositionswitch: %d", posswitch.value);
		// Send position uint. Wait for 10 ticks for space to become available if necessary.
		if (xQueueGenericSend(ui_config.queue_ui_mode, ( void * ) &posswitch.value, (TickType_t) 10, queueSEND_TO_BACK ) != pdPASS) {
			//ESP_LOGI(TAG, "Failed to post mode message, even after 10 ticks.");
		}
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}

