/* ========================================================================= */
/* [INCL] Includes                                                           */
/* ========================================================================= */
#include "TLS3001.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "math.h"
#include "gamma_correction.h"

/* ========================================================================= */
/* [DEFS] Defines                                                            */
/* ========================================================================= */
#define TLS3001_DEBUG_PRINTS_EN 1

/* ========================================================================= */
/* [TYPE] Type definitions                                                   */
/* ========================================================================= */
typedef struct {
    bool ch1_initiated;
    bool ch2_initiated;
} TLS3001_internal_state_t;

/* ========================================================================= */
/* [PFDE] Private functions declaration                                      */
/* ========================================================================= */
static esp_err_t s_SPI_init(TLS3001_handle_s *TLS3001_handle);
static esp_err_t s_init_spi_data_buffer(void **spi_manchester_data_p, uint32_t byte_len);
static void s_deinit_spi_data_buffer(void *spi_manchester_data_p);
static void s_TLS3001_clear_all_pixels(TLS3001_handle_s *TLS3001_handle_channel);
static void * s_pack_manchester_data_segment(uint8_t *spi_mem_data_p_start, uint64_t data_in, uint32_t bit_length_manch, bool last_segment_flag);
static void s_TLS3001_prep_color_packet(uint8_t *spi_tx_data_start, uint16_t *color_data, uint16_t num_color_pixels);
static esp_err_t s_TLS3001_send_packet(uint8_t *data, uint32_t length, spi_device_handle_t spi_handle);
static esp_err_t s_TLS3001_send_color_packet(uint8_t *spi_tx_data_start, uint16_t num_pixels, spi_device_handle_t spi_handle);
static esp_err_t s_TLS3001_send_resetsynch_packet(spi_device_handle_t spi_handle);
static void s_TLS3001_task(void *arg);

/* ========================================================================= */
/* [GLOB] Global variables                                                   */
/* ========================================================================= */
static const char * TAG = "TLS3001";

static uint8_t reset_cmd[8];
static uint8_t synch_cmd[8];
static uint8_t start_cmd[8];

static spi_device_handle_t spi_ch1;
static spi_device_handle_t spi_ch2;

static QueueSetHandle_t xQueueSet;
static QueueSetMemberHandle_t xActivatedMember;

static TLS3001_handle_s TLS3001_handle_ch1 = {0};
static TLS3001_handle_s TLS3001_handle_ch2 = {0};

static TLS3001_internal_state_t TLS3001_state_keeper = { 0 };
/* ========================================================================= */
/* [PFUN] Private functions implementations                                  */
/* ========================================================================= */
static void s_TLS3001_task(void *arg)
{
	pixel_message_s *pixel_message_incomming_ch1_p;
    pixel_message_s *pixel_message_incomming_ch2_p;
	bool ch1_got_color = false;
    bool ch2_got_color = false;
	    
	while(1) {

		//Check incomming data queue set. Could be from cmd task or some other communication channel.
		//NOTE: The incomming data will be a POINTER to structure containing the data.
        xActivatedMember = xQueueSelectFromSet( xQueueSet, 300 / portTICK_PERIOD_MS );

        if(xActivatedMember == TLS3001_handle_ch1.TLS3001_input_queue && TLS3001_state_keeper.ch1_initiated)
        {
            xQueueReceive( xActivatedMember, &(pixel_message_incomming_ch1_p), 0 );
            
            #if TLS3001_DEBUG_PRINTS_EN 
                ESP_LOGI(TAG, "Recieved pixel data on ch1 queue. Length: %u", pixel_message_incomming_ch1_p->pixel_len);
            #endif

            //Process the pixel data. Fill the SPI buffer with new data.
            //ESP_LOGD(TAG, "Encoding and filling pixel data to spi tx buffer. Sending over SPI");
            s_TLS3001_prep_color_packet(TLS3001_handle_ch1.spi_tx_data_start, pixel_message_incomming_ch1_p->color_data_p, pixel_message_incomming_ch1_p->pixel_len);

            //Send colors. Blocks until all data is sent.
            s_TLS3001_send_color_packet(TLS3001_handle_ch1.spi_tx_data_start,pixel_message_incomming_ch1_p->pixel_len, TLS3001_handle_ch1.spi_handle);

            ch1_got_color = true;
        }

        else if(xActivatedMember == TLS3001_handle_ch2.TLS3001_input_queue && TLS3001_state_keeper.ch2_initiated)
        {
            xQueueReceive( xActivatedMember, &(pixel_message_incomming_ch2_p), 0 );
            
            #if TLS3001_DEBUG_PRINTS_EN
                ESP_LOGI(TAG, "Recieved pixel data on ch2 queue. Length: %u", pixel_message_incomming_ch2_p->pixel_len);
            #endif
            //Process the pixel data. Fill the SPI buffer with new data.
            //ESP_LOGD(TAG, "Encoding and filling pixel data to spi tx buffer. Sending over SPI");
            s_TLS3001_prep_color_packet(TLS3001_handle_ch2.spi_tx_data_start, pixel_message_incomming_ch2_p->color_data_p, pixel_message_incomming_ch2_p->pixel_len);

            //Send colors. Blocks until all data is sent.
            s_TLS3001_send_color_packet(TLS3001_handle_ch2.spi_tx_data_start,pixel_message_incomming_ch2_p->pixel_len, TLS3001_handle_ch2.spi_handle);

            ch2_got_color = true;
        }
        else
        {
            if(ch1_got_color)
            {
                s_TLS3001_send_color_packet(TLS3001_handle_ch1.spi_tx_data_start,pixel_message_incomming_ch1_p->pixel_len, TLS3001_handle_ch1.spi_handle);
            }
            if(ch2_got_color)
            {
                s_TLS3001_send_color_packet(TLS3001_handle_ch2.spi_tx_data_start,pixel_message_incomming_ch2_p->pixel_len, TLS3001_handle_ch2.spi_handle);
            }
        }
    }
}

static esp_err_t s_TLS3001_ch1_init(uint16_t num_pixels)
{
	esp_err_t ret;
	uint16_t ch1_buffer_mem_size_byte;

	TLS3001_handle_ch1.num_pixels = num_pixels;
	TLS3001_handle_ch1.spi_channel = HSPI_HOST;
	TLS3001_handle_ch1.spi_handle = spi_ch1;
	TLS3001_handle_ch1.spi_freq = 500000;
	TLS3001_handle_ch1.spi_mosi_pin = CH1_PIN_NUM_MOSI;
	TLS3001_handle_ch1.spi_clk_pin = CH1_PIN_NUM_CLK;
    TLS3001_handle_ch1.spi_dma_channel = 1;

	//Create a queue capable of containing 10 pointer to pixel_message_s structure.
   	//This structure should be passed by pointer as it contains a lot of data.
	TLS3001_handle_ch1.TLS3001_input_queue=xQueueCreate(TLS3001_TASKQUEUE_CH1_LEN, sizeof(pixel_message_s *));
    if(TLS3001_handle_ch1.TLS3001_input_queue == NULL){
		ESP_LOGE(__func__, "xQueueCreate() failed");
    }
    xQueueAddToSet((QueueSetMemberHandle_t) TLS3001_handle_ch1.TLS3001_input_queue, xQueueSet);

	//Calculate SPI buffer size
	ch1_buffer_mem_size_byte = (uint16_t ) ceil(((TLS3001_handle_ch1.num_pixels*PIXEL_DATA_LEN_SPI)+START_CMD_LEN_SPI)/8);

	if (s_SPI_init(&TLS3001_handle_ch1) != ESP_OK) {
		printf("Error initializing spi channel 1\n");
		while (1) {
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
	}
    #if TLS3001_DEBUG_PRINTS_EN
        ESP_LOGI(TAG, "SPI ch1 init successful!");
    #endif
    
	ret = s_init_spi_data_buffer(&TLS3001_handle_ch1.spi_tx_data_start, ch1_buffer_mem_size_byte);
	if(ret != ESP_OK)
	{
		ESP_LOGE(__func__, "init_spi_data_buffer(): returned %d", ret);
		return ret;
	}

    #if TLS3001_DEBUG_PRINTS_EN
        ESP_LOGI(TAG, "ch1 sending resetsynch packet");
    #endif

	s_TLS3001_send_resetsynch_packet(TLS3001_handle_ch1.spi_handle);
	ets_delay_us(SYNCH_DELAY_PER_PIXEL * TLS3001_handle_ch1.num_pixels); 	//min delay of 28.34us times the number of pixels

	s_TLS3001_clear_all_pixels(&TLS3001_handle_ch1);

	ESP_LOGI(TAG, "TLS3001 ch1 initiated! Pixels: %d. Buffer memory size: %d", TLS3001_handle_ch1.num_pixels, ch1_buffer_mem_size_byte);

	return ESP_OK;
}

static esp_err_t s_TLS3001_ch2_init(uint16_t num_pixels)
{
	esp_err_t ret;
	uint16_t ch2_buffer_mem_size_byte;

	TLS3001_handle_ch2.num_pixels = num_pixels;
	TLS3001_handle_ch2.spi_channel = VSPI_HOST;
	TLS3001_handle_ch2.spi_handle = spi_ch2;
	TLS3001_handle_ch2.spi_freq = 500000;
	TLS3001_handle_ch2.spi_mosi_pin = CH2_PIN_NUM_MOSI;
	TLS3001_handle_ch2.spi_clk_pin = CH2_PIN_NUM_CLK;
    TLS3001_handle_ch2.spi_dma_channel = 2;

	//Create a queue capable of containing 10 pointer to pixel_message_s structure.
   	//This structure should be passed by pointer as it contains a lot of data.
	TLS3001_handle_ch2.TLS3001_input_queue=xQueueCreate(TLS3001_TASKQUEUE_CH2_LEN, sizeof(pixel_message_s *));
    if(TLS3001_handle_ch2.TLS3001_input_queue == NULL){
		ESP_LOGE(__func__, "xQueueCreate() failed");
    }
    xQueueAddToSet((QueueSetMemberHandle_t)TLS3001_handle_ch2.TLS3001_input_queue, xQueueSet);

	//Calculate SPI buffer size
	ch2_buffer_mem_size_byte = (uint16_t ) ceil(((TLS3001_handle_ch2.num_pixels*PIXEL_DATA_LEN_SPI)+START_CMD_LEN_SPI)/8);

	if (s_SPI_init(&TLS3001_handle_ch2) != ESP_OK) {
		printf("Error initializing spi channel 2\n");
		while (1) {
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
	}
    #if TLS3001_DEBUG_PRINTS_EN
        ESP_LOGI(TAG, "SPI ch2 init successful!");
    #endif
    
	ret = s_init_spi_data_buffer(&TLS3001_handle_ch2.spi_tx_data_start, ch2_buffer_mem_size_byte);
	if(ret != ESP_OK)
	{
		ESP_LOGE(__func__, "init_spi_data_buffer(): returned %d", ret);
		return ret;
	}

    #if TLS3001_DEBUG_PRINTS_EN
        ESP_LOGI(TAG, "ch2 sending resetsynch packet");
    #endif

	s_TLS3001_send_resetsynch_packet(TLS3001_handle_ch2.spi_handle);
	ets_delay_us(SYNCH_DELAY_PER_PIXEL * TLS3001_handle_ch2.num_pixels); 	//min delay of 28.34us times the number of pixels

	s_TLS3001_clear_all_pixels(&TLS3001_handle_ch2);

	ESP_LOGI(TAG, "TLS3001 ch2 initiated! Pixels: %d. Buffer memory size: %d", TLS3001_handle_ch2.num_pixels, ch2_buffer_mem_size_byte);

	return ESP_OK;
}

static esp_err_t s_SPI_init(TLS3001_handle_s *TLS3001_handle)
{
	esp_err_t ret;

	spi_bus_config_t buscfg = {
		.miso_io_num = -1,
		.mosi_io_num = TLS3001_handle->spi_mosi_pin,
		.sclk_io_num = TLS3001_handle->spi_clk_pin,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};
	
	//New config parameters for DMA
	spi_device_interface_config_t devcfg = {
		.command_bits = 0,
		.address_bits = 0,
		.dummy_bits = 0,
		.clock_speed_hz = TLS3001_handle->spi_freq,
		 	//Clock out at 6 MHz				
		.mode = 1, 
							//SPI mode 1		                               
		.spics_io_num = -1,
		 			//-1 if manually toggle CS 			
		.queue_size = 1, 
					//We want to be able to queue 4 transactions at a time        
		.cs_ena_posttrans = 0, 
				//CS hold post trans in us
		.cs_ena_pretrans = 0, 
				//CS setup time in us
		.flags = (SPI_DEVICE_HALFDUPLEX) | (SPI_DEVICE_3WIRE),
		.input_delay_ns = 0	
		//.pre_cb = pre_cb_func,		//pre interrupt for manually toggle CS
	    //.post_cb = post_cb_func			//post interrupt for manually toggle CS
	};
		
	//Initialize the SPI bus with DMA channel
	ret = spi_bus_initialize(TLS3001_handle->spi_channel, &buscfg, TLS3001_handle->spi_dma_channel);
	if (ret != ESP_OK)
	{
		ESP_LOGE(__func__, "spi_bus_initialize(): returned %d", ret);
	}
	assert(ret == ESP_OK);
	
	//Attach the device to the SPI bus
	ret = spi_bus_add_device(TLS3001_handle->spi_channel, &devcfg, &TLS3001_handle->spi_handle);
	if (ret != ESP_OK)
	{
		ESP_LOGE(__func__, "spi_bus_add_device(): returned %d", ret);
	}
	assert(ret == ESP_OK);
	
	return ESP_OK;
}

static esp_err_t s_init_spi_data_buffer(void **spi_manchester_data_p, uint32_t byte_len)
{
	*spi_manchester_data_p = heap_caps_malloc(byte_len, (MALLOC_CAP_DMA | MALLOC_CAP_32BIT));

	if(*spi_manchester_data_p == NULL)
	{
		return ESP_FAIL;
	}

	return ESP_OK;
}

static void s_deinit_spi_data_buffer(void *spi_manchester_data_p)
{
	heap_caps_free(spi_manchester_data_p);
}

static void s_TLS3001_clear_all_pixels(TLS3001_handle_s *TLS3001_handle_channel)
{
	//Send zero to ALL pixels (all dark)
	s_TLS3001_prep_color_packet(TLS3001_handle_channel->spi_tx_data_start, NULL, TLS3001_handle_channel->num_pixels);
	//Send colors. Blocks until all data is sent.
	s_TLS3001_send_color_packet(TLS3001_handle_channel->spi_tx_data_start,TLS3001_handle_channel->num_pixels, TLS3001_handle_channel->spi_handle);
}

static void s_TLS3001_prep_color_packet(uint8_t *spi_tx_data_start, uint16_t *color_data, uint16_t num_color_pixels)
{
	uint8_t *spi_tx_data_last;

	uint16_t color_data_local = 0;
	//Todo: make sure that pixel1_red starts with 0b0......
    //Color data will be on the form: [red,green,blue,red,green,blue,...]

	spi_tx_data_last = s_pack_manchester_data_segment(spi_tx_data_start, START_CMD, START_CMD_LEN_MANCH, false);

	//For loop for filling 1 color of data at a time.
	for (size_t i = 0; i < (num_color_pixels*3); i++)
	{
	
		if(color_data != NULL)
		{
			#ifdef USE_GAMMA_CORR
				color_data_local = gamma_lookup(*(color_data+i));
			#else
				color_data_local = *(color_data+i);
			#endif /* MACRO */
		}
		else
		{
			color_data_local = 0x00;
		}


		spi_tx_data_last = s_pack_manchester_data_segment(spi_tx_data_start, (uint64_t)color_data_local, COLOR_DATA_LEN_MANCH, false);	
		
		if (i == ((num_color_pixels*3)-1))	//Last color in data. Call pack_manchester_data_segment with last_segment_flag = true.
		{
			spi_tx_data_last = s_pack_manchester_data_segment(spi_tx_data_start, (uint64_t)color_data_local, COLOR_DATA_LEN_MANCH, true);
		}		
	
	}

} 

//Generic function for packing manchester data bits to memory. Aligns correctly in memory with SPI transfers
static void *s_pack_manchester_data_segment(uint8_t *spi_mem_data_p_start, uint64_t data_in, uint32_t bit_length_manch, bool last_segment_flag)
{
	
	//ESP32 is a little endian chip whose lowest byte is stored in the very beginning of the address for a uint16 and uint32 variables. 
	//SPI sends MSB first, meaning that it will send bit 7->0 in the memory. 

	//This function takes a color data word (uint64_t) that could be data for one pixel or a start command, encodes it to manchester code, and stores it in memory from the starting point pointed by *spi_mem_data_p_start.
	//The function also packs the manchester bits. I.e. if there are any remaining manchester bits at the end that did not fill an entire byte that could be written to memory, they are stored to the next iteration of the function. 
	//If there are remaining bits left at the end AND the argument last_segment_flag is true, then they get padded with zeros and written to memory. 
	//These last zeros will be transmitted over SPI (since it can't transmit less than 1 byte), but it won't really matter since the manchester decoder will se "nothing".

	static uint8_t manch_byte_temp = 0;
	static uint8_t manch_byte_pointer = 8;	
	static uint32_t mem_p_offset = 0;
	uint8_t *spi_mem_data_p_last = spi_mem_data_p_start + mem_p_offset;

	for (size_t i = 0; i < bit_length_manch; i++)
	{
		manch_byte_pointer = manch_byte_pointer - 2;	//Offset with 2 bits so that a manchester bit can fit (1=10, 0=01)

		//See issue for explanation of what goes on here: https://github.com/mikbeng/esp32_TLS3001_spi/issues/3
		manch_byte_temp |= (MANCHESTER_VALUE( (uint8_t)((data_in >> (bit_length_manch - i -1)) & 0x01) ) << manch_byte_pointer);
		
		if (manch_byte_pointer == 0)	//byte full. Write byte to memory
		{
			*(spi_mem_data_p_start+mem_p_offset) = manch_byte_temp;
			spi_mem_data_p_last = spi_mem_data_p_start+mem_p_offset;
			mem_p_offset ++;	//Offset-pointer variable points to next free memory

			manch_byte_temp = 0;	//reset temporary byte 
			manch_byte_pointer = 8;
		}
			
	}

	if((manch_byte_pointer != 0) && (last_segment_flag == true))
	{
		//We have remaining manchester data that didnt make it to one entire byte

		//Write last remaining data. Empty bits will be zero.
		*(spi_mem_data_p_start+mem_p_offset) = manch_byte_temp;
		spi_mem_data_p_last = spi_mem_data_p_start+mem_p_offset;

		manch_byte_temp = 0;	//reset temporary byte 
		manch_byte_pointer = 8;
		mem_p_offset = 0;	//reset pointer offset
	
	}

	//Return pointer to last data.
	return spi_mem_data_p_last;	
}

static esp_err_t s_TLS3001_send_packet(uint8_t *data, uint32_t length, spi_device_handle_t spi_handle)
{
	esp_err_t ret;
	static spi_transaction_t t;

	memset(&t, 0, sizeof(t));        	//Zero out the transaction
	t.tx_buffer = data;
	t.length = length;
	t.rxlength = 0;

	ret = spi_device_transmit(spi_handle, &t);
	//ret = spi_device_polling_transmit(spi_handle, &t); 	//Transmit!
	if(ret != ESP_OK)
	{
		ESP_LOGW(__func__, "spi_device_transmit(): returned %d", ret);
		return ret;
	}
	
	return ESP_OK;
}

static esp_err_t s_TLS3001_send_color_packet(uint8_t *spi_tx_data_start, uint16_t num_pixels, spi_device_handle_t spi_handle)
{
	//Function for sending start and colorpacket from memory pointed by arg self->spi_tx_data_start
	esp_err_t ret;
	ret = spi_device_acquire_bus(spi_handle, portMAX_DELAY);
	if(ret != ESP_OK)
	{
		ESP_LOGE(__func__, "spi_device_acquire_bus(): returned %d", ret);
		return ret;
	}

	ret = s_TLS3001_send_packet(spi_tx_data_start, ((num_pixels*PIXEL_DATA_LEN_SPI)+START_CMD_LEN_SPI), spi_handle);
	if(ret != ESP_OK)
	{
		ESP_LOGE(__func__, "TLS3001_send_packet(): returned %d", ret);
		return ret;
	}

	ets_delay_us(150); 

	ret = s_TLS3001_send_packet((uint8_t *)&start_cmd, START_CMD_LEN_SPI, spi_handle);
	if(ret != ESP_OK)
	{
		ESP_LOGE(__func__, "TLS3001_send_packet(): returned %d", ret);
		return ret;
	}

	spi_device_release_bus(spi_handle);	

	return ESP_OK;

}

static esp_err_t s_TLS3001_send_resetsynch_packet(spi_device_handle_t spi_handle) 
{
	// Prepare the reset and synch commands
	s_pack_manchester_data_segment(reset_cmd, RESET_CMD, RESET_CMD_LEN_MANCH, true);	
	s_pack_manchester_data_segment(synch_cmd, SYNCH_CMD, SYNCH_CMD_LEN_MANCH, true);
	s_pack_manchester_data_segment(start_cmd, START_CMD, START_CMD_LEN_MANCH, true);

	// See https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/spi_master.html#bus-acquiring
	ESP_ERROR_CHECK(spi_device_acquire_bus(spi_handle, portMAX_DELAY));

	//Send RESET	
	ESP_ERROR_CHECK(s_TLS3001_send_packet(reset_cmd, RESET_CMD_LEN_SPI,spi_handle));			
	
	//Delays 1ms. 
	/*The ROM function ets_delay_us() (defined in rom/ets_sys.h) will allow you to busy-wait for a correct number of microseconds. 
	Note that this is busy-waiting, so unlike vTaskDelay it does not allow other tasks to run (it just burns CPU cycles.)
	*/
	ets_delay_us(1000); 										
	
	//Send SYNCH
	ESP_ERROR_CHECK(s_TLS3001_send_packet(synch_cmd, SYNCH_CMD_LEN_SPI,spi_handle));

	spi_device_release_bus(spi_handle);


	return ESP_OK;
}

/* ========================================================================= */
/* [FUNC] Functions implementations                                          */
/* ========================================================================= */

esp_err_t TLS3001_init(uint16_t num_pixels_ch1, uint16_t num_pixels_ch2)
{

    xQueueSet = xQueueCreateSet(TLS3001_TASKQUEUE_CH1_LEN + TLS3001_TASKQUEUE_CH2_LEN);
    configASSERT( xQueueSet );

    if(num_pixels_ch1 > 0)
    {
        s_TLS3001_ch1_init(num_pixels_ch1);
        TLS3001_state_keeper.ch1_initiated = true;
    }
    
    if(num_pixels_ch2 > 0)
    {
        s_TLS3001_ch2_init(num_pixels_ch2);
        TLS3001_state_keeper.ch2_initiated = true;
    }

    if(num_pixels_ch1 <= 0 && num_pixels_ch2 <= 0)
    {
        ESP_LOGE(TAG, "Error init. Number of pixels must be at least > 0 on either ch1 and/or ch2");
        return ESP_FAIL;
    }
    
    xTaskCreate(&s_TLS3001_task, "TLS3001_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);

    return ESP_OK;
}

esp_err_t TLS3001_send_to_queue(pixel_message_s *pixel_message_packet_p, uint8_t channel)
{
    if(channel == 1)
    {
        if(TLS3001_state_keeper.ch1_initiated != true)
        {
            ESP_LOGE(TAG, "Unable to post data on ch1 queue since ch1 is not initiated!");
            return ESP_FAIL;
        }
        //Send copy of pointer to pixel_message_s structure to TLS3001 task
        if(xQueueSend(TLS3001_handle_ch1.TLS3001_input_queue, (void *) &pixel_message_packet_p,(TickType_t )10))
        {
            #if TLS3001_DEBUG_PRINTS_EN 
                ESP_LOGI(TAG, "successfully posted pattern data on ch1 queue");
            #endif
        }
        else
        {
            ESP_LOGW(TAG, "Ch1 Queue full. Did not post any data!");
        }        
    }
    else if(channel == 2)
    {
        if(TLS3001_state_keeper.ch2_initiated != true)
        {
            ESP_LOGE(TAG, "Unable to post data on ch2 queue since ch2 is not initiated!");
            return ESP_FAIL;
        }
        //Send copy of pointer to pixel_message_s structure to TLS3001 task
        if(xQueueSend(TLS3001_handle_ch2.TLS3001_input_queue, (void *) &pixel_message_packet_p,(TickType_t )10))
        {
            #if TLS3001_DEBUG_PRINTS_EN 
                ESP_LOGI(TAG, "successfully posted pattern data on ch2 queue");
            #endif
        }
        else
        {
            ESP_LOGW(TAG, "Ch2 Queue full. Did not post any data!");
        }            
    }
    else
    {
        ESP_LOGE(TAG, "Wrong channel selection. Channel argument: %d", channel);
        return ESP_FAIL;
    }
	
	return ESP_OK;
}