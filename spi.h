/*
 * spi.h
 *
 *  Created on: Feb 2, 2019
 *      Author: Pavel
 */

#ifndef MAIN_SPI_H_
#define MAIN_SPI_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h> /* memset */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_app_trace.h"
#include "esp_log.h"
#include "esp_err.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "ICM-20602.h"
#include "AD-7797BRUZ.h"
#include "esp_heap_alloc_caps.h"
#include "mssg.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_common.h"
#include "pb.h"
#include "tcp_server.h"
#include "esp_heap_caps.h"
#include "esp_heap_caps_init.h"
#include "button.h"

#define PIN_NUM_VSPI_Q 19 //miso
#define PIN_NUM_VSPI_D 23 //mosi
#define PIN_NUM_CLK  18
#define PIN_NUM_CS0   5 // local acc
#define PIN_NUM_CS1   4 // remote acc
#define PIN_NUM_CS2   2 // adc
#define NUM_OF_FIELDS	8000
#define MAX_PRTBUF_SIZE		100
#define EL_IN_BURST		32 // change ICM20602_FIFO_WM_TH if you are changing that
#define DMA_BUFF_SIZE	15


spi_device_handle_t spi1;
spi_device_handle_t spi2;

uint8_t  dlpf_acc;
TaskHandle_t xTaskToNotify;
const esp_partition_t *partition;
uint8_t data_size[2 * NUM_OF_FIELDS + 34];
EventGroupHandle_t SpiEventGroup;
void spi_setup(spi_device_handle_t * spi1, spi_device_handle_t * spi2, spi_device_handle_t * spi3);
void get_data_acc(spi_device_handle_t * spi, uint8_t * dma_addr,  uint8_t * tr, int16_t * result);
void accel_init(spi_device_handle_t * spi);
void acc_who_i_am(spi_device_handle_t * spi, uint8_t i);
void adc_setup(spi_device_handle_t * spi2);
void get_data(void *pvParameter);
void get_data_adc(spi_device_handle_t *spi, uint32_t * buf);
void get_data_acc_fifo(spi_device_handle_t * spi, int8_t * dma_buf);
void test(spi_device_handle_t * spi);
int16_t read_low_high_byte(uint8_t count, int8_t * dma_buf);
uint8_t check_intr(spi_device_handle_t * spi);



#endif /* MAIN_SPI_H_ */
