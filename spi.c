/*
 * spi.c
 *
 *  Created on: Feb 2, 2019
 *      Author: Pavel
 */
#include "spi.h"


uint8_t who_am_i[3];
uint8_t ad_rec[10];
uint8_t buffer[100];
size_t dma;
int16_t gyro;
int16_t temp;
Accel a;
int16_t result[100];
int32_t transf;

void spi_setup(spi_device_handle_t * spi1, spi_device_handle_t * spi2, spi_device_handle_t * spi3)
{
	spi_bus_config_t buscfg={
				        .miso_io_num=PIN_NUM_VSPI_Q,
				        .mosi_io_num=PIN_NUM_VSPI_D,
				        .sclk_io_num=PIN_NUM_CLK,
				        .quadwp_io_num=-1,
				        .quadhd_io_num=-1,
				        .max_transfer_sz=4094*16,
				    };
	spi_device_interface_config_t devcfg={
				        .clock_speed_hz=10*1000*1000,           //Clock out at 2 MHz
				        .command_bits=0,
						.address_bits=0,
						.mode=3,                                //SPI mode 0
						.cs_ena_pretrans = 1,
						.cs_ena_posttrans = 1,
				        .spics_io_num=PIN_NUM_CS0,               //CS pin
				        .queue_size=70,  	//We want to be able to queue 7 transactions at a time
						.flags=0,
				    };
	ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, 1));
	ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, spi1));
	devcfg.spics_io_num = PIN_NUM_CS1;
	ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, spi2));
	devcfg.spics_io_num = PIN_NUM_CS2;
	ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, spi3));
}



void accel_init(spi_device_handle_t * spi)
{
	spi_transaction_t trans;
	spi_transaction_t * r_trans;
	memset(&trans, 0, sizeof(spi_transaction_t));
	trans.flags=SPI_TRANS_USE_TXDATA;
	trans.length=16;


	trans.tx_data[0]=ICM20602_SMPLRT_DIV;
	trans.tx_data[1]= 0;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.tx_data[0]=ICM20602_CONFIG;
	trans.tx_data[1] = 1; // DLPF gyro
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.tx_data[0] = ICM20602_GYRO_CONFIG;
	trans.tx_data[1] = (1<<3);
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.tx_data[0]=ICM20602_ACCEL_CONFIG;
	trans.tx_data[1]= (1<<4)|(1<<3); // +-16g
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.tx_data[0]=ICM20602_ACCEL_CONFIG2;
	trans.tx_data[1]=1<<3; //  DLPF acc
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.tx_data[0]=ICM20602_FIFO_EN;
	trans.tx_data[1]=0; //(1<<3) | (1<<4) ; // enable write acc data | gyro data
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.tx_data[0]=ICM20602_FIFO_WM_TH1;
	trans.tx_data[1]=0;// 0x1;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.tx_data[0]=ICM20602_FIFO_WM_TH2;
	trans.tx_data[1]= 0;// 0xC0; // treshold that trigers intr, 448 bytes (7*2*32)
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.tx_data[0] = ICM20602_ACCEL_INTEL_CTRL;
	trans.tx_data[1] = 0; // intell control
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.tx_data[0]=ICM20602_USER_CTRL;
	trans.tx_data[1]= 0; //(1<<6); // fifo enable | reset fifo
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.tx_data[0]=ICM20602_PWR_MGMT_1;
	trans.tx_data[1]= 1; //cycle mode | auto select clock
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.tx_data[0]=ICM20602_I2C_IF;
	trans.tx_data[1]=(1<<6);
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

//	trans.tx_data[0]=ICM20602_INT_ENABLE;
//	trans.tx_data[1]=(1<<2);
//	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
//	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));



}

void acc_who_i_am(spi_device_handle_t * spi, uint8_t i)
{
	spi_transaction_t trans;
	memset(&trans, 0, sizeof(spi_transaction_t));
	trans.tx_data[0] = (1<<7)| (ICM20602_WHO_AM_I);
	trans.length = 16;
	trans.rxlength=8;
	trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	spi_transaction_t * r_trans;
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));
	who_am_i[0] = r_trans->rx_data[0];
	who_am_i[1] = r_trans->rx_data[1];
	who_am_i[2] = r_trans->rx_data[2];

//	trans.addr = (0x80 | ICM20602_ACCEL_CONFIG);
//	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
//	memset(&r_trans, 0, sizeof(spi_transaction_t));
//	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));
//
//	trans.addr = (0x80 | 0x1A);
//	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
//	memset(&r_trans, 0, sizeof(spi_transaction_t));
//	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

//	who_am_i[i] = r_trans->rx_data[0];

}



//void adc_setup(spi_device_handle_t * spi)
//{
//	spi_transaction_t trans;
//	memset(&trans, 0, sizeof(spi_transaction_t));
//	trans.addr = (AD_7797_MODE); // write to MODE
//	trans.tx_data[0] = 0;
//	trans.tx_data[1] = 0xF; // slowest update rate
//	trans.length = 24;
//	trans.flags = SPI_TRANS_USE_TXDATA;
//	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
//	spi_transaction_t * r_trans;
//	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));
//	memset(&trans, 0, sizeof(spi_transaction_t));
//	trans.addr = (AD_7797_CONFG); // write to configuration
//	trans.tx_data[0] = (1<<4);
//	trans.tx_data[1] = (1<<0) | (1<<1) | (1<<2); // bipolar
//	trans.length = 24;
//	trans.flags = SPI_TRANS_USE_TXDATA;
//	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
//	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));
//
//
//}

void IRAM_ATTR get_data(void *pvParameter)
{
	SpiEventGroup = xEventGroupCreate();
	xTaskToNotify = NULL;
//	spi_device_handle_t spi1;
//	spi_device_handle_t spi2;
	spi_device_handle_t spi3;
	uint8_t * dma_buf = (uint8_t *)heap_caps_malloc(DMA_BUFF_SIZE, MALLOC_CAP_DMA);
	uint8_t * addr_dma_buf = (uint8_t *)heap_caps_malloc(DMA_BUFF_SIZE, MALLOC_CAP_DMA);
	int16_t a_buf[6];
	uint64_t time;
	uint32_t num1 = 0;
	uint32_t num2 = 0;
	uint32_t adc_buf;
	uint8_t buf[MAX_PRTBUF_SIZE];

	spi_setup(&spi1, &spi2, &spi3);
//	while(1)
//	{
//	acc_who_i_am(&spi1, 0);
//	vTaskDelay(1 / portTICK_PERIOD_MS);
//	}
//	accel_init(&spi1);
//	accel_init(&spi2);
//	while(1)
//	{
//	get_data_acc(&spi1, addr_dma_buf,  dma_buf, a_buf);
//	}
	//	while(1)
//	{
//		test(&spi2);
//		vTaskDelay(1 / portTICK_PERIOD_MS);
//	}

	pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
	partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
	esp_partition_erase_range(partition, 0, partition->size);



	while(1)
	{

		xEventGroupWaitBits(SpiEventGroup,
						BIT5 | BIT2,  // BIT5 -- button, BIT2 -- setting received
						pdTRUE,         // should be cleared before returning.
						pdTRUE,        //  wait for both bits, either bit will do.
						portMAX_DELAY );

		xEventGroupClearBits(SpiEventGroup, BIT5 | BIT2);
		accel_init(&spi1);
		accel_init(&spi2);

		num1 = 0;
		num2 = 0;
//	*********Blink 3 times*****************************************
		gpio_set_level(26, 1);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		for(int i = 0; i < 2; i++)
		{
	   		gpio_set_level(26, 0);
	   		vTaskDelay(1000 / portTICK_PERIOD_MS);
	   		gpio_set_level(26, 1);
	   		vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
		gpio_set_level(26, 0);

//	***************************************************************

		vTaskSuspend(button_task_handle);
		timer_start(0, 0);
		timer_set_counter_value(0,0,0);
		while(num1 < NUM_OF_FIELDS || num2 < NUM_OF_FIELDS)
		{
			if(num1 < NUM_OF_FIELDS)
    		{
//    			memset(dma_buf, 0, DMA_BUFF_SIZE);
//    			memset(buf, 0, sizeof(buf));
//    			memset(a_buf, 0, sizeof(a_buf));
//    			memset(&stream, 0, sizeof(pb_ostream_t));
    			get_data_acc(&spi1, addr_dma_buf, dma_buf, a_buf);
    			timer_get_counter_value(0,0, &time);
    			a.last_msg = false;
    			a.a_x = a_buf[0];
    			a.a_y = a_buf[1];
    			a.a_z = a_buf[2];
    			a.g_x = a_buf[3];
    			a.g_y = a_buf[4];
    			a.g_z = a_buf[5];
    			a.number = num1;
    			a.up = true;
 				a.time = (uint32_t) ((time) / 80); // to microsec
    			size_t d_size = 0;
    			pb_get_encoded_size(&d_size, Accel_fields, &a);
    			data_size[num1] = (uint8_t)d_size;
    			stream = pb_ostream_from_buffer(buf, MAX_PRTBUF_SIZE);
    			pb_encode(&stream, Accel_fields, &a);
				esp_partition_write(partition, (sizeof(buf))*num1, buf, sizeof(buf));
    			num1++;
    		}

			if(num2 < NUM_OF_FIELDS)
    		{
//    			memset(dma_buf, 0, DMA_BUFF_SIZE);
//    			memset(buf, 0, sizeof(buf));
//    			memset(a_buf, 0, sizeof(a_buf));
//    			memset(&stream, 0, sizeof(pb_ostream_t));
    			get_data_acc(&spi2, addr_dma_buf, dma_buf, a_buf);
    			timer_get_counter_value(0,0, &time);
    			a.last_msg = false;
    			a.a_x = a_buf[0];
    			a.a_y = a_buf[1];
    			a.a_z = a_buf[2];
    			a.g_x = a_buf[3];
    			a.g_y = a_buf[4];
    			a.g_z = a_buf[5];
    			a.number = num2;
    			a.up = false;
 				a.time = (uint32_t) ((time) / 80); // to microsec
    			size_t d_size = 0;
    			pb_get_encoded_size(&d_size, Accel_fields, &a);
    			data_size[num2+NUM_OF_FIELDS] = (uint8_t)d_size;
    			stream = pb_ostream_from_buffer(buf, MAX_PRTBUF_SIZE);
    			pb_encode(&stream, Accel_fields, &a);
				esp_partition_write(partition, (sizeof(buf))*(num2+NUM_OF_FIELDS), buf, sizeof(buf));
    			num2++;
    		}
    	}
		xEventGroupSetBits(SpiEventGroup, BIT1);
		indic(3);
		vTaskResume(button_task_handle);
	}
}

int16_t read_low_high_byte(uint8_t count, int8_t * dma_buf)
{
	int16_t high_b = dma_buf[2 * count];
	int16_t low_b = dma_buf[2 * count + 1];
	int16_t x = low_b | (high_b<<8);
	return x;
}
void test(spi_device_handle_t * spi)
{
	uint8_t * buffer_dma = (uint8_t *)heap_caps_malloc(100, MALLOC_CAP_DMA);
	memset(buffer_dma, 0, 100);
	spi_transaction_t trans;
	memset(&trans, 0, sizeof(spi_transaction_t));
	uint8_t addr = (1<<7)| ICM20602_GYRO_XOUT_H;
	trans.tx_buffer = &addr;
	trans.length = 34*8;
	trans.rxlength=33*8;
	trans.flags = 0;
	trans.rx_buffer = buffer_dma;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	spi_transaction_t * r_trans;
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));
	for(int i = 0; i< 100; i++)
		{
			buffer[i] = buffer_dma[i];
		}
	heap_caps_free(buffer_dma);
}

void get_data_acc_fifo(spi_device_handle_t * spi, int8_t * dma_buf)
{

	spi_transaction_t trans;
	memset(&trans, 0, sizeof(spi_transaction_t));
	uint8_t addr = (1<<7) | ICM20602_FIFO_R_W;
	trans.tx_buffer = &addr;
	trans.rx_buffer=dma_buf;
	trans.length = DMA_BUFF_SIZE*8+8;
	trans.rxlength = DMA_BUFF_SIZE*8;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	spi_transaction_t * r_trans1;
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans1, portMAX_DELAY));
}

uint8_t check_intr(spi_device_handle_t * spi)
{
	spi_transaction_t trans;
	uint8_t intr;// = pvPortMallocCaps(8, MALLOC_CAP_DMA);

	memset(&trans, 0, sizeof(spi_transaction_t));
	uint8_t addr = (1<<7) | ICM20602_FIFO_WM_INT;
	trans.tx_buffer = &addr;
	trans.rx_buffer=&intr;
	trans.length = 16;
	trans.rxlength=8;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	spi_transaction_t * r_trans1;
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans1, portMAX_DELAY));
	if(intr & (1<<6))
	{
		return(1);
	}
	else
		return(0);
}

void get_data_acc(spi_device_handle_t * spi, uint8_t * dma_addr,  uint8_t * tr, int16_t * a_buf)
{
	spi_transaction_t trans;
	spi_transaction_t * r_trans1;
	memset(&trans, 0, sizeof(spi_transaction_t));
	//***********data ready?*****************
	trans.tx_data[0] = (1<<7) | ICM20602_INT_STATUS;
	trans.length = 16;
	trans.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;


	while(1)
	{
		ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
		ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans1, portMAX_DELAY));
		if(r_trans1->rx_data[1] & 1)
		{
			break;
		}
	}
	//**********reading data*****************
	memset(dma_addr, 0, 15);
	dma_addr[0] = (1<<7) | ICM20602_ACCEL_XOUT_H;
	trans.tx_buffer = dma_addr;
	trans.rx_buffer = tr;
	trans.length = 15*8;
	trans.rxlength = 15*8;
	trans.flags = 0;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans1, portMAX_DELAY));
	a_buf[0] = (tr[1]<<8) | tr[2]; // acc_x
	a_buf[1] = (tr[3]<<8) | tr[4]; // acc_y
	a_buf[2] = (tr[5]<<8) | tr[6]; // acc_z
	a_buf[3] = (tr[9]<<8) | tr[10]; // gyro_x
	a_buf[4] = (tr[11]<<8) | tr[12]; // gyro_y
	a_buf[5] = (tr[13]<<8) | tr[14]; // gyro_z
	for (int i = 0; i<15; i++)
	{
		buffer[i] = tr[i];
	}
	for (int i = 0; i < 6; i++)
	{
		result[i] = a_buf[i];
	}
//	memcpy(result, tr, 3);
//	heap_caps_free(tr);
//	return(result);
}

void get_data_adc(spi_device_handle_t *spi, uint32_t * buf)
{
	spi_transaction_t trans[2];
	memset(&trans[0], 0, sizeof(spi_transaction_t));
	trans[0].tx_data[0] = 0xFF;
	trans[0].tx_data[1] = 0xFF;
	trans[0].tx_data[2] = 0xFF;
	trans[0].tx_data[3] = 0xFF;
	trans[0].length = 32;
	trans[0].flags = SPI_TRANS_USE_TXDATA;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans[0], portMAX_DELAY));
	spi_transaction_t * r_trans;
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	memset(&trans[0], 0, sizeof(spi_transaction_t));
	trans[0].tx_data[0] = (0x7C & ((1<<6) | AD_7797_ID )); // read id
	trans[0].length = 8;
	trans[0].flags = SPI_TRANS_USE_TXDATA;
	memset(&trans[1], 0, sizeof(spi_transaction_t));
	trans[1].rxlength=8;
	trans[1].length = 8;
	trans[1].rx_data[0] = 0;
	trans[1].flags = SPI_TRANS_USE_RXDATA;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans[0], portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans[1], portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));
	ad_rec[0] = r_trans->rx_data[0];

	memset(&trans[0], 0, sizeof(spi_transaction_t));
	trans[0].tx_data[0] = (0x7C & ((1<<6) | AD_7797_MODE )); // read id
	trans[0].length = 8;
	trans[0].flags = SPI_TRANS_USE_TXDATA;
	memset(&trans[1], 0, sizeof(spi_transaction_t));
	trans[1].rxlength=16;
	trans[1].length = 16;
	trans[1].flags = SPI_TRANS_USE_RXDATA;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans[0], portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans[1], portMAX_DELAY));
//	spi_transaction_t * r_trans;
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));
	ad_rec[1] = r_trans->rx_data[0];
	ad_rec[2] = r_trans->rx_data[1];

	memset(&trans[0], 0, sizeof(spi_transaction_t));
	trans[0].tx_data[0] = (0x7C & ((1<<6) | AD_7797_CONFG )); // read id
	trans[0].length = 8;
	trans[0].flags = SPI_TRANS_USE_TXDATA;
	memset(&trans[1], 0, sizeof(spi_transaction_t));
	trans[1].rxlength=16;
	trans[1].length = 16;
	trans[1].flags = SPI_TRANS_USE_RXDATA;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans[0], portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans[1], portMAX_DELAY));
//	spi_transaction_t * r_trans;
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));
	ad_rec[3] = r_trans->rx_data[0];
	ad_rec[4] = r_trans->rx_data[1];

	memset(&trans[0], 0, sizeof(spi_transaction_t));
	trans[0].tx_data[0] = (0x7C & ((1<<6) | AD_7797_STATUS )); // read id
	trans[0].length = 8;
	trans[0].flags = SPI_TRANS_USE_TXDATA;
	memset(&trans[1], 0, sizeof(spi_transaction_t));
	trans[1].rxlength=16;
	trans[1].length = 16;
	trans[1].flags = SPI_TRANS_USE_RXDATA;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans[0], portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans[1], portMAX_DELAY));
//	spi_transaction_t * r_trans;
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));
	ad_rec[5] = r_trans->rx_data[0];

}
