#include "tcp_server.h"

#define LISTENQ 2
#define MESSAGE "Hello TCP Client!!"
#define AP_SSID "ESP_32"
#define AP_PASSPHARSE "12345678"
#define AP_SSID_HIDDEN 0
#define AP_MAX_CONNECTIONS 4
#define AP_AUTHMODE WIFI_AUTH_OPEN // the passpharese should be atleast 8 chars long
#define AP_BEACON_INTERVAL 100 // in milli seconds


const int CLIENT_CONNECTED_BIT = BIT0;
const int CLIENT_DISCONNECTED_BIT = BIT1;
const int AP_STARTED_BIT = BIT2;
static const char *TAG = "tcp_server";
Accel a4;
//size_t buf_size;
//size_t s;
//size_t delta;


esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_AP_START:
		printf("Event:ESP32 is started in AP mode\n");
        xEventGroupSetBits(wifi_event_group, AP_STARTED_BIT);
        break;

	case SYSTEM_EVENT_AP_STACONNECTED:
		xEventGroupSetBits(wifi_event_group, CLIENT_CONNECTED_BIT);
		break;

	case SYSTEM_EVENT_AP_STADISCONNECTED:
		xEventGroupSetBits(wifi_event_group, CLIENT_DISCONNECTED_BIT);
		break;
    default:
        break;
    }
    return ESP_OK;
}

void start_dhcp_server(){

    // initialize the tcp stack
	tcpip_adapter_init();
    // stop DHCP server
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
    // assign a static IP to the network interface
    tcpip_adapter_ip_info_t info;
    memset(&info, 0, sizeof(info));
    IP4_ADDR(&info.ip, 192, 168, 0, 1);
    IP4_ADDR(&info.gw, 192, 168, 0, 1);//ESP acts as router, so gw addr will be its own addr
    IP4_ADDR(&info.netmask, 255, 255, 255, 0);
    ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &info));
    // start the DHCP server
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
    printf("DHCP server started \n");
}
void initialise_wifi_in_ap(void)
{
    esp_log_level_set("wifi", ESP_LOG_NONE); // disable wifi driver logging
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_AP) );

    // configure the wifi connection and start the interface
	wifi_config_t ap_config = {
        .ap = {
            .ssid = AP_SSID,
            .password = AP_PASSPHARSE,
			.ssid_len = 0,
			.channel = 0,
			.authmode = AP_AUTHMODE,
			.ssid_hidden = AP_SSID_HIDDEN,
			.max_connection = AP_MAX_CONNECTIONS,
			.beacon_interval = AP_BEACON_INTERVAL,
        },
    };
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK( esp_wifi_start() );
    printf("ESP WiFi started in AP mode \n");
}

void indic(int count)
{
    for(int i = 0; i < count; i++)
    {
    	gpio_set_level(26, 1);
    	vTaskDelay(50 / portTICK_PERIOD_MS);
    	gpio_set_level(26, 0);
    	vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
}

void IRAM_ATTR tcp_server(void *pvParameters)
{
    ESP_LOGI(TAG,"tcp_server task started \n");
    struct sockaddr_in tcpServerAddr;
    tcpServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    tcpServerAddr.sin_family = AF_INET;
    tcpServerAddr.sin_port = htons( 3000 );
    int s, r;
    uint8_t buf[MAX_PRTBUF_SIZE];
    uint8_t recv_buf=0;
    uint8_t setting_buf = 0;
    static struct sockaddr_in remote_addr;
    static unsigned int socklen;
    socklen = sizeof(remote_addr);
    int cs;//client socket
    xEventGroupWaitBits(wifi_event_group,AP_STARTED_BIT,false,true,portMAX_DELAY);
    while(1){
        s = socket(AF_INET, SOCK_STREAM, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.\n");
//            indic(1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket\n");
         if(bind(s, (struct sockaddr *)&tcpServerAddr, sizeof(tcpServerAddr)) != 0) {
            ESP_LOGE(TAG, "... socket bind failed errno=%d \n", errno);
            close(s);
//            indic(2);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket bind done \n");
        if(listen (s, LISTENQ) != 0) {
            ESP_LOGE(TAG, "... socket listen failed errno=%d \n", errno);
            close(s);
//            indic(3);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        while(1)
        {

            cs=accept(s,(struct sockaddr *)&remote_addr, &socklen);
            ESP_LOGI(TAG,"New connection request,Request data:");
            //set O_NONBLOCK so that recv will return, otherwise we need to impliment message end
            //detection logic. If know the client message format you should instead impliment logic
            //detect the end of message
            fcntl(cs,F_SETFL,O_NONBLOCK);
        	uint16_t r = 0;
        	indic(2);
        	uint16_t delay = 0;

//        	uint8_t n = 0;
//        	partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
//        	xEventGroupWaitBits(
//        	            	                 SpiEventGroup,    // The event group being tested.
//        	            	                 BIT0 | BIT3,  // The bits within the event group to wait for.
//        	            	                 pdTRUE,         // BIT_0 and BIT_4 should be cleared before returning.
//        	            	                 pdFALSE,        // Don't wait for both bits, either bit will do.
//        	            	                 portMAX_DELAY);
//        	pb_istream_t stream_in = pb_istream_from_buffer(buf, sizeof(buf));
        	Accel a = Accel_init_default;
        	pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
        	memset(buf, 0, sizeof(buf));
//        	size_t g_size;
//        	pb_get_encoded_size(&g_size, Accel_fields, &a);
//        	pb_encode(&stream, Accel_fields, &a);
//        	s = 0;
//        	while(s < g_size)
//        	{
//        	 	s = write(cs , buf, g_size);
//        	}
        	while(1)
        	{
        		r = 0;
        		memset(&setting_buf, 0, sizeof(uint8_t));
        		memset(data_size, 0, sizeof(data_size));
        		while(~setting_buf & (1<<7))
        		{
        			r = recv(cs, &setting_buf, sizeof(uint8_t), MSG_WAITALL);
        		}

        		dlpf_acc = 0;
        		xEventGroupSetBits(SpiEventGroup, BIT2); // BIT2 -- received setting
        		if((setting_buf & ((1<<2)|(1<<1)|(1))) == 1)
        		{
        			accel_init(&spi1);
        			accel_init(&spi2);
        			vTaskDelay(1 / portTICK_PERIOD_MS);
                	while(1)
                	{
        //        		get_data_acc_fifo(&spi2, test_buf);
                		stream = pb_ostream_from_buffer(buf, sizeof(buf));
                		uint8_t * tr = (uint8_t *)heap_caps_malloc(DMA_BUFF_SIZE, MALLOC_CAP_DMA);
                		uint8_t * dma_addr = (uint8_t *)heap_caps_malloc(DMA_BUFF_SIZE, MALLOC_CAP_DMA);
                		int16_t a_buf[6];
                		get_data_acc(&spi2, dma_addr, tr, a_buf);
                		a.last_msg = false;
                		a.a_x = a_buf[0];
                		a.a_y = a_buf[1];
                		a.a_z = a_buf[2];
                		a.g_x = a_buf[3];
                		a.g_y = a_buf[4];
                		a.g_z = a_buf[5];
                		memset(buf, 0, sizeof(buf));
                		size_t g_size;
                		pb_get_encoded_size(&g_size, Accel_fields, &a);
                		pb_encode(&stream, Accel_fields, &a);
                		s = 0;

              			while(s < g_size)
               			{
               				s = write(cs , buf, g_size);
               			}
              			memset(&stream, 0, sizeof(pb_ostream_t));
              			heap_caps_free(tr);
              			heap_caps_free(dma_addr);
              			indic(1);
                	}


        		}
                else
                {
        		xEventGroupWaitBits(SpiEventGroup,    // The event group being tested.
        							BIT1,  // BIT1 -- recording completed
        							pdTRUE,         // should be cleared before returning.
        							pdFALSE,        // Don't wait for both bits, either bit will do.
        							portMAX_DELAY );
        		xEventGroupClearBits(SpiEventGroup, BIT1);

        		for(uint32_t i = 0; i < 2 * NUM_OF_FIELDS; i++)
        		{
        			if(data_size[i] == 0)
        			{
        				continue;
        			}
        			while(1)
        			{
        				memset(buf, 0, sizeof(buf));
        				esp_partition_read(partition, sizeof(buf)*i, buf, sizeof(buf));
//            		memset(&stream_in, 0, sizeof(pb_istream_t));
//            		stream_in = pb_istream_from_buffer(buf, sizeof(buf));
//            		if (!pb_decode(&stream_in, Accel_fields, &a4))
//            		{
//            			continue;
//            		}
//               		a.last_msg = false;
//               		a.a_x = -32768;
//               		a.a_y = 6;
//               		a.a_z = 32768;
//               		a.time = 1;
//               		a.number = 1;
//               		a.up = true;
//               		buf_size = sizeof(buf);
//               		pb_get_encoded_size(&data_size, Accel_fields, &a);
//               		memset(buf, 0, sizeof(buf));
//               		pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
//               		pb_encode(&stream, Accel_fields, &a);
//               		memset(&stream_in, 0, sizeof(pb_istream_t));
//               		stream_in = pb_istream_from_buffer(buf, sizeof(buf));
//               		pb_decode(&stream_in, Accel_fields, &a4);
//               		s = stream.bytes_written;
//               		delta = data_size - s;
//            		vTaskDelay(delay / portTICK_PERIOD_MS);

        				s = 0;
        				r = 0;
        				while(s < data_size[i])
        				{
        					s = write(cs , buf, data_size[i]);
        				}

        				while(1)
        				{
        					memset(&recv_buf, 0, 1);
        					r = recv(cs, &recv_buf, sizeof(uint8_t), MSG_WAITALL);
        					if(recv_buf == 0)
        					{
        						continue;
        					}
        					else if(recv_buf == 0x18 || recv_buf == 0xE7)
        					{
        						break; // received right response
        					}
        				}
        				if(recv_buf == 0x18)
        				{
        					if(delay > 0)
        					{
        						delay--;
        					}
//        				indic(1);
        					break; // success
        				}
        				else
        				{
        					delay++;
        					continue; // failure
        				}
        			}
//        		vTaskDelay(200 / portTICK_PERIOD_MS);

        		}
        		while(1)
        		{
        			a = (Accel)Accel_init_default;
        			a.last_msg = true;
        			memset(buf, 0, sizeof(buf));
        			stream = pb_ostream_from_buffer(buf, sizeof(buf));
        			pb_encode(&stream, Accel_fields, &a);
        			vTaskDelay(100 / portTICK_PERIOD_MS);
        			s = 0;
        			r = 0;
        			while(s < stream.bytes_written)
        			{
        				s = write(cs , buf, stream.bytes_written);
        			}
        			while(1)
        			{
        				memset(&recv_buf, 0, 1);
        				r = recv(cs, &recv_buf, sizeof(uint8_t), MSG_WAITALL);
        				if(recv_buf == 0)
        				{
        					continue;
        				}
        				else if(recv_buf == 0x18 || recv_buf == 0xE7)
        				{
        					break; // received right response
        				}
        			}
        			if(recv_buf == 0x18)
        			{
        				indic(1);
        				break; // success
        			}
        			else
        			{
        				continue; // failure
        			}
        		}
        		esp_partition_erase_range(partition, 0, partition->size);
        		indic(3);
        	}
        	}
       		close(cs);

        }
        ESP_LOGI(TAG, "... server will be opened in 5 seconds");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "...tcp_client task closed\n");
}
