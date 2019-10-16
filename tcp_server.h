#ifndef MY_TCP_SERVER_H_
#define MY_TCP_SERVER_H_

#include <stdio.h>
#include <string.h>
#include <sys/fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
//#include "soft_AP_DHCP.h"
#include "spi.h"

EventGroupHandle_t wifi_event_group;
void tcp_server(void *pvParameters);
esp_err_t event_handler(void *ctx, system_event_t *event);
void start_dhcp_server();
void initialise_wifi_in_ap(void);
void indic(int count);

#endif /* MAIN_TCP_SERVER_H_ */
