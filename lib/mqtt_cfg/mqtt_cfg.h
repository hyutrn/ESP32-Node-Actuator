#ifndef _MQTT_CFG_H_
#define _MQTT_CFG_H_

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
//#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "wifi_pro.h"
#include "server_cfg.h"
#include "dht11.h"
#include "light_sen.h"
#include "mois.h"
#include "client_cfg.h"

extern esp_mqtt_client_handle_t client;

void dataSend(esp_mqtt_client_handle_t client);
esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event);
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void mqtt_app_start(void);
#endif 