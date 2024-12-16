#ifndef _WIFI_PRO_H_
#define _WIFI_PRO_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "string.h"
#include <sys/param.h>
#include "esp_wifi.h"
#include <stdbool.h>

#include "lwip/err.h"
#include "lwip/sys.h"
#include "cJSON.h"
#include "server_cfg.h"
#include "project_system_files.h"
#include "mqtt_cfg.h"
#include "client_cfg.h"
#include "shared.h"
#include "nvs_storage.h"

//ssid và password cho kết nối STA 
extern char sta_ssid[20];
extern char sta_password[20];
extern char ip_gateway[30];
extern int sta_flag;
extern int ip_flag;
extern esp_netif_ip_info_t ip_info;
extern char ip_address[100];
extern esp_err_t flag_ap;

#define WIFI_STA_CONNECTED_BIT BIT0

extern EventGroupHandle_t wifi_event_group;

void scann();
esp_err_t wifi_sta_mode(void);
esp_err_t wifi_ap_mode(void);
void sta_connect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);
void ap_connect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);
void initialise_wifi(void);

#endif 
