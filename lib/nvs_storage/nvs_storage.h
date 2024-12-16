#ifndef NVS_STORAGE_H_
#define NVS_STORAGE_H_

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "string.h"
#include <sys/param.h>
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "cJSON.h"
#include "wifi_pro.h"
#include "project_system_files.h"
#include "client_cfg.h"
#include "shared.h"

// Hàm lưu SSID và Password vào NVS
void save_wifi_credentials(const char* ssid, const char* password);

// Hàm đọc SSID và Password từ NVS
bool load_wifi_credentials(char* ssid, size_t ssid_size, char* password, size_t password_size);


#endif 