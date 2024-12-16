#ifndef _SERVER_CFG_H_
#define _SERVER_CFG_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "string.h"
#include <sys/param.h>
#include "esp_wifi.h"
#include <stdbool.h>

#include "lwip/err.h"
#include "lwip/sys.h"
#include "cJSON.h"
#include "wifi_pro.h"
#include "project_system_files.h"
#include "client_cfg.h"
#include "shared.h"
#include "nvs_storage.h"

esp_err_t get_url_server(httpd_req_t *req);
esp_err_t post_handler(httpd_req_t *req);
httpd_handle_t server_start(void);
void server_stop(httpd_handle_t server_handle);

extern int flag_call_mqtt;
// Biến kiểm tra trạng thái
extern int server_initialized;
extern int register_node;
extern esp_err_t check_post_client;


#endif 