#ifndef _CLIENT_CFG_H_
#define _CLIENT_CFG_H_

#include <string.h>
#include <sys/param.h>
#include <stdlib.h>
#include <ctype.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "esp_http_client.h"
#include "server_cfg.h"
#include "wifi_pro.h"

extern int client_flag;

//GET
esp_err_t client_event_get_handler(esp_http_client_event_handle_t evt);
esp_err_t client_get();

//POST
esp_err_t client_event_post_handler(esp_http_client_event_handle_t evt);
esp_err_t client_post();

#endif