#include "wifi_pro.h"

//WIFI
esp_netif_t *netif_ap = NULL;
esp_netif_t *netif_sta = NULL;

char sta_ssid[20];
char sta_password[20];
char ip_gateway[30];
int sta_flag;
int ip_flag;
char ip_address[100];
esp_netif_ip_info_t ip_info;
esp_err_t flag_ap;

EventGroupHandle_t wifi_event_group;
int CONNECTED_BIT = BIT0;

//Scan WiFi 
void scann(httpd_req_t *req) {
    // configure and run the scan process in blocking mode
    wifi_scan_config_t scan_config = {
        .ssid = 0,
        .bssid = 0,
        .channel = 0,
        .show_hidden = false
    };
    printf("Start scanning...");
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
    printf(" completed!\n");

    vTaskDelay(1000/portTICK_PERIOD_MS);
    // get the list of APs found in the last scan
    uint16_t ap_num;
    wifi_ap_record_t ap_records[20];
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_num));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, ap_records));
    if(ap_num > 0) {
        // Create a JSON object to store the SSID list and status
        cJSON *root = cJSON_CreateObject();
        if (root == NULL) {
            printf("Failed to create JSON object.\n");
            return;
        }

        // Create a JSON array to store the SSID objects
        cJSON *ssidArray = cJSON_CreateArray();
        if (ssidArray == NULL) {
            printf("Failed to create JSON array.\n");
            cJSON_Delete(root);
            return;
        }

        // Add each SSID object to the JSON array
        for (int i = 0; i < ap_num; i++) {
            cJSON *ssidObject = cJSON_CreateObject();
            cJSON_AddItemToObject(ssidObject, "ssid", cJSON_CreateString((const char *)ap_records[i].ssid));
            cJSON_AddItemToArray(ssidArray, ssidObject);
        }   

        // Add the SSID array and status to the root object
        cJSON_AddItemToObject(root, "ssid_available", ssidArray);
        cJSON_AddNumberToObject(root, "status", 200);

        // Convert the JSON object to a JSON string
        char *jsonString = cJSON_PrintUnformatted(root);

        if (jsonString != NULL) {
            printf("JSON:\n%s\n", jsonString);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, jsonString, strlen(jsonString));
            free(jsonString);
        }   

        cJSON_Delete(root);
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
    else {
        //create json {"status": 204}
        //send json by httpd_resp_send()
        // Create a JSON object for the response
        cJSON *root = cJSON_CreateObject();
        if (root == NULL) {
            printf("Failed to create JSON object.\n");
            return;
        }

        // Add the status to the root object
        cJSON_AddNumberToObject(root, "status", 204);

        // Convert the JSON object to a JSON string
        char *jsonString = cJSON_PrintUnformatted(root);

        if (jsonString != NULL) {
            printf("JSON:\n%s\n", jsonString);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, jsonString, strlen(jsonString));
            free(jsonString);
        }
        cJSON_Delete(root);
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}

//Khởi tạo AP Mode 
esp_err_t wifi_ap_mode(void) {
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "NODE",
            .ssid_len = 0,
            .max_connection = 1,
            .password = "12345678",
            .authmode = WIFI_AUTH_WPA2_PSK,
            .channel = 1
        }
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));

    return ESP_OK;
}

//Khởi tạo STA Mode 
//sua thanh esp_err_t 
esp_err_t wifi_sta_mode(void) {
    wifi_config_t wifi_config = {
        .sta = {
            .scan_method = WIFI_ALL_CHANNEL_SCAN,  // Thay đổi phương thức quét theo yêu cầu của bạn
            .bssid_set = false,  // Không đặt địa chỉ MAC của AP
            .channel = 0,  // Kênh AP không xác định
            .listen_interval = 0,  // Đặt theo giá trị mặc định
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,  // Sắp xếp AP theo tín hiệu
            .threshold = {0},  // Không áp dụng ngưỡng
            .pmf_cfg = {  // Cấu hình PMF
                .capable = true,
                .required = false,
            },
        }
    };
    strncpy((char*)wifi_config.sta.ssid, sta_ssid, sizeof(wifi_config.sta.ssid) - 1);  // Sao chép SSID
    strncpy((char*)wifi_config.sta.password, sta_password, sizeof(wifi_config.sta.password) - 1);  // Sao chép mật khẩu

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_connect());

    return ESP_OK;
}

//handler cho các sự kiện wifi 
//sta
void sta_connect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{

    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        sta_flag = 1;
        /*
        if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
            ip_flag = 1;
            esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
            esp_netif_get_ip_info(sta_netif, &ip_info);
            sprintf(ip_address, "%d.%d.%d.%d", IP2STR(&ip_info.ip));
            printf("STA IP: %s\n", ip_address);
        }
        */
        //printf("STA connected successfully.\n");

        // Lưu SSID và Password vào NVS
        save_wifi_credentials(sta_ssid, sta_password);

        // Đặt bit sự kiện
        xEventGroupSetBits(wifi_event_group, WIFI_STA_CONNECTED_BIT);

    }
    else {
        sta_flag = 0;
    }
}
//ap
void ap_connect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    printf("ap connected\n");
}

//Khoi tao WiFi, Server 
//demo khi chua co ssid password
void initialise_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    netif_ap = esp_netif_create_default_wifi_ap();
    assert(netif_ap);
    netif_sta = esp_netif_create_default_wifi_sta();
    assert(netif_sta);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        WIFI_EVENT_AP_STACONNECTED,
                                                        &ap_connect_handler,
                                                        NULL,
                                                        NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        WIFI_EVENT_STA_CONNECTED,
                                                        &sta_connect_handler,
                                                        NULL,
                                                        NULL));                                                        

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Kiểm tra và load SSID, Password từ NVS
    if (load_wifi_credentials(sta_ssid, sizeof(sta_ssid), sta_password, sizeof(sta_password))) {
        printf("Connecting to saved WiFi: SSID=%s\n", sta_ssid);
        flag_ap = wifi_ap_mode();
        wifi_sta_mode();

        // Chờ sự kiện kết nối STA
        printf("Waiting for STA to connect...\n");
        EventBits_t bits = xEventGroupWaitBits(
            wifi_event_group,
            WIFI_STA_CONNECTED_BIT,
            pdFALSE, // Không xóa bit sau khi chờ
            pdTRUE,  // Chờ tất cả bit được đặt
            portMAX_DELAY // Chờ vô hạn
        );

        if (bits & WIFI_STA_CONNECTED_BIT) {
            printf("STA connected successfully! Proceeding...\n");
            
        }
        server_start();
    } else {
        printf("No saved WiFi credentials. Starting in AP mode.\n");
        flag_ap = wifi_ap_mode(); 
        server_start();
    }
}
