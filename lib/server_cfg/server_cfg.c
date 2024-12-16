#include "server_cfg.h"
#include <stdio.h>

int flag_call_mqtt;
int server_initialized;
int register_node = 0;
esp_err_t check_post_client;

//SERVER 
//GET FILE
esp_err_t get_url_server(httpd_req_t *req) {
    printf("This is the handler for the <%s> URI", req->uri);
    // Mở file index.html
    FILE *file = fopen("/spiffs/index.html", "r");
    // Lấy độ dài của file
    long file_size = calculate_file_length(file);
    // Cấp phát bộ nhớ cho biến file_content
    char *file_content = malloc((file_size + 1) * sizeof(char));
    // Đọc file và lưu vào biến file_content
    fread(file_content, file_size, 1, file);
    // Gửi phản hồi HTTP với dữ liệu HTML
    httpd_resp_send(req, file_content, file_size);
    // Giải phóng bộ nhớ cho biến file_content
    free(file_content);
    // Đóng file
    fclose(file);
    return ESP_OK;
}

esp_err_t get_css_file_handler(httpd_req_t* req) {
    printf("This is the handler for the <%s> URI", req->uri);
    // set type of file to response
    httpd_resp_set_type(req, "text/css");
    // Mở file style.css
    FILE *file = fopen("/spiffs/style.css", "r");
    // Lấy độ dài của file
    long file_size = calculate_file_length(file);
    // Cấp phát bộ nhớ cho biến file_content
    char *file_content = malloc((file_size + 1) * sizeof(char));
    // Đọc file và lưu vào biến file_content
    fread(file_content, file_size, 1, file);
    // Gửi phản hồi HTTP với dữ liệu HTML
    httpd_resp_send(req, file_content, file_size);
    // Giải phóng bộ nhớ cho biến file_content
    free(file_content);
    // Đóng file
    fclose(file);

    return ESP_OK;
}

esp_err_t get_js_file_handler(httpd_req_t* req) {
    printf("This is the handler for the <%s> URI", req->uri);
    // set type of file to response
    httpd_resp_set_type(req, "text/javascript");
    // Mở file style.css
    FILE *css_file = fopen("/spiffs/app.js", "r");
    // Lấy độ dài của file
    long css_file_size = calculate_file_length(css_file);
    // Cấp phát bộ nhớ cho biến file_content
    char *css_file_content = malloc((css_file_size + 1) * sizeof(char));
    // Đọc file và lưu vào biến file_content
    fread(css_file_content, css_file_size, 1, css_file);
    // Gửi phản hồi HTTP với dữ liệu HTML
    httpd_resp_send(req, css_file_content, css_file_size);
    // Giải phóng bộ nhớ cho biến file_content
    free(css_file_content);
    // Đóng file
    fclose(css_file);
    return ESP_OK;
}

//SSID  AVAILABLE
esp_err_t get_ssid_available(httpd_req_t *req) {
    scann(req);    
    return ESP_OK;
}

//LOGIN WIFI
esp_err_t post_login_wifi(httpd_req_t *req) {
    //Đọc dữ liệu từ Body
    char content [100];
    size_t recv_size = MIN(req->content_len, sizeof(content));
    int ret = httpd_req_recv(req, content, recv_size);
    if(ret <= 0) {
        if(ret == HTTPD_SOCK_ERR_TIMEOUT) {
            //timeout 
            httpd_resp_send_408(req);
        }
    }
    httpd_req_recv(req, content, sizeof(content));
    printf("ESP-HTTP: Data received:\n");
    printf(content);
    printf("\n");
    //Nhận URL, tách key và value từ params nhận được
    printf("ESP-HTTP: Params received: \n");
    //lấy độ dài chuỗi truy vấn
    size_t query_len = httpd_req_get_url_query_len(req);
    uint32_t query[query_len + 1];
    //Lấy chuỗi truy vấn sau dấu "?"
    if (httpd_req_get_url_query_str(req, query, query_len + 1) == ESP_OK) {
        //Lấy value từ key trong param
        printf("STA MODE: \n");
        if (httpd_query_key_value(query, "ssid", sta_ssid, sizeof(sta_ssid)) == ESP_OK &&
            httpd_query_key_value(query, "password", sta_password, sizeof(sta_password)) == ESP_OK) {
            printf("SSID: %s, Password: %s\n", sta_ssid, sta_password);
            const esp_err_t connected_check = wifi_sta_mode();
            vTaskDelay(7000/portTICK_PERIOD_MS);
            //Response 
            if(connected_check == ESP_OK) {
                if(sta_flag == 1) {
                    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
                    esp_netif_get_ip_info(sta_netif, &ip_info);
                    sprintf(ip_address, "%d.%d.%d.%d", IP2STR(&ip_info.ip));
                    printf("STA IP: %s\n", ip_address);

                    //create json {"status": 200}
                    // Create a JSON object for the response
                    cJSON *root = cJSON_CreateObject();
                    if (root == NULL) {
                        printf("Failed to create JSON object.\n");
                    }
                                            
                    //Add the ip_gateway to the root object 
                    cJSON_AddStringToObject(root, "ip", ip_address);

                    // Add the status to the root object
                    cJSON_AddNumberToObject(root, "status", 200);

                    // Convert the JSON object to a JSON string
                    char *jsonString = cJSON_PrintUnformatted(root);

                    if (jsonString != NULL) {
                        printf("Response login wifi to user:\n%s\n", jsonString);
                        httpd_resp_set_type(req, "application/json");
                        httpd_resp_send(req, jsonString, strlen(jsonString));
                        free(jsonString);
                    }
                    cJSON_Delete(root);
                    vTaskDelay(2000/portTICK_PERIOD_MS);
                }
            }
            else {
                printf("case not connected and send error\n");
                //create json {"status": 401} 
                // Create a JSON object for the response
                cJSON *root = cJSON_CreateObject();
                if (root == NULL) {
                    printf("Failed to create JSON object.\n");
                }

                // Add the status to the root object
                cJSON_AddNumberToObject(root, "status", 401);

                // Convert the JSON object to a JSON string
                char *jsonString = cJSON_PrintUnformatted(root);

                if (jsonString != NULL) {
                    printf("Response login wifi to user:\n%s\n", jsonString);
                    httpd_resp_set_type(req, "application/json");
                    httpd_resp_send(req, jsonString, strlen(jsonString));
                    free(jsonString);
                }
                cJSON_Delete(root);
                vTaskDelay(2000/portTICK_PERIOD_MS);
            }
        }
    }
    // Gán bit sự kiện cho EVENT_CLIENT_POSTED
    xEventGroupSetBits(shared_event_group, EVENT_CLIENT_POSTED);
    return ESP_OK;
}

//REGISTER NODE
esp_err_t post_register_node(httpd_req_t *req) {
    //Đọc dữ liệu từ Body
    char content [100];
    size_t recv_size = MIN(req->content_len, sizeof(content));
    int ret = httpd_req_recv(req, content, recv_size);
    if(ret <= 0) {
        if(ret == HTTPD_SOCK_ERR_TIMEOUT) {
            //timeout 
            httpd_resp_send_408(req);
        }
    }
    httpd_req_recv(req, content, sizeof(content));
    printf("ESP-HTTP: Data received:\n");
    printf(content);
    printf("\n");
    //Params
    printf("ESP-HTTP: Params received: \n");
    //lấy độ dài chuỗi truy vấn
    size_t query_len = httpd_req_get_url_query_len(req);
    char query[query_len + 1];
    //Lấy chuỗi truy vấn sau dấu "?"
    if (httpd_req_get_url_query_str(req, query, query_len + 1) == ESP_OK) {
        //Lấy value từ key trong param
        if (httpd_query_key_value(query, "ip_gateway", ip_gateway, sizeof(ip_gateway)) == ESP_OK) {
            printf("ip_gateway: %s\n", ip_gateway);
            esp_err_t check_client = client_post(); 
            if(check_client == ESP_OK) {
                // Create a JSON object for the response
                cJSON *root = cJSON_CreateObject();
                if (root == NULL) {
                    printf("Failed to create JSON object.\n");
                }

                // Add the status to the root object
                cJSON_AddNumberToObject(root, "status", 200);

                // Convert the JSON object to a JSON string
                char *jsonString = cJSON_PrintUnformatted(root);

                if (jsonString != NULL) {
                    printf("Response rigister gateway to user:\n%s\n", jsonString);
                    httpd_resp_set_type(req, "application/json");
                    httpd_resp_send(req, jsonString, strlen(jsonString));
                    free(jsonString);
                }
                cJSON_Delete(root);
                vTaskDelay(2000/portTICK_PERIOD_MS);
            }
            else {
                // Create a JSON object for the response
                cJSON *root = cJSON_CreateObject();
                if (root == NULL) {
                    printf("Failed to create JSON object.\n");
                }

                // Add the status to the root object
                cJSON_AddNumberToObject(root, "status", 401);

                // Convert the JSON object to a JSON string
                char *jsonString = cJSON_PrintUnformatted(root);

                if (jsonString != NULL) {
                    printf("Response rigister gateway to user:\n%s\n", jsonString);
                    httpd_resp_set_type(req, "application/json");
                    httpd_resp_send(req, jsonString, strlen(jsonString));
                    free(jsonString);
                }
                cJSON_Delete(root);
                vTaskDelay(2000/portTICK_PERIOD_MS);
            }
        }
    }
    check_post_client = ESP_OK;

    return ESP_OK;
}

httpd_handle_t server_start(void) {
    // Init SPIFFS files
    files_spiffs_init(NULL, 5, true);
    httpd_config_t server_cfg = HTTPD_DEFAULT_CONFIG();

    //GET URL_SERVER
    httpd_uri_t uri_get_url_server = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = get_url_server,
        .user_ctx = NULL
    };
    //Load file 
    httpd_uri_t get_css_file = {
        .uri       = "/style.css",
        .method    = HTTP_GET,
        .handler   = get_css_file_handler,
        .user_ctx  = NULL,
    };
    httpd_uri_t get_js_file = {
        .uri       = "/app.js",
        .method    = HTTP_GET,
        .handler   = get_js_file_handler,
        .user_ctx  = NULL,
    };

    //GET/ssid_available
    httpd_uri_t uri_ssid_available = {
        .uri = "/ssid_available",
        .method = HTTP_GET,
        .handler = get_ssid_available,
        .user_ctx = NULL
    };

    //POST login wifi 
    httpd_uri_t uri_post_login_wifi = {
        .uri = "/login_wifi",
        .method = HTTP_POST,
        .handler = post_login_wifi,
        .user_ctx = NULL,
    };

    //POST register 
    httpd_uri_t uri_post_register_node = {
        .uri = "/register_node",
        .method = HTTP_POST,
        .handler = post_register_node, 
        .user_ctx = NULL
    };

    //

    httpd_handle_t server_handle = NULL;

    if(httpd_start(&server_handle, &server_cfg) == ESP_OK){
        server_initialized = 1; // Đặt trạng thái thành công
        httpd_register_uri_handler(server_handle, &uri_get_url_server);
        httpd_register_uri_handler(server_handle, &get_css_file);
        httpd_register_uri_handler(server_handle, &get_js_file);
        httpd_register_uri_handler(server_handle, &uri_ssid_available);
        httpd_register_uri_handler(server_handle, &uri_post_login_wifi);
        httpd_register_uri_handler(server_handle, &uri_post_register_node);
    }
    else {
        server_initialized = 0; // Đặt trạng thái thất bại
    }
    return server_handle;
}

void server_stop(httpd_handle_t server_handle) {
    if(server_handle) {
        httpd_stop(server_handle);
    }
}