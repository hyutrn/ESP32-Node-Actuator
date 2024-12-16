#include "client_cfg.h"

int client_flag;

//GET
esp_err_t client_event_get_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        printf("HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data);
        break;

    default:
        break;
    }
    return ESP_OK;
}

esp_err_t client_get()
{
    // Tạo chuỗi URL cho phương thức POST
    //char client_get_url[100];
    //snprintf(client_get_url, sizeof(client_get_url), "http://%s/get", ip_gateway);

    esp_http_client_config_t config_get = {
        .url = "client_get_url",
        .method = HTTP_METHOD_GET,
        .cert_pem = NULL,
        .event_handler = client_event_get_handler};
        
    esp_http_client_handle_t client = esp_http_client_init(&config_get);
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);

    return ESP_OK;
}

//POST
esp_err_t client_event_post_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        printf("HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data);
        //Xu ly json
        if(evt->data_len > 0) {
            client_flag = 1;
        }
        break;

    default:
        break;
    }
    return ESP_OK;
}

esp_err_t client_post()
{
    // Tạo chuỗi URL cho phương thức POST
    char client_post_url[100];
    snprintf(client_post_url, sizeof(client_post_url), "http://%s/?features=sensor", ip_gateway);
    printf("%s\n", client_post_url);

    esp_http_client_config_t config_post = {
        .host = ip_gateway,
        .port = 8000,
        .path = "/nodes/register_nodes?features=sensor",
        .method = HTTP_METHOD_POST,
        .cert_pem = NULL,
        .event_handler = client_event_post_handler};

    esp_http_client_handle_t client = esp_http_client_init(&config_post);

    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
    if(client_flag == 1) {
        return ESP_OK;
    }
    else {
        return ESP_FAIL;
    }
}
