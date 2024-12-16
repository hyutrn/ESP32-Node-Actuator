#include "mqtt_cfg.h" 

#if CONFIG_BROKER_CERTIFICATE_OVERRIDDEN == 1 
static const uint8_t mqtt_cert_key_pem_start[] = "-----BEGIN CERTIFICATE-----\n" CONFIG_BROKER_CERTIFICATE_OVERRIDDEN "\n-----END CERTIFICATE-----";
#else
extern const uint8_t mqtt_cert_key_pem_start[]   asm("_binary_mqtt_cert_key_pem_start");
#endif
extern const uint8_t mqtt_cert_key_pem_end[]   asm("_binary_mqtt_cert_key_pem_end");

esp_mqtt_client_handle_t client = NULL;

esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI("MQTTWSS", "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        //ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        //ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        //ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI("MQTTWSS", "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI("MQTTWSS", "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "Hello broker", 0, 0, 0);
        ESP_LOGI("MQTTWSS", "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI("MQTTWSS", "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI("MQTTWSS", "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI("MQTTWSS", "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI("MQTTWSS", "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI("MQTTWSS", "Other event id:%d", event->event_id);
        break;
    }
    return ESP_OK;
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    /* The argument passed to esp_mqtt_client_register_event can de accessed as handler_args*/
    ESP_LOGD("MQTTWSS", "Event dispatched from event loop base=%s, event_id=%" PRId32, base, event_id);
    mqtt_event_handler_cb(event_data);
}

void dataSend(esp_mqtt_client_handle_t client){
        //Publish to MQTT Broker
    // Tạo JSON để publish
    char json_data[128];
    //snprintf(json_data, sizeof(json_data),
    //         "{\"temperature\": %.2f, \"moisture\": %d, \"light\": %d}",
    //         temperature, moisture, light);

    // Publish dữ liệu tới broker
    int msg_id = esp_mqtt_client_publish(client, "/sensor/data", "Hello", 0, 1, 0);
    if (msg_id != -1) {
        ESP_LOGI("MQTTWSS", "Data published successfully, msg_id=%d", msg_id);
    } else {
        ESP_LOGE("MQTTWSS", "Failed to publish data");
    }
}

/*
void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "wss://mqtt.eclipseprojects.io:443/mqtt",
        //.cert_pem = (const char *)mqtt_cert_key_pem_start,
        .broker.verification.certificate = (const char *)mqtt_cert_key_pem_start,
        .broker.verification.certificate_len = strlen((const char*)mqtt_cert_key_pem_start),
        .session.keepalive = 90
    };

    ESP_LOGI("MQTTWSS", "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    //The last argument may be used to pass data to the event handler, in this example mqtt_event_handler
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    esp_mqtt_client_start(client);
}
*/

void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "wss://mqtt.eclipseprojects.io:443/mqtt",
        .broker.verification.certificate = (const char *)mqtt_cert_key_pem_start,
        .broker.verification.certificate_len = strlen((const char *)mqtt_cert_key_pem_start),
        .session.keepalive = 90
    };

    ESP_LOGI("MQTTWSS", "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}
