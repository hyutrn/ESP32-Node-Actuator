#include "mqtt_cfg.h" 

#if CONFIG_BROKER_CERTIFICATE_OVERRIDDEN == 1 
static const uint8_t mqtt_cert_key_pem_start[] = "-----BEGIN CERTIFICATE-----\n" CONFIG_BROKER_CERTIFICATE_OVERRIDDEN "\n-----END CERTIFICATE-----";
#else
extern const uint8_t mqtt_cert_key_pem_start[]   asm("_binary_mqtt_cert_key_pem_start");
#endif
extern const uint8_t mqtt_cert_key_pem_end[]   asm("_binary_mqtt_cert_key_pem_end");

esp_mqtt_client_handle_t client = NULL;
bool motor_1 = 0;
bool motor_2 = 0;
bool motor_3 = 0;
bool motor_4 = 0;

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
        //printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        //printf("DATA=%.*s\r\n", event->data_len, event->data);
        // Gọi hàm updateMotorStatus
        updateMotorStatus(client, event->topic, event->data, event->data_len);
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
    //Publish status 4 buttons to MQTT Broker
    // Tạo JSON chứa trạng thái 4 nút nhấn
    cJSON *status_json = cJSON_CreateObject();
    cJSON_AddBoolToObject(status_json, "motor_1", motor_1);
    cJSON_AddBoolToObject(status_json, "motor_2", motor_2);
    cJSON_AddBoolToObject(status_json, "motor_3", motor_3);
    cJSON_AddBoolToObject(status_json, "motor_4", motor_4);

    // Chuyển đổi JSON thành chuỗi
    char *status_str = cJSON_PrintUnformatted(status_json);

    // Gửi dữ liệu JSON lên topic "actuator/status/"
    esp_mqtt_client_publish(client, "actuator/status/", status_str, 0, 1, 0);

    ESP_LOGI("MQTTWSS", "Published: %s", status_str);

    // Giải phóng bộ nhớ JSON
    cJSON_Delete(status_json);
    free(status_str);
}

void updateMotorStatus(esp_mqtt_client_handle_t client, const char *topic, const char *data, int data_len) {
    // Kiểm tra topic để đảm bảo đúng topic "actuator/control/"
    if (strncmp(topic, "actuator/control/", strlen("actuator/control/")) == 0) {
        // Parse dữ liệu JSON từ MQTT payload
        char *payload = strndup(data, data_len); // Tạo chuỗi từ payload
        cJSON *json = cJSON_Parse(payload);

        if (json != NULL) {
            // Cập nhật trạng thái motor nếu key tồn tại
            cJSON *motor_1_json = cJSON_GetObjectItem(json, "motor_1");
            cJSON *motor_2_json = cJSON_GetObjectItem(json, "motor_2");
            cJSON *motor_3_json = cJSON_GetObjectItem(json, "motor_3");
            cJSON *motor_4_json = cJSON_GetObjectItem(json, "motor_4");

            if (cJSON_IsBool(motor_1_json)) motor_1 = cJSON_IsTrue(motor_1_json);
            if (cJSON_IsBool(motor_2_json)) motor_2 = cJSON_IsTrue(motor_2_json);
            if (cJSON_IsBool(motor_3_json)) motor_3 = cJSON_IsTrue(motor_3_json);
            if (cJSON_IsBool(motor_4_json)) motor_4 = cJSON_IsTrue(motor_4_json);

            ESP_LOGI("MQTTWSS", "Updated motor statuses: motor_1=%d, motor_2=%d, motor_3=%d, motor_4=%d", motor_1, motor_2, motor_3, motor_4);
        } else {
            ESP_LOGE("MQTTWSS", "Failed to parse JSON payload");
        }

        // Giải phóng bộ nhớ
        cJSON_Delete(json);
        free(payload);
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

/*
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
*/

void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://mqtt.flespi.io",
        .broker.address.hostname = "mqtt://mqtt.flespi.io",
        .broker.address.port = 1883,
        .credentials.username = "ejkH5zHIZGRWVmg8cjgSKuDeWZoNhhgzrRl6BsOixEEEIgJ6bauQbQYGcPs5vyUd",
        .credentials.client_id = "NODE SENSOR",
        .credentials.authentication.password = NULL,
        .session.keepalive = 90
    };


    ESP_LOGI("MQTTWSS", "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

