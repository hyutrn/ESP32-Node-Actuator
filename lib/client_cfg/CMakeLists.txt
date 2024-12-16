set(SOURCES "client_cfg.c")
set(pri_set esp_http_client)

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "."
                    REQUIRES ${pri_set}
                    REQUIRES wifi_pro
                    REQUIRES server_cfg
                    REQUIRES dht11
                    REQUIRES mqtt_cfg
                    REQUIRES json
)