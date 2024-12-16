set(SOURCES "nvs_storage.c")

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash
                    REQUIRES server_cfg
                    REQUIRES wifi_pro
                    REQUIRES mqtt
                    REQUIRES dht11
                    REQUIRES esp_system
)
