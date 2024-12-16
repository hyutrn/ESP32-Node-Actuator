    set(SOURCES "wifi_pro.c")
    set(pri_set esp_wifi)

    idf_component_register(SRCS ${SOURCES}
                        INCLUDE_DIRS "."
                        REQUIRES ${pri_set}
                        REQUIRES nvs_flash
                        REQUIRES json
                        REQUIRES server_cfg
                        REQUIRES project_system_files
                        REQUIRES mqtt_cfg
                        REQUIRES event_lib
                        REQUIRES nvs_storage
                        )

