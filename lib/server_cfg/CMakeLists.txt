set(SOURCES "server_cfg.c")
set(pri_set esp_http_server)

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "."
                    REQUIRES ${pri_set}
                    REQUIRES nvs_flash
                    REQUIRES json
                    REQUIRES wifi_pro
                    REQUIRES project_system_files
                    REQUIRES client_cfg
                    REQUIRES event_lib
                    REQUIRES nvs_storage
                    )