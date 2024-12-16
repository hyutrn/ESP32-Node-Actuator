set(SOURCES "shared.c")

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "."
                    REQUIRES wifi_pro
                    REQUIRES mqtt_cfg
                    REQUIRES server_cfg
                    REQUIRES json
                    REQUIRES nvs_storage
)