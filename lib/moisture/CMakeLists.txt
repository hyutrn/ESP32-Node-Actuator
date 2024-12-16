set(SOURCES "mois.c")
set(pri_set driver)

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "."
                    REQUIRES ${pri_set}
                    REQUIRES wifi_pro
                    REQUIRES mqtt_cfg
                    REQUIRES json
)