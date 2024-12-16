set(pri_req)

idf_component_register(SRCS "project_system_files.c"
                    INCLUDE_DIRS "."
                    REQUIRES ${pri_req} spiffs
                    )

spiffs_create_partition_image(storage ../../spiffs_data FLASH_IN_PROJECT)