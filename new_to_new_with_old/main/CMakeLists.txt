
idf_component_register(SRCS "ota_example_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash app_update)
