

idf_component_register(SRCS "src/user_wifi.c"
                            "src/user_sniffer.c"
                            "src/user_uart.c"
                            "src/user_sim800c.c"
                            "src/user_radar.c"
                            "src/user_i2c.c"
                            "src/user_bmp.c"
                            "src/user_bt.c"
                            "src/user_nvs.c"
                            "src/user_ota.c"
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash bt app_update)
                    
