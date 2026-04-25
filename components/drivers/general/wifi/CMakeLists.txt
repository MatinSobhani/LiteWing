idf_component_register(SRCS "wifi_esp32.c"
                            "litewing_espnow.c"
                      INCLUDE_DIRS "." "include"
                      REQUIRES crazyflie platform config esp_wifi esp_timer)
