idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES mqtt esp_adc driver esp_wifi esp_event esp_netif nvs_flash pca9685 wpa_supplicant)