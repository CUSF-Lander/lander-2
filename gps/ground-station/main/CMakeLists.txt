idf_component_register(SRCS "main.cpp" "esp_now_receiver.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi esp_event nvs_flash driver esp_timer comms)
