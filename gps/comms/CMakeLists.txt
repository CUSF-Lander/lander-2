idf_component_register(
        SRCS "comm.cpp" "uart.cpp" "Parser.cpp" "structs.h"
        INCLUDE_DIRS "."
        REQUIRES driver freertos esp_system log esp_wifi esp_event esp_netif
)