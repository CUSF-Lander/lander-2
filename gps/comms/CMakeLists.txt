idf_component_register(
        SRCS "uart.cpp" "Parser.cpp" "structs.h"
        INCLUDE_DIRS "."
        REQUIRES driver freertos esp_system log
)