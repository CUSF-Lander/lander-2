#ifndef PARSER_H
#define PARSER_H

#include <stdint.h> //Provides the definition of uint8_t
#include <stddef.h> //Provides the definition of size_t
#include <string>
#include <vector>
#include <string_view>
#include "esp_log.h"
#include "structs.h"
class Parser {
    private:
        uint8_t buffer[256];
        size_t buffer_index;
        constexpr static const double a {6378137.0};
        constexpr static const double e_2{0.00669437999014};
        std::vector<std::string> tokenize(const uint8_t* data, size_t len);
        void convertToCart(const double& lat,const  double& lon, const double& alt, const double& hae);
        GpsData gps_data;
    public:
        void parse(const uint8_t* data, size_t len);
};

#endif // PARSER_H