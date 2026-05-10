#include "Parser.h"
#include "structs.h"
#include <string>
#include <vector>
#include <string_view>
#include "esp_log.h"
#include <cstdlib>
#include <cmath>
#include <ctype.h>

/** An example of the message sentence is 
'$GNGGA,215905.00,5212.98222110,N,00006.28559124,E,7,15,1.7,21.8945,M,47.0843,M,,*46   


'$GNGGA,220314.00,5212.98222110,N,00006.28559124,E,7,16,1.6,21.8945,M,47.0843,M,,*48

I (1919) Parser: Token 0: GNGGA
I (1919) Parser: Token 1: 220246.00
I (1919) Parser: Token 2: 5212.98222110
I (1929) Parser: Token 3: N
I (1929) Parser: Token 4: 00006.28559124
I (1929) Parser: Token 5: E
I (1929) Parser: Token 6: 7
I (1939) Parser: Token 7: 16
I (1939) Parser: Token 8: 1.6
I (1939) Parser: Token 9: 21.8945
I (1949) Parser: Token 10: M
I (1949) Parser: Token 11: 47.0843
I (1949) Parser: Token 12: M
I (1949) Parser: Token 13: 
I (1959) Parser: Token 14: 

*/

std::vector<std::string> Parser::tokenize(const uint8_t* data, size_t len) {
    std::string sentence = std::string(reinterpret_cast<const char*>(data), len);

    // Strip leading '$' and trailing checksum "*XX\r\n"
    size_t start = (sentence.front() == '$') ? 1 : 0; // Access the first character of the string
    size_t end = sentence.find('*');
    if (end == std::string::npos) end = sentence.size();
    sentence = sentence.substr(start, end - start);

    std::vector<std::string> tokens;
    size_t pos = 0;
    while (pos <= sentence.size()) {
        size_t comma = sentence.find(',', pos);
        if (comma == std::string::npos) comma = sentence.size();
        tokens.push_back(sentence.substr(pos, comma - pos));
        pos = comma + 1;
    }
    return tokens;
}

void Parser::convertToCart(const double& lat, const double& lon, const double& alt, const double& hae){
    // Convert lat/lon from DDDMM.MMMMM to decimal degrees
    ESP_LOGI("Parser", "Converting to Cartesian: lat=%.6f, lon=%.6f, alt=%.2f", lat, lon, alt);
    gps_data.altitude = alt;

    double lat_deg_d = std::floor(lat / 100.0);
    double lat_min   = lat - (lat_deg_d * 100.0);
    double lat_dec   = lat_deg_d + (lat_min / 60.0);

    double lon_deg_d = std::floor(lon / 100.0);
    double lon_min   = lon - (lon_deg_d * 100.0);
    double lon_dec   = lon_deg_d + (lon_min / 60.0);
    
    // ESP_LOGI("Parser", "lat_deg_d=%.8f, lat_min=%.8f, lat_dec=%.8f",
    //      lat_deg_d, lat_min, lat_dec);
    // ESP_LOGI("Parser", "lon_deg_d=%.8f, lon_min=%.8f, lon_dec=%.8f",
    //      lon_deg_d, lon_min, lon_dec);
    // Convert to radians
    double lat_rad = lat_dec * M_PI / 180.0;
    double lon_rad = lon_dec * M_PI / 180.0;
    // ESP_LOGI("Parser", "e2=%.14f", e_2);
    // ESP_LOGI("Parser", "a=%.2f", a);
    // ESP_LOGI("Parser", "lat_rad=%.8f, lon_rad=%.8f", lat_rad, lon_rad);
    // Calculate Cartesian coordinates
    double altitude_ecef = alt + hae;
    double N = a / std::sqrt(1 - e_2 * std::sin(lat_rad) * std::sin(lat_rad));
    double x = (N + altitude_ecef) * std::cos(lat_rad) * std::cos(lon_rad);
    double y = (N + altitude_ecef) * std::cos(lat_rad) * std::sin(lon_rad);
    double z = ((1 - e_2) * N + altitude_ecef) * std::sin(lat_rad);
    // ESP_LOGI("Parser", "N=%.4f, cos_lat=%.10f, cos_lon=%.10f, sin_lat=%.10f",
    //      N, std::cos(lat_rad), std::cos(lon_rad), std::sin(lat_rad));
    // ESP_LOGI("Parser", "ACTIVE FUNCTION: x=%.2f, y=%.2f, z=%.2f", x, y, z);
    gps_data.x = x;
    gps_data.y = y;
    gps_data.z = z;

}



void Parser::parse(const uint8_t* data, size_t len) {
    std::vector<std::string> tokens = tokenize(data, len);
    if (tokens.empty()) return;
    
    //handling the type of command:
    std::string_view type(tokens[0]);
    if (type == std::string_view("GNGGA")){
        if (std::stod(tokens[6]) < 1) {
            ESP_LOGW("Parser", "Invalid GPS fix quality: %s", tokens[6].c_str());
            return;
        }
        else{
            if (std::stod(tokens[6]) >=4 && std::stod(tokens[6]) <=5){
                RTK = true;
            }
            convertToCart(std::stod(tokens[2]), std::stod(tokens[4]), std::stod(tokens[9]), std::stod(tokens[11]));
            ESP_LOGI("Parser", "Parsed GNGGA: x=%.2f, y=%.2f, z=%.2f", gps_data.x, gps_data.y, gps_data.z);
        }
    }
    else {
        return;
    }
}

const GpsData& Parser::getGpsData(void){
    return gps_data;
}
