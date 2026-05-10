#ifndef STRUCTS_H
#define STRUCTS_H

// Define a struct to hold the GPS data
struct GpsData {
    double x; // X coordinate in meters
    double y; // Y coordinate in meters
    double z; // Z coordinate in meters
    double altitude; // Altitude in meters
    double heading; // Heading in degrees
};
#endif // STRUCTS_H