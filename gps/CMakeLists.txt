cmake_minimum_required(VERSION 3.16)

# Select which firmware app to build from the GPS root.
# Use: -DGPS_APP=rover or -DGPS_APP=ground-station
set(GPS_APP "ground-station" CACHE STRING "Firmware app to build (rover|ground-station)") 
set_property(CACHE GPS_APP PROPERTY STRINGS rover ground-station)

if(NOT GPS_APP STREQUAL "rover" AND NOT GPS_APP STREQUAL "ground-station")
    message(FATAL_ERROR "Invalid GPS_APP='${GPS_APP}'. Use rover or ground-station.")
endif()

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/comms"
    "${CMAKE_CURRENT_LIST_DIR}/${GPS_APP}/main"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(gps)
