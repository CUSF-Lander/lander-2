cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../comms")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rover
        LANGUAGES CXX C
        VERSION 0.1.0
        DESCRIPTION "Rover firmware for GPS tracking")