# AerialKit on the ESP32, as an ESP-IDF project.
#
# This file is the platform, not the firmware: it hands IDF the ESP32 port and
# the board, and everything portable arrives through main/CMakeLists.txt
# unchanged. The portable core has never needed to know which chip it is on, and
# this is where that claim gets tested.

cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(aerialkit-esp32)
