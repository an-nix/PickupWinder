# Source component for PickupWinder ESP32 firmware.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS ${app_sources}
                        REQUIRES driver)
