cmake_minimum_required(VERSION 3.28)

if(TARGET kvasir_devices)
    return()
endif()

add_library(kvasir_devices INTERFACE)
target_include_directories(kvasir_devices INTERFACE src)
add_library(kvasir::devices ALIAS kvasir_devices)
