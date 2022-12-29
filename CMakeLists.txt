cmake_minimum_required(VERSION 3.15)
cmake_policy(SET CMP0048 NEW)

project(eeprom-driver LANGUAGES CXX C ASM)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
        include
        )

target_link_libraries(${PROJECT_NAME} INTERFACE)
