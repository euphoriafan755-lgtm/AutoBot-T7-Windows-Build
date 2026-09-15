cmake_minimum_required(VERSION 3.21)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)

project(AutoBotT7 VERSION 0.1.0)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if (DEFINED GEODE_SDK)
    set(AUTOBOT_GEODE_SDK "${GEODE_SDK}")
elseif (DEFINED ENV{GEODE_SDK})
    set(AUTOBOT_GEODE_SDK "$ENV{GEODE_SDK}")
else()
    message(FATAL_ERROR "Geode SDK not found. Set GEODE_SDK or the GEODE_SDK environment variable.")
endif()

message(STATUS "Using Geode SDK: ${AUTOBOT_GEODE_SDK}")
add_subdirectory(${AUTOBOT_GEODE_SDK} ${CMAKE_CURRENT_BINARY_DIR}/geode)
setup_geode_mod(${PROJECT_NAME})
