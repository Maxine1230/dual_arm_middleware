cmake_minimum_required(VERSION 3.16)
project(dual_arm_middleware VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# Compile options
add_compile_options(
  -Wall
  -Wextra
  -Wpedantic
  $<$<CONFIG:Debug>:-g>
  $<$<CONFIG:Release>:-O2>
)

# Thread support (required for std::thread)
find_package(Threads REQUIRED)

# nlohmann/json 单头文件（下载到 third_party/nlohmann/json.hpp）
set(NLOHMANN_JSON_HPP "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann/json.hpp")
if(NOT EXISTS "${NLOHMANN_JSON_HPP}")
  message(STATUS "Downloading nlohmann/json.hpp single-header...")
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann")
  file(DOWNLOAD
    https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp
    "${NLOHMANN_JSON_HPP}"
    SHOW_PROGRESS
    STATUS _json_dl
    TLS_VERIFY ON
  )
  list(GET _json_dl 0 _dl_ok)
  if(NOT _dl_ok EQUAL 0)
    message(FATAL_ERROR "Failed to download json.hpp; place json.hpp in third_party/nlohmann/ manually.")
  endif()
endif()

# Include paths
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

# Subdirectories
add_subdirectory(src)

# Tests
option(BUILD_TESTS "Build test programs" ON)
if(BUILD_TESTS)
  add_subdirectory(tests)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Project: ${PROJECT_NAME} v${PROJECT_VERSION}")
