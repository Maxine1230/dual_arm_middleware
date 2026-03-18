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

# nlohmann/json (header-only, vendored or system)
find_package(nlohmann_json 3.2.0 QUIET)
if(NOT nlohmann_json_FOUND)
  message(STATUS "nlohmann_json not found via cmake, using FetchContent")
  include(FetchContent)
  FetchContent_Declare(json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp
    DOWNLOAD_NO_EXTRACT TRUE
    DOWNLOAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann
  )
  # Fallback: just add include path if header already present
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
