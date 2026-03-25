add_library(config STATIC config_loader.cpp)
target_include_directories(config PUBLIC
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/third_party
)
