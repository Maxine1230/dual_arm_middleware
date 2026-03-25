add_library(arm_state STATIC arm_state_hub.cpp)
target_include_directories(arm_state PUBLIC ${CMAKE_SOURCE_DIR}/src)
