add_library(comm STATIC serial_port.cpp)
target_include_directories(comm PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(comm PUBLIC logger Threads::Threads)
