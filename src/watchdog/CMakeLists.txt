add_library(watchdog STATIC watchdog.cpp)
target_include_directories(watchdog PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(watchdog PUBLIC logger Threads::Threads)
