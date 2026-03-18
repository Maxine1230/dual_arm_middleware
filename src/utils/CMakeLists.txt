add_library(utils STATIC state_machine.cpp)
target_include_directories(utils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(utils PUBLIC Threads::Threads)
# ring_buffer.hpp 和 thread_pool.hpp 是 header-only，随 utils 一起暴露
