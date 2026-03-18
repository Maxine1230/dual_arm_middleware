add_library(trajectory STATIC trajectory_generator.cpp)
target_include_directories(trajectory PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(trajectory PUBLIC logger utils Threads::Threads)
