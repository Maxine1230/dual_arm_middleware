add_library(logger STATIC logger.cpp)
target_include_directories(logger PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(logger PUBLIC Threads::Threads)
