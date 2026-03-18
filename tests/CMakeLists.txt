add_executable(test_ring_buffer test_ring_buffer.cpp)
target_include_directories(test_ring_buffer PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(test_ring_buffer PRIVATE Threads::Threads)

add_executable(test_logger test_logger.cpp)
target_include_directories(test_logger PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(test_logger PRIVATE logger Threads::Threads)

add_executable(test_state_machine test_state_machine.cpp)
target_include_directories(test_state_machine PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(test_state_machine PRIVATE utils Threads::Threads)
