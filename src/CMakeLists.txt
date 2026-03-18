add_subdirectory(logger)
add_subdirectory(utils)
add_subdirectory(comm)
add_subdirectory(trajectory)
add_subdirectory(watchdog)

add_executable(middleware main.cpp)

target_link_libraries(middleware
  PRIVATE
    logger
    utils
    comm
    trajectory
    watchdog
    Threads::Threads
)
