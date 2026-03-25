add_subdirectory(logger)
add_subdirectory(utils)
add_subdirectory(comm)
add_subdirectory(trajectory)
add_subdirectory(watchdog)
add_subdirectory(config)
add_subdirectory(state)

add_executable(middleware main.cpp)

target_link_libraries(middleware
  PRIVATE
  logger
  utils
  comm
  trajectory
  watchdog
  config
  arm_state
  Threads::Threads
)
