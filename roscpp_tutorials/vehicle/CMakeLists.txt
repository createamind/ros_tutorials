include_directories(${catkin_INCLUDE_DIRS})
add_executable(vehicle vehicle.cpp)
target_link_libraries(vehicle ${catkin_LIBRARIES})
install(TARGETS vehicle
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
