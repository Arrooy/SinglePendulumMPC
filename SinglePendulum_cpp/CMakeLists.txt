

add_executable(SinglePendulumMPC main.cpp ActuationModelSinglePendulum.cpp ActuationModelSinglePendulum.h CostModelSinglePendulum.cpp CostModelSinglePendulum.h Controller.cpp Controller.h)
target_include_directories(SinglePendulumMPC PUBLIC ${EIGEN3_INCLUDE_DIRS} ${pinocchio_INCLUDE_DIRS} ${crocoddyl_INCLUDE_DIRS})
target_link_libraries(SinglePendulumMPC PUBLIC ${pinocchio_LIBRARIES} ${crocoddyl_LIBRARIES} yaml-cpp)
target_link_libraries(SinglePendulumMPC LINK_PUBLIC odrive_cpp)