cmake_minimum_required(VERSION 3.1)
project(SinglePendulumMPC)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lyaml-cpp -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR -DBOOST_MPL_LIMIT_LIST_SIZE=30 -DPINOCCHIO_WITH_URDFDOM ")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE RELEASE)
#set(CMAKE_BUILD_TYPE DEBUG)

#target_compile_options(kre -Wall -Wextra -Wpedantic -Werror)

find_package(Eigen3 3.3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(crocoddyl REQUIRED)
FIND_PACKAGE(yaml-cpp REQUIRED)

find_package(PkgConfig REQUIRED)
find_package( Threads )
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)
include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})

add_subdirectory(Plot)
add_subdirectory(ODrive_cpp)
add_subdirectory(SinglePendulum_cpp)

