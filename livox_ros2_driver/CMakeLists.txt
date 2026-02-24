# Copyright(c) 2020 livoxtech limited.

cmake_minimum_required(VERSION 3.5)
project(livox_ros2_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# Printf version info
include(cmake/version.cmake)
project(${PROJECT_NAME} VERSION ${LIVOX_ROS_DRIVER_VERSION} LANGUAGES CXX)
message(STATUS "${PROJECT_NAME} version: ${LIVOX_ROS_DRIVER_VERSION}")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(PCL REQUIRED)
find_package(livox_sdk REQUIRED)

# check apr
find_package(PkgConfig)
pkg_check_modules(APR apr-1)
if (APR_FOUND)
	message(${APR_INCLUDE_DIRS})
	message(${APR_LIBRARIES})
endif (APR_FOUND)

## make sure the livox_sdk_static library is installed
find_library(LIVOX_SDK_LIBRARY liblivox_sdk_static.a /usr/local/lib)

## PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Shared library (data handling, LDS, LDDC, etc. — everything except the node main)
ament_auto_add_library(${PROJECT_NAME} SHARED
  livox_ros2_driver/lvx_file.cpp
  livox_ros2_driver/ldq.cpp
  livox_ros2_driver/lds.cpp
  livox_ros2_driver/lds_lvx.cpp
  livox_ros2_driver/lds_lidar.cpp
  livox_ros2_driver/lds_hub.cpp
  livox_ros2_driver/lddc.cpp
  timesync/timesync.cpp
  timesync/user_uart/user_uart.cpp
  common/comm/comm_protocol.cpp
  common/comm/sdk_protocol.cpp
  common/comm/gps_protocol.cpp
)

target_compile_options(${PROJECT_NAME}
  PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wno-missing-field-initializers -fPIC>
)

target_include_directories(${PROJECT_NAME} PRIVATE ${livox_sdk_INCLUDE_DIRS})

# include file directories
target_include_directories(${PROJECT_NAME} PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${APR_INCLUDE_DIRS}
  common
  common/rapidjson
  comon/rapdidxml
  common/comm
  timesync
  timesync/user_uart
  livox_ros2_driver
)

# link libraries
target_link_libraries(${PROJECT_NAME}
	${livox_sdk_LIBRARIES}
	${Boost_LIBRARY}
	${PCL_LIBRARIES}
	${APR_LIBRARIES}
)

# Lifecycle node executable
add_executable(${PROJECT_NAME}_node
  livox_ros2_driver/livox_ros2_driver.cpp
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${livox_sdk_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  common
  common/comm
  timesync
  livox_ros2_driver
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  pcl_conversions
  livox_interfaces
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
