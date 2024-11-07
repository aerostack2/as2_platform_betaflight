# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.5)

# Set the project name
set(PROJECT_NAME as2_platform_betaflight)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set Release as default build type if not set
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  as2_core
  as2_msgs
  std_srvs
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  nav_msgs
)

# add a submodules/msp DIRECTORY

find_package(PkgConfig REQUIRED)
pkg_search_module(PKG_MSP REQUIRED IMPORTED_TARGET msp)

include_directories(${PKG_MSP_INCLUDE_DIRS})
link_directories(${PKG_MSP_LIBRARY_DIRS})

message(WARNING "PKG_MSP_INCLUDE_DIRS: ${PKG_MSP_INCLUDE_DIRS}")
message(WARNING "PKG_MSP_LIBRARIES: ${PKG_MSP_LIBRARIES}")
message(WARNING "PKG_MSP_LIBRARY_DIRS: ${PKG_MSP_LIBRARY_DIRS}")
message(WARNING "PKG_MSP_LDFLAGS: ${PKG_MSP_LDFLAGS}")

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Include necessary directories
include_directories(
  include
  include/${PROJECT_NAME}
  ${PKG_MSP_INCLUDE_DIRS}
)

# Set source files
set(SOURCE_CPP_FILES
  src/betaflight_platform.cpp
  src/thrust_map.cpp
)

# Create the node executable
add_executable(${PROJECT_NAME}_node
  src/betaflight_platform_node.cpp
  ${SOURCE_CPP_FILES})

target_link_libraries(${PROJECT_NAME}_node
  ${PKG_MSP_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

# # Create the library
add_library(${PROJECT_NAME} STATIC ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} ${PKG_MSP_LIBRARIES})

# Install the headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the node executable
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# # Install the launch directory
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

# Install the config directory
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

# Build tests if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(tests)
endif()

# Create the ament package
ament_package()
