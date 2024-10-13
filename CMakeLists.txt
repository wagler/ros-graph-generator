cmake_minimum_required(VERSION 3.8)
project(ros-graph-generator)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Protobuf REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add the include directory
# Include directories
include_directories(
    include
    ${Protobuf_INCLUDE_DIRS}
)

# # Generate protobuf files
# set(PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/protobuf/graph.proto")
# protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# Get the source files
file(GLOB_RECURSE SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" 
	"${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc" 

)
message(${SOURCE_FILES})

add_executable(ros_graph_generator ${SOURCE_FILES})
ament_target_dependencies(ros_graph_generator rclcpp std_msgs Protobuf)
#target_link_libraries(ros_graph_generator ${Protobuf_LIBRARIES})

install(TARGETS
  ros_graph_generator
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
