cmake_minimum_required(VERSION 2.8.3)
project(led_controller)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

catkin_package(
	CATKIN_DEPENDS
	message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(led_pub
	src/led_pub.cpp
)

add_dependencies(led_pub ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})

target_link_libraries(led_pub
	${catkin_LIBRARIES}
)



