cmake_minimum_required(VERSION 2.8.3)
project(parameter_tutorials)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

add_service_files(FILES
	SrvTutorial.srv
)

generate_messages(DEPENDENCIES	
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS
	message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(parameter_server
	src/parameter_server.cpp
)

add_dependencies(parameter_server ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})

target_link_libraries(parameter_server
	${catkin_LIBRARIES}
)

add_executable(parameter_client
	src/parameter_client.cpp
)

add_dependencies(parameter_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(parameter_client
	${catkin_LIBRARIES}
)


