cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
)

add_message_files(FILES
	simple_msg.msg
)

generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
)

include_directories(
	include ${catkin_INCLUDE_DIRS}
)

add_executable(talker
    src/talker.cpp
)

add_dependencies(talker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(talker
	${catkin_LIBRARIES}
)
add_executable(listener
	src/listener.cpp
)
add_dependencies(listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(listener
	${catkin_LIBRARIES}
)
