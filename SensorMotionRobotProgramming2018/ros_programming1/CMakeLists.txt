cmake_minimum_required(VERSION 2.8.3)
project(ros_programming1)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
)

add_message_files(
   FILES
   rand_msg.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_pub src/pub.cpp)

add_dependencies(ros_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ros_pub
   ${catkin_LIBRARIES}
)


add_executable(ros_sub src/sub.cpp)

add_dependencies(ros_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ros_sub
   ${catkin_LIBRARIES}
)

