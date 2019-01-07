cmake_minimum_required(VERSION 2.8.3)
project(service_tutorial)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
)


add_service_files(
   FILES
   test_srv.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(server src/server.cpp)

add_dependencies(server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(server
   ${catkin_LIBRARIES}
)

add_executable(client src/client.cpp)

add_dependencies(client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(client
   ${catkin_LIBRARIES}
)
