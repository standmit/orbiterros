cmake_minimum_required(VERSION 3.0.2)
project(orbiterros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_srvs
    tf2_msgs
    tf2_ros
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_static_transform_publisher src/static_transform_publisher.cpp)
add_dependencies(${PROJECT_NAME}_static_transform_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_static_transform_publisher ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_static_transform_publisher PROPERTIES OUTPUT_NAME static_transform_publisher)


install(
    TARGETS ${PROJECT_NAME}_static_transform_publisher
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES orbiter.launch debug.config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
