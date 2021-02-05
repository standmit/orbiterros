cmake_minimum_required(VERSION 3.0.2)
project(orbiterros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rviz
    std_srvs
    tf2_msgs
    tf2_ros
)

find_package(OGRE QUIET)
if(OGRE_FOUND)
    if(${OGRE_VERSION} VERSION_LESS 1.11)
        set(OGRE_INCLUDE_DIRS ${OGRE_INCLUDE_DIRS} ${OGRE_Overlay_INCLUDE_DIRS})
        set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${OGRE_Overlay_LIBRARIES})
    endif()
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(X REQUIRED OGRE)
    set(OGRE_VERSION ${X_OGRE_VERSION})
    set(OGRE_INCLUDE_DIRS ${X_INCLUDE_DIRS})
    message(STATUS "OGRE_INCLUDE_DIRS=${OGRE_INCLUDE_DIRS}")
    
    if(NOT DEFINED OGRE_LIBRARIES)
        set(_OGRE_LIBRARIES)
        foreach(_lib ${X_LIBRARIES})
            set(_lib_tag "OGRE_RVIZ_${_lib}")
            find_library(${_lib_tag}
                NAMES ${_lib}
                HINTS ${X_LIBRARY_DIRS}
                PATHS ${X_LIBRARY_DIRS}
            )
            list(APPEND _OGRE_LIBRARIES ${${_lib_tag}})
        endforeach(_lib)
        set(OGRE_LIBRARIES ${_OGRE_LIBRARIES} CACHE STRING "Absolute paths to OGRE libs")
    endif(NOT DEFINED OGRE_LIBRARIES)
    message(STATUS "OGRE_LIBRARIES=${OGRE_LIBRARIES}")
endif()

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)

catkin_package(
    LIBRARIES
        ${PROJECT_NAME}_rviz_plugin
)

include_directories(
    src/rviz_plugin
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_static_transform_publisher src/static_transform_publisher.cpp)
add_dependencies(${PROJECT_NAME}_static_transform_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_static_transform_publisher ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_static_transform_publisher PROPERTIES OUTPUT_NAME static_transform_publisher)

add_library(${PROJECT_NAME}_rviz_plugin src/rviz_plugin/scale_panel.cpp)
target_link_libraries(${PROJECT_NAME}_rviz_plugin ${QT_LIBRARIES} ${catkin_LIBRARIES})


install(
    TARGETS
        ${PROJECT_NAME}_static_transform_publisher
        ${PROJECT_NAME}_rviz_plugin
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
    FILES
        orbiter.launch debug.config
        rviz_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
