
# @author Ryo Hanai

option(BUILD_SAMPLE_HIRO_CONTROLLER "Building a sample hiro controller for teaching plugin" ON)
if(WIN32)
  option(USE_ROS "Use ROS" OFF)
else()
  option(USE_ROS "Use ROS" ON)
endif()

if(NOT BUILD_SAMPLE_HIRO_CONTROLLER)
  return()
endif()

set(CONTROLLER_DIRS ../../SampleHiroControllerPlugin)

if(NOT ENABLE_PYTHON)
  find_package(PythonLibs REQUIRED)
endif()

if(USE_ROS)
  find_package(Eigen3 REQUIRED)

  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    trajectory_msgs
    geometric_shapes
    interactive_markers
    moveit_core
    moveit_ros_planning_interface
    moveit_visual_tools
    pluginlib
    )

    add_definitions(-DROS_ON)
endif(USE_ROS)

if(UNIX)
set(CMAKE_CXX_FLAGS "-std=c++11")
# set(CMAKE_CXX_FLAGS "-DBOOST_NO_EXCEPTIONS")
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_INCLUDE_CURRENT_DIR ON)
# set(CMAKE_AUTOMOC ON)
endif(UNIX)

set(libname CnoidSampleHiroControllerPlugin)

set(sources
  SampleHiroControllerPlugin.cpp
  SampleHiroController.cpp
  UR3dualController.cpp
  ControllerFramework.cpp
  )

set(headers
  SampleHiroController.h
  UR3dualController.h
  ControllerFramework.h
  Interpolator.h
  gettext.h
  )

set(includes
  ../teachingPlugin/Teaching
  ${PYTHON_INCLUDE_DIRS}
  )
set(libs
  CnoidBodyPlugin
  CnoidTeachingPlugin
  )

if(USE_ROS)
  set(sources
    ${sources}
    FollowTrajectoryControllerROS.cpp
    FollowTrajectoryControllerUR3Dual.cpp
    #MoveItControllerUR3Dual.cpp
    )
  set(headers
    ${headers}
    FollowTrajectoryControllerROS.h
    FollowTrajectoryControllerUR3Dual.h
    #MoveItControllerUR3Dual.h
    )
  set(includes
    ${includes}
    ${catkin_INCLUDE_DIRS}
    )
  set(libs
    ${libs}
    ${catkin_LIBRARIES}
    )
 endif(USE_ROS)

include_directories(${includes})
add_cnoid_plugin(${libname} SHARED ${sources} ${headers})
target_link_libraries(${libname} ${libs})

apply_common_setting_for_plugin(${libname} "${headers}")

