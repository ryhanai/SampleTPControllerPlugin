
# @author Ryo Hanai

option(BUILD_SAMPLE_HIRO_CONTROLLER "Building a sample hiro controller for teaching plugin" ON)

if(NOT BUILD_SAMPLE_HIRO_CONTROLLER)
  return()
endif()

set(CONTROLLER_DIRS ../../SampleHiroControllerPlugin)

if(NOT ENABLE_PYTHON)
  find_package(PythonLibs REQUIRED)
endif()

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
  ControllerFramework.cpp
  )

set(headers
  SampleHiroController.h
  ControllerFramework.h
  Interpolator.h
  gettext.h
  )

include_directories(../teachingPlugin/Teaching ${PYTHON_INCLUDE_DIRS})
add_cnoid_plugin(${libname} SHARED ${sources} ${headers})
#target_link_libraries(${libname} CnoidTeachingPlugin) # This doesn't work. why???
target_link_libraries(${libname} CnoidBodyPlugin CnoidTeachingPlugin)

apply_common_setting_for_plugin(${libname} "${headers}")

