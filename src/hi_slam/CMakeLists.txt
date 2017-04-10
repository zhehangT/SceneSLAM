cmake_minimum_required(VERSION 2.8.3)
project(hi_slam)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
  tf2
  tf2_ros
  pluginlib
  dynamic_reconfigure
  roslib
)

find_package(yaml-cpp REQUIRED)

generate_dynamic_reconfigure_options(
    cfg/hi_slam.cfg
)



catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES Test
   CATKIN_DEPENDS
#   roscpp
#  DEPENDS system_lib
)

Message("YAML_INCLUDE:${YAML_CPP_INCLUDE_DIR}")

include_directories(
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${YAML_CPP_INCLUDE_DIR}
)


set(LIBS 
${catkin_LIBRARIES}
${YAML_CPP_LIBRARIES}
)


add_executable(test_manual
src/test_manual.cpp
src/software_bus_manual.cpp
)
target_link_libraries(test_manual
${LIBS})

add_dependencies(test_manual ${PROJECT_NAME}_gencfg)

add_executable(main
src/main.cpp
src/software_bus.cpp
)
target_link_libraries(main
${LIBS})



add_executable(test_slam
src/test_slam.cpp
)
target_link_libraries(test_slam
${LIBS})


add_executable(test_config
src/test_config.cpp
    )

target_link_libraries(test_config
${LIBS}
    )




