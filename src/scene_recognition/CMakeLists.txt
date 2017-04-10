cmake_minimum_required(VERSION 2.8.3)
project(scene_recognition)


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



find_package(catkin REQUIRED COMPONENTS
    roscpp
    hi_slam
    pluginlib
    std_msgs
    cv_bridge
    )

find_package(OpenCV 3.0 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scene_recognition
#  CATKIN_DEPENDS
#  hi_slam
#  pluginlib
#  std_msgs
#  DEPENDS system_lib
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

set(LIBS
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )

add_library(scene_recognition
    src/scene_recognition.cpp
    )

target_link_libraries(scene_recognition
    ${LIBS}
    )


add_executable(main
    src/main
    src/scene_recognition.cpp
    )

target_link_libraries(main
    ${LIBS}
    )
