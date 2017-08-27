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

LIST(APPEND CMAKE_MODULE_PATH /home/tzh/caffe/build/install/share/Caffe/)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    hi_slam
    pluginlib
    std_msgs
    cv_bridge
    
    )

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

find_package(Caffe REQUIRED)


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
    ${Caffe_INCLUDE_DIRS}
    )

set(LIBS
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Caffe_LIBRARIES}
    )

add_definitions(${Caffe_DEFINITIONS})    # ex. -DCPU_ONLY


add_library(scene_recognition
    src/scene_recognition.cpp
    )

target_link_libraries(scene_recognition
    ${LIBS}
    )

add_library(scene_recognition_cnn
    src/classification_alexnet.cpp
    src/scene_recognition_cnn.cpp
    )

target_link_libraries(scene_recognition_cnn
    ${LIBS}
    )


add_executable(test_scene_recongnition
    src/classification_alexnet.cpp
    src/test_scene_recongnition.cpp
    src/scene_recognition.cpp
    src/scene_recognition_cnn.cpp
    )
target_link_libraries(test_scene_recongnition
    ${LIBS}
    )


add_executable(experiment
    src/test.cpp
    )
target_link_libraries(experiment
    ${LIBS}
    )


add_executable(experiment2
    src/test2.cpp
    )
target_link_libraries(experiment2
    ${LIBS}
    )



add_executable(caffe_demo src/classification.cpp)
target_link_libraries(caffe_demo 
    ${LIBS}
    )


add_executable(test_classification_alexnet
  src/test_classification_alexnet.cpp
  src/classification_alexnet.cpp
)
target_link_libraries(test_classification_alexnet
    ${LIBS}
)