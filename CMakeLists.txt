cmake_minimum_required(VERSION 3.5)

project (classAggregator)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lgdal -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Wno-deprecated-declarations -Wno-unused-variable")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include 
                    ${CMAKE_CURRENT_SOURCE_DIR}/masa_protocol 
                    ${CMAKE_CURRENT_SOURCE_DIR}/tracker_CLASS)

#-------------------------------------------------------------------------------
# CUDA
#-------------------------------------------------------------------------------
find_package(CUDA 9.0 REQUIRED)
SET(CUDA_SEPARABLE_COMPILATION ON)
#set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -arch=sm_30 --compiler-options '-fPIC'")
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} --maxrregcount=32)

find_package(CUDNN REQUIRED)

# compile
cuda_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CUDA_INCLUDE_DIRS} ${CUDNN_INCLUDE_DIRS})


#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOPENCV")

find_package(yaml-cpp REQUIRED)

include_directories(/usr/include/gdal)

#-------------------------------------------------------------------------------
# Submodules
#-------------------------------------------------------------------------------
add_subdirectory(tracker_CLASS/c++)
#add_subdirectory(masa_protocol)


#-------------------------------------------------------------------------------
# Build Libraries
#-------------------------------------------------------------------------------
#include_directories("~/repos/cereal/include" ${CMAKE_CURRENT_SOURCE_DIR}/tracker_CLASS/c++/src /usr/include/python2.7)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include 
                    ${CMAKE_CURRENT_SOURCE_DIR}/tracker_CLASS/c++/include 
                    ${CMAKE_CURRENT_SOURCE_DIR}/masa_protocol/include)
include_directories(${CUDA_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${NVINFER_INCLUDES} ${CUDNN_INCLUDE_DIRS})

file(GLOB classAggregator_SRC "src/*.cpp")
set(classAggregator_LIBS  ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDNN_LIBRARIES} ${OpenCV_LIBS} yaml-cpp gdal class-tracker)
add_library(classAggregator SHARED ${classAggregator_SRC})
target_link_libraries(classAggregator ${classAggregator_LIBS})


#-------------------------------------------------------------------------------
# Build executables
#-------------------------------------------------------------------------------
add_executable(main demo/main.cpp)
target_link_libraries(main classAggregator)