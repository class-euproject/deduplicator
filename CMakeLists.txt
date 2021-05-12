cmake_minimum_required(VERSION 3.5)
project (classAggregator)

set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lgdal -pthread -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Wno-deprecated-declarations -Wno-unused-variable")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTKPROJ_PATH='\"${CMAKE_CURRENT_SOURCE_DIR}/tkCommon/\"'")

#-------------------------------------------------------------------------------
# CUDA
#-------------------------------------------------------------------------------
#find_package(CUDA 9.0 REQUIRED)
#SET(CUDA_SEPARABLE_COMPILATION ON)
#set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -arch=sm_30 --compiler-options '-fPIC'")
#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} --maxrregcount=32)

#find_package(CUDNN REQUIRED)

# compile
#cuda_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CUDA_INCLUDE_DIRS} ${CUDNN_INCLUDE_DIRS})

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#find_package(OpenCV REQUIRED)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOPENCV")

#find_package(yaml-cpp REQUIRED)

include_directories(/usr/include/gdal)

# include_directories(/usr/include/python2.7)
find_package(Python3 COMPONENTS Interpreter Development)
message("Python_FOUND:${Python3_FOUND}")
message("Python_VERSION:${Python3_VERSION}")
message("Python_Development_FOUND:${Python3_Development_FOUND}")
message("Python_LIBRARIES:${Python3_LIBRARIES}")
message("Python_INCLUDE_DIRS:${Python3_INCLUDE_DIRS}")

#-------------------------------------------------------------------------------
# Submodules
#-------------------------------------------------------------------------------

#add_subdirectory(tkCommon)
add_subdirectory(class-tracker)
add_subdirectory(libnabo)
#add_subdirectory(masa_protocol)


#-------------------------------------------------------------------------------
# Build Libraries
#-------------------------------------------------------------------------------
#include_directories("~/repos/cereal/include" ${CMAKE_CURRENT_SOURCE_DIR}/class-tracker/src /usr/include/python2.7)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/class-tracker/include
        #${CMAKE_CURRENT_SOURCE_DIR}/tkCommon/include
	${CMAKE_CURRENT_SOURCE_DIR}/libnabo/nabo
        ${CMAKE_CURRENT_SOURCE_DIR}/masa_protocol/include)
#include_directories(${CUDA_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${CUDNN_INCLUDE_DIRS})

#file(GLOB classAggregator_SRC "src/*.cpp") # AggregatorViewer.h and AggregatorViewer.cpp should be discarded
#list(REMOVE_ITEM classAggregator_SRC ${CMAKE_SOURCE_DIR}/src/modules.cpp)
#set(classAggregator_LIBS  ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDNN_LIBRARIES} ${OpenCV_LIBS} yaml-cpp gdal tkCommon class-tracker)
#set(classAggregator_LIBS ${OpenCV_LIBS} yaml-cpp gdal class-tracker)
#message("${classAggregator_LIBS}")
#add_library(classAggregator SHARED ${classAggregator_SRC})
#target_link_libraries(classAggregator ${classAggregator_LIBS})

find_package(pybind11 REQUIRED)
include_directories(BEFORE ${Python3_INCLUDE_DIRS} ./include)
pybind11_add_module(deduplicator SHARED src/modules.cpp src/Deduplicator.cpp src/utils.cpp src/ClassAggregatorMessage.cpp)
target_link_libraries(deduplicator PUBLIC class-tracker gdal nabo ${Python3_LIBRARIES})


#-------------------------------------------------------------------------------
# Build executables
#-------------------------------------------------------------------------------

#add_executable(class-aggregator demo/main.cpp)
#target_link_libraries(class-aggregator classAggregator)

#add_executable(test-aggregator demo/test.cpp)
#target_link_libraries(test-aggregator classAggregator)
