cmake_minimum_required(VERSION 3.5)
project (classAggregator)

set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lgdal -pthread")  # -O3
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Wno-deprecated-declarations -Wno-unused-variable")


#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(yaml-cpp REQUIRED)

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

add_subdirectory(class-tracker)
add_subdirectory(libnabo)


#-------------------------------------------------------------------------------
# Build Libraries
#-------------------------------------------------------------------------------
#include_directories("~/repos/cereal/include" ${CMAKE_CURRENT_SOURCE_DIR}/class-tracker/src /usr/include/python2.7)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
		${CMAKE_CURRENT_SOURCE_DIR}/class-tracker/include
		#${CMAKE_CURRENT_SOURCE_DIR}/tkCommon/include
		${CMAKE_CURRENT_SOURCE_DIR}/libnabo/nabo
		${CMAKE_CURRENT_SOURCE_DIR}/masa_protocol/include)

find_package(pybind11 REQUIRED)
find_package(yaml-cpp REQUIRED)
include_directories(BEFORE ${Python3_INCLUDE_DIRS} ./include)
pybind11_add_module(deduplicator SHARED src/modules.cpp src/Deduplicator.cpp src/utils.cpp src/ClassAggregatorMessage.cpp)
target_link_libraries(deduplicator PUBLIC class-tracker gdal yaml-cpp nabo ${Python3_LIBRARIES})