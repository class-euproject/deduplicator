message("-- Found classAggregator")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} --std=c++11 -fPIC")

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDNN REQUIRED)

set(tkDNN_INCLUDE_DIRS 
	${CUDA_INCLUDE_DIRS} 
	${OPENCV_INCLUDE_DIRS} 
    ${CUDNN_INCLUDE_DIRS}
)

set(classAggregator_LIBRARIES 
    classAggregator 
    #kernels 
    ${CUDA_LIBRARIES} 
    ${CUDA_CUBLAS_LIBRARIES}
	${CUDNN_LIBRARIES}
	${OpenCV_LIBS}
)

set(classAggregator_FOUND true)
