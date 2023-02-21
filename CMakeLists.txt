cmake_minimum_required(VERSION 2.8)
project( LaserDotMeasure_Origin )
set(CMAKE_CXX_FLAGS "-std=c++11")
find_package( OpenCV REQUIRED )
find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()
include_directories( include
		     ${OpenCV_INCLUDE_DIRS}
		     ${Pylon_INCLUDE_DIRS}
	             ${pylon_camera_INCLUDE_DIRS}
 )
add_executable( LaserDotMeasure_Origin LaserDotMeasure_Origin.cpp )
target_link_libraries( LaserDotMeasure_Origin
		       ${OpenCV_LIBS}
	               ${Pylon_LIBRARIES}
                       ${pylon_camera_INCLUDE_DIRS}
                      
 )

add_executable( LaserDotMeasure_Pixel LaserDotMeasure_Pixel.cpp )
target_link_libraries( LaserDotMeasure_Pixel
		       ${OpenCV_LIBS}
	               ${Pylon_LIBRARIES}
                       ${pylon_camera_INCLUDE_DIRS}
                      
 )
