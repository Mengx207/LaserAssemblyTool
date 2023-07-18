cmake_minimum_required(VERSION 2.8)
project( LaserDotMeasure )
set(CMAKE_CXX_FLAGS "-std=c++11")
SET("OpenCV_DIR" "/usr/share/OpenCV")
find_package( OpenCV REQUIRED )
find_package(Pylon QUIET)
find_package(Eigen3 REQUIRED)
if (NOT ${Pylon_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()
link_directories(${OpenCV_LIBRARY_DIRS})
include_directories( include
		     		${OpenCV_INCLUDE_DIRS}
		     		${Pylon_INCLUDE_DIRS}
	             	${pylon_camera_INCLUDE_DIRS}

 )


add_executable( arucolocate arucolocate.cpp)
target_link_libraries( arucolocate
		       ${OpenCV_LIBS}
	               ${Pylon_LIBRARIES}
                       ${pylon_camera_INCLUDE_DIRS}

)


add_executable( laserbeamguide laserbeamguide.cpp imgpro.cpp gencal.cpp)
target_link_libraries( laserbeamguide
		       ${OpenCV_LIBS}
	               ${Pylon_LIBRARIES}
                       ${pylon_camera_INCLUDE_DIRS}

)

add_executable( laserplaneguide laserplaneguide.cpp imgpro.cpp gencal.cpp)
target_link_libraries( laserplaneguide
		       ${OpenCV_LIBS}
	               ${Pylon_LIBRARIES}
                       ${pylon_camera_INCLUDE_DIRS}

)

add_executable( laserverification laserverification.cpp imgpro.cpp gencal.cpp)
target_link_libraries( laserverification
		       ${OpenCV_LIBS}
	               ${Pylon_LIBRARIES}
                       ${pylon_camera_INCLUDE_DIRS}

)




