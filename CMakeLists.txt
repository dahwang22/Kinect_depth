cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)
project("My Project")

set(CMAKE_CXX_FLAGS "-std=c++11")

# Set cmake prefix path to enable cmake to find freenect2
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)
find_package(freenect2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonInterp 2.7 REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)

message(STATUS "PYTHON_LIBRARIES = ${PYTHON_LIBRARIES}")
message(STATUS "PYTHON_EXECUTABLE = ${PYTHON_EXECUTABLE}")
message(STATUS "PYTHON_INCLUDE_DIRS = ${PYTHON_INCLUDE_DIRS}")

INCLUDE_DIRECTORIES("/usr/include/libusb-1.0/")
LINK_DIRECTORIES("/usr/lib/x86_64-linux-gnu")

INCLUDE_DIRECTORIES(
	${freenect2_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	"/usr/local/include"
	"/usr/local/include/eigen3"
	${PYTHON_INCLUDE_DIRS}
)
Link_DIRECTORIES(
	${PCL_LIBRARY_DIRS}
)
add_definitions(
	${PCL_DEFINITIONS}
)

add_executable(kinectOpencv kinectOpencv.cpp)
target_link_libraries(kinectOpencv ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})

ADD_EXECUTABLE( generate_pointcloud generatePointCloud.cpp )
TARGET_LINK_LIBRARIES( generate_pointcloud ${OpenCV_LIBS}  ${PCL_LIBRARIES}) 

add_executable(colordepth colordepth.cpp)
target_include_directories(colordepth PUBLIC ${OPENGL_INCLUDE_DIR})
target_link_libraries(colordepth ${OPENGL_glu_LIBRARY} ${OpenCV_LIBS} pthread Eigen3::Eigen)

