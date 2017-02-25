cmake_minimum_required(VERSION 2.8.3)
project(take_picture_cart)

find_package(roscpp REQUIRED)
find_package(roslib REQUIRED)
find_package(catkin REQUIRED COMPONENTS cwru_davinci_traj_streamer cwru_davinci_kinematics roslib message_generation std_msgs sensor_msgs cwru_opencv_common roscpp)
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

#find_package(catkin_simple REQUIRED)

#catkin_simple()
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} ${LIBUSB_1_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} include)
catkin_package(
	INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
	DEPENDS roscpp roslib
	LIBRARIES playfile_record
	CATKIN_DEPENDS cwru_davinci_traj_streamer cwru_davinci_kinematics roscpp roslib message_runtime std_msgs sensor_msgs

)


# Executables


add_executable(take_picture_cart src/take_picture_cart.cpp )

target_link_libraries(take_picture_cart ${catkin_LIBRARIES})

