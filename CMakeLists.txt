cmake_minimum_required(VERSION 2.8.3)
project(kinnect_mapper)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs message_filters pcl_ros)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(example_publisher src/example_publisher.cpp)
target_link_libraries(example_publisher ${catkin_LIBRARIES})

add_executable(example_subscriber src/example_subscriber.cpp)
target_link_libraries(example_subscriber ${catkin_LIBRARIES})

add_executable(kinect_mapper src/kinect_mapper.cpp src/KinectToLaserAdapter.cpp src/LinearAlgebraUtils.cpp)
target_link_libraries(kinect_mapper ${catkin_LIBRARIES} ${OpenCV_LIBS} -L/opt/ros/indigo/lib/ -lcv_bridge)

add_executable(pcd2ply src/pcd2ply.cpp)
target_link_libraries(pcd2ply ${catkin_LIBRARIES})