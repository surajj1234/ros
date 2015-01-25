cmake_minimum_required(VERSION 2.8.3)
project(baby_gate)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation image_transport sensor_msgs cv_bridge)

## Generate messages in the 'msg' folder
add_message_files(FILES Object_Params.msg Filtered_Params.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime) 

include_directories(include ${catkin_INCLUDE_DIRS})


## Declare a cpp executable
add_executable(kinect_imaq_depth src/kinect_imaq_depth.cpp)
add_executable(kinect_imaq_disparity src/kinect_imaq_disparity.cpp)
add_executable(object_detector src/object_detector.cpp)
add_executable(tracker src/tracker.cpp src/Serial.cpp src/SerialComms.cpp)
add_executable(visualizer src/visualizer.cpp)

## Specify libraries to link a library or executable target against
#add_library(SerialComms src/SerialComms.cpp src/Serial.cpp include/Serial.h include/SerialComms.h)

#link_directories(/home/suraj/ros/catkin_ws/src/baby_gate/lib) # ${catkin_LINK_DIRS})
target_link_libraries(kinect_imaq_depth ${catkin_LIBRARIES})
target_link_libraries(kinect_imaq_disparity ${catkin_LIBRARIES})
target_link_libraries(object_detector ${catkin_LIBRARIES})
target_link_libraries(tracker ${catkin_LIBRARIES})
target_link_libraries(visualizer ${catkin_LIBRARIES})

##add_dependencies(kinect_imaq_depth baby_gate_generate_messages_cpp)





