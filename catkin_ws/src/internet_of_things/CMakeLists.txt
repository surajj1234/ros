cmake_minimum_required(VERSION 2.8.3)
project(internet_of_things)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation image_transport sensor_msgs cv_bridge)

## Generate messages in the 'msg' folder
#add_message_files(FILES Object_Params.msg Filtered_Params.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime) 

include_directories(include ${catkin_INCLUDE_DIRS})


## Declare a cpp executable
add_executable(baby_monitor src/baby_monitor.cpp)

target_link_libraries(baby_monitor ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS src/voice_recognition.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS src/mamaRoo_bt_controller.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS src/origami_network_controller.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


