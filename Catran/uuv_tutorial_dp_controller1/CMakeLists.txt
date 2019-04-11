cmake_minimum_required(VERSION 2.8.3)
project(uuv_tutorial_dp_controller1)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy


)


catkin_package()

catkin_install_python(PROGRAMS scripts/tutorial_dp_controller.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)



