cmake_minimum_required(VERSION 2.8.3)
project(python_executables)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  computeTF2frames.py
  convBP_VLP.py
  convM1600_VLP.py
  convMID70_M1600.py
  imu_filter.py
  navtogps.py
  Odom_publisher.py
  Unitree_tranformer.py
  interpolate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
