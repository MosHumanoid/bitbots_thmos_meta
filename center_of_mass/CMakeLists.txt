cmake_minimum_required(VERSION 2.8.3)
project(center_of_mass)


find_package(catkin REQUIRED COMPONENTS
  urdfdom_py
  rospy
  bitbots_docs
)


catkin_python_setup()

catkin_package()

enable_bitbots_docs()

include_directories()
