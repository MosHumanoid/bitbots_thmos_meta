cmake_minimum_required(VERSION 2.8.3)
project(humanoid_league_field_rqt)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure std_msgs humanoid_league_msgs rospy bitbots_docs)



catkin_python_setup()

## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
   cfg/field_rqt_params.cfg
)

catkin_package()

enable_bitbots_docs()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/2d_field
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
