cmake_minimum_required(VERSION 2.8.3)
project(humanoid_league_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  actionlib
  actionlib_msgs
  bitbots_docs
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    Animation.msg
    Audio.msg
    BallInImage.msg
    BallInImageArray.msg
    GameState.msg
    GoalPostInImage.msg
    GoalPostInImageArray.msg
    HeadMode.msg
    LineInformationInImage.msg
    LineInformationRelative.msg
    LineIntersectionInImage.msg
    LineIntersectionRelative.msg
    LineSegmentInImage.msg
    LineSegmentRelative.msg
    ObstacleInImage.msg
    ObstacleInImageArray.msg
    ObstacleRelative.msg
    ObstacleRelativeArray.msg
    PoseWithCertainty.msg
    PoseWithCertaintyArray.msg
    PoseWithCertaintyStamped.msg
    RegionOfInterestWithImage.msg
    RobotControlState.msg
    Strategy.msg
    TeamData.msg
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  PlayAnimation.action
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   trajectory_msgs
   actionlib_msgs
 )

catkin_package(
   CATKIN_DEPENDS
   message_runtime
   std_msgs
   geometry_msgs
   sensor_msgs
   actionlib_msgs
   trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

enable_bitbots_docs()
