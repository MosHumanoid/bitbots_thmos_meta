# An obstacle relative to the robot

# Main color of the obstacle, to differentiate between robots and other things like human legs
# Something we cant classify
uint8 UNDEFINED = 0
# robot without known color
uint8 ROBOT_UNDEFINED = 1
uint8 ROBOT_MAGENTA = 2
uint8 ROBOT_CYAN = 3
# A human legs, e.g. from the referee
uint8 HUMAN = 4
# Black poles which are normally used for technical challenges
uint8 POLE = 5
uint8 type

# The number of the robot, if it is a robot and if it can be read. Put in 0 if not known
uint8 playerNumber

# Position (in meters)
humanoid_league_msgs/PoseWithCertainty pose

# Educated guess of the width and height (in meters)
float32 width
float32 height
