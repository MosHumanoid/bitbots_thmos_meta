# Information about a special line feature on the field

# The type defines which kind of intersection is present
uint8 UNDEFINED=0
uint8 L=1
uint8 T=2
uint8 X=3

uint8 type

# The pose of the line intersection and its certainty
humanoid_league_msgs/PoseWithCertainty pose
