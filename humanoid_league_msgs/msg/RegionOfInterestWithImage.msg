# A region of interest including the image data and its position in the original image

std_msgs/Header header

# The image data of the specific ROI (has the same size as the ROI)
sensor_msgs/Image image

# The region of interest describing the size and placement of the region in the original image.
sensor_msgs/RegionOfInterest regionOfInterest

# Width and height of the original image the ROI is taken from.
# This can be especially useful when visualizing the ROI in context
uint16 original_width
uint16 original_height
