# BoolMultiArray
std_msgs/MultiArrayLayout layout
# FIXME: PAE: it seems that this definition does not make much sense (data fields from layout up to data
# seem to be duplicated definitions from MultiArrayLayout and MultiArrayLDimension)  but I tried to avoid
# changing it during porting to ROS2 only removing leading spaces to fix idl conversion.
std_msgs/MultiArrayDimension[] dim
string label
uint32 size
uint32 stride
uint32 data_offset
bool[] data