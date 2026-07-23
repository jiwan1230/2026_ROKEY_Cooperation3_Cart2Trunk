std_msgs/Header header

string map_id
uint32 sequence

geometry_msgs/Pose trunk_pose
geometry_msgs/Vector3 inner_size

float32 resolution

Box3D[] occupied_boxes
geometry_msgs/Point[] boundary_points
geometry_msgs/Point[] obstacle_points

# Optional low-resolution voxel occupancy (leave size_x/y/z = 0 when unused)
uint32 size_x
uint32 size_y
uint32 size_z
uint8[] occupancy

float32 free_volume
float32 map_quality
