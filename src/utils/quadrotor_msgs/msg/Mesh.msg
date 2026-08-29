# the mesh id, starts from "1".
uint32 trajectory_id

# the vertex matrix of the mesh, dimension is N*3
uint32 vertex_num
std_msgs/Float64MultiArray vertex_matrix

# the face matrix of the mesh, dimension is M*3
uint32 face_num
std_msgs/Int64MultiArray face_matrix
