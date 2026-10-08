# Diagnostic-only snapshot, sampled at WorldUpdateEnd. Poses are world-relative.
# No command, contact-success, or close-permission semantics.
std_msgs/Header header
string source_instance
uint64 world_epoch
string vehicle_name
uint64 vehicle_id
bool joint_measurement_valid
string[] link_names
uint64[] link_ids
geometry_msgs/Pose[] link_poses
string[] joint_names
float64[] joint_position
float64[] joint_velocity
string target_name
bool target_present
uint64 target_id
geometry_msgs/Pose target_pose
bool body_velocity_valid
geometry_msgs/Vector3 body_linear_velocity_world
geometry_msgs/Vector3 body_angular_velocity_world
