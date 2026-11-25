std_msgs/Header header
uint64 generation
geometry_msgs/Pose target
uint32 seeds_tried
uint32 seeds_total
bool complete
IKSolution[] solutions
# Counts are per candidate checked, not unique rejected configurations.
# A candidate may contribute to both collision categories.
uint32 self_collision_rejections
uint32 environment_collision_rejections
uint32 check_errors
string last_rejection
