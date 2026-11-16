# State presented to the factor graph before optimization.
std_msgs/Header header

# Shared numeric index used by the X(index), V(index), and B(index) GTSAM keys.
uint64 state_index

# State-vector layout:
#   pose, velocity, accelerometer_bias, gyroscope_bias, gravity
# Gravity is the full vector in m/s^2, not only its Unit3 direction.
geometry_msgs/Pose pose
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 accelerometer_bias
geometry_msgs/Vector3 gyroscope_bias
geometry_msgs/Vector3 gravity

# Motion contributed by this source since its previous graph pose. The transform is
# expressed in the previous body frame and is intended to be composed over time.
geometry_msgs/Pose pose_delta
