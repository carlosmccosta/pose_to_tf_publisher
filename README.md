pose_to_tf_publisher
====================

Package that publishes tf from poses (geometry_msgs::PoseStamped, geometry_msgs::PoseWithCovarianceStamped, nav_msgs::Odometry).
TF messages can be published continuously, with a timeout counted after the last valid pose estimation or one message for each pose estimation.
