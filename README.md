pose_to_tf_publisher
====================

Package that publishes TF from poses ([geometry_msgs::PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html), [geometry_msgs::PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html), [nav_msgs::Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html), [Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html) or from a text file).

It can also republish / remap a given TF chain into another (with the possibility to add a time offset to the TF message -> useful to synchronize ground truth poses).

TF messages can be published continuously, with a timeout counted after the last valid pose estimation or one message for each pose received.
