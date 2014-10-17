/**\file pose_to_tf_publisher.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <pose_to_tf_publisher/pose_to_tf_publisher.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace pose_to_tf_publisher {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
PoseToTFPublisher::PoseToTFPublisher() :
		publish_rate_(100),
		publish_last_pose_tf_timeout_seconds_(-1.0),
		last_pose_time_(0),
		invert_tf_transform_(false),
		invert_tf_hierarchy_(false),
		transform_pose_to_map_frame_id_(true),
		tf_collector_(ros::Duration(600.0)),
		number_tfs_published_(0) {
}

PoseToTFPublisher::~PoseToTFPublisher() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <ros integration functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void PoseToTFPublisher::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	node_handle_ = node_handle;
	private_node_handle_ = private_node_handle;

	private_node_handle_->param("publish_rate", publish_rate_, 100.0);
	private_node_handle_->param("publish_last_pose_tf_timeout_seconds", publish_last_pose_tf_timeout_seconds_, -1.0);
	double tf_lookup_timeout;
	private_node_handle_->param("tf_lookup_timeout", tf_lookup_timeout, 0.5);
	tf_lookup_timeout_.fromSec(tf_lookup_timeout);

	private_node_handle_->param("pose_stamped_topic", pose_stamped_topic_, std::string(""));
	private_node_handle_->param("pose_with_covariance_stamped_topic", pose_with_covariance_stamped_topic_, std::string("/initialpose"));
	private_node_handle_->param("odometry_topic", odometry_topic_, std::string(""));
	private_node_handle_->param("poses_filename", poses_filename_, std::string(""));

	private_node_handle_->param("map_frame_id", map_frame_id_, std::string("map"));
	private_node_handle_->param("odom_frame_id", odom_frame_id_, std::string("odom"));
	private_node_handle_->param("base_link_frame_id", base_link_frame_id_, std::string("base_link"));

	private_node_handle_->param("invert_tf_transform", invert_tf_transform_, false);
	private_node_handle_->param("invert_tf_hierarchy", invert_tf_hierarchy_, false);
	private_node_handle_->param("transform_pose_to_map_frame_id", transform_pose_to_map_frame_id_, true);

	std::string child_frame = (odom_frame_id_.empty() ? base_link_frame_id_ : odom_frame_id_);

	transform_stamped_.header.frame_id = invert_tf_hierarchy_ ? child_frame : map_frame_id_;
	transform_stamped_.child_frame_id = invert_tf_hierarchy_ ? map_frame_id_ : child_frame;
	transform_stamped_.transform.translation.x = 0.0;
	transform_stamped_.transform.translation.y = 0.0;
	transform_stamped_.transform.translation.z = 0.0;
	transform_stamped_.transform.rotation.x = 0.0;
	transform_stamped_.transform.rotation.y = 0.0;
	transform_stamped_.transform.rotation.z = 0.0;
	transform_stamped_.transform.rotation.w = 1.0;
}


void PoseToTFPublisher::publishInitialPoseFromParameterServer() {
	// initial pose tf
	double x, y, z, roll, pitch, yaw;
	bool initial_pose_in_map_to_base;
	private_node_handle_->param("initial_pose_in_map_to_base", initial_pose_in_map_to_base, true);
	private_node_handle_->param("initial_x", x, 0.0);
	private_node_handle_->param("initial_y", y, 0.0);
	private_node_handle_->param("initial_z", z, 0.0);
	private_node_handle_->param("initial_roll", roll, 0.0);
	private_node_handle_->param("initial_pitch", pitch, 0.0);
	private_node_handle_->param("initial_yaw", yaw, 0.0);

	if (initial_pose_in_map_to_base) {
		publishTFFromMapToBasePose(x, y, z, roll, pitch, yaw);
	} else {
		publishTFFromMapToOdomPose(x, y, z, roll, pitch, yaw);
	}
}


void PoseToTFPublisher::startPublishingTF() {
	if (transform_stamped_.header.frame_id.empty() || transform_stamped_.child_frame_id.empty()) {
		ROS_ERROR_STREAM("Source and target frames can't be empty [ map_frame_id: " << map_frame_id_ << " | base_link_frame_id: " << base_link_frame_id_ << " ]");
		return;
	}

	if (!poses_filename_.empty()) {
		if (startPublishingTFFromFile(poses_filename_)) {
			return;
		} else {
			ROS_ERROR_STREAM("Invalid poses file: " << poses_filename_);
		}
	}

	startPublishingTFFromPoseTopics();
}


bool PoseToTFPublisher::startPublishingTFFromFile(std::string poses_filename) {
	std::ifstream input_stream(poses_filename.c_str());
	if (input_stream.is_open()) {
		geometry_msgs::Pose current_pose;
		geometry_msgs::Pose next_pose;
		double current_pose_time;
		double next_pose_time;

		if (getPoseFromFile(current_pose, current_pose_time, input_stream)) {
			ROS_INFO_STREAM("Publishing tf [ " << transform_stamped_.header.frame_id << " -> " << transform_stamped_.child_frame_id << " ] from file " << poses_filename);

			ros::Time::waitForValid();

			publishTFFromPose(current_pose, map_frame_id_, ros::Time(current_pose_time));

			while (getPoseFromFile(next_pose, next_pose_time, input_stream)) {
				double current_time = ros::Time::now().toSec();
				double delay_for_next_pose_publish = next_pose_time - current_time - 0.005; // give some time to publish tf and send it before its timestamp
				if (delay_for_next_pose_publish > -3.0) { // discard old tfs
					if (delay_for_next_pose_publish > 0.0 && ros::Time::now().sec != 0) { ros::Duration(delay_for_next_pose_publish).sleep(); }

					current_pose = next_pose;
					current_pose_time = next_pose_time;
					publishTFFromPose(current_pose, map_frame_id_, ros::Time(current_pose_time));
				} else {
					ROS_WARN_STREAM("Dropping pose from file [ current time: " << current_time << " | pose time: " << next_pose_time << " ]");
				}
			}

			return true;
		}
	}

	return false;
}


bool PoseToTFPublisher::getPoseFromFile(geometry_msgs::Pose& pose_out, double& timestamp_out, std::ifstream& input_stream) {
	std::string line;
	while (std::getline(input_stream, line)) {
		std::stringstream ss(line);
		double time;
		if ((ss >> timestamp_out) &&
			(ss >> pose_out.position.x) &&
			(ss >> pose_out.position.y) &&
			(ss >> pose_out.position.z) &&
			(ss >> pose_out.orientation.x) &&
			(ss >> pose_out.orientation.y) &&
			(ss >> pose_out.orientation.z) &&
			(ss >> pose_out.orientation.w) ) {
			return true;
		}
	}

	return false;
}


void PoseToTFPublisher::startPublishingTFFromPoseTopics() {
	publishInitialPoseFromParameterServer();
	std::stringstream ss;
	if (!pose_stamped_topic_.empty()) {
		ss << " " << pose_stamped_topic_;
		pose_stamped_subscriber_ = node_handle_->subscribe(pose_stamped_topic_, 5, &pose_to_tf_publisher::PoseToTFPublisher::publishTFFromPoseStamped, this);
	}

	if (!pose_with_covariance_stamped_topic_.empty()) {
		ss << " " << pose_with_covariance_stamped_topic_;
		pose_with_covariance_stamped_subscriber_ = node_handle_->subscribe(pose_with_covariance_stamped_topic_, 5,
				&pose_to_tf_publisher::PoseToTFPublisher::publishTFFromPoseWithCovarianceStamped, this);
	}

	if (!odometry_topic_.empty()) {
		ss << " " << odometry_topic_;
		odometry_subscriber_ = node_handle_->subscribe(odometry_topic_, 5, &pose_to_tf_publisher::PoseToTFPublisher::publishTFFromOdometry, this);
	}

	ROS_INFO_STREAM("Publishing tf [ " << transform_stamped_.header.frame_id << " -> " << transform_stamped_.child_frame_id << " ] from pose topics [" << ss.str() << " ]");

	ros::Rate publish_rate(publish_rate_);
	while (ros::ok()) {
		sendTF();
		publish_rate.sleep();
		ros::spinOnce();
	}
}


void PoseToTFPublisher::stopPublishingTFFromPoseTopics() {
	if (!pose_stamped_topic_.empty()) {
		pose_stamped_subscriber_.shutdown();
	}

	if (!pose_with_covariance_stamped_topic_.empty()) {
		pose_with_covariance_stamped_subscriber_.shutdown();
	}

	if (!odometry_topic_.empty()) {
		odometry_subscriber_.shutdown();
	}
}


void PoseToTFPublisher::publishTFFromPose(const geometry_msgs::Pose& pose, const std::string& frame_id, const ros::Time& pose_time) {
	ros::Time pose_time_updated = pose_time;
	if (pose_time.sec == 0 && pose_time.nsec == 0) { // time in the future to override any poses coming from the localization node
		pose_time_updated = ros::Time::now() + ros::Duration(0.25);
		ROS_INFO("Reseting initial pose...");
	}

	if (pose_time_updated < last_pose_time_ || frame_id.empty()) {
		ROS_WARN_STREAM("Dropping new pose with time [" << pose_time_updated << "] and frame_id [" << frame_id << "]");
		return;
	}

	tf2::Transform transform_pose(
			tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
			tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));

	if (transform_pose_to_map_frame_id_ && frame_id != map_frame_id_) {
		// transform to map (global frame reference)
		ROS_WARN_STREAM("Pose received in " << frame_id << " frame instead of " << map_frame_id_);
		tf2::Transform transform_pose_to_map;
		if (!tf_collector_.lookForTransform(transform_pose_to_map, map_frame_id_, frame_id, pose_time, tf_lookup_timeout_)) {
			ROS_WARN_STREAM("Dropping new pose with time [" << pose_time_updated << "] because there isn't tf between [" << map_frame_id_ << "] and [" << frame_id << "]");
			return;
		}

		transform_pose *= transform_pose_to_map;
	}

	publishTF(transform_pose, pose_time, tf_lookup_timeout_);
	last_pose_time_ = pose_time_updated;
}


void PoseToTFPublisher::publishTFFromPoseStamped(const geometry_msgs::PoseStampedConstPtr& pose) {
	publishTFFromPose(pose->pose, pose->header.frame_id, pose->header.stamp);
}


void PoseToTFPublisher::publishTFFromPoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {
	publishTFFromPose(pose->pose.pose, pose->header.frame_id, pose->header.stamp);
}


void PoseToTFPublisher::publishTFFromOdometry(const nav_msgs::OdometryConstPtr& odom) {
	publishTFFromPose(odom->pose.pose, odom->header.frame_id, odom->header.stamp);
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </ros integration functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <tf update functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool PoseToTFPublisher::addOdometryDisplacementToTransform(tf2::Transform& transform, const ros::Time& time_of_transform, const ros::Time& target_time) {
	tf2::Transform odometry_tf_from_pose_time_to_now;
	if (tf_collector_.lookForTransform(odometry_tf_from_pose_time_to_now, base_link_frame_id_, target_time, base_link_frame_id_, time_of_transform, map_frame_id_, tf_lookup_timeout_)) {
		// include odometry displacement from pose publish time to current time
		transform = transform * odometry_tf_from_pose_time_to_now.inverse();
		return true;
	}

	return false;
}


bool PoseToTFPublisher::sendTF(bool check_pose_timeout) {
	ros::Time current_time = ros::Time::now();
	if (!check_pose_timeout || publish_last_pose_tf_timeout_seconds_ <= 0.0 || (current_time - last_pose_time_).toSec() <= publish_last_pose_tf_timeout_seconds_ || last_pose_time_.toSec() < 1.0) {
		transform_stamped_.header.seq = number_tfs_published_++;
		transform_stamped_.header.stamp = current_time;
		transform_broadcaster_.sendTransform(transform_stamped_);
		return true;
	}

	ROS_WARN_STREAM_THROTTLE(1.0, "Pose to TF publisher reached timeout for last valid pose" \
			<< "\n\tTF translation -> [ x: " << transform_stamped_.transform.translation.x << " | y: " << transform_stamped_.transform.translation.y << " | z: " << transform_stamped_.transform.translation.z << " ]" \
			<< "\n\tTF orientation -> [ qx: " << transform_stamped_.transform.rotation.x << " | qy: " << transform_stamped_.transform.rotation.y << " | qz: " << transform_stamped_.transform.rotation.z << " | qw: " << transform_stamped_.transform.rotation.w << " ]" \
			<< "\n\tCurrent time: " << current_time \
			<< "\n\tLast pose time: " << last_pose_time_);
	return false;
}

bool PoseToTFPublisher::updateTFMessage(tf2::Transform& transform) {
	if (boost::math::isfinite(transform.getOrigin().getX()) &&
		boost::math::isfinite(transform.getOrigin().getY()) &&
		boost::math::isfinite(transform.getOrigin().getZ()) &&
		boost::math::isfinite(transform.getRotation().getX()) &&
		boost::math::isfinite(transform.getRotation().getY()) &&
		boost::math::isfinite(transform.getRotation().getZ()) &&
		boost::math::isfinite(transform.getRotation().getW())) {

		if (transform.getRotation().getX() == 0.0 && transform.getRotation().getY() == 0.0 && transform.getRotation().getZ() == 0.0 && transform.getRotation().getW() == 0.0) {
			ROS_WARN("Dropped pose with invalid quaternion!");
			return false;
		}

		transform.getRotation().normalize();
		laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformTF2ToMsg(transform, transform_stamped_.transform);
		ROS_DEBUG_STREAM("Updating TF between " << transform_stamped_.header.frame_id << " and " << transform_stamped_.child_frame_id \
				<< "\n\tTF translation -> [ x: " << transform_stamped_.transform.translation.x << " | y: " << transform_stamped_.transform.translation.y << " | z: " << transform_stamped_.transform.translation.z << " ]" \
				<< "\n\tTF orientation -> [ qx: " << transform_stamped_.transform.rotation.x << " | qy: " << transform_stamped_.transform.rotation.y << " | qz: " << transform_stamped_.transform.rotation.z << " | qw: " << transform_stamped_.transform.rotation.w << " ]");
		return true;
	}

	ROS_WARN("Dropped pose with NaN numbers!");
	return false;
}


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </tf update functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <pose to tf functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void PoseToTFPublisher::publishTFFromMapToOdomPose(double x, double y, double z, double roll, double pitch, double yaw) {
	tf2::Quaternion orientation;
	orientation.setRPY(roll, pitch, yaw);

	tf2::Transform transform_map_to_odom = tf2::Transform(orientation, tf2::Vector3(x, y, z));

	if (invert_tf_transform_) {
		transform_map_to_odom = transform_map_to_odom.inverse();
	}

	if (updateTFMessage(transform_map_to_odom)) {
		last_pose_time_ = ros::Time::now();
		sendTF(false);

		ROS_INFO_STREAM("Published global pose from map to odom estimate [ x: " << x << ", y: " << y << ", z: " << z
				<< " | r: " << roll << ", p: " << pitch << ", y: " << yaw
				<< " | qx: " << orientation.x() << ", qy: " << orientation.y() << ", qz: " << orientation.z() << ", qw: " << orientation.w() << " ]");
	}
}


void PoseToTFPublisher::publishTFFromMapToBasePose(double x, double y, double z, double roll, double pitch, double yaw) {
	tf2::Quaternion orientation;
	orientation.setRPY(roll, pitch, yaw);
	tf2::Transform transform(orientation, tf2::Vector3(x, y, z));

	if (updateTFMessage(transform)) {
		last_pose_time_ = ros::Time::now();

		if (publishTF(transform, ros::Time::now(), ros::Duration(5)), false) {
			ROS_INFO_STREAM("Published global pose from map to base estimate [ x: " << x << ", y: " << y << ", z: " << z \
					<< " | r: " << roll << ", p: " << pitch << ", y: " << yaw \
					<< " | qx: " << orientation.x() << ", qy: " << orientation.y() << ", qz: " << orientation.z() << ", qw: " << orientation.w() << " ]");
		} else {
			ros::Time end_time = ros::Time::now() + ros::Duration(10);
			ros::Duration wait_duration(0.005);

			while (ros::Time::now() < end_time) {
				if (publishTF(transform, ros::Time::now(), tf_lookup_timeout_, false)) {
					ROS_INFO_STREAM("Published global pose from map to base estimate [ x: " << x << ", y: " << y << ", z: " << z \
							<< " || r: " << roll << ", p: " << pitch << ", y: " << yaw \
							<< " || qx: " << orientation.x() << ", qy: " << orientation.y() << ", qz: " << orientation.z() << ", qw: " << orientation.w() << " ]");
					return;
				}
				wait_duration.sleep();
			}

			ROS_WARN_STREAM("Failed to find TF between " << odom_frame_id_ << " and " << base_link_frame_id_ << " when setting initial pose");
			publishTFFromMapToOdomPose(x, y, z, roll, pitch, yaw);
		}
	}
}


bool PoseToTFPublisher::publishTF(const tf2::Transform& transform_base_link_to_map, ros::Time tf_time, ros::Duration tf_timeout, bool check_pose_timeout) {
	tf2::Transform transform = transform_base_link_to_map;

	if (!base_link_frame_id_.empty() && !odom_frame_id_.empty()) {
		tf2::Transform transform_odom_to_base_link;
		if (!tf_collector_.lookForTransform(transform_odom_to_base_link, base_link_frame_id_, odom_frame_id_, tf_time, tf_timeout)) {
			ROS_WARN_STREAM("Dropping new pose with time [" << tf_time << "] because there isn't tf between [" << odom_frame_id_ << "] and [" << base_link_frame_id_ << "]");
			return false;
		}

		// base_to_map = base_to_odom * odom_to_map
		// odom_to_map = base_to_map * odom_to_base)
		transform = transform_base_link_to_map * transform_odom_to_base_link;
	}

	if (invert_tf_transform_) {
		transform = transform.inverse();
	}

	if (updateTFMessage(transform)) {
		last_pose_time_ = tf_time;
		return sendTF(check_pose_timeout);
	} else {
		return false;
	}
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </pose to tf functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

} /* namespace pose_to_tf_publisher */
