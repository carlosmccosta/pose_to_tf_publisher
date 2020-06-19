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
PoseToTFPublisher::PoseToTFPublisher(ros::Duration tf_buffer_duration) :
		float_update_field_(RotationYaw),
		float_update_field_orientation_in_degrees_(0.0f),
		publish_rate_(100),
		update_transform_timestamp_when_republishing_tf_(true),
		publish_last_pose_tf_timeout_seconds_(-1.0),
		tf_time_offset_(0.0),
		last_pose_time_(0),
		last_pose_arrival_time_(0),
		last_pose_time_valid_(false),
		invert_tf_transform_(false),
		invert_tf_hierarchy_(false),
		transform_pose_to_map_frame_id_(true),
		tf_collector_(tf_buffer_duration),
		number_tfs_published_(0) {
}

PoseToTFPublisher::~PoseToTFPublisher() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <ros integration functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void PoseToTFPublisher::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	node_handle_ = node_handle;
	private_node_handle_ = private_node_handle;
	configuration_namespace_ = configuration_namespace;

	private_node_handle_->param(configuration_namespace_ + "publish_rate", publish_rate_, 100.0);
	private_node_handle_->param(configuration_namespace_ + "update_transform_timestamp_when_republishing_tf", update_transform_timestamp_when_republishing_tf_, true);
	private_node_handle_->param(configuration_namespace_ + "publish_last_pose_tf_timeout_seconds", publish_last_pose_tf_timeout_seconds_, -1.0);
	double tf_time_offset = 0.0;
	private_node_handle_->param(configuration_namespace_ + "tf_time_offset", tf_time_offset, 0.0);
	tf_time_offset_ = ros::Duration(tf_time_offset);
	double tf_lookup_timeout;
	private_node_handle_->param(configuration_namespace_ + "tf_lookup_timeout", tf_lookup_timeout, 0.5);
	tf_lookup_timeout_.fromSec(tf_lookup_timeout);

	private_node_handle_->param(configuration_namespace_ + "pose_stamped_topic", pose_stamped_topic_, std::string(""));
	private_node_handle_->param(configuration_namespace_ + "pose_with_covariance_stamped_topic", pose_with_covariance_stamped_topic_, std::string("/initialpose"));
	private_node_handle_->param(configuration_namespace_ + "odometry_topic", odometry_topic_, std::string(""));
	private_node_handle_->param(configuration_namespace_ + "tf_topic", tf_topic_, std::string("/tf"));
	private_node_handle_->param(configuration_namespace_ + "float_topic", float_topic_, std::string(""));
	private_node_handle_->param(configuration_namespace_ + "float_update_field_orientation_in_degrees", float_update_field_orientation_in_degrees_, false);
	std::string float_update_field;
	private_node_handle_->param(configuration_namespace_ + "float_update_field", float_update_field, std::string("RotationPitch"));

	if (float_update_field == "TranslationX") {
		float_update_field_ = TranslationX;
	} else if (float_update_field == "TranslationY") {
		float_update_field_ = TranslationY;
	} else if (float_update_field == "TranslationZ") {
		float_update_field_ = TranslationZ;
	} else if (float_update_field == "RotationYaw") {
		float_update_field_ = RotationYaw;
	} else if (float_update_field == "RotationPitch") {
		float_update_field_ = RotationPitch;
	} else if (float_update_field == "RotationRoll") {
		float_update_field_ = RotationRoll;
	} else if (float_update_field == "RotationQuaternionX") {
		float_update_field_ = RotationQuaternionX;
	} else if (float_update_field == "RotationQuaternionY") {
		float_update_field_ = RotationQuaternionY;
	} else if (float_update_field == "RotationQuaternionZ") {
		float_update_field_ = RotationQuaternionZ;
	} else if (float_update_field == "RotationQuaternionW") {
		float_update_field_ = RotationQuaternionW;
	}

	private_node_handle_->param(configuration_namespace_ + "poses_filename", poses_filename_, std::string(""));

	private_node_handle_->param(configuration_namespace_ + "map_frame_id", map_frame_id_, std::string("map"));
	private_node_handle_->param(configuration_namespace_ + "odom_frame_id", odom_frame_id_, std::string("odom"));
	private_node_handle_->param(configuration_namespace_ + "base_link_frame_id", base_link_frame_id_, std::string("base_link"));
	private_node_handle_->param(configuration_namespace_ + "transform_tf_message_source", transform_tf_message_source_, std::string(""));
	private_node_handle_->param(configuration_namespace_ + "transform_tf_message_target", transform_tf_message_target_, std::string(""));

	private_node_handle_->param(configuration_namespace_ + "invert_tf_transform", invert_tf_transform_, false);
	private_node_handle_->param(configuration_namespace_ + "invert_tf_hierarchy", invert_tf_hierarchy_, false);
	private_node_handle_->param(configuration_namespace_ + "transform_pose_to_map_frame_id", transform_pose_to_map_frame_id_, true);
	private_node_handle_->param(configuration_namespace_ + "discard_older_poses", discard_older_poses_, true);

	updateTFMessageFrames();
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
	bool initial_pose_in_base_to_map;
	bool publish_initial_pose;
	private_node_handle_->param(configuration_namespace_ + "publish_initial_pose", publish_initial_pose, true);

	if (publish_initial_pose) {
		private_node_handle_->param(configuration_namespace_ + "initial_pose_in_base_to_map", initial_pose_in_base_to_map, true);
		private_node_handle_->param(configuration_namespace_ + "initial_x", x, 0.0);
		private_node_handle_->param(configuration_namespace_ + "initial_y", y, 0.0);
		private_node_handle_->param(configuration_namespace_ + "initial_z", z, 0.0);
		private_node_handle_->param(configuration_namespace_ + "initial_roll", roll, 0.0);
		private_node_handle_->param(configuration_namespace_ + "initial_pitch", pitch, 0.0);
		private_node_handle_->param(configuration_namespace_ + "initial_yaw", yaw, 0.0);

		if (initial_pose_in_base_to_map) {
			publishTFFromBaseToMapPose(x, y, z, roll, pitch, yaw);
		} else {
			publishTFFromOdomToMapPose(x, y, z, roll, pitch, yaw);
		}
	}
}


void PoseToTFPublisher::startPublishingTF() {
	if (transform_stamped_.header.frame_id.empty() || transform_stamped_.child_frame_id.empty()) {
		ROS_ERROR_STREAM("Source and target frames can't be empty [ map_frame_id: " << map_frame_id_ << " | base_link_frame_id: " << base_link_frame_id_ << " ]");
		return;
	}

	ros::Time::waitForValid();

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
		pose_with_covariance_stamped_subscriber_ = node_handle_->subscribe(pose_with_covariance_stamped_topic_, 5, &pose_to_tf_publisher::PoseToTFPublisher::publishTFFromPoseWithCovarianceStamped, this);
	}

	if (!odometry_topic_.empty()) {
		ss << " " << odometry_topic_;
		odometry_subscriber_ = node_handle_->subscribe(odometry_topic_, 5, &pose_to_tf_publisher::PoseToTFPublisher::publishTFFromOdometry, this);
	}

	if (!float_topic_.empty()) {
		ss << " " << float_topic_;
		float_subscriber_ = node_handle_->subscribe(float_topic_, 5, &pose_to_tf_publisher::PoseToTFPublisher::publishTFFromFloat, this);
	}

	if (!tf_topic_.empty() && !transform_tf_message_source_.empty() && !transform_tf_message_target_.empty()) {
		ss << " " << tf_topic_;
		tf_subscriber_ = node_handle_->subscribe(tf_topic_, 5, &pose_to_tf_publisher::PoseToTFPublisher::publishTFFromTF, this);
	}

	ROS_INFO_STREAM("Publishing tf [ " << transform_stamped_.header.frame_id << " -> " << transform_stamped_.child_frame_id << " ] from pose topics [" << ss.str() << " ]");

	if (publish_rate_ > 0) {
		ros::Rate publish_rate(publish_rate_);
		while (ros::ok()) {
			if (update_transform_timestamp_when_republishing_tf_) transform_stamped_.header.stamp = ros::Time::now();
			sendTF();
			publish_rate.sleep();
			ros::spinOnce();
		}
	} else {
		ros::spin();
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

	if (!float_topic_.empty()) {
		float_subscriber_.shutdown();
	}
}


void PoseToTFPublisher::publishTFFromPose(const geometry_msgs::Pose& pose, const std::string& frame_id, const ros::Time& pose_time) {
	ros::Time pose_time_updated = pose_time;
	if (pose_time.sec == 0 && pose_time.nsec == 0) { // time in the future to override any poses coming from the localization node
		pose_time_updated = ros::Time::now() + ros::Duration(0.5);
		ROS_INFO("Reseting tf initial pose...");
	}

	if ((discard_older_poses_ && pose_time_updated < last_pose_time_) || frame_id.empty()) {
		ROS_WARN_STREAM("Dropping new pose with time [" << pose_time_updated << "] and frame_id [" << frame_id << "]");
		return;
	}

	tf2::Transform transform_pose(
			tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w).normalize(),
			tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));

	if (transform_pose_to_map_frame_id_ && frame_id != map_frame_id_) {
		// transform to map (global frame reference)
		ROS_WARN_STREAM("Pose received in " << frame_id << " frame instead of " << map_frame_id_);
		tf2::Transform transform_pose_to_map;
		if (!tf_collector_.lookForTransform(transform_pose_to_map, map_frame_id_, frame_id, pose_time_updated, tf_lookup_timeout_)) {
			ROS_WARN_STREAM("Dropping new pose with time [" << pose_time_updated << "] because there isn't tf between [" << map_frame_id_ << "] and [" << frame_id << "]");
			return;
		}

		transform_pose = transform_pose_to_map * transform_pose;
	}

	publishTF(transform_pose, pose_time_updated, pose_time_updated, tf_lookup_timeout_);
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


void PoseToTFPublisher::publishTFFromTF(const tf2_msgs::TFMessageConstPtr tf_message) {
	for (size_t i = 0; i < tf_message->transforms.size(); ++i) {
		if (tf_message->transforms[i].child_frame_id == transform_tf_message_source_ && tf_message->transforms[i].header.frame_id == transform_tf_message_target_) {
			if (!odom_frame_id_.empty() || invert_tf_transform_) {
				tf2::Transform transform;
				laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformMsgToTF2(tf_message->transforms[i].transform, transform);
				publishTF(transform, tf_message->transforms[i].header.stamp, tf_message->transforms[i].header.stamp, tf_lookup_timeout_, false);
			} else {
				last_pose_time_ = tf_message->transforms[i].header.stamp;
				last_pose_arrival_time_ = ros::Time::now();
				last_pose_time_valid_ = true;
				transform_stamped_.header.stamp = tf_message->transforms[i].header.stamp + tf_time_offset_;
				transform_stamped_.transform.translation.x = tf_message->transforms[i].transform.translation.x;
				transform_stamped_.transform.translation.y = tf_message->transforms[i].transform.translation.y;
				transform_stamped_.transform.translation.z = tf_message->transforms[i].transform.translation.z;
				transform_stamped_.transform.rotation.x = tf_message->transforms[i].transform.rotation.x;
				transform_stamped_.transform.rotation.y = tf_message->transforms[i].transform.rotation.y;
				transform_stamped_.transform.rotation.z = tf_message->transforms[i].transform.rotation.z;
				transform_stamped_.transform.rotation.w = tf_message->transforms[i].transform.rotation.w;
				sendTF(false);
			}

			return;
		}
	}
}


void PoseToTFPublisher::publishTFFromFloat(const std_msgs::Float64ConstPtr& float64) {
	switch (float_update_field_) {
		case TranslationX: { transform_stamped_.transform.translation.x = float64->data; break; }
		case TranslationY: { transform_stamped_.transform.translation.y = float64->data; break; }
		case TranslationZ: { transform_stamped_.transform.translation.z = float64->data; break; }
		case RotationQuaternionX: { transform_stamped_.transform.rotation.x = float64->data; break; }
		case RotationQuaternionY: { transform_stamped_.transform.rotation.y = float64->data; break; }
		case RotationQuaternionZ: { transform_stamped_.transform.rotation.z = float64->data; break; }
		case RotationQuaternionW: { transform_stamped_.transform.rotation.w = float64->data; break; }
		default: { break; }
	}


	tf2::Quaternion new_orientation(transform_stamped_.transform.rotation.x, transform_stamped_.transform.rotation.y, transform_stamped_.transform.rotation.z, transform_stamped_.transform.rotation.w);
	new_orientation.normalize();

	if (float_update_field_ == RotationRoll || float_update_field_ == RotationPitch || float_update_field_ == RotationYaw) {
		double float_in_radians = float_update_field_orientation_in_degrees_ ? angles::from_degrees(float64->data) : float64->data;
		tf2::Matrix3x3 matrix(new_orientation);
		double roll, pitch, yaw;
		matrix.getRPY(roll, pitch, yaw);

		if(float_update_field_ == RotationRoll) {
			roll = float_in_radians;
		} else if(float_update_field_ == RotationPitch) {
			pitch = float_in_radians;
		} else if(float_update_field_ == RotationYaw) {
			yaw = float_in_radians;
		}

		new_orientation.setRPY(roll, pitch, yaw);
		new_orientation.normalize();
	}

	transform_stamped_.transform.rotation.x = new_orientation.getX();
	transform_stamped_.transform.rotation.y = new_orientation.getY();
	transform_stamped_.transform.rotation.z = new_orientation.getZ();
	transform_stamped_.transform.rotation.w = new_orientation.getW();

	transform_stamped_.header.stamp = ros::Time::now();
	sendTF(true);
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
	if (!last_pose_time_valid_) {
		return false;
	}

	ros::Time current_time = ros::Time::now();
	if (!check_pose_timeout || publish_last_pose_tf_timeout_seconds_ <= 0.0 || (current_time - last_pose_arrival_time_).toSec() <= publish_last_pose_tf_timeout_seconds_ || last_pose_arrival_time_.toSec() < 1.0) {
		transform_stamped_.header.seq = number_tfs_published_++;
//		transform_stamped_.header.stamp = current_time;
		transform_broadcaster_.sendTransform(transform_stamped_);
		return true;
	}

	ROS_WARN_STREAM_THROTTLE(1.0, "Pose to TF publisher reached timeout for last valid pose" \
			<< "\n\tTF translation -> [ x: " << transform_stamped_.transform.translation.x << " | y: " << transform_stamped_.transform.translation.y << " | z: " << transform_stamped_.transform.translation.z << " ]" \
			<< "\n\tTF orientation -> [ qx: " << transform_stamped_.transform.rotation.x << " | qy: " << transform_stamped_.transform.rotation.y << " | qz: " << transform_stamped_.transform.rotation.z << " | qw: " << transform_stamped_.transform.rotation.w << " ]" \
			<< "\n\tCurrent time: " << current_time \
			<< "\n\tLast pose time: " << last_pose_time_ \
			<< "\n\tLast pose arrival time: " << last_pose_arrival_time_);
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

		laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformTF2ToMsg(transform, transform_stamped_.transform);
		ROS_DEBUG_STREAM("Updating TF between " << transform_stamped_.header.frame_id << " and " << transform_stamped_.child_frame_id << " with time [" << transform_stamped_.header.stamp << "]" \
				<< "\tTF translation -> [ x: " << transform_stamped_.transform.translation.x << " | y: " << transform_stamped_.transform.translation.y << " | z: " << transform_stamped_.transform.translation.z << " ]" \
				<< "\tTF orientation -> [ qx: " << transform_stamped_.transform.rotation.x << " | qy: " << transform_stamped_.transform.rotation.y << " | qz: " << transform_stamped_.transform.rotation.z << " | qw: " << transform_stamped_.transform.rotation.w << " ]");
		return true;
	}

	ROS_WARN("Dropped pose with NaN numbers!");
	return false;
}

void PoseToTFPublisher::updateTFMessageFrames() {
    std::string child_frame = (odom_frame_id_.empty() ? base_link_frame_id_ : odom_frame_id_);
    transform_stamped_.header.frame_id = invert_tf_hierarchy_ ? child_frame : map_frame_id_;
    transform_stamped_.child_frame_id = invert_tf_hierarchy_ ? map_frame_id_ : child_frame;
}


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </tf update functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <pose to tf functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void PoseToTFPublisher::publishTFFromOdomToMapPose(double x, double y, double z, double roll, double pitch, double yaw) {
	tf2::Quaternion orientation;
	orientation.setRPY(roll, pitch, yaw);
	tf2::Transform transform_map_to_odom(orientation, tf2::Vector3(x, y, z));

	if (invert_tf_transform_) {
		transform_map_to_odom = transform_map_to_odom.inverse();
	}

	if (updateTFMessage(transform_map_to_odom)) {
		last_pose_time_ = ros::Time::now();
		last_pose_arrival_time_ = ros::Time::now();
		last_pose_time_valid_ = true;
		transform_stamped_.header.stamp = ros::Time::now();
		sendTF(false);

		ROS_INFO_STREAM("Published global pose from map to odom estimate [ x: " << x << " | y: " << y << " | z: " << z
				<< " || r: " << roll << " | p: " << pitch << " | y: " << yaw
				<< " || qx: " << orientation.x() << " | qy: " << orientation.y() << " | qz: " << orientation.z() << " | qw: " << orientation.w() << " ]");
	}
}


void PoseToTFPublisher::publishTFFromBaseToMapPose(double x, double y, double z, double roll, double pitch, double yaw) {
	tf2::Quaternion orientation;
	orientation.setRPY(roll, pitch, yaw);
	tf2::Transform transform(orientation, tf2::Vector3(x, y, z));

	if (updateTFMessage(transform)) {
		last_pose_time_ = ros::Time::now();
		last_pose_arrival_time_ = ros::Time::now();
		last_pose_time_valid_ = true;
		ros::Time end_time = ros::Time::now() + ros::Duration(10);
		ros::Duration wait_duration(0.005);

		while (ros::Time::now() < end_time) {
			if (publishTF(transform, ros::Time::now(), ros::Time(0.0), tf_lookup_timeout_, false)) {
				ROS_INFO_STREAM("Published global pose from map to base estimate [ x: " << x << " | y: " << y << " | z: " << z \
						<< " || r: " << roll << " | p: " << pitch << " | y: " << yaw \
						<< " || qx: " << orientation.x() << " | qy: " << orientation.y() << " | qz: " << orientation.z() << " | qw: " << orientation.w() << " ]");
				return;
			}
			wait_duration.sleep();
		}

		ROS_WARN_STREAM("Failed to find TF between " << odom_frame_id_ << " and " << base_link_frame_id_ << " when setting initial pose");
		publishTFFromOdomToMapPose(x, y, z, roll, pitch, yaw);
	}
}


bool PoseToTFPublisher::publishTF(const tf2::Transform& transform_base_link_to_map, ros::Time tf_time, ros::Time tf_odom_time, ros::Duration tf_timeout, bool check_pose_timeout) {
	tf2::Transform transform = transform_base_link_to_map;

	if (!base_link_frame_id_.empty() && !odom_frame_id_.empty() && !retrieveTFOdomToMap(transform_base_link_to_map, tf_odom_time, transform, tf_timeout)) {
		ROS_WARN_STREAM("Dropping new pose with time [" << tf_time << "] because there isn't tf between [" << odom_frame_id_ << "] and [" << base_link_frame_id_ << "]");
		return false;
	}

	if (invert_tf_transform_) {
		transform = transform.inverse();
	}

	if (updateTFMessage(transform)) {
		last_pose_time_ = tf_time;
		last_pose_arrival_time_ = ros::Time::now();
		last_pose_time_valid_ = true;
		transform_stamped_.header.stamp = tf_time + tf_time_offset_;
		return sendTF(check_pose_timeout);
	} else {
		return false;
	}
}


bool PoseToTFPublisher::retrieveTFOdomToMap(const tf2::Transform& transform_base_link_to_map, ros::Time tf_time, tf2::Transform& transform_odom_to_map_out, ros::Duration tf_timeout) {
	if (!base_link_frame_id_.empty() && !odom_frame_id_.empty()) {
		tf2::Transform transform_odom_to_base_link;
		if (tf_timeout.toSec() <= 0.01) { tf_timeout = tf_lookup_timeout_; }
		if (tf_collector_.lookForTransform(transform_odom_to_base_link, base_link_frame_id_, odom_frame_id_, tf_time, tf_timeout)) {
			transform_odom_to_map_out = transform_base_link_to_map * transform_odom_to_base_link;
			return true;
		}
	}

	return false;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </pose to tf functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

} /* namespace pose_to_tf_publisher */
