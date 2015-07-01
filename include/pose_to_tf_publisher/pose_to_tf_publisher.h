#pragma once

/**\file pose_to_tf_publisher.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <string>
#include <sstream>
#include <vector>
#include <fstream>

// ROS includes
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <angles/angles.h>


// external libs includes
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

// project includes
#include <laserscan_to_pointcloud/tf_collector.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace pose_to_tf_publisher {
// ##############################################################################   tf_publisher   #############################################################################
/**
 * \brief Description...
 */
class PoseToTFPublisher {
	// ========================================================================   <public-section>   ===========================================================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <typedefs>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		typedef boost::shared_ptr< PoseToTFPublisher > Ptr;
		typedef boost::shared_ptr< const PoseToTFPublisher > ConstPtr;
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </typedefs>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <enums>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		enum FloatUpdateField {
			TranslationX, TranslationY, TranslationZ,
			RotationRoll, RotationPitch, RotationYaw,
			RotationQuaternionX, RotationQuaternionY, RotationQuaternionZ, RotationQuaternionW
		};
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </enums>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constants>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constants>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		PoseToTFPublisher(ros::Duration tf_buffer_duration = ros::Duration(600.0));
		virtual ~PoseToTFPublisher();
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <ros integration functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		virtual void setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace = "");
		void publishInitialPoseFromParameterServer();
		void startPublishingTF();
		bool startPublishingTFFromFile(std::string poses_filename);
		bool getPoseFromFile(geometry_msgs::Pose& pose_out, double& timestamp_out, std::ifstream& input_stream);
		void startPublishingTFFromPoseTopics();
		void stopPublishingTFFromPoseTopics();

		void publishTFFromPose(const geometry_msgs::Pose& pose, const std::string& frame_id, const ros::Time& pose_time);
		void publishTFFromPoseStamped(const geometry_msgs::PoseStampedConstPtr& pose);
		void publishTFFromPoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);
		void publishTFFromOdometry(const nav_msgs::OdometryConstPtr& odom);
		void publishTFFromTF(const tf2_msgs::TFMessageConstPtr tf_message);
		void publishTFFromFloat(const std_msgs::Float64ConstPtr& float64);
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </ros integration functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <tf update functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		bool addOdometryDisplacementToTransform(tf2::Transform& transform, const ros::Time& time_of_transform, const ros::Time& target_time);
		bool sendTF(bool check_pose_timeout = true);
		bool updateTFMessage(tf2::Transform& transform);
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </tf update functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <pose to tf functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		void publishTFFromOdomToMapPose(double x, double y, double z = 0, double roll = 0, double pitch = 0, double yaw = 0);
		void publishTFFromBaseToMapPose(double x, double y, double z = 0, double roll = 0, double pitch = 0, double yaw = 0);
		bool publishTF(const tf2::Transform& transform_base_link_to_map, ros::Time tf_time = ros::Time::now(), ros::Time tf_odom_time = ros::Time::now(), ros::Duration tf_timeout = ros::Duration(0.1), bool check_pose_timeout = true);
		bool retrieveTFOdomToMap(const tf2::Transform& transform_base_link_to_map, ros::Time tf_time, tf2::Transform& transform_odom_to_map_out, ros::Duration tf_timeout = ros::Duration(0));
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </pose to tf functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <gets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		inline std::string& getBaseLinkFrameId() { return base_link_frame_id_; }
		inline std::string& getInitialPoseTopic() { return pose_stamped_topic_; }
		inline std::string& getInitialPoseWithCovarianceStampedTopic() { return pose_with_covariance_stamped_topic_; }
		inline std::string& getMapFrameId() { return map_frame_id_; }
		inline std::string& getOdomFrameId() { return odom_frame_id_; }
		inline double getPublishRate() const { return publish_rate_; }
		laserscan_to_pointcloud::TFCollector& getTfCollector() { return tf_collector_; }
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </gets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <sets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		inline void setPublishRate(double publish_rate) { publish_rate_ = publish_rate; }
		inline void setBaseLinkFrameId(const std::string& base_link_frame_id) { base_link_frame_id_ = base_link_frame_id; }
		inline void setOdomFrameId(const std::string& odom_frame_id) { odom_frame_id_ = odom_frame_id; }
		inline void setMapFrameId(const std::string& map_frame_id) { map_frame_id_ = map_frame_id; }
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </sets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// ========================================================================   </public-section>  ===========================================================================

	// ========================================================================   <protected-section>   ========================================================================
	protected:
	// ========================================================================   </protected-section>  ========================================================================

	// ========================================================================   <private-section>   ==========================================================================
	private:
		// configuration fields
		std::string pose_stamped_topic_;
		std::string pose_with_covariance_stamped_topic_;
		std::string odometry_topic_;
		std::string tf_topic_;
		std::string float_topic_;
		FloatUpdateField float_update_field_;
		bool float_update_field_orientation_in_degrees_;

		std::string poses_filename_;

		double publish_rate_;
		double publish_last_pose_tf_timeout_seconds_;
		ros::Duration tf_time_offset_;
		ros::Duration tf_lookup_timeout_;
		ros::Time last_pose_time_;
		ros::Time last_pose_arrival_time_;
		bool last_pose_time_valid_;
		bool invert_tf_transform_;
		bool invert_tf_hierarchy_;
		bool transform_pose_to_map_frame_id_;

		// frame reference ids
		std::string map_frame_id_;
		std::string odom_frame_id_;
		std::string base_link_frame_id_;
		std::string transform_tf_message_source_;
		std::string transform_tf_message_target_;

		// state fields
		laserscan_to_pointcloud::TFCollector tf_collector_;
		size_t number_tfs_published_;
		geometry_msgs::TransformStamped transform_stamped_;

		// ros communication fields
		ros::NodeHandlePtr node_handle_;
		ros::NodeHandlePtr private_node_handle_;
		ros::Subscriber pose_stamped_subscriber_;
		ros::Subscriber pose_with_covariance_stamped_subscriber_;
		ros::Subscriber odometry_subscriber_;
		ros::Subscriber float_subscriber_;
		ros::Subscriber tf_subscriber_;
		tf2_ros::TransformBroadcaster transform_broadcaster_;
	// ========================================================================   </private-section>  ==========================================================================
};

} /* namespace pose_to_tf_publisher */
