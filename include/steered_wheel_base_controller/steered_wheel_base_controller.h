#ifndef _SWBC_SWBC_H_
#define _SWBC_SWBC_H_

#include <algorithm>
#include <exception>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>

#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>

#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>

#include <urdf/model.h>

#include "steered_wheel_base_controller/joint_base.h"
#include "steered_wheel_base_controller/joint_types/pos_joint.h"
#include "steered_wheel_base_controller/joint_types/vel_joint.h"
#include "steered_wheel_base_controller/joint_types/pid_joint.h"
#include "steered_wheel_base_controller/wheel.h"
#include "steered_wheel_base_controller/util.h"

namespace SWBC
{
	// Steered-wheel base controller
	class SteeredWheelBaseController : public controller_interface::MultiInterfaceController<
																hardware_interface::EffortJointInterface,
																hardware_interface::PositionJointInterface,
																hardware_interface::VelocityJointInterface>
	{
	public:
	  SteeredWheelBaseController();

	  virtual std::string getHardwareInterfaceType() const;

	  // These are real-time safe.
	  virtual void starting(const ros::Time& time);
	  virtual void update(const ros::Time& time, const ros::Duration& period);
	  virtual void stopping(const ros::Time& time);

	private:
	  struct VelCmd     // Velocity command
	  {
		double x_vel;   // X velocity component. Unit: m/s.
		double y_vel;   // Y velocity component. Unit: m/s.
		double yaw_vel; // Yaw velocity. Unit: rad/s.

		// last_vel_cmd_time is the time at which the most recent velocity command
		// was received.
		ros::Time last_vel_cmd_time;
	  };

	  static const std::string DEF_ROBOT_DESC_NAME;
	  static const std::string DEF_BASE_LINK;
	  static const double DEF_CMD_VEL_TIMEOUT;

	  static const double DEF_LIN_SPEED_LIMIT;
	  static const double DEF_LIN_ACCEL_LIMIT;
	  static const double DEF_LIN_DECEL_LIMIT;

	  static const double DEF_YAW_SPEED_LIMIT;
	  static const double DEF_YAW_ACCEL_LIMIT;
	  static const double DEF_YAW_DECEL_LIMIT;

	  static const double DEF_FULL_AXLE_SPEED_ANG;
	  static const double DEF_ZERO_AXLE_SPEED_ANG;

	  static const double DEF_WHEEL_DIA_SCALE;

	  static const double DEF_ODOM_PUB_FREQ;
	  static const bool DEF_PUB_ODOM_TO_BASE;
	  static const std::string DEF_ODOM_FRAME;
	  static const std::string DEF_BASE_FRAME;
	  static const double DEF_INIT_X;
	  static const double DEF_INIT_Y;
	  static const double DEF_INIT_YAW;
	  static const double DEF_SD;

	  static const Eigen::Vector2d X_DIR;

	  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle& ctrlr_nh);
	  void velCmdCB(const geometry_msgs::TwistConstPtr& vel_cmd);

	  Eigen::Vector2d enforceLinLimits(const Eigen::Vector2d& desired_vel,
								const double delta_t, const double inv_delta_t);
	  double enforceYawLimits(const double desired_vel,
							  const double delta_t, const double inv_delta_t);
	  void ctrlWheels(const Eigen::Vector2d& lin_vel, const double yaw_vel,
					  const ros::Duration& period);
	  void compOdometry(const ros::Time& time, const double inv_delta_t);

	  std::vector<Wheel> wheels_;

	  // Linear motion limits
	  bool has_lin_speed_limit_;
	  double lin_speed_limit_;
	  bool has_lin_accel_limit_;
	  double lin_accel_limit_;
	  bool has_lin_decel_limit_;
	  double lin_decel_limit_;

	  // Yaw limits
	  bool has_yaw_speed_limit_;
	  double yaw_speed_limit_;
	  bool has_yaw_accel_limit_;
	  double yaw_accel_limit_;
	  bool has_yaw_decel_limit_;
	  double yaw_decel_limit_;

	  double hermite_scale_, hermite_offset_;

	  Eigen::Vector2d last_lin_vel_; // Last linear velocity. Unit: m/s.
	  double last_yaw_vel_;   // Last yaw velocity. Unit: rad/s.

	  // Velocity command member variables
	  VelCmd vel_cmd_;
	  realtime_tools::RealtimeBuffer<VelCmd> vel_cmd_buf_;
	  bool vel_cmd_timeout_enabled_;
	  ros::Duration vel_cmd_timeout_;
	  ros::Subscriber vel_cmd_sub_;

	  // Odometry
	  bool comp_odom_;              // Compute odometry
	  bool pub_odom_to_base_;       // Publish the odometry to base frame transform
	  ros::Duration odom_pub_period_;    // Odometry publishing period
	  Eigen::Affine2d init_odom_to_base_;  // Initial odometry to base frame transform
	  Eigen::Affine2d odom_to_base_;       // Odometry to base frame transform
	  Eigen::Affine2d odom_rigid_transf_;
	  // wheel_pos_ contains the positions of the wheel's steering axles.
	  // The positions are relative to the centroid of the steering axle positions
	  // in the base link's frame. neg_wheel_centroid is the negative version of
	  // that centroid.
	  Eigen::Matrix2Xd wheel_pos_;
	  Eigen::Vector2d neg_wheel_centroid_;
	  Eigen::MatrixX2d new_wheel_pos_;
	  realtime_tools::RealtimePublisher<nav_msgs::Odometry> odom_pub_;
	  realtime_tools::RealtimePublisher<tf::tfMessage> odom_tf_pub_;
	  ros::Time last_odom_pub_time_, last_odom_tf_pub_time_;
	};
	
	const std::string SteeredWheelBaseController::DEF_ROBOT_DESC_NAME = "robot_description";
	const std::string SteeredWheelBaseController::DEF_BASE_LINK = "base_link";
	const double SteeredWheelBaseController::DEF_CMD_VEL_TIMEOUT = 0.5;

	const double SteeredWheelBaseController::DEF_LIN_SPEED_LIMIT = 1;
	const double SteeredWheelBaseController::DEF_LIN_ACCEL_LIMIT = 1;
	const double SteeredWheelBaseController::DEF_LIN_DECEL_LIMIT = -1;

	const double SteeredWheelBaseController::DEF_YAW_SPEED_LIMIT = 1;
	const double SteeredWheelBaseController::DEF_YAW_ACCEL_LIMIT = 1;
	const double SteeredWheelBaseController::DEF_YAW_DECEL_LIMIT = -1;

	const double SteeredWheelBaseController::DEF_FULL_AXLE_SPEED_ANG = 0.7854;
	const double SteeredWheelBaseController::DEF_ZERO_AXLE_SPEED_ANG = 1.5708;

	const double SteeredWheelBaseController::DEF_WHEEL_DIA_SCALE = 1;

	const double SteeredWheelBaseController::DEF_ODOM_PUB_FREQ = 30;
	const bool SteeredWheelBaseController::DEF_PUB_ODOM_TO_BASE = true;
	const std::string SteeredWheelBaseController::DEF_ODOM_FRAME = "odom";
	const std::string SteeredWheelBaseController::DEF_BASE_FRAME = "base_link";
	const double SteeredWheelBaseController::DEF_INIT_X = 0;
	const double SteeredWheelBaseController::DEF_INIT_Y = 0;
	const double SteeredWheelBaseController::DEF_INIT_YAW = 0;
	const double SteeredWheelBaseController::DEF_SD = 0;

	// X direction
	const Eigen::Vector2d SteeredWheelBaseController::X_DIR = Eigen::Vector2d::UnitX();
}

#endif
