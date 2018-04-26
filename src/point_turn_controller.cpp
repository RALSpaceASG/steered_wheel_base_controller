/// \file point_turn_controller.cpp
///
/// \brief Steered-wheel base controller
///
/// This file contains the source code for PointTurnController,
/// a base controller for mobile robots. It works with bases that have two or
/// more independently-steerable driven wheels and zero or more omnidirectional
/// passive wheels (e.g. swivel casters).
///
/// Subscribed Topics:
///     cmd_vel (geometry_msgs/Twist)
///         Velocity command, defined in the frame specified by the base_link
///         parameter.
///         The angular.z field specifies the base's desired angular velocity,
///         measured in radians per second.
///
/// Published Topics:
///     odom (nav_msgs/Odometry)
///         Odometry.
///
/// Parameters:
///     ~robot_description_name (string, default: robot_description)
///         Name of a parameter on the Parameter Server. The named parameter's
///         value is URDF data that describes the robot.
///     ~base_link (string, default: base_link)
///         Link that specifies the frame in which cmd_vel is defined.
///         The link specified by base_link must exist in the robot's URDF
///         data.
///     ~cmd_vel_timeout (float, default: 0.5)
///         If cmd_vel_timeout is greater than zero and this controller does
///         not receive a velocity command for more than cmd_vel_timeout
///         seconds, wheel motion is paused until a command is received.
///         If cmd_vel_timeout is less than or equal to zero, the command
///         timeout is disabled.
///
///     ~yaw_speed_limit (float, default: 1.0)
///         Yaw speed limit. If yaw_speed_limit is less than zero, the yaw
///         speed limit is disabled. Unit: rad/s.
///     ~yaw_acceleration_limit (float, default: 1.0)
///         Yaw acceleration limit. If yaw_acceleration_limit is less than
///         zero, the yaw acceleration limit is disabled. Unit: rad/s**2.
///     ~yaw_deceleration_limit (float, default: -1.0)
///         Yaw deceleration limit. If yaw_deceleration_limit is less than or
///         equal to zero, the yaw deceleration limit is disabled.
///         Unit: rad/s**2.
///
///     ~full_axle_speed_angle (float, default: 0.7854)
///         If the difference between a wheel's desired and measured steering
///         angles is less than or equal to full_axle_speed_angle, the wheel's
///         axle will rotate at the speed determined by the current velocity
///         command, subject to the speed, acceleration, and deceleration
///         limits. full_axle_speed_angle must be less than
///         zero_axle_speed_angle. Range: [0, pi]. Unit: radian.
///     ~zero_axle_speed_angle (float, default: 1.5708)
///         If the difference between a wheel's desired and measured steering
///         angles is greater than or equal to zero_axle_speed_angle, the
///         wheel's axle will stop rotating, subject to the deceleration
///         limits. zero_axle_speed_angle must be greater than
///         full_axle_speed_angle. Range: [0, pi]. Unit: radian.
///
///     ~wheels (sequence of mappings, default: empty)
///         Two or more steered wheels.
///
///         Key-Value Pairs:
///
///         steering_joint (string)
///             Steering joint.
///         axle_joint (string)
///             Axle joint.
///         diameter (float)
///             Wheel diameter. It must be greater than zero. Unit: meter.
///     ~wheel_diameter_scale (float, default: 1.0)
///         Scale applied to each wheel's diameter. It is used to correct for
///         tire deformation. wheel_diameter_scale must be greater than zero.
///     ~pid_gains/<joint name> (mapping, default: empty)
///         PID controller gains for the specified joint. Needed only for
///         effort-controlled joints and velocity-controlled steering joints.
///
///     ~odometry_publishing_frequency (float, default: 30.0)
///         Odometry publishing frequency. If it is less than or equal to zero,
///         odometry computation is disabled.  Unit: hertz.
///     ~publish_odometry_to_base_transform (bool, default: true)
///         If "true," publish the transform from <odometry_frame> to
///         <base_frame>. The transform is published to /tf.
///     ~odometry_frame (string, default: odom)
///         Odometry frame.
///     ~base_frame (string, default: base_link)
///         Base frame in the <odometry_frame>-to-<base_frame> transform
///         provided by this controller. base_frame allows the controller to
///         publish transforms from odometry_frame to a frame that is not a
///         link in the robot's URDF data. For example, base_frame can be set
///         to "base_footprint". This controller computes coordinates in
///         <base_link> and they are not transformed into <base_frame>, so the
///         transform from <base_link> to <base_frame> should consist a
///         translation along the z axis only. The x and y translation values
///         and all the rotation values should be zero.
///     ~initial_x (float, default: 0.0)
///         X coordinate of the base frame's initial position in the odometry
///         frame. Unit: meter.
///     ~initial_y (float, default: 0.0)
///         Y coordinate of the base frame's initial position in the odometry
///         frame. Unit: meter.
///     ~initial_yaw (float, default: 0.0)
///         Initial orientation of the base frame in the odometry frame.
///         Range: [-pi, pi]. Unit: radian.
///     ~x_sd (float, default: 0.0)
///         Standard deviation of the x coordinate of the base frame's position
///         in the odometry frame. x_sd must be greater than or equal to zero.
///         Unit: meter.
///     ~y_sd (float, default: 0.0)
///         Standard deviation of the y coordinate of the base frame's position
///         in the odometry frame. y_sd must be greater than or equal to zero.
///         Unit: meter.
///     ~yaw_sd (float, default: 0.0)
///         Standard deviation of the yaw angle of the base frame's
///         orientation in the odometry frame. yaw_sd must be greater than or
///         equal to zero. Unit: radian.
///     ~x_speed_sd (float, default: 0.0)
///         Standard deviation of the base's x speed in the base frame.
///         x_speed_sd must be greater than or equal to zero. Unit: m/s.
///     ~y_speed_sd (float, default: 0.0)
///         Standard deviation of the base's y speed in the base frame.
///         y_speed_sd must be greater than or equal to zero. Unit: m/s.
///     ~yaw_speed_sd (float, default: 0.0)
///         Standard deviation of the base's yaw speed in the base frame.
///         yaw_speed_sd must be greater than or equal to zero. Unit: rad/s.
///
/// Provided tf Transforms:
///     <odometry_frame> to <base_frame>
///         Specifies the base frame's pose in the odometry frame.
///         This transform is provided only if odometry computation is enabled
///         and publish_odometry_to_base_transform is "true."
//
// Copyright (c) 2013-2015 Wunderkammer Laboratory
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "steered_wheel_base_controller/point_turn_controller.h"

using SWBC::joint_types::JointBase;
using SWBC::joint_types::PosJoint;
using SWBC::joint_types::VelJoint;
using SWBC::joint_types::PIDJoint;
using SWBC::Wheel;
using namespace SWBC::util;

using std::runtime_error;
using std::set;
using std::string;

using boost::math::sign;
using std::shared_ptr;

using Eigen::Affine2d;
using Eigen::Matrix2d;
using Eigen::Vector2d;

using geometry_msgs::TwistConstPtr;

using hardware_interface::RobotHW;

using hardware_interface::EffortJointInterface;
using hardware_interface::PositionJointInterface;
using hardware_interface::VelocityJointInterface;

using realtime_tools::RealtimeBuffer;
using realtime_tools::RealtimePublisher;

using ros::Duration;
using ros::NodeHandle;
using ros::Time;

using XmlRpc::XmlRpcValue;

namespace SWBC
{
	PointTurnController::PointTurnController()
	{
		state_ = CONSTRUCTED;
	}

	string PointTurnController::getHardwareInterfaceType() const
	{
	  return "";
	}

	void PointTurnController::starting(const Time& time)
	{
	  for (auto &wheel : wheels_)
		wheel.initJoints();

	  last_lin_vel_ = Vector2d(0, 0);
	  last_yaw_vel_ = 0;

	  if (comp_odom_)
	  {
		last_odom_pub_time_ = time;
		last_odom_tf_pub_time_ = time;
	  }

	  vel_cmd_.x_vel = 0;
	  vel_cmd_.y_vel = 0;
	  vel_cmd_.yaw_vel = 0;
	  vel_cmd_.last_vel_cmd_time = time;
	  vel_cmd_buf_.initRT(vel_cmd_);
	}

	void PointTurnController::update(const Time& time,
											const Duration& period)
	{
	  const double delta_t = period.toSec();
	  if (delta_t <= 0)
		return;

	  vel_cmd_ = *(vel_cmd_buf_.readFromRT());
	  Vector2d desired_lin_vel;
	  double desired_yaw_vel;
	  if (!vel_cmd_timeout_enabled_ ||
		  time - vel_cmd_.last_vel_cmd_time <= vel_cmd_timeout_)
	  {
		desired_lin_vel = Vector2d(vel_cmd_.x_vel, vel_cmd_.y_vel);
		desired_yaw_vel = vel_cmd_.yaw_vel;
	  }
	  else
	  {
		// Too much time has elapsed since the last velocity command was received.
		// Stop the robot
		desired_lin_vel.setZero();
		desired_yaw_vel = 0;
	  }
	  const double inv_delta_t = 1 / delta_t;

	  const Vector2d lin_vel = enforceLinLimits(desired_lin_vel,
												delta_t, inv_delta_t);
	  const double yaw_vel = enforceYawLimits(desired_yaw_vel,
											  delta_t, inv_delta_t);
	  ctrlWheels(lin_vel, yaw_vel, period);
	  if (comp_odom_)
		compOdometry(time, inv_delta_t);
	}

	void PointTurnController::stopping(const Time& /*time*/)
	{
		for (auto &wheel : wheels_)
			wheel.stop();
	}

	// Initialize this steered-wheel base controller.
	bool PointTurnController:: init(RobotHW *robot_hw, NodeHandle& ctrlr_nh)
	{
	  EffortJointInterface   *const eff_joint_iface = robot_hw->get<EffortJointInterface>();
	  PositionJointInterface *const pos_joint_iface = robot_hw->get<PositionJointInterface>();
	  VelocityJointInterface *const vel_joint_iface = robot_hw->get<VelocityJointInterface>();

	  string robot_desc_name;
	  ctrlr_nh.param("robot_description_name", robot_desc_name,
					 DEF_ROBOT_DESC_NAME);
	  urdf::Model urdf_model;
	  if (!urdf_model.initParam(robot_desc_name))
		throw runtime_error("The URDF data was not found.");
	  KDL::Tree model_tree;
	  if (!kdl_parser::treeFromUrdfModel(urdf_model, model_tree))
		throw runtime_error("The kinematic tree could not be created.");

	  string base_link;
	  ctrlr_nh.param("base_link", base_link, DEF_BASE_LINK);
	  double timeout;
	  ctrlr_nh.param("cmd_vel_timeout", timeout, DEF_CMD_VEL_TIMEOUT);
	  vel_cmd_timeout_enabled_ = timeout > 0;
	  if (vel_cmd_timeout_enabled_)
		vel_cmd_timeout_.fromSec(timeout);

	  ctrlr_nh.param("linear_speed_limit", lin_speed_limit_, DEF_LIN_SPEED_LIMIT);
	  has_lin_speed_limit_ = lin_speed_limit_ >= 0;
	  ctrlr_nh.param("linear_acceleration_limit", lin_accel_limit_,
					 DEF_LIN_ACCEL_LIMIT);
	  has_lin_accel_limit_ = lin_accel_limit_ >= 0;
	  // For safety, a valid deceleration limit must be greater than zero.
	  ctrlr_nh.param("linear_deceleration_limit", lin_decel_limit_,
					 DEF_LIN_DECEL_LIMIT);
	  has_lin_decel_limit_ = lin_decel_limit_ > 0;

	  ctrlr_nh.param("yaw_speed_limit", yaw_speed_limit_, DEF_YAW_SPEED_LIMIT);
	  has_yaw_speed_limit_ = yaw_speed_limit_ >= 0;
	  ctrlr_nh.param("yaw_acceleration_limit", yaw_accel_limit_,
					 DEF_YAW_ACCEL_LIMIT);
	  has_yaw_accel_limit_ = yaw_accel_limit_ >= 0;
	  // For safety, a valid deceleration limit must be greater than zero.
	  ctrlr_nh.param("yaw_deceleration_limit", yaw_decel_limit_,
					 DEF_YAW_DECEL_LIMIT);
	  has_yaw_decel_limit_ = yaw_decel_limit_ > 0;

	  ctrlr_nh.param("full_axle_speed_angle", hermite_offset_,
					 DEF_FULL_AXLE_SPEED_ANG);
	  if (hermite_offset_ < 0 || hermite_offset_ > M_PI)
		throw runtime_error("full_axle_speed_angle must be in the range [0, pi].");
	  double zero_axle_speed_ang;
	  ctrlr_nh.param("zero_axle_speed_angle", zero_axle_speed_ang,
					 DEF_ZERO_AXLE_SPEED_ANG);
	  if (zero_axle_speed_ang < 0 || zero_axle_speed_ang > M_PI)
		throw runtime_error("zero_axle_speed_angle must be in the range [0, pi].");
	  if (hermite_offset_ >= zero_axle_speed_ang)
	  {
		throw runtime_error("full_axle_speed_angle must be less than "
							"zero_axle_speed_angle.");
	  }
	  hermite_scale_ = 1 / (zero_axle_speed_ang - hermite_offset_);

	  // Wheels

	  XmlRpcValue wheel_param_list;
	  if (!ctrlr_nh.getParam("wheels", wheel_param_list))
		throw runtime_error("No wheels were specified.");
	  if (wheel_param_list.getType() != XmlRpcValue::TypeArray)
		throw runtime_error("The specified list of wheels is invalid.");
	  if (wheel_param_list.size() < 2)
		throw runtime_error("At least two wheels must be specified.");

	  double wheel_dia_scale;
	  ctrlr_nh.param("wheel_diameter_scale", wheel_dia_scale, DEF_WHEEL_DIA_SCALE);
	  if (wheel_dia_scale <= 0)
	  {
		throw runtime_error("The specified wheel diameter scale is less than or "
							"equal to zero.");
	  }

	  for (int i = 0; i < wheel_param_list.size(); i++)
	  {
		XmlRpcValue& wheel_params = wheel_param_list[i];
		if (wheel_params.getType() != XmlRpcValue::TypeStruct)
		  throw runtime_error("The specified list of wheels is invalid.");

		if (!wheel_params.hasMember("steering_joint"))
		  throw runtime_error("A steering joint was not specified.");
		XmlRpcValue& xml_steer_joint = wheel_params["steering_joint"];
		if (!xml_steer_joint.valid() ||
			xml_steer_joint.getType() != XmlRpcValue::TypeString)
		{
		  throw runtime_error("An invalid steering joint was specified.");
		}
		const string steer_joint_name = xml_steer_joint;
		urdf::JointConstSharedPtr steer_joint =
		  urdf_model.getJoint(steer_joint_name);
		if (steer_joint == NULL)
		{
		  throw runtime_error("Steering joint \"" + steer_joint_name +
							  "\" was not found in the URDF data.");
		}
		const string steer_link = steer_joint->child_link_name;

		if (!wheel_params.hasMember("axle_joint"))
		  throw runtime_error("An axle joint was not specified.");
		XmlRpcValue& xml_axle_joint = wheel_params["axle_joint"];
		if (!xml_axle_joint.valid() ||
			xml_axle_joint.getType() != XmlRpcValue::TypeString)
		{
		  throw runtime_error("An invalid axle joint was specified.");
		}
		const string axle_joint_name = xml_axle_joint;

		if (!wheel_params.hasMember("diameter"))
		  throw runtime_error("A wheel diameter was not specified.");
		XmlRpcValue& xml_dia = wheel_params["diameter"];
		if (!xml_dia.valid())
		  throw runtime_error("An invalid wheel diameter was specified.");
		double dia;
		switch (xml_dia.getType())
		{
		  case XmlRpcValue::TypeInt:
			{
			  const int tmp = xml_dia;
			  dia = tmp;
			}
			break;
		  case XmlRpcValue::TypeDouble:
			dia = xml_dia;
			break;
		  default:
			throw runtime_error("An invalid wheel diameter was specified.");
		}
		if (dia <= 0)
		{
		  throw runtime_error("A specified wheel diameter is less than or "
							  "equal to zero.");
		}
		// Circumference
		const double circ = (2 * M_PI) * (wheel_dia_scale * dia) / 2;

		wheels_.push_back(Wheel(model_tree, base_link, steer_link,
								getJoint(steer_joint_name, true,
										 ctrlr_nh, urdf_model,
										 eff_joint_iface, pos_joint_iface,
										 vel_joint_iface),
								getJoint(axle_joint_name, false,
										 ctrlr_nh, urdf_model,
										 eff_joint_iface, pos_joint_iface,
										 vel_joint_iface), circ));
	  }

	  // Odometry
	  double odom_pub_freq;
	  ctrlr_nh.param("odometry_publishing_frequency", odom_pub_freq,
					 DEF_ODOM_PUB_FREQ);
	  comp_odom_ = odom_pub_freq > 0;
	  if (comp_odom_)
	  {
		odom_pub_period_ = Duration(1 / odom_pub_freq);
		ctrlr_nh.param("publish_odometry_to_base_transform", pub_odom_to_base_,
					   DEF_PUB_ODOM_TO_BASE);

		double init_x, init_y, init_yaw;
		ctrlr_nh.param("initial_x", init_x, DEF_INIT_X);
		ctrlr_nh.param("initial_y", init_y, DEF_INIT_Y);
		ctrlr_nh.param("initial_yaw", init_yaw, DEF_INIT_YAW);
		double x_sd, y_sd, yaw_sd;
		ctrlr_nh.param("x_sd", x_sd, DEF_SD);
		ctrlr_nh.param("y_sd", y_sd, DEF_SD);
		ctrlr_nh.param("yaw_sd", yaw_sd, DEF_SD);
		double x_speed_sd, y_speed_sd, yaw_speed_sd;
		ctrlr_nh.param("x_speed_sd", x_speed_sd, DEF_SD);
		ctrlr_nh.param("y_speed_sd", y_speed_sd, DEF_SD);
		ctrlr_nh.param("yaw_speed_sd", yaw_speed_sd, DEF_SD);

		init_odom_to_base_.setIdentity();
		init_odom_to_base_.rotate(clamp(init_yaw, -M_PI, M_PI));
		init_odom_to_base_.translation() = Vector2d(init_x, init_y);
		odom_to_base_ = init_odom_to_base_;
		odom_rigid_transf_.setIdentity();

		wheel_pos_.resize(2, wheels_.size());
		for (size_t col = 0; col < wheels_.size(); col++)
		  wheel_pos_.col(col) = wheels_[col].pos();
		const Vector2d centroid = wheel_pos_.rowwise().mean();
		wheel_pos_.colwise() -= centroid;
		neg_wheel_centroid_ = -centroid;
		new_wheel_pos_.resize(wheels_.size(), 2);

		string odom_frame, base_frame;
		ctrlr_nh.param("odometry_frame", odom_frame, DEF_ODOM_FRAME);
		ctrlr_nh.param("base_frame", base_frame, DEF_BASE_FRAME);

		odom_pub_.msg_.header.frame_id = odom_frame;
		odom_pub_.msg_.child_frame_id = base_frame;

		odom_pub_.msg_.pose.pose.position.z = 0;

		odom_pub_.msg_.pose.covariance.assign(0);
		odom_pub_.msg_.pose.covariance[0] = x_sd * x_sd;
		odom_pub_.msg_.pose.covariance[7] = y_sd * y_sd;
		odom_pub_.msg_.pose.covariance[35] = yaw_sd * yaw_sd;

		odom_pub_.msg_.twist.twist.linear.z = 0;
		odom_pub_.msg_.twist.twist.angular.x = 0;
		odom_pub_.msg_.twist.twist.angular.y = 0;

		odom_pub_.msg_.twist.covariance.assign(0);
		odom_pub_.msg_.twist.covariance[0] = x_speed_sd * x_speed_sd;
		odom_pub_.msg_.twist.covariance[7] = y_speed_sd * y_speed_sd;
		odom_pub_.msg_.twist.covariance[35] = yaw_speed_sd * yaw_speed_sd;

		odom_pub_.init(ctrlr_nh, "odom", 1);

		if (pub_odom_to_base_)
		{
		  odom_tf_pub_.msg_.transforms.resize(1);
		  geometry_msgs::TransformStamped& odom_tf_trans =
			odom_tf_pub_.msg_.transforms[0];
		  odom_tf_trans.header.frame_id = odom_pub_.msg_.header.frame_id;
		  odom_tf_trans.child_frame_id = odom_pub_.msg_.child_frame_id;
		  odom_tf_trans.transform.translation.z = 0;
		  odom_tf_pub_.init(ctrlr_nh, "/tf", 1);
		}
	  }

	  vel_cmd_sub_ = ctrlr_nh.subscribe("cmd_vel", 1,
										&PointTurnController::velCmdCB,
										this);
	  return true;
	}

	// Velocity command callback
	void PointTurnController::velCmdCB(const TwistConstPtr& vel_cmd)
	{
	  vel_cmd_.x_vel = vel_cmd->linear.x;
	  vel_cmd_.y_vel = vel_cmd->linear.y;
	  vel_cmd_.yaw_vel = vel_cmd->angular.z;
	  vel_cmd_.last_vel_cmd_time = Time::now();
	  vel_cmd_buf_.writeFromNonRT(vel_cmd_);
	}

	// Enforce linear motion limits.
	Vector2d PointTurnController::
	enforceLinLimits(const Vector2d& desired_vel,
					 const double delta_t, const double inv_delta_t)
	{
	  Vector2d vel = desired_vel;
	  if (has_lin_speed_limit_)
	  {
		const double vel_mag = vel.norm();
		if (vel_mag > lin_speed_limit_)
		  vel = (vel / vel_mag) * lin_speed_limit_;
	  }

	  Vector2d accel = (vel - last_lin_vel_) * inv_delta_t;

	  if (accel.dot(last_lin_vel_) >= 0)
	  {
		// Acceleration

		if (has_lin_accel_limit_)
		{
		  const double accel_mag = accel.norm();
		  if (accel_mag > lin_accel_limit_)
		  {
			accel = (accel / accel_mag) * lin_accel_limit_;
			vel = last_lin_vel_ + accel * delta_t;
		  }
		}
	  }
	  else
	  {
		// Deceleration

		if (has_lin_decel_limit_)
		{
		  const double accel_mag = accel.norm();
		  if (accel_mag > lin_decel_limit_)
		  {
			accel = (accel / accel_mag) * lin_decel_limit_;
			vel = last_lin_vel_ + accel * delta_t;
		  }
		}
		if (vel.dot(last_lin_vel_) < 0)
		  vel = Vector2d(0, 0);
	  }

	  last_lin_vel_ = vel;
	  return vel;
	}

	double PointTurnController::enforceYawLimits(const double desired_vel,
														const double delta_t,
														const double inv_delta_t)
	{
	  double vel = desired_vel;
	  if (has_yaw_speed_limit_)
		vel = clamp(vel, -yaw_speed_limit_, yaw_speed_limit_);

	  double accel = (vel - last_yaw_vel_) * inv_delta_t;

	  const double accel_sign = sign(accel);
	  const double last_yaw_vel_sign = sign(last_yaw_vel_);
	  if (accel_sign == last_yaw_vel_sign || last_yaw_vel_sign == 0)
	  {
		// Acceleration

		if (has_yaw_accel_limit_ && fabs(accel) > yaw_accel_limit_)
		{
		  accel = accel_sign * yaw_accel_limit_;
		  vel = last_yaw_vel_ + accel * delta_t;
		}
	  }
	  else
	  {
		// Deceleration

		if (has_yaw_decel_limit_ && fabs(accel) > yaw_decel_limit_)
		{
		  accel = accel_sign * yaw_decel_limit_;
		  vel = last_yaw_vel_ + accel * delta_t;
		}
		if (sign(vel) != last_yaw_vel_sign)
		  vel = 0;
	  }

	  last_yaw_vel_ = vel;
	  return vel;
	}

	// Control the wheels.
	void PointTurnController::ctrlWheels(const Vector2d& lin_vel,
												const double yaw_vel,
												const Duration& period)
	{
	  // Align the wheels so that they are tangent to circles centered
	  // at "center".

	  Vector2d center;
	  center.setZero();

	  std::vector<double> radii;
	  double min_speed_gain = 1;
	  for (auto &wheel : wheels_)
	  {
		Vector2d vec = wheel.pos();
		vec -= center;
		const double radius = vec.norm();
		radii.push_back(radius);
		double theta;
		if (radius > 0)
		{
		  vec /= radius;
		  theta =
		  copysign(acos(vec.dot(PointTurnController::X_DIR)), vec.y()) +
		  M_PI_2;
		}
		else
		{
		  theta = 0;
		}

		const double speed_gain =
		  wheel.ctrlSteering(theta, period, hermite_scale_, hermite_offset_);
		if (speed_gain < min_speed_gain)
		  min_speed_gain = speed_gain;
	  }

	  const double lin_speed_gain = min_speed_gain * yaw_vel;
	  size_t i = 0;
	  for (auto &wheel : wheels_)
	  {
		wheel.ctrlAxle(lin_speed_gain * radii[i++], period);
	  }
	}

	// Compute odometry.
	void PointTurnController::compOdometry(const Time& time,
											const double inv_delta_t)
	{
	  // Compute the rigid transform from wheel_pos_ to new_wheel_pos_.

	  for (size_t row = 0; row < wheels_.size(); row++)
		new_wheel_pos_.row(row) = wheels_[row].pos() + wheels_[row].getDeltaPos();
	  const Eigen::RowVector2d new_wheel_centroid =
		new_wheel_pos_.colwise().mean();
	  new_wheel_pos_.rowwise() -= new_wheel_centroid;

	  const Matrix2d h = wheel_pos_ * new_wheel_pos_;
	  const Eigen::JacobiSVD<Matrix2d> svd(h, Eigen::ComputeFullU |
										   Eigen::ComputeFullV);
	  Matrix2d rot = svd.matrixV() * svd.matrixU().transpose();
	  if (rot.determinant() < 0)
		rot.col(1) *= -1;

	  odom_rigid_transf_.matrix().block(0, 0, 2, 2) = rot;
	  odom_rigid_transf_.translation() =
		rot * neg_wheel_centroid_ + new_wheel_centroid.transpose();
	  odom_to_base_ = odom_to_base_ * odom_rigid_transf_;

	  const double odom_x = odom_to_base_.translation().x();
	  const double odom_y = odom_to_base_.translation().y();
	  const double odom_yaw = atan2(odom_to_base_(1, 0), odom_to_base_(0, 0));

	  // Publish the odometry.

	  geometry_msgs::Quaternion orientation;
	  bool orientation_comped = false;

	  // tf
	  if (pub_odom_to_base_ && time - last_odom_tf_pub_time_ >= odom_pub_period_ &&
		  odom_tf_pub_.trylock())
	  {
		orientation = tf::createQuaternionMsgFromYaw(odom_yaw);
		orientation_comped = true;

		geometry_msgs::TransformStamped& odom_tf_trans =
		  odom_tf_pub_.msg_.transforms[0];
		odom_tf_trans.header.stamp = time;
		odom_tf_trans.transform.translation.x = odom_x;
		odom_tf_trans.transform.translation.y = odom_y;
		odom_tf_trans.transform.rotation = orientation;

		odom_tf_pub_.unlockAndPublish();
		last_odom_tf_pub_time_ = time;
	  }

	  // odom
	  if (time - last_odom_pub_time_ >= odom_pub_period_ && odom_pub_.trylock())
	  {
		if (!orientation_comped)
		  orientation = tf::createQuaternionMsgFromYaw(odom_yaw);

		odom_pub_.msg_.header.stamp = time;
		odom_pub_.msg_.pose.pose.position.x = odom_x;
		odom_pub_.msg_.pose.pose.position.y = odom_y;
		odom_pub_.msg_.pose.pose.orientation = orientation;

		odom_pub_.msg_.twist.twist.linear.x =
		  odom_rigid_transf_.translation().x() * inv_delta_t;
		odom_pub_.msg_.twist.twist.linear.y =
		  odom_rigid_transf_.translation().y() * inv_delta_t;
		odom_pub_.msg_.twist.twist.angular.z =
		  atan2(odom_rigid_transf_(1, 0), odom_rigid_transf_(0, 0)) * inv_delta_t;

		odom_pub_.unlockAndPublish();
		last_odom_pub_time_ = time;
	  }
	}
} // namespace steered_wheel_base_controller

PLUGINLIB_EXPORT_CLASS(SWBC::PointTurnController, controller_interface::ControllerBase)
