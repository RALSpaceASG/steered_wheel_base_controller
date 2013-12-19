/// \file steered_wheel_base_controller.cpp
///
/// \brief Steered-wheel base controller
///
/// This file contains the source code for SteeredWheelBaseController,
/// a base controller for mobile robots. It works with bases that have two or
/// more independently-steerable driven wheels and zero or more omnidirectional
/// passive wheels (e.g. swivel casters).
///
/// Subscribed Topics:
///     cmd_vel (geometry_msgs/Twist)
///         Velocity command, defined in the frame specified by the base_frame
///         parameter. The linear.x and linear.y fields specify the base's
///         desired linear velocity, measured in meters per second.
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
///     ~base_frame (string, default: base_link)
///         Frame in which cmd_vel is defined.
///     ~cmd_vel_timeout (float, default: 0.5)
///         If cmd_vel_timeout is greater than zero and this controller does
///         not receive a velocity command for more than cmd_vel_timeout
///         seconds, wheel motion is paused until a command is received.
///         If cmd_vel_timeout is less than or equal to zero, the command
///         timeout is disabled.
///
///     ~linear_speed_limit (float, default: 1.0)
///         Linear speed limit. If linear_speed_limit is less than zero, the
///         linear speed limit is disabled. Unit: m/s.
///     ~linear_acceleration_limit (float, default: 1.0)
///         Linear acceleration limit. If linear_acceleration_limit is less
///         than zero, the linear acceleration limit is disabled. Unit: m/s**2.
///     ~linear_deceleration_limit (float, default: -1.0)
///         Linear deceleration limit. If linear_deceleration_limit is less
///         than or equal to zero, the linear deceleration limit is disabled.
///         Unit: m/s**2.
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
///     ~odometry_frame (string, default: odom)
///         Odometry frame.
///     ~initial_x (float, default: 0.0)
///         X coordinate of the base's initial position in the odometry frame.
///         Unit: meter.
///     ~initial_y (float, default: 0.0)
///         Y coordinate of the base's initial position in the odometry frame.
///         Unit: meter.
///     ~initial_yaw (float, default: 0.0)
///         Initial orientation of the base in the odometry frame.
///         Unit: radian.
///
/// Provided tf Transforms:
///     <odometry_frame> to <base_frame>
///         Specifies the base's pose in the odometry frame.
//
// Copyright (c) 2013 Wunderkammer Laboratory
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

#include <algorithm>
#include <exception>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <angles/angles.h>

#include <boost/foreach.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <boost/shared_ptr.hpp>

#include <control_toolbox/pid.h>
#include <controller_interface/controller_base.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

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

using std::runtime_error;
using std::set;
using std::string;

using boost::shared_ptr;

using Eigen::Affine2d;
using Eigen::Matrix2d;
using Eigen::Vector2d;

using geometry_msgs::TwistConstPtr;

using hardware_interface::JointHandle;
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

namespace
{

void addClaimedResources(hardware_interface::HardwareInterface *const hw_iface,
                         set<string>& claimed_resources)
{
  if (hw_iface == NULL)
    return;
  const set<string> claims = hw_iface->getClaims();
  claimed_resources.insert(claims.begin(), claims.end());
  hw_iface->clearClaims();
}

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

double hermite(const double t)
{
  if (t <= 0)
    return 0;
  if (t >= 1)
    return 1;
  return (-2 * t + 3) * t * t;  // -2t**3 + 3t**2
}

class Joint
{
public:
  virtual ~Joint() {}
  virtual void init() {}

  bool isValidPos(const double pos) const
    {return pos >= lower_limit_ && pos <= upper_limit_;}
  double getPos() const {return handle_.getPosition();}
  virtual void setPos(const double pos, const Duration& period) {}

  virtual void setVel(const double vel, const Duration& period) = 0;

protected:
  Joint(const JointHandle& handle,
        const shared_ptr<const urdf::Joint> urdf_joint);

  JointHandle handle_;
  const double lower_limit_, upper_limit_;  // Unit: radian
};

// Position-controlled joint
class PosJoint : public Joint
{
public:
  PosJoint(const JointHandle& handle,
           const shared_ptr<const urdf::Joint> urdf_joint) :
    Joint(handle, urdf_joint) {}

  virtual void init();
  virtual void setPos(const double pos, const Duration& period);
  virtual void setVel(const double vel, const Duration& period);

private:
  double pos_;
};

// Velocity-controlled joint. VelJoint is used for axles only.
class VelJoint : public Joint
{
public:
  VelJoint(const JointHandle& handle,
           const shared_ptr<const urdf::Joint> urdf_joint) :
    Joint(handle, urdf_joint) {}

  virtual void setVel(const double vel, const Duration& period);
};

// An object of class PIDJoint is a joint controlled by a PID controller.
class PIDJoint : public Joint
{
public:
  PIDJoint(const JointHandle& handle,
           const shared_ptr<const urdf::Joint> urdf_joint,
           const NodeHandle& ctrlr_nh);

  virtual void init();
  virtual void setPos(const double pos, const Duration& period);
  virtual void setVel(const double vel, const Duration& period);

private:
  const int type_;  // URDF joint type
  control_toolbox::Pid pid_ctrlr_;
};

// An object of class Wheel is a steered wheel.
class Wheel
{
public:
  Wheel(const KDL::Tree& tree,
        const string& base_frame, const string& steer_frame,
        const shared_ptr<Joint> steer_joint,
        const shared_ptr<Joint> axle_joint,
        const double circ);

  const Vector2d& pos() const {return pos_;}
  Vector2d getDeltaPos();

  void initJoints();
  void ctrlSteering(const Duration& period);
  void ctrlSteering(const double theta_desired, const Duration& period);
  void ctrlAxle(const double lin_speed, const Duration& period) const;

private:
  static const double ZERO_AXLE_VEL_ANG;

  void initPos(const KDL::Tree& tree, const string& base_frame);

  string steer_frame_;  // Steering frame
  Vector2d pos_;        // Wheel's position in the base frame

  shared_ptr<Joint> steer_joint_;   // Steering joint
  shared_ptr<Joint> axle_joint_;
  double theta_steer_;              // Steering joint position
  double last_theta_steer_desired_; // Last desired steering joint position
  double last_theta_axle_;          // Last axle joint position

  double radius_;         // Unit: meter.
  double inv_radius_;     // Inverse of radius_
  double axle_vel_gain_;  // Axle velocity gain
};

Joint::Joint(const JointHandle& handle,
             const shared_ptr<const urdf::Joint> urdf_joint) :
  handle_(handle), lower_limit_(urdf_joint->limits->lower),
  upper_limit_(urdf_joint->limits->upper)
{
  // Do nothing.
}

// Initialize this joint.
void PosJoint::init()
{
  pos_ = handle_.getPosition();
}

// Specify this joint's position.
void PosJoint::setPos(const double pos, const Duration& /* period */)
{
  pos_ = pos;
  handle_.setCommand(pos_);
}

// Specify this joint's velocity.
void PosJoint::setVel(const double vel, const Duration& period)
{
  pos_ += vel * period.toSec();
  handle_.setCommand(pos_);
}

// Specify this joint's velocity.
void VelJoint::setVel(const double vel, const Duration& /* period */)
{
  handle_.setCommand(vel);
}

PIDJoint::PIDJoint(const JointHandle& handle,
                   const shared_ptr<const urdf::Joint> urdf_joint,
                   const NodeHandle& ctrlr_nh) :
  Joint(handle, urdf_joint), type_(urdf_joint->type)
{
  const NodeHandle nh(ctrlr_nh, "pid_gains/" + handle.getName());
  if (!pid_ctrlr_.init(nh))
  {
    throw runtime_error("No PID gain values for \"" + handle.getName() +
                        "\" were found.");
  }
}

// Initialize this joint.
void PIDJoint::init()
{
  pid_ctrlr_.reset();
}

// Specify this joint's position.
void PIDJoint::setPos(const double pos, const Duration& period)
{
  const double curr_pos = handle_.getPosition();

  double error;
  switch (type_)
  {
    case urdf::Joint::REVOLUTE:
      angles::shortest_angular_distance_with_limits(curr_pos, pos,
                                                    lower_limit_, upper_limit_,
                                                    error);
      break;
    case urdf::Joint::CONTINUOUS:
      error = angles::shortest_angular_distance(curr_pos, pos);
      break;
    default:
      error = pos - curr_pos;
      break;
  }

  handle_.setCommand(pid_ctrlr_.computeCommand(error, period));
}

// Specify this joint's velocity.
void PIDJoint::setVel(const double vel, const Duration& period)
{
  const double error = vel - handle_.getVelocity();
  handle_.setCommand(pid_ctrlr_.computeCommand(error, period));
}

// Wheel axle velocities vary from full velocity (|steering angle| = 0) to
// zero velocity (|steering angle| >= ZERO_AXLE_VEL_ANG).
// ZERO_AXLE_VEL_ANG unit: radian.
const double Wheel::ZERO_AXLE_VEL_ANG = M_PI / 8;

Wheel::Wheel(const KDL::Tree& tree,
             const string& base_frame, const string& steer_frame,
             const shared_ptr<Joint> steer_joint,
             const shared_ptr<Joint> axle_joint,
             const double circ)
{
  steer_frame_ = steer_frame;
  initPos(tree, base_frame);

  steer_joint_ = steer_joint;
  axle_joint_ = axle_joint;
  theta_steer_ = steer_joint_->getPos();
  last_theta_steer_desired_ = theta_steer_;
  last_theta_axle_ = axle_joint_->getPos();

  radius_ = circ / (2 * M_PI);
  inv_radius_ = 1 / radius_;
  axle_vel_gain_ = 0;
}

// Return the difference between this wheel's current position and its
// position when getDeltaPos() was last called. The returned vector is defined
// in the base frame.
Vector2d Wheel::getDeltaPos()
{
  const double theta_axle = axle_joint_->getPos();
  const double delta_theta_axle = theta_axle - last_theta_axle_;
  last_theta_axle_ = theta_axle;
  const double vec_mag = delta_theta_axle * radius_;
  return Vector2d(cos(theta_steer_), sin(theta_steer_)) * vec_mag;
}

// Initialize this wheel's steering and axle joints.
void Wheel::initJoints()
{
  steer_joint_->init();
  axle_joint_->init();
}

// Maintain the position of this wheel's steering joint.
void Wheel::ctrlSteering(const Duration& period)
{
  ctrlSteering(last_theta_steer_desired_, period);
}

// Control this wheel's steering joint. theta_desired range: [-pi, pi].
void Wheel::ctrlSteering(const double theta_desired, const Duration& period)
{
  last_theta_steer_desired_ = theta_desired;

  // Find the minimum rotation that will align the wheel with theta_desired.
  theta_steer_ = steer_joint_->getPos();
  const double theta_diff = fabs(theta_desired - theta_steer_);
  double theta;
  if (theta_diff > M_PI_2)
  {
    theta = theta_desired - copysign(M_PI, theta_desired);
    axle_vel_gain_ = -1;
  }
  else
  {
    theta = theta_desired;
    axle_vel_gain_ = 1;
  }

  // Keep theta within its valid range.
  if (!steer_joint_->isValidPos(theta))
  {
    theta -= copysign(M_PI, theta);
    axle_vel_gain_ = -axle_vel_gain_;
  }

  steer_joint_->setPos(theta, period);
  axle_vel_gain_ *= 1 - hermite(fabs(theta - theta_steer_) /
                                ZERO_AXLE_VEL_ANG);
}

// Control this wheel's axle joint.
void Wheel::ctrlAxle(const double lin_speed, const Duration& period) const
{
  const double ang_vel = axle_vel_gain_ * inv_radius_ * lin_speed;
  axle_joint_->setVel(ang_vel, period);
}

// Initialize pos_.
void Wheel::initPos(const KDL::Tree& tree, const string& base_frame)
{
  KDL::Chain chain;
  if (!tree.getChain(base_frame, steer_frame_, chain))
  {
    throw runtime_error("No kinematic chain was found from \"" + base_frame +
                        "\" to \"" + steer_frame_ + "\".");
  }

  const unsigned int num_joints = chain.getNrOfJoints();
  KDL::JntArray joint_positions(num_joints);
  for (unsigned int i = 0; i < num_joints; i++)
    joint_positions(i) = 0;

  KDL::ChainFkSolverPos_recursive solver(chain);
  KDL::Frame frame;
  if (solver.JntToCart(joint_positions, frame) < 0)
  {
    throw runtime_error("The position of steering frame \"" + steer_frame_ +
                        "\" in base frame \"" + base_frame +
                        "\" was not found.");
  }
  pos_ = Vector2d(frame.p.x(), frame.p.y());
}

// Create an object of class Joint that corresponds to the URDF joint specified
// by joint_name.
shared_ptr<Joint> getJoint(const string& joint_name, const bool is_steer_joint,
                           const NodeHandle& ctrlr_nh,
                           const urdf::Model& urdf_model,
                           EffortJointInterface *const eff_joint_iface,
                           PositionJointInterface *const pos_joint_iface,
                           VelocityJointInterface *const vel_joint_iface)
{
  if (eff_joint_iface != NULL)
  {
    JointHandle handle;
    bool handle_found;
    try
    {
      handle = eff_joint_iface->getHandle(joint_name);
      handle_found = true;
    }
    catch (...)
    {
      handle_found = false;
    }

    if (handle_found)
    {
      const shared_ptr<const urdf::Joint> urdf_joint =
        urdf_model.getJoint(joint_name);
      if (urdf_joint == NULL)
      {
        throw runtime_error("\"" + joint_name +
                            "\" was not found in the URDF data.");
      }

      shared_ptr<Joint> joint(new PIDJoint(handle, urdf_joint, ctrlr_nh));
      return joint;
    }
  }

  if (pos_joint_iface != NULL)
  {
    JointHandle handle;
    bool handle_found;
    try
    {
      handle = pos_joint_iface->getHandle(joint_name);
      handle_found = true;
    }
    catch (...)
    {
      handle_found = false;
    }

    if (handle_found)
    {
      const shared_ptr<const urdf::Joint> urdf_joint =
        urdf_model.getJoint(joint_name);
      if (urdf_joint == NULL)
      {
        throw runtime_error("\"" + joint_name +
                            "\" was not found in the URDF data.");
      }

      shared_ptr<Joint> joint(new PosJoint(handle, urdf_joint));
      return joint;
    }
  }

  if (vel_joint_iface != NULL)
  {
    JointHandle handle;
    bool handle_found;
    try
    {
      handle = vel_joint_iface->getHandle(joint_name);
      handle_found = true;
    }
    catch (...)
    {
      handle_found = false;
    }

    if (handle_found)
    {
      const shared_ptr<const urdf::Joint> urdf_joint =
        urdf_model.getJoint(joint_name);
      if (urdf_joint == NULL)
      {
        throw runtime_error("\"" + joint_name +
                            "\" was not found in the URDF data.");
      }

      if (is_steer_joint)
      {
        shared_ptr<Joint> joint(new PIDJoint(handle, urdf_joint, ctrlr_nh));
        return joint;
      }
      shared_ptr<Joint> joint(new VelJoint(handle, urdf_joint));
      return joint;
    }
  }

  throw runtime_error("No handle for \"" + joint_name + "\" was found.");
}

} // namespace

namespace steered_wheel_base_controller
{

// Steered-wheel base controller
class SteeredWheelBaseController : public controller_interface::ControllerBase
{
public:
  SteeredWheelBaseController();

  // These are not real-time safe.
  virtual bool initRequest(RobotHW *const robot_hw,
                           NodeHandle& root_nh, NodeHandle& ctrlr_nh,
                           set<string>& claimed_resources);
  virtual string getHardwareInterfaceType() const;

  // These are real-time safe.
  virtual void starting(const Time& time);
  virtual void update(const Time& time, const Duration& period);

private:
  struct VelCmd     // Velocity command
  {
    double x_vel;   // X velocity component. Unit: m/s.
    double y_vel;   // Y velocity component. Unit: m/s.
    double yaw_vel; // Yaw velocity. Unit: rad/s.

    // last_vel_cmd_time is the time at which the most recent velocity command
    // was received.
    Time last_vel_cmd_time;
  };

  static const string DEF_ROBOT_DESC_NAME;
  static const string DEF_BASE_FRAME;
  static const double DEF_CMD_VEL_TIMEOUT;

  static const double DEF_LIN_SPEED_LIMIT;
  static const double DEF_LIN_ACCEL_LIMIT;
  static const double DEF_LIN_DECEL_LIMIT;

  static const double DEF_YAW_SPEED_LIMIT;
  static const double DEF_YAW_ACCEL_LIMIT;
  static const double DEF_YAW_DECEL_LIMIT;

  static const double DEF_WHEEL_DIA_SCALE;

  static const double DEF_ODOM_PUB_FREQ;
  static const string DEF_ODOM_FRAME;
  static const double DEF_INIT_X;
  static const double DEF_INIT_Y;
  static const double DEF_INIT_YAW;

  static const Vector2d X_DIR;

  void init(EffortJointInterface *const eff_joint_iface,
            PositionJointInterface *const pos_joint_iface,
            VelocityJointInterface *const vel_joint_iface,
            NodeHandle& ctrlr_nh);
  void velCmdCB(const TwistConstPtr& vel_cmd);

  Vector2d enforceLinLimits(const Vector2d& desired_vel,
                            const double delta_t, const double inv_delta_t);
  double enforceYawLimits(const double desired_vel,
                          const double delta_t, const double inv_delta_t);
  void ctrlWheels(const Vector2d& lin_vel, const double yaw_vel,
                  const Duration& period);
  void compOdometry(const Time& time, const double inv_delta_t);

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

  Vector2d last_lin_vel_; // Last linear velocity. Unit: m/s.
  double last_yaw_vel_;   // Last yaw velocity. Unit: rad/s.

  // Velocity command member variables
  VelCmd vel_cmd_;
  RealtimeBuffer<VelCmd> vel_cmd_buf_;
  bool vel_cmd_timeout_enabled_;
  Duration vel_cmd_timeout_;
  ros::Subscriber vel_cmd_sub_;

  // Odometry
  bool comp_odom_;              // Compute odometry
  Duration odom_pub_period_;    // Odometry publishing period
  Affine2d init_odom_to_base_;  // Initial odometry to base frame transform
  Affine2d odom_to_base_;       // Odometry to base frame transform
  Affine2d odom_affine_;
  double last_odom_x_, last_odom_y_, last_odom_yaw_;
  // wheel_pos_ contains the positions of the wheel's steering axles.
  // The positions are relative to the centroid of the steering axle positions
  // in the base frame. neg_wheel_centroid is the negative version of that
  // centroid.
  Eigen::Matrix2Xd wheel_pos_;
  Vector2d neg_wheel_centroid_;
  Eigen::MatrixX2d new_wheel_pos_;
  RealtimePublisher<nav_msgs::Odometry> odom_pub_;
  RealtimePublisher<tf::tfMessage> odom_tf_pub_;
  Time last_odom_pub_time_, last_odom_tf_pub_time_;
};

const string SteeredWheelBaseController::DEF_ROBOT_DESC_NAME =
  "robot_description";
const string SteeredWheelBaseController::DEF_BASE_FRAME = "base_link";
const double SteeredWheelBaseController::DEF_CMD_VEL_TIMEOUT = 0.5;

const double SteeredWheelBaseController::DEF_LIN_SPEED_LIMIT = 1;
const double SteeredWheelBaseController::DEF_LIN_ACCEL_LIMIT = 1;
const double SteeredWheelBaseController::DEF_LIN_DECEL_LIMIT = -1;

const double SteeredWheelBaseController::DEF_YAW_SPEED_LIMIT = 1;
const double SteeredWheelBaseController::DEF_YAW_ACCEL_LIMIT = 1;
const double SteeredWheelBaseController::DEF_YAW_DECEL_LIMIT = -1;

const double SteeredWheelBaseController::DEF_WHEEL_DIA_SCALE = 1;

const double SteeredWheelBaseController::DEF_ODOM_PUB_FREQ = 30.0;
const string SteeredWheelBaseController::DEF_ODOM_FRAME = "odom";
const double SteeredWheelBaseController::DEF_INIT_X = 0;
const double SteeredWheelBaseController::DEF_INIT_Y = 0;
const double SteeredWheelBaseController::DEF_INIT_YAW = 0;

// X direction
const Vector2d SteeredWheelBaseController::X_DIR = Vector2d::UnitX();

SteeredWheelBaseController::SteeredWheelBaseController()
{
  state_ = CONSTRUCTED;
}

bool SteeredWheelBaseController::initRequest(RobotHW *const robot_hw,
                                             NodeHandle& /* root_nh */,
                                             NodeHandle& ctrlr_nh,
                                             set<string>& claimed_resources)
{
  if (state_ != CONSTRUCTED)
  {
    ROS_ERROR("The steered-wheel base controller could not be created.");
    return false;
  }

  EffortJointInterface *const eff_joint_iface =
    robot_hw->get<EffortJointInterface>();
  PositionJointInterface *const pos_joint_iface =
    robot_hw->get<PositionJointInterface>();
  VelocityJointInterface *const vel_joint_iface =
    robot_hw->get<VelocityJointInterface>();

  if (eff_joint_iface != NULL)
    eff_joint_iface->clearClaims();
  if (pos_joint_iface != NULL)
    pos_joint_iface->clearClaims();
  if (vel_joint_iface != NULL)
    vel_joint_iface->clearClaims();

  try
  {
    init(eff_joint_iface, pos_joint_iface, vel_joint_iface, ctrlr_nh);
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }

  claimed_resources.clear();
  addClaimedResources(eff_joint_iface, claimed_resources);
  addClaimedResources(pos_joint_iface, claimed_resources);
  addClaimedResources(vel_joint_iface, claimed_resources);

  state_ = INITIALIZED;
  return true;
}

string SteeredWheelBaseController::getHardwareInterfaceType() const
{
  return "";
}

void SteeredWheelBaseController::starting(const Time& time)
{
  last_lin_vel_ = Vector2d(0, 0);
  last_yaw_vel_ = 0;

  BOOST_FOREACH(Wheel& wheel, wheels_)
    wheel.initJoints();

  if (comp_odom_)
  {
    odom_to_base_ = init_odom_to_base_;
    odom_affine_.setIdentity();

    last_odom_x_ = odom_to_base_.translation().x();
    last_odom_y_ = odom_to_base_.translation().y();
    last_odom_yaw_ = atan2(odom_to_base_(1, 0), odom_to_base_(0, 0));
    last_odom_pub_time_ = time;
    last_odom_tf_pub_time_ = time;
  }

  vel_cmd_.x_vel = 0;
  vel_cmd_.y_vel = 0;
  vel_cmd_.yaw_vel = 0;
  vel_cmd_.last_vel_cmd_time = time;
  vel_cmd_buf_.initRT(vel_cmd_);
}

void SteeredWheelBaseController::update(const Time& time,
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
    // Stop the robot.
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

// Initialize this steered-wheel base controller.
void SteeredWheelBaseController::
init(EffortJointInterface *const eff_joint_iface,
     PositionJointInterface *const pos_joint_iface,
     VelocityJointInterface *const vel_joint_iface,
     NodeHandle& ctrlr_nh)
{
  string robot_desc_name;
  ctrlr_nh.param("robot_description_name", robot_desc_name,
                 DEF_ROBOT_DESC_NAME);
  urdf::Model urdf_model;
  if (!urdf_model.initParam(robot_desc_name))
    throw runtime_error("The URDF data was not found.");
  KDL::Tree model_tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, model_tree))
    throw runtime_error("The kinematic tree could not be created.");

  string base_frame;
  ctrlr_nh.param("base_frame", base_frame, DEF_BASE_FRAME);
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
    const shared_ptr<const urdf::Joint> steer_joint =
      urdf_model.getJoint(steer_joint_name);
    if (steer_joint == NULL)
    {
      throw runtime_error("Steering joint \"" + steer_joint_name +
                          "\" was not found in the URDF data.");
    }
    const string steer_frame = steer_joint->child_link_name;

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

    wheels_.push_back(Wheel(model_tree, base_frame, steer_frame,
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

    double init_x, init_y, init_yaw;
    ctrlr_nh.param("initial_x", init_x, DEF_INIT_X);
    ctrlr_nh.param("initial_y", init_y, DEF_INIT_Y);
    ctrlr_nh.param("initial_yaw", init_yaw, DEF_INIT_YAW);
    init_odom_to_base_.setIdentity();
    init_odom_to_base_.rotate(clamp(init_yaw, -M_PI, M_PI));
    init_odom_to_base_.translation() = Vector2d(init_x, init_y);

    wheel_pos_.resize(2, wheels_.size());
    for (size_t col = 0; col < wheels_.size(); col++)
      wheel_pos_.col(col) = wheels_[col].pos();
    const Vector2d centroid = wheel_pos_.rowwise().mean();
    wheel_pos_.colwise() -= centroid;
    neg_wheel_centroid_ = -centroid;
    new_wheel_pos_.resize(wheels_.size(), 2);

    string odom_frame;
    ctrlr_nh.param("odometry_frame", odom_frame, DEF_ODOM_FRAME);
    odom_pub_.msg_.header.frame_id = odom_frame;
    odom_pub_.msg_.child_frame_id = base_frame;
    odom_pub_.msg_.pose.pose.position.z = 0;
    odom_pub_.msg_.twist.twist.linear.z = 0;
    odom_pub_.msg_.twist.twist.angular.x = 0;
    odom_pub_.msg_.twist.twist.angular.y = 0;
    odom_pub_.init(ctrlr_nh, "odom", 1);

    odom_tf_pub_.msg_.transforms.resize(1);
    geometry_msgs::TransformStamped& odom_tf_trans =
      odom_tf_pub_.msg_.transforms[0];
    odom_tf_trans.header.frame_id = odom_pub_.msg_.header.frame_id;
    odom_tf_trans.child_frame_id = odom_pub_.msg_.child_frame_id;
    odom_tf_trans.transform.translation.z = 0;
    odom_tf_pub_.init(ctrlr_nh, "/tf", 1);
  }

  vel_cmd_sub_ = ctrlr_nh.subscribe("cmd_vel", 1,
                                    &SteeredWheelBaseController::velCmdCB,
                                    this);
}

// Velocity command callback
void SteeredWheelBaseController::velCmdCB(const TwistConstPtr& vel_cmd)
{
  vel_cmd_.x_vel = vel_cmd->linear.x;
  vel_cmd_.y_vel = vel_cmd->linear.y;
  vel_cmd_.yaw_vel = vel_cmd->angular.z;
  vel_cmd_.last_vel_cmd_time = Time::now();
  vel_cmd_buf_.writeFromNonRT(vel_cmd_);
}

// Enforce linear motion limits.
Vector2d SteeredWheelBaseController::
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

  if (last_lin_vel_.dot(accel) >= 0)
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
  }

  last_lin_vel_ = vel;
  return vel;
}

double SteeredWheelBaseController::enforceYawLimits(const double desired_vel,
                                                    const double delta_t,
                                                    const double inv_delta_t)
{
  double vel = desired_vel;
  if (has_yaw_speed_limit_)
    vel = clamp(vel, -yaw_speed_limit_, yaw_speed_limit_);

  double accel = (vel - last_yaw_vel_) * inv_delta_t;

  const double accel_sign = boost::math::sign(accel);
  if (boost::math::sign(last_yaw_vel_) == accel_sign)
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
  }

  last_yaw_vel_ = vel;
  return vel;
}

// Control the wheels.
void SteeredWheelBaseController::ctrlWheels(const Vector2d& lin_vel,
                                            const double yaw_vel,
                                            const Duration& period)
{
  const double lin_speed = lin_vel.norm();

  if (yaw_vel == 0)
  {
    if (lin_speed > 0)
    {
      // Point the wheels in the same direction.
      const Vector2d dir = lin_vel / lin_speed;
      const double theta =
        copysign(acos(dir.dot(SteeredWheelBaseController::X_DIR)), dir.y());
      BOOST_FOREACH(Wheel& wheel, wheels_)
      {
        wheel.ctrlSteering(theta, period);
        wheel.ctrlAxle(lin_speed, period);
      }
    }
    else
    {
      // Stop wheel rotation.
      BOOST_FOREACH(Wheel& wheel, wheels_)
      {
        wheel.ctrlSteering(period);
        wheel.ctrlAxle(0, period);
      }
    }
  }
  else  // The yaw velocity is nonzero.
  {
    // Align the wheels so that they are tangent to circles centered
    // at "center".

    Vector2d center;
    if (lin_speed > 0)
    {
      const Vector2d dir = lin_vel / lin_speed;
      center = Vector2d(-dir.y(), dir.x()) * lin_speed / yaw_vel;
    }
    else
    {
      center.setZero();
    }

    BOOST_FOREACH(Wheel& wheel, wheels_)
    {
      Vector2d vec = wheel.pos();
      vec -= center;
      const double radius = vec.norm();
      double theta;
      if (radius > 0)
      {
        vec /= radius;
        theta =
          copysign(acos(vec.dot(SteeredWheelBaseController::X_DIR)), vec.y()) +
          M_PI_2;
      }
      else
      {
        theta = 0;
      }
                        
      wheel.ctrlSteering(theta, period);
      wheel.ctrlAxle(yaw_vel * radius, period);
    }
  }
}

// Compute odometry.
void SteeredWheelBaseController::compOdometry(const Time& time,
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

  odom_affine_.matrix().block(0, 0, 2, 2) = rot;
  odom_affine_.translation() =
    rot * neg_wheel_centroid_ + new_wheel_centroid.transpose();
  odom_to_base_ = odom_to_base_ * odom_affine_;

  const double odom_x = odom_to_base_.translation().x();
  const double odom_y = odom_to_base_.translation().y();
  const double odom_yaw = atan2(odom_to_base_(1, 0), odom_to_base_(0, 0));

  // Publish the odometry.

  geometry_msgs::Quaternion orientation;
  bool orientation_comped = false;

  // tf
  if (time - last_odom_tf_pub_time_ >= odom_pub_period_ &&
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
      (odom_x - last_odom_x_) * inv_delta_t;
    odom_pub_.msg_.twist.twist.linear.y =
      (odom_y - last_odom_y_) * inv_delta_t;
    odom_pub_.msg_.twist.twist.angular.z =
      (odom_yaw - last_odom_yaw_) * inv_delta_t;

    odom_pub_.unlockAndPublish();
    last_odom_pub_time_ = time;
  }

  last_odom_x_ = odom_x;
  last_odom_y_ = odom_y;
  last_odom_yaw_ = odom_yaw;
}

} // namespace steered_wheel_base_controller

PLUGINLIB_EXPORT_CLASS(steered_wheel_base_controller::\
SteeredWheelBaseController, controller_interface::ControllerBase)
