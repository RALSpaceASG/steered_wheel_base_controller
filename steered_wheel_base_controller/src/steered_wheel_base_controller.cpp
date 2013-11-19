/// \file steered_wheel_base_controller.cpp
///
/// \brief Steered-wheel base controller
///
/// This file contains the source code for steered_wheel_base_controller,
/// a base controller for mobile robots. It works with bases that have one or
/// more independently-steerable driven wheels and zero or more omnidirectional
/// passive wheels (e.g. swivel casters).
//
// Subscribed Topics:
//     cmd_vel (geometry_msgs/Twist)
//         Velocity command, defined in the frame specified by the base_frame
//         parameter. The linear.x and linear.y fields specify the base's
//         desired linear velocity, measured in meters per second.
//         The angular.z field specifies the base's desired angular velocity,
//         measured in radians per second.
//
// Parameters:
//     ~base_frame (string, default: base_link)
//         Frame in which cmd_vel is defined.
//
//     ~wheels (list, default: empty)
//         Zero or more wheels may be specified.
//
//         steering_frame (string, default: "")
//             steering_frame's z axis is collinear with the wheel's steering
//             axis. Its axes are parallel to those of base_frame.
//         minimum_steering_angle (float, default: \todo)
//             \todo
//         maximum_steering_angle (float, default: \todo)
//             \todo
//         steering_controller (string, default: "")
//             Steering controller.
//         axle_controller (string, default: "")
//             Axle controller.
//         diameter (float, default: 1.0)
//             Wheel diameter. It must be greater than zero. Unit: meter.
//
//     ~linear_speed_limit (float, default: 1.0)
//         \todo. Unit: m/s.
//     ~linear_acceleration_limit (float, default: 1.0)
//         \todo
//     ~linear_deceleration_limit (float, default: -1.0)
//         \todo
//
//     ~yaw_speed_limit (float, default: 1.0)
//         \todo. Unit: rad/s.
//     ~yaw_acceleration_limit (float, default: 1.0)
//         \todo
//     ~yaw_deceleration_limit (float, default: -1.0)
//         \todo
//
//     ~cmd_vel_timeout (float, default: 0.5)
//         If cmd_vel_timeout is greater than zero and this controller does
//         not receive a velocity command for more than cmd_vel_timeout
//         seconds, wheel motion is paused until a command is received.
//         If cmd_vel_timeout is less than or equal to zero, the command
//         timeout is disabled.
//
// \todo: Services? Transforms?
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

#include <angles/angles.h>
#include <boost/foreach.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <control_toolbox/pid.h>
#include <controller_interface/controller_base.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <urdf/model.h>

using std::runtime_error;
using std::set;
using std::string;
using std::vector;

using geometry_msgs::TwistConstPtr;

using hardware_interface::JointHandle;
using hardware_interface::RobotHW;

using hardware_interface::EffortJointInterface;
using hardware_interface::PositionJointInterface;
using hardware_interface::VelocityJointInterface;

using realtime_tools::RealtimeBuffer;

using ros::Duration;
using ros::NodeHandle;
using ros::Time;

namespace
{

// Two-element vector. Deriving Vec2 from
// boost::numeric::ublas::c_vector<double, 2> does not work.
class Vec2
{
public:
  Vec2() {}
  Vec2(const double x, const double y) { vec_[0] = x; vec_[1] = y;}
  Vec2(const boost::numeric::ublas::c_vector<double, 2>& vec) : vec_(vec) {}

  double x() const {return vec_[0];}
  double y() const {return vec_[1];}
  void x(const double val) {vec_[0] = val;}
  void y(const double val) {vec_[1] = val;}

  double magnitude() const {return norm_2(vec_);}
  double dot_prod(const Vec2& vec) const {return inner_prod(vec_, vec.vec_);}

  Vec2 operator +(const Vec2& val) const {return Vec2(vec_ + val.vec_);}
  Vec2 operator -(const Vec2& val) const {return Vec2(vec_ - val.vec_);}
  Vec2 operator *(double val) const {return Vec2(vec_ * val);}
  Vec2 operator /(double val) const {return Vec2(vec_ / val);}

  Vec2& operator -=(const Vec2& val) {vec_ -= val.vec_; return *this;}
  Vec2& operator *=(double val) {vec_ *= val; return *this;}
  Vec2& operator /=(double val) {vec_ /= val; return *this;}

private:
  boost::numeric::ublas::c_vector<double, 2> vec_;
};

class Joint
{
public:
  virtual ~Joint() {}

  virtual void init() = 0;
  double getPos() const {return handle_.getPosition();}
  virtual void setPos(const double pos, const Duration& period) = 0;
  virtual void setVel(const double vel, const Duration& period) = 0;

protected:
  Joint(const JointHandle& handle) : handle_(handle) {}

  JointHandle handle_;
};

// An object of class PIDJoint is a joint controlled by a PID controller.
class PIDJoint : public Joint
{
public:
  virtual void init();
  virtual void setPos(const double pos, const Duration& period);
  virtual void setVel(const double vel, const Duration& period);

protected:
  PIDJoint(const JointHandle& handle, const urdf::Joint *const urdf_joint,
           const NodeHandle& pid_ctrlr_nh, const bool pid_ctrlr_required);

private:
  const int type_;
  const double lower_limit_, upper_limit_;

  control_toolbox::Pid pid_ctrlr_;
};

// Effort-controlled joint
class EffJoint : public PIDJoint
{
public:
  EffJoint(const JointHandle& handle, const urdf::Joint *const urdf_joint,
           const NodeHandle& pid_ctrlr_nh) :
    PIDJoint(handle, urdf_joint, pid_ctrlr_nh, true) {}
};

// Position-controlled joint
class PosJoint : public Joint
{
public:
  PosJoint(const JointHandle& handle) : Joint(handle) {}

  virtual void init();
  virtual void setPos(const double pos, const Duration& period);
  virtual void setVel(const double vel, const Duration& period);

private:
  double pos_;
};

// Velocity-controlled joint
class VelJoint : public PIDJoint
{
public:
  VelJoint(const JointHandle& handle, const urdf::Joint *const urdf_joint,
           const NodeHandle& pid_ctrlr_nh) :
    PIDJoint(handle, urdf_joint, pid_ctrlr_nh, false) {}

  virtual void setVel(const double vel, const Duration& period);
};

// An object of class Wheel is a steered wheel.
class Wheel
{
public:
  Wheel(const double circ, const char *const steer_axle_joint_name,
        Joint *const steer_joint, Joint *const axle_joint);

  void initPos(const tf::TransformListener& tfl, const char *const base_frame);
  const Vec2& getPos() const {return pos_;}
  void initJoints();
  void ctrlSteering(const double theta_desired, const Duration& period);
  void ctrlAxle(const double lin_speed, const Duration& period) const;

private:
  // \todo These should be parameters.
  static const double THETA_MIN;
  static const double THETA_MAX;

  string steer_axle_joint_name_;  // Steering axle joint name
  Vec2 pos_;  // Position

  boost::shared_ptr<Joint> steer_joint_;  // Steering joint
  boost::shared_ptr<Joint> axle_joint_;

  double axle_vel_gain_;  // Axle velocity gain
  // lin_to_ang_ is a multiplier that converts this wheel's linear speed into
  // the axle's angular velocity.
  double lin_to_ang_;
};

// These values are from cerberus.urdf.xacro.
const double Wheel::THETA_MIN = 1.5 * -M_PI_2;  // Minimum steering angle
const double Wheel::THETA_MAX = 1.5 * M_PI_2;   // Maximum steering angle

double hermite(const double t)
{
  if (t <= 0)
    return 0;
  if (t >= 1)
    return 1;
  return (-2 * t + 3) * t * t;  // -2t**3 + 3t**2
}

PIDJoint::PIDJoint(const JointHandle& handle,
                   const urdf::Joint *const urdf_joint,
                   const NodeHandle& pid_ctrlr_nh,
                   const bool pid_ctrlr_required) :
  Joint(handle), type_(urdf_joint->type),
  lower_limit_(urdf_joint->limits->lower),
  upper_limit_(urdf_joint->limits->upper)
{
  if (!pid_ctrlr_.init(pid_ctrlr_nh))
  {
    if (pid_ctrlr_required)
    {
      throw runtime_error("No PID gain values for \"" + handle_.getName() +
                          "\" were found.");
    }
  }
}

void PIDJoint::init()
{
  pid_ctrlr_.reset();
}

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

void PIDJoint::setVel(const double vel, const Duration& period)
{
  const double error = vel - handle_.getVelocity();
  handle_.setCommand(pid_ctrlr_.computeCommand(error, period));
}

void PosJoint::init()
{
  pos_ = handle_.getPosition();
}

void PosJoint::setPos(const double pos, const Duration& /* period */)
{
  pos_ = pos;
  handle_.setCommand(pos_);
}

void PosJoint::setVel(const double vel, const Duration& period)
{
  pos_ += vel * period.toSec();
  handle_.setCommand(pos_);
}

void VelJoint::setVel(const double vel, const Duration& /* period */)
{
  handle_.setCommand(vel);
}

Wheel::Wheel(const double circ, const char *const steer_axle_joint_name,
             Joint *const steer_joint, Joint *const axle_joint) :
  steer_axle_joint_name_(steer_axle_joint_name),
  steer_joint_(steer_joint), axle_joint_(axle_joint)
{
  steer_axle_joint_name_ = steer_axle_joint_name;
  axle_vel_gain_ = 0;
  lin_to_ang_ = (2 * M_PI) / circ;
}

void Wheel::initPos(const tf::TransformListener& tfl,
                    const char *const base_frame)
{
  while (true)
  {
    try
    {
      tf::StampedTransform trans;
      tfl.lookupTransform(base_frame, steer_axle_joint_name_, ros::Time(0),
                          trans);
      pos_ = Vec2(trans.getOrigin().x(), trans.getOrigin().y());
      break;
    }
    catch (...)
    {
      // Do nothing.
    }
  }
}

void Wheel::initJoints()
{
  steer_joint_->init();
  axle_joint_->init();
}

// Control this wheel's steering joint. theta_desired range: [-pi, pi].
void Wheel::ctrlSteering(const double theta_desired, const Duration& period)
{
  // Find the minimum rotation that will align the wheel with theta_desired.
  const double theta_measured = steer_joint_->getPos();
  const double theta_diff = fabs(theta_desired - theta_measured);
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
  if (theta < THETA_MIN || theta > THETA_MAX)
  {
    theta -= copysign(M_PI, theta);
    axle_vel_gain_ = -axle_vel_gain_;
  }

  steer_joint_->setPos(theta, period);
  // \todo Static const for MI_PI / 8?
  axle_vel_gain_ *= 1 - hermite(fabs(theta - theta_measured) / (M_PI / 8));
}

// Control this wheel's axle joint.
void Wheel::ctrlAxle(const double lin_speed, const Duration& period) const
{
  const double ang_vel = axle_vel_gain_ * lin_to_ang_ * lin_speed;
  axle_joint_->setVel(ang_vel, period);
}

void addClaimedResources(hardware_interface::HardwareInterface *const hw_iface,
                         set<string>& claimed_resources)
{
  if (hw_iface == NULL)
    return;
  const set<string> claims = hw_iface->getClaims();
  claimed_resources.insert(claims.begin(), claims.end());
  hw_iface->clearClaims();
}

Joint *getJoint(const string& joint_name,
                const NodeHandle& ctrlr_nh, const urdf::Model& urdf_model,
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
      boost::shared_ptr<const urdf::Joint> urdf_joint =
        urdf_model.getJoint(joint_name);
      if (urdf_joint == NULL)
      {
        throw runtime_error("\"" + joint_name +
                            "\" was not found in the URDF data.");
      }

      const NodeHandle pid_ctrlr_nh(ctrlr_nh, "pid");
      return new EffJoint(handle, urdf_joint.get(), pid_ctrlr_nh);
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
      return new PosJoint(handle);
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
      // \todo scoped_ptr?
      boost::shared_ptr<const urdf::Joint> urdf_joint =
        urdf_model.getJoint(joint_name);
      if (urdf_joint == NULL)
      {
        throw runtime_error("\"" + joint_name +
                            "\" was not found in the URDF data.");
      }

      const NodeHandle pid_ctrlr_nh(ctrlr_nh, "pid");
      return new VelJoint(handle, urdf_joint.get(), pid_ctrlr_nh);
    }
  }

  throw runtime_error("No handle for \"" + joint_name + "\" was found.");
}

double enforceLimits(const double desired_vel,
                     const double delta_t, const double inv_delta_t,
                     const bool speed_limit_exists, const double speed_limit,
                     const bool accel_limit_exists, const double accel_limit,
                     const bool decel_limit_exists, const double decel_limit,
                     double *const last_vel)
{
  double vel = desired_vel;
  if (speed_limit_exists)
    vel = std::min(std::max(vel, -speed_limit), speed_limit);

  const double last_vel_2 = *last_vel;
  double accel = (vel - last_vel_2) * inv_delta_t;

  const double accel_sign = boost::math::sign(accel);
  if (boost::math::sign(last_vel_2) == accel_sign)
  {
    // Acceleration

    if (accel_limit_exists && fabs(accel) > accel_limit)
    {
      accel = accel_sign * accel_limit;
      vel = last_vel_2 + accel * delta_t;
    }
  }
  else
  {
    // Deceleration

    if (decel_limit_exists && fabs(accel) > decel_limit)
    {
      accel = accel_sign * decel_limit;
      vel = last_vel_2 + accel * delta_t;
    }
  }

  *last_vel = vel;
  return vel;
}

Vec2 enforceLimits(const Vec2& desired_vel,
                   const double delta_t, const double inv_delta_t,
                   const bool speed_limit_exists, const double speed_limit,
                   const bool accel_limit_exists, const double accel_limit,
                   const bool decel_limit_exists, const double decel_limit,
                   Vec2 *const last_vel)
{
  Vec2 vel = desired_vel;
  if (speed_limit_exists)
  {
    const double vel_mag = vel.magnitude();
    if (vel_mag > speed_limit)
      vel = (vel / vel_mag) * speed_limit;
  }

  const Vec2& last_vel_2 = *last_vel;
  Vec2 accel = (vel - last_vel_2) * inv_delta_t;

  const double vel_dot_accel = last_vel_2.dot_prod(accel);
  if (vel_dot_accel >= 0)
  {
    // Acceleration

    if (accel_limit_exists)
    {
      const double accel_mag = accel.magnitude();
      if (accel_mag > accel_limit)
      {
        accel = (accel / accel_mag) * accel_limit;
        vel = last_vel_2 + accel * delta_t;
      }
    }
  }
  else
  {
    // Deceleration

    if (decel_limit_exists)
    {
      const double accel_mag = accel.magnitude();
      if (accel_mag > decel_limit)
      {
        accel = (accel / accel_mag) * decel_limit;
        vel = last_vel_2 + accel * delta_t;
      }
    }
  }

  *last_vel = vel;
  return vel;
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
  };

  static const double DEF_LIN_SPEED_LIMIT;
  static const double DEF_LIN_ACCEL_LIMIT;
  static const double DEF_LIN_DECEL_LIMIT;

  static const double DEF_YAW_SPEED_LIMIT;
  static const double DEF_YAW_ACCEL_LIMIT;
  static const double DEF_YAW_DECEL_LIMIT;

  static const Vec2 X_DIR;  // X direction

  void init(EffortJointInterface *const eff_joint_iface,
            PositionJointInterface *const pos_joint_iface,
            VelocityJointInterface *const vel_joint_iface,
            NodeHandle& ctrlr_nh);
  void velCmdCB(const TwistConstPtr& vel_cmd);

  void ctrlWheels(const Vec2& lin_vel, const double yaw_vel,
                  const Duration& period);

  bool wheel_pos_initted_;  // Wheel positions initialized
  vector<Wheel> wheels_;

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

  VelCmd vel_cmd_;                      // Velocity command
  RealtimeBuffer<VelCmd> vel_cmd_buf_;  // Velocity command buffer
  ros::Subscriber vel_cmd_sub_;         // Velocity command subscriber

  Vec2 last_lin_vel_;   // Last linear velocity. Unit: m/s.
  double last_yaw_vel_; // Last yaw velocity. Unit: rad/s.
};

const double SteeredWheelBaseController::DEF_LIN_SPEED_LIMIT = 1;
const double SteeredWheelBaseController::DEF_LIN_ACCEL_LIMIT = 1;
const double SteeredWheelBaseController::DEF_LIN_DECEL_LIMIT = -1;

const double SteeredWheelBaseController::DEF_YAW_SPEED_LIMIT = 1;
const double SteeredWheelBaseController::DEF_YAW_ACCEL_LIMIT = 1;
const double SteeredWheelBaseController::DEF_YAW_DECEL_LIMIT = -1;

const Vec2 SteeredWheelBaseController::X_DIR(1, 0);

SteeredWheelBaseController::SteeredWheelBaseController()
{
  state_ = CONSTRUCTED;
  wheel_pos_initted_ = false;
}

bool SteeredWheelBaseController::initRequest(RobotHW *const robot_hw,
                                             NodeHandle& /* root_nh */,
                                             NodeHandle& ctrlr_nh,
                                             set<string>& claimed_resources)
{
  if (state_ != CONSTRUCTED)
  {
    ROS_ERROR("todo");
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
  vel_cmd_.x_vel = 0;
  vel_cmd_.y_vel = 0;
  vel_cmd_.yaw_vel = 0;
  vel_cmd_buf_.initRT(vel_cmd_);

  last_lin_vel_ = Vec2(0, 0);
  last_yaw_vel_ = 0;

  BOOST_FOREACH(Wheel& wheel, wheels_)
    wheel.initJoints();
}

void SteeredWheelBaseController::update(const Time& time,
                                        const Duration& period)
{
  if (!wheel_pos_initted_)
    return;
  const double delta_t = period.toSec();
  if (delta_t <= 0)
    return;

  vel_cmd_ = *(vel_cmd_buf_.readFromRT());
  const Vec2 lin_vel(vel_cmd_.x_vel, vel_cmd_.y_vel);
  const double inv_delta_t = 1 / delta_t;

  const Vec2 lin_vel_2 =
    enforceLimits(lin_vel, delta_t, inv_delta_t,
                  has_lin_speed_limit_, lin_speed_limit_,
                  has_lin_accel_limit_, lin_accel_limit_,
                  has_lin_decel_limit_, lin_decel_limit_,
                  &last_lin_vel_);
  const double yaw_vel =
    enforceLimits(vel_cmd_.yaw_vel, delta_t, inv_delta_t,
                  has_yaw_speed_limit_, yaw_speed_limit_,
                  has_yaw_accel_limit_, yaw_accel_limit_,
                  has_yaw_decel_limit_, yaw_decel_limit_,
                  &last_yaw_vel_);

  ctrlWheels(lin_vel_2, yaw_vel, period);
}

void SteeredWheelBaseController::
init(EffortJointInterface *const eff_joint_iface,
     PositionJointInterface *const pos_joint_iface,
     VelocityJointInterface *const vel_joint_iface,
     NodeHandle& ctrlr_nh)
{
  // \todo

  string front_steer_joint_name;
  if (!ctrlr_nh.getParam("front_steering_joint_name", front_steer_joint_name))
    throw runtime_error("Steering joint name not found.");
  string front_axle_joint_name;
  if (!ctrlr_nh.getParam("front_axle_joint_name", front_axle_joint_name))
    throw runtime_error("Axle joint name not found.");

  string left_steer_joint_name;
  if (!ctrlr_nh.getParam("left_steering_joint_name", left_steer_joint_name))
    throw runtime_error("Steering joint name not found.");
  string left_axle_joint_name;
  if (!ctrlr_nh.getParam("left_axle_joint_name", left_axle_joint_name))
    throw runtime_error("Axle joint name not found.");

  string right_steer_joint_name;
  if (!ctrlr_nh.getParam("right_steering_joint_name", right_steer_joint_name))
    throw runtime_error("Steering joint name not found.");
  string right_axle_joint_name;
  if (!ctrlr_nh.getParam("right_axle_joint_name", right_axle_joint_name))
    throw runtime_error("Axle joint name not found.");

  urdf::Model urdf_model;
  // \todo Get the robot description string from a parameter.
  if (!urdf_model.initParam("robot_description"))
    throw runtime_error("The URDF data was not found.");

  const double circ = (2 * M_PI) * 0.254 / 2; // \todo
  wheels_.push_back(Wheel(circ, "front_steer_axle",
                          getJoint(front_steer_joint_name,
                                   ctrlr_nh, urdf_model,
                                   eff_joint_iface, pos_joint_iface,
                                   vel_joint_iface),
                          getJoint(front_axle_joint_name,
                                   ctrlr_nh, urdf_model,
                                   eff_joint_iface, pos_joint_iface,
                                   vel_joint_iface)));
  wheels_.push_back(Wheel(circ, "left_steer_axle",
                          getJoint(left_steer_joint_name,
                                   ctrlr_nh, urdf_model,
                                   eff_joint_iface, pos_joint_iface,
                                   vel_joint_iface),
                          getJoint(left_axle_joint_name,
                                   ctrlr_nh, urdf_model,
                                   eff_joint_iface, pos_joint_iface,
                                   vel_joint_iface)));
  wheels_.push_back(Wheel(circ, "right_steer_axle",
                          getJoint(right_steer_joint_name,
                                   ctrlr_nh, urdf_model,
                                   eff_joint_iface, pos_joint_iface,
                                   vel_joint_iface),
                          getJoint(right_axle_joint_name,
                                   ctrlr_nh, urdf_model,
                                   eff_joint_iface, pos_joint_iface,
                                   vel_joint_iface)));

  ctrlr_nh.param("linear_speed_limit", lin_speed_limit_, DEF_LIN_SPEED_LIMIT);
  has_lin_speed_limit_ = lin_speed_limit_ >= 0;
  ctrlr_nh.param("linear_acceleration_limit", lin_accel_limit_,
                 DEF_LIN_ACCEL_LIMIT);
  has_lin_accel_limit_ = lin_accel_limit_ >= 0;
  ctrlr_nh.param("linear_deceleration_limit", lin_decel_limit_,
                 DEF_LIN_DECEL_LIMIT);
  // For safety, the deceleration limit must be greater than zero.
  has_lin_decel_limit_ = lin_decel_limit_ > 0;

  ctrlr_nh.param("yaw_speed_limit", yaw_speed_limit_, DEF_YAW_SPEED_LIMIT);
  has_yaw_speed_limit_ = yaw_speed_limit_ >= 0;
  ctrlr_nh.param("yaw_acceleration_limit", yaw_accel_limit_,
                 DEF_YAW_ACCEL_LIMIT);
  has_yaw_accel_limit_ = yaw_accel_limit_ >= 0;
  ctrlr_nh.param("yaw_deceleration_limit", yaw_decel_limit_,
                 DEF_YAW_DECEL_LIMIT);
  // For safety, the deceleration limit must be greater than zero.
  has_yaw_decel_limit_ = yaw_decel_limit_ > 0;

  vel_cmd_sub_ = ctrlr_nh.subscribe("cmd_vel", 1,
                                    &SteeredWheelBaseController::velCmdCB,
                                    this);
}

// Velocity command callback
void SteeredWheelBaseController::velCmdCB(const TwistConstPtr& vel_cmd)
{
  if (!wheel_pos_initted_)
  {
    tf::TransformListener tfl;
    BOOST_FOREACH(Wheel& wheel, wheels_)
      wheel.initPos(tfl, "base_link");  // \todo Use parameter.
    wheel_pos_initted_ = true;
  }

  vel_cmd_.x_vel = vel_cmd->linear.x;
  vel_cmd_.y_vel = vel_cmd->linear.y;
  vel_cmd_.yaw_vel = vel_cmd->angular.z;
  vel_cmd_buf_.writeFromNonRT(vel_cmd_);
}

// Control the wheels.
void SteeredWheelBaseController::ctrlWheels(const Vec2& lin_vel,
                                            const double yaw_vel,
                                            const Duration& period)
{
  const double lin_speed = lin_vel.magnitude();

  if (yaw_vel == 0)
  {
    if (lin_speed > 0)
    {
      // Point the wheels in the same direction.
      const Vec2 dir = lin_vel / lin_speed;
      const double theta =
        copysign(acos(dir.dot_prod(SteeredWheelBaseController::X_DIR)),
                 dir.y());
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
        wheel.ctrlAxle(0, period);
    }
  }
  else  // The yaw velocity is nonzero.
  {
    // Align the wheels so that they are tangent to circles centered
    // at "center".

    Vec2 center;
    if (lin_speed > 0)
    {
      const Vec2 dir = lin_vel / lin_speed;
      center.x(-dir.y());
      center.y(dir.x());
      center *= lin_speed / yaw_vel;
    }
    else
    {
      center.x(0);
      center.y(0);
    }

    BOOST_FOREACH(Wheel& wheel, wheels_)
    {
      Vec2 vec = wheel.getPos();
      vec -= center;
      const double radius = vec.magnitude();
      double theta;
      if (radius > 0)
      {
        vec /= radius;
        theta =
          copysign(acos(vec.dot_prod(SteeredWheelBaseController::X_DIR)),
                   vec.y()) + M_PI_2;
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

} // namespace steered_wheel_base_controller

PLUGINLIB_EXPORT_CLASS(steered_wheel_base_controller::\
SteeredWheelBaseController, controller_interface::ControllerBase)
