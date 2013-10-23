// steered_wheel_base_controller.cpp
//
// This file contains the source code for steered_wheel_base_controller,
// a base controller for mobile robots. It works with bases that have one or
// more independently-steerable driven wheels and zero or more omnidirectional
// passive wheels (e.g. swivel casters).
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
//     ~linear_deceleration_limit (float, default: 1.0)
//         \todo
//     ~linear_jerk_limit (float, default: 0.0)
//         \todo
//
//     ~angular_speed_limit (float, default: 1.0)
//         \todo. Unit: rad/s.
//     ~angular_acceleration_limit (float, default: 1.0)
//         \todo
//     ~angular_deceleration_limit (float, default: 1.0)
//         \todo
//     ~angular_jerk_limit (float, default: 0.0)
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
#include <control_toolbox/pid.h>
#include <controller_interface/controller_base.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>

using std::auto_ptr;
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

using ros::Duration;
using ros::NodeHandle;
using ros::Time;

namespace
{

class Joint
{
public:
  virtual ~Joint() {}

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
  PosJoint(const JointHandle& handle);

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

PosJoint::PosJoint(const JointHandle& handle) : Joint(handle)
{
  pos_ = std::numeric_limits<double>::quiet_NaN();
}

void PosJoint::setPos(const double pos, const Duration& /* period */)
{
  pos_ = pos;
  handle_.setCommand(pos_);
}

void PosJoint::setVel(const double vel, const Duration& period)
{
  if (std::isnan(pos_))
    pos_ = handle_.getPosition();
  pos_ += vel * period.toSec();
  handle_.setCommand(pos_);
}

void VelJoint::setVel(const double vel, const Duration& /* period */)
{
  handle_.setCommand(vel);
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

auto_ptr<Joint> getJoint(const string& joint_name,
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
      boost::shared_ptr<const urdf::Joint> urdf_joint =
        urdf_model.getJoint(joint_name);
      if (urdf_joint == NULL)
      {
        throw runtime_error("\"" + joint_name +
                            "\" was not found in the URDF data.");
      }

      const NodeHandle pid_ctrlr_nh(ctrlr_nh, "pid");
      return auto_ptr<Joint>(new EffJoint(handle, urdf_joint.get(),
                                          pid_ctrlr_nh));
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
      return auto_ptr<Joint>(new PosJoint(handle));
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
      boost::shared_ptr<const urdf::Joint> urdf_joint =
        urdf_model.getJoint(joint_name);
      if (urdf_joint == NULL)
      {
        throw runtime_error("\"" + joint_name +
                            "\" was not found in the URDF data.");
      }

      const NodeHandle pid_ctrlr_nh(ctrlr_nh, "pid");
      return auto_ptr<Joint>(new VelJoint(handle, urdf_joint.get(),
                                          pid_ctrlr_nh));
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
  SteeredWheelBaseController() {state_ = CONSTRUCTED;}

  // These are not real-time safe.
  virtual bool initRequest(RobotHW *const robot_hw,
                           NodeHandle& root_nh, NodeHandle& ctrlr_nh,
                           set<string>& claimed_resources);
  virtual string getHardwareInterfaceType() const;

  // These are real-time safe.
  virtual void starting(const Time& time);
  virtual void update(const Time& time, const Duration& period);

private:
  void init(EffortJointInterface *const eff_joint_iface,
            PositionJointInterface *const pos_joint_iface,
            VelocityJointInterface *const vel_joint_iface,
            NodeHandle& ctrlr_nh);
  void velCmdCB(const TwistConstPtr& vel_cmd);

  // \todo
  auto_ptr<Joint> front_steer_joint_, front_axle_joint_;
  auto_ptr<Joint> left_steer_joint_, left_axle_joint_;
  auto_ptr<Joint> right_steer_joint_, right_axle_joint_;

  ros::Subscriber vel_cmd_sub_; // Velocity command subscriber
};

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
  // \todo
#ifdef NOTDEF
  desired_pos_ = joint_.getPosition();
  pid_ctrlr_.reset();
#endif
}

void SteeredWheelBaseController::update(const Time& time,
                                        const Duration& period)
{
  // Control steering angles.
  // \todo
  // Control axle velocities.
  // \todo

  // Send joint commands to the hardware.

  // \todo
  front_steer_joint_->setPos(0, period);
  left_steer_joint_->setPos(0, period);
  right_steer_joint_->setPos(0, period);
#ifndef NOTDEF
  front_axle_joint_->setVel(16.6821, period);
  left_axle_joint_->setVel(16.6821, period);
  right_axle_joint_->setVel(16.6821, period);
#else
  front_axle_joint_->setVel(0, period);
  left_axle_joint_->setVel(0, period);
  right_axle_joint_->setVel(0, period);
#endif

#ifdef NOTDEF
  for (vector<VelJoint>::const_iterator joint = vel_joints_.begin(); joint != vel_joints_.end();
       joint++)
  {
    joint->setVel(3.14);
  }
#endif
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

  front_steer_joint_ = getJoint(front_steer_joint_name, ctrlr_nh, urdf_model,
                                eff_joint_iface, pos_joint_iface,
                                vel_joint_iface);
  front_axle_joint_ = getJoint(front_axle_joint_name, ctrlr_nh, urdf_model,
                               eff_joint_iface, pos_joint_iface,
                               vel_joint_iface);

  left_steer_joint_ = getJoint(left_steer_joint_name, ctrlr_nh, urdf_model,
                               eff_joint_iface, pos_joint_iface,
                               vel_joint_iface);
  left_axle_joint_ = getJoint(left_axle_joint_name, ctrlr_nh, urdf_model,
                              eff_joint_iface, pos_joint_iface,
                              vel_joint_iface);

  right_steer_joint_ = getJoint(right_steer_joint_name, ctrlr_nh, urdf_model,
                                eff_joint_iface, pos_joint_iface,
                                vel_joint_iface);
  right_axle_joint_ = getJoint(right_axle_joint_name, ctrlr_nh, urdf_model,
                               eff_joint_iface, pos_joint_iface,
                               vel_joint_iface);

  vel_cmd_sub_ = ctrlr_nh.subscribe("cmd_vel", 1,
                                    &SteeredWheelBaseController::velCmdCB,
                                    this);
}

// Velocity command callback
void SteeredWheelBaseController::velCmdCB(const TwistConstPtr& vel_cmd)
{
  // \todo
#ifdef NOTDEF
  desired_pos_ = vel_cmd->data;
#endif
}

} // namespace steered_wheel_base_controller

PLUGINLIB_EXPORT_CLASS(steered_wheel_base_controller::\
SteeredWheelBaseController, controller_interface::ControllerBase)
