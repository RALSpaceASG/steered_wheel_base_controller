#ifndef _SWBC_UTIL_H_
#define _SWBC_UTIL_H_

#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>

#include "steered_wheel_base_controller/joint_base.h"
#include "steered_wheel_base_controller/joint_types/pos_joint.h"
#include "steered_wheel_base_controller/joint_types/vel_joint.h"
#include "steered_wheel_base_controller/joint_types/pid_joint.h"

namespace SWBC
{
	namespace util
	{
		void addClaimedResources(	hardware_interface::HardwareInterface *const hw_iface,
									std::set<std::string>& claimed_resources);									
		double clamp(const double val, const double min_val, const double max_val);
		boost::shared_ptr<joint_types::JointBase> getJoint(	const std::string& joint_name, const bool is_steer_joint,
										const ros::NodeHandle& ctrlr_nh,
										const urdf::Model& urdf_model,
										hardware_interface::EffortJointInterface *const eff_joint_iface,
										hardware_interface::PositionJointInterface *const pos_joint_iface,
										hardware_interface::VelocityJointInterface *const vel_joint_iface);
		
	}	
}

#endif
