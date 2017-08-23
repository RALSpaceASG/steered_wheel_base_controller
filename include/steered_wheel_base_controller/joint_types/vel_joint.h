#ifndef _SWBC_JOINT_TYPES_VEL_JOINT_H_
#define _SWBC_JOINT_TYPES_VEL_JOINT_H_

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <urdf/model.h>

#include "steered_wheel_base_controller/joint_base.h"

namespace SWBC
{
	namespace joint_types
	{	
		// Velocity-controlled joint. VelJoint is used for axles only.
		class VelJoint : public JointBase
		{
			public:
				VelJoint(const hardware_interface::JointHandle& handle,
				urdf::JointConstSharedPtr urdf_joint)
					: JointBase(handle, urdf_joint) {}

				virtual void init() {stop();}
				virtual void stop();
				virtual void setVel(const double vel, const ros::Duration& period);
		};
	}
}

#endif
