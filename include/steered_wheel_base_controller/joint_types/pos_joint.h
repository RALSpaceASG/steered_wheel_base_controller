#ifndef _SWBC_JOINT_TYPES_POS_JOINT_H_
#define _SWBC_JOINT_TYPES_POS_JOINT_H_

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <urdf/model.h>

#include "steered_wheel_base_controller/joint_base.h"

namespace SWBC
{
	namespace joint_types
	{	
		// Position-controlled joint 
		class PosJoint : public JointBase
		{
			public:
				PosJoint(const hardware_interface::JointHandle& handle,
				const std::shared_ptr<const urdf::Joint> urdf_joint) :
				JointBase(handle, urdf_joint) {}

				virtual void init();
				virtual void stop();
				virtual void setPos(const double pos, const ros::Duration& period);
				virtual void setVel(const double vel, const ros::Duration& period);

			private:
				double pos_;
		};
	}
}

#endif
