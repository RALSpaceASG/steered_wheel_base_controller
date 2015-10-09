#include "steered_wheel_base_controller/joint_types/vel_joint.h"

using ros::Duration;

namespace SWBC
{
	namespace joint_types
	{	
		// Stop this joint's motion.
		void VelJoint::stop()
		{
			handle_.setCommand(0);
		}

		// Specify this joint's velocity.
		void VelJoint::setVel(const double vel, const Duration& /* period */)
		{
			handle_.setCommand(vel);
		}
	}
}
