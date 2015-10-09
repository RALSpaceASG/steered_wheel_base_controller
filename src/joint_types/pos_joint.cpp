#include "steered_wheel_base_controller/joint_types/pos_joint.h"

using ros::Duration;

namespace SWBC
{
	namespace joint_types
	{	
		// Initialize this joint.
		void PosJoint::init()
		{
			pos_ = getPos();
			stop();
		}

		// Stop this joint's motion.
		void PosJoint::stop()
		{
			handle_.setCommand(getPos());
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
	}
}
