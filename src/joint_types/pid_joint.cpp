#include "steered_wheel_base_controller/joint_types/pid_joint.h"

using ros::Duration;
using ros::NodeHandle;
using std::shared_ptr;
using hardware_interface::JointHandle;
using std::runtime_error;

namespace SWBC
{
	namespace joint_types
	{	
		PIDJoint::PIDJoint(	const JointHandle& handle,
							urdf::JointConstSharedPtr urdf_joint,
							const NodeHandle& ctrlr_nh) :
								JointBase(handle, urdf_joint), type_(urdf_joint->type)
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
			stop();
		}

		// Stop this joint's motion.
		void PIDJoint::stop()
		{
			// The command passed to setCommand() might be an effort value or a
			// velocity. In either case, the correct command to pass here is zero.
			handle_.setCommand(0);
		}

		// Specify this joint's position.
		void PIDJoint::setPos(const double pos, const Duration& period)
		{
			const double curr_pos = getPos();

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
	}
}
