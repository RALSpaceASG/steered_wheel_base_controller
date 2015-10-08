#ifndef _SWBC_PID_JOINT_H_
#define _SWBC_PID_JOINT_H_

#include <angles/angles.h>
#include <boost/shared_ptr.hpp>
#include <control_toolbox/pid.h>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <urdf/model.h>

#include "steered_wheel_base_controller/joint_base.h"

namespace SWBC
{
	namespace joint_types
	{	
		// An object of class PIDJoint is a joint controlled by a PID controller.
		class PIDJoint : public JointBase
		{
			public:
				PIDJoint(const hardware_interface::JointHandle& handle,
				const boost::shared_ptr<const urdf::Joint> urdf_joint,
				const ros::NodeHandle& ctrlr_nh);

				virtual void init();
				virtual void stop();
				virtual void setPos(const double pos, const ros::Duration& period);
				virtual void setVel(const double vel, const ros::Duration& period);

			private:
				const int type_;  // URDF joint type
				control_toolbox::Pid pid_ctrlr_;
		};
	}
}

#endif
