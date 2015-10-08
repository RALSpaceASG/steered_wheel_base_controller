#ifndef _SWBC_JOINT_BASE_H_
#define _SWBC_JOINT_BASE_H_

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <urdf/model.h>

namespace SWBC
{
	namespace JointTypes
	{
		class JointBase
		{
			public:
				virtual ~JointBase() {}
				virtual void init() = 0;
				virtual void stop() = 0;

				bool isValidPos(const double pos) const;
				double getPos() const {return handle_.getPosition();}

				virtual void setPos(const double pos, const ros::Duration& period) {}
				virtual void setVel(const double vel, const ros::Duration& period) = 0;

			protected:
				JointBase(const hardware_interface::JointHandle& handle,
				const boost::shared_ptr<const urdf::Joint> urdf_joint);

				hardware_interface::JointHandle handle_;
				const bool is_continuous_;
				const double lower_limit_, upper_limit_;  // Unit: radian
		};
	}
}

#endif
