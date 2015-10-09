#include "steered_wheel_base_controller/joint_base.h"

using hardware_interface::JointHandle;
using boost::shared_ptr;

namespace SWBC
{
	namespace joint_types
	{		
		JointBase::JointBase(const JointHandle& handle, const shared_ptr<const urdf::Joint> urdf_joint) 
				: handle_(handle), is_continuous_(urdf_joint->type == urdf::Joint::CONTINUOUS),
				lower_limit_(urdf_joint->limits->lower),
				upper_limit_(urdf_joint->limits->upper)
		{
		// Do nothing.
		}

		bool JointBase::isValidPos(const double pos) const
		{
			if (is_continuous_)
				return true;
			return pos >= lower_limit_ && pos <= upper_limit_;
		}
	}
}
