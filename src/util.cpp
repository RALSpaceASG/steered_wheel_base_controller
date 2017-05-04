#include "steered_wheel_base_controller/util.h"

using boost::shared_ptr;
using hardware_interface::EffortJointInterface;
using hardware_interface::PositionJointInterface;
using hardware_interface::VelocityJointInterface;
using hardware_interface::JointHandle;
using ros::NodeHandle;
using std::set;
using std::string;
using std::runtime_error;
using std::min;
using std::max;
using namespace SWBC::joint_types;

namespace SWBC
{
	namespace util
	{
		double clamp(const double val, const double min_val, const double max_val)
		{
			return min(max(val, min_val), max_val);
		}

		// Create an object of class Joint that corresponds to the URDF joint specified
		// by joint_name.
		shared_ptr<JointBase> getJoint(	const string& joint_name, const bool is_steer_joint,
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
					const shared_ptr<const urdf::Joint> urdf_joint =
					urdf_model.getJoint(joint_name);
					if (urdf_joint == NULL)
					{
						throw runtime_error("\"" + joint_name +
						"\" was not found in the URDF data.");
					}

					shared_ptr<JointBase> joint(new PIDJoint(handle, urdf_joint, ctrlr_nh));
					return joint;
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
				{
					const shared_ptr<const urdf::Joint> urdf_joint =
					urdf_model.getJoint(joint_name);
					if (urdf_joint == NULL)
					{
						throw runtime_error("\"" + joint_name +
						"\" was not found in the URDF data.");
					}

					shared_ptr<JointBase> joint(new PosJoint(handle, urdf_joint));
					return joint;
				}
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
					const shared_ptr<const urdf::Joint> urdf_joint =
					urdf_model.getJoint(joint_name);
					if (urdf_joint == NULL)
					{
						throw runtime_error("\"" + joint_name +
						"\" was not found in the URDF data.");
					}

					if (is_steer_joint)
					{
						shared_ptr<JointBase> joint(new PIDJoint(handle, urdf_joint, ctrlr_nh));
						return joint;
					}
					shared_ptr<JointBase> joint(new VelJoint(handle, urdf_joint));
					return joint;
				}
			}

			throw runtime_error("No handle for \"" + joint_name + "\" was found.");
		}
	}
}
