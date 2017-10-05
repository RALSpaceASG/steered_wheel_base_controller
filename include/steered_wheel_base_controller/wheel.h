#ifndef _SWBC_WHEEL_H_
#define _SWBC_WHEEL_H_

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "steered_wheel_base_controller/joint_base.h"

namespace SWBC
{
	// An object of class Wheel is a steered wheel.
	class Wheel
	{
		public:
			Wheel(const KDL::Tree& tree,
				  const std::string& base_link, const std::string& steer_link,
				  const std::shared_ptr<joint_types::JointBase> steer_joint,
				  const std::shared_ptr<joint_types::JointBase> axle_joint,
				  const double circ);

			const Eigen::Vector2d& pos() const {return pos_;}
			Eigen::Vector2d getDeltaPos();

			void initJoints();
			void stop() const;
			double ctrlSteering(const ros::Duration& period, const double hermite_scale,
								const double hermite_offset);
			double ctrlSteering(const double theta_desired, const ros::Duration& period,
								const double hermite_scale, const double hermite_offset);
			void ctrlAxle(const double lin_speed, const ros::Duration& period) const;

		private:
			void initPos(const KDL::Tree& tree, const std::string& base_link);
			double hermite(const double t);

			std::string steer_link_; // Steering link
			Eigen::Vector2d pos_;      // Wheel's position in the base link's frame

			std::shared_ptr<joint_types::JointBase> steer_joint_;   // Steering joint
			std::shared_ptr<joint_types::JointBase> axle_joint_;
			double theta_steer_;              // Steering joint position
			double last_theta_steer_desired_; // Last desired steering joint position
			double last_theta_axle_;          // Last axle joint position

			double radius_;         // Unit: meter.
			double inv_radius_;     // Inverse of radius_
			double axle_vel_gain_;  // Axle velocity gain
	};
}

#endif
