#include "steered_wheel_base_controller/wheel.h"

using boost::shared_ptr;
using Eigen::Vector2d;
using ros::Duration;
using std::string;
using std::runtime_error;
using SWBC::joint_types::JointBase;

namespace SWBC
{
	Wheel::Wheel(	const KDL::Tree& tree,
					const string& base_link, const string& steer_link,
					const shared_ptr<JointBase> steer_joint,
					const shared_ptr<JointBase> axle_joint,
					const double circ)
	{
		steer_link_ = steer_link;
		initPos(tree, base_link);

		steer_joint_ = steer_joint;
		axle_joint_ = axle_joint;
		theta_steer_ = steer_joint_->getPos();
		last_theta_steer_desired_ = theta_steer_;
		last_theta_axle_ = axle_joint_->getPos();

		radius_ = circ / (2 * M_PI);
		inv_radius_ = 1 / radius_;
		axle_vel_gain_ = 0;
	}

	// Return the difference between this wheel's current position and its
	// position when getDeltaPos() was last called. The returned vector is defined
	// in the base link's frame.
	Vector2d Wheel::getDeltaPos()
	{
		const double theta_axle = axle_joint_->getPos();
		const double delta_theta_axle = theta_axle - last_theta_axle_;
		last_theta_axle_ = theta_axle;
		const double vec_mag = delta_theta_axle * radius_;
		return Vector2d(cos(theta_steer_), sin(theta_steer_)) * vec_mag;
	}

	// Initialize this wheel's steering and axle joints.
	void Wheel::initJoints()
	{
		steer_joint_->init();
		axle_joint_->init();
	}

	// Stop this wheel's motion.
	void Wheel::stop() const
	{
		steer_joint_->stop();
		axle_joint_->stop();
	}

	// Maintain the position of this wheel's steering joint. Return a linear speed
	// gain value based on the difference between the desired steering angle and
	// the actual steering angle.
	double Wheel::ctrlSteering(	const Duration& period, const double hermite_scale,
								const double hermite_offset)
	{
		return ctrlSteering(last_theta_steer_desired_, period, hermite_scale,
		hermite_offset);
	}

	// Control this wheel's steering joint. theta_desired range: [-pi, pi].
	// Return a linear speed gain value based on the difference between the
	// desired steering angle and the actual steering angle.
	double Wheel::ctrlSteering(	const double theta_desired, const Duration& period,
								const double hermite_scale,
								const double hermite_offset)
	{
		last_theta_steer_desired_ = theta_desired;

		// Find the minimum rotation that will align the wheel with theta_desired.
		theta_steer_ = steer_joint_->getPos();
		const double theta_diff = fabs(theta_desired - theta_steer_);
		double theta;
		if (theta_diff > M_PI_2)
		{
			theta = theta_desired - copysign(M_PI, theta_desired);
			axle_vel_gain_ = -1;
		}
		else
		{
			theta = theta_desired;
			axle_vel_gain_ = 1;
		}

		// Keep theta within its valid range.
		if (!steer_joint_->isValidPos(theta))
		{
			theta -= copysign(M_PI, theta);
			axle_vel_gain_ = -axle_vel_gain_;
		}

		steer_joint_->setPos(theta, period);
		return 1 - hermite(hermite_scale * (fabs(theta - theta_steer_) - hermite_offset));
	}

	// Control this wheel's axle joint.
	void Wheel::ctrlAxle(const double lin_speed, const Duration& period) const
	{
		const double ang_vel = axle_vel_gain_ * inv_radius_ * lin_speed;
		axle_joint_->setVel(ang_vel, period);
	}

	// Initialize pos_.
	void Wheel::initPos(const KDL::Tree& tree, const string& base_link)
	{
		KDL::Chain chain;
		if (!tree.getChain(base_link, steer_link_, chain))
		{
			throw runtime_error("No kinematic chain was found from \"" + base_link +
			"\" to \"" + steer_link_ + "\".");
		}

		const unsigned int num_joints = chain.getNrOfJoints();
		KDL::JntArray joint_positions(num_joints);
		for (unsigned int i = 0; i < num_joints; i++)
			joint_positions(i) = 0;

		KDL::ChainFkSolverPos_recursive solver(chain);
		KDL::Frame frame;
		if (solver.JntToCart(joint_positions, frame) < 0)
		{
			throw runtime_error("The position of steering link \"" + steer_link_ +
			"\" in base link frame \"" + base_link +
			"\" was not found.");
		}
		pos_ = Vector2d(frame.p.x(), frame.p.y());
	}
	
	double Wheel::hermite(const double t)
	{
		if (t <= 0)
			return 0;
		if (t >= 1)
			return 1;
		return (-2 * t + 3) * t * t;  // -2t**3 + 3t**2
	}

}
