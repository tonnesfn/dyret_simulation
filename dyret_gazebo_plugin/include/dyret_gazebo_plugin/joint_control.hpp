#pragma once

#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace dyret {
	class JointControl {
		public:
			JointControl(const std::string n,
					const gazebo::physics::JointControllerPtr c,
					const gazebo::common::PID pid): name(n), ctrl(c) {
				SetPID(pid);
			}

			void SetPID(const gazebo::common::PID pid);

		private:
			// Name of this joint
			std::string name;
			// Pointer to Gazebo joint controller
			gazebo::physics::JointControllerPtr ctrl;
			// Reference to position PID controller for joint
			gazebo::common::PID pid;
	};
}
