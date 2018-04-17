#include "dyret_gazebo_plugin/joint_control.hpp"

namespace dyret {
	void JointControl::SetPID(const gazebo::common::PID pid) {
		this->pid = pid;
		ctrl->SetPositionPID(name, this->pid);
	}
}
