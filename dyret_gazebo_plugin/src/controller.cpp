#include "dyret_gazebo_plugin/controller.hpp"

#include <cmath>
#include <dyret_common/ServoState.h>
#include <dyret_common/ServoStateArray.h>
#include <functional>
#include <gazebo/gazebo.hh>
#include <ros/advertise_options.h>

GZ_REGISTER_MODEL_PLUGIN(dyret::GazeboDyretController)

namespace dyret {

	GazeboDyretController::~GazeboDyretController() {
		// Clear connection to Gazebo
		conn.reset();
		// Clear ROS callback queue
		queue.clear();
		queue.disable();
		// Shutdown ROS node if it has been initialized
		if(node != nullptr){
			node->shutdown();
		}
		// Wait for asynchronous spinner to stop
		if(spin != nullptr){
			spin->stop();
		}
		// Drop connection to ROS
		node.reset();
	}

	void GazeboDyretController::Load(gazebo::physics::ModelPtr model_, sdf::ElementPtr sdf_) {
		if(!ros::isInitialized()) {
			ROS_FATAL("ROS has not been initialized!");
			gzerr << "ROS has not been initialized! GazeboDyretController exiting!\n";
			return;
		}
		// Store pointer to model
		model = model_;
		sdf = sdf_;
		ctrl = model->GetJointController();
		// Check that the model contains enough joints before proceeding
		if(ctrl->GetJoints().size() != 12 + 8) {
			ROS_FATAL("Model does not contain all expected joints!");
			return;
		}
		if(!InitROS()) {
			ROS_FATAL("Could not initiate ROS subsystem");
			return;
		}
		if(!LoadParameters()) {
			ROS_FATAL("Could not load parameters");
			return;
		}
		// This ensures that our "UpdateJoints" method is called after each
		// simulation tick
		conn = gazebo::event::Events::ConnectWorldUpdateBegin(
				std::bind(&GazeboDyretController::UpdateJoints, this));
		ROS_INFO("Dyret Gazebo Controller loaded");
	}

	bool GazeboDyretController::LoadParameters() {
		if(sdf->HasElement("updateRate")) {
			publishRate = sdf->Get<double>("updateRate");
			ROS_DEBUG("'updateRate' set to %.1f", publishRate);
		}
		if(sdf->HasElement("pid")) {
			auto pid = sdf->GetElement("pid");
			// Can default PID configuration be set?
			bool set_default = true;
			while(pid != nullptr) {
				const auto value = pid->Get<ignition::math::Vector3d>();
				const auto pid_value = gazebo::common::PID(value[0], value[1], value[2]);
				if(pid->HasAttribute("type")) {
					auto attr = pid->GetAttribute("type")->GetAsString();
					if(attr == "mx64") {
						mx64Pid = pid_value;
					} else if(attr == "mx106") {
						mx106Pid = pid_value;
					} else if(attr == "extension_short") {
						extShortPid = pid_value;
					} else if(attr == "extension_long") {
						extLongPid = pid_value;
					} else {
						ROS_WARN_STREAM("Unknown PID type: " << attr);
					}
					set_default = false;
				} else if (set_default) {
					mx64Pid = pid_value;
					mx106Pid = pid_value;
					extShortPid = pid_value;
					extLongPid = pid_value;
				} else {
					ROS_WARN_STREAM("Could not use PID configuration '"
							<< pid->ToString("CFG")
							<< "' because default is set");
				}
				pid = pid->GetNextElement("pid");
			}
		}

		// Update PID controller for all joints
		ctrl->SetPositionPID("dyret::fl_joint1", mx64Pid);
		ctrl->SetPositionPID("dyret::fr_joint1", mx64Pid);
		ctrl->SetPositionPID("dyret::bl_joint1", mx64Pid);
		ctrl->SetPositionPID("dyret::br_joint1", mx64Pid);

		ctrl->SetPositionPID("dyret::fl_joint2", mx106Pid);
		ctrl->SetPositionPID("dyret::fl_joint3", mx106Pid);
		ctrl->SetPositionPID("dyret::fr_joint2", mx106Pid);
		ctrl->SetPositionPID("dyret::fr_joint3", mx106Pid);
		ctrl->SetPositionPID("dyret::bl_joint2", mx106Pid);
		ctrl->SetPositionPID("dyret::bl_joint3", mx106Pid);
		ctrl->SetPositionPID("dyret::br_joint2", mx106Pid);
		ctrl->SetPositionPID("dyret::br_joint3", mx106Pid);

		ctrl->SetPositionPID("dyret::fl_ext1", extShortPid);
		ctrl->SetPositionPID("dyret::fr_ext1", extShortPid);
		ctrl->SetPositionPID("dyret::bl_ext1", extShortPid);
		ctrl->SetPositionPID("dyret::br_ext1", extShortPid);

		ctrl->SetPositionPID("dyret::fl_ext2", extLongPid);
		ctrl->SetPositionPID("dyret::fr_ext2", extLongPid);
		ctrl->SetPositionPID("dyret::bl_ext2", extLongPid);
		ctrl->SetPositionPID("dyret::br_ext2", extLongPid);

		// Set initial set-point so that robot doesn't fall
		Default(true);
		ROS_DEBUG("LoadParameters finished");
		return true;
	}

	bool GazeboDyretController::InitROS() {
		// Create node handle for ROS
		node = std::make_unique<ros::NodeHandle>(model->GetName());
		// Set custom callback queue
		node->setCallbackQueue(&queue);
		// Create asynchronous spinner with custom callback queue
		// To minimize contention and resource usage set 1 thread:
		spin = std::make_unique<ros::AsyncSpinner>(1, &queue);
		spin->start();
		// Advertise reset service
		auto reset_opts = ros::AdvertiseServiceOptions::create<std_srvs::SetBool>(
				"/" + this->model->GetName() + "/reset",
				std::bind(&GazeboDyretController::Reset, this,
					std::placeholders::_1, std::placeholders::_2),
				ros::VoidPtr(), &queue);
		reset_service = node->advertiseService(reset_opts);
		if(reset_service == nullptr) {
			ROS_ERROR("Could not create reset service");
			return false;
		}
		// Create state publisher
		auto state_opts = ros::AdvertiseOptions::create<dyret_common::ServoStateArray>(
				"/" + this->model->GetName() + "/servoStates",
				1, NULL, NULL, ros::VoidPtr(), &queue);
		state_pub = node->advertise(state_opts);
		if(state_pub == nullptr) {
			ROS_ERROR("Could not create 'servoStates' publisher");
			return false;
		}
		// Create configure service
		auto config_opts = ros::AdvertiseServiceOptions::create<dyret_common::ConfigureServos>(
				"/" + this->model->GetName() + "/configure",
				std::bind(&GazeboDyretController::Configure, this,
					std::placeholders::_1, std::placeholders::_2),
				ros::VoidPtr(), &queue);
		config_service = node->advertiseService(config_opts);
		if(config_service == nullptr) {
			ROS_ERROR("Could not create configure service");
			return false;
		}
		ROS_DEBUG("InitROS finished");
		return true;
	}

	void GazeboDyretController::Default(bool reset_prismatic) {
		for(const auto& joint : JOINT_NAMES) {
			double position = 0.0;
			node->getParam(joint.substr(7), position);
			ctrl->SetJointPosition(joint, position);
			ctrl->SetPositionTarget(joint, position);
		}
		// If user desires we reset the extension joints as well
		if(reset_prismatic) {
			for(const auto& ext : EXT_NAMES) {
				double position = 0.0;
				node->getParam(ext.substr(7), position);
				ctrl->SetJointPosition(ext, position);
				ctrl->SetPositionTarget(ext, position);
			}
		}
	}

	bool GazeboDyretController::Reset(const std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp) {
		{
			// NOTE: If method could affect 'UpdateJoints' must take mutex!
			std::lock_guard<std::mutex> guard(mutex);
			// Reset joints
			Default(req.data);
		}
		resp.success = true;
		return true;
	}

	bool GazeboDyretController::Configure(
			const dyret_common::ConfigureServos::Request& req,
			dyret_common::ConfigureServos::Response& resp) {
		// NOTE: If method could affect 'UpdateJoints' must take mutex!
		std::lock_guard<std::mutex> guard(mutex);
		auto joints = ctrl->GetJoints();
		for(const auto& cfg: req.configurations) {
			if(cfg.id > 12) {
				ROS_WARN("Unknown servo ID: %d", cfg.id);
				continue;
			}
			auto name = JOINT_NAMES[cfg.id];
			auto joint = joints[name];
			switch(cfg.type) {
				case dyret_common::ServoConfig::TYPE_DISABLE_LOG:
				case dyret_common::ServoConfig::TYPE_ENABLE_LOG:
				case dyret_common::ServoConfig::TYPE_ENABLE_TORQUE:
				case dyret_common::ServoConfig::TYPE_DISABLE_TORQUE:
					ROS_WARN("Operation (%d) not supported in simulation", cfg.type);
					break;
				case dyret_common::ServoConfig::TYPE_SET_SPEED:
					if(cfg.parameters.size() < 1) {
						resp.status = dyret_common::ConfigureServos::Response::STATUS_PARAMETER;
						resp.message = "Expected one parameter to set speed";
						return true;
					} else {
						joint->SetVelocityLimit(0, cfg.parameters[0]);
					}
					break;
				case dyret_common::ServoConfig::TYPE_SET_PID:
					if(cfg.parameters.size() < 3) {
						resp.status = dyret_common::ConfigureServos::Response::STATUS_PARAMETER;
						resp.message = "Expected at least 3 parameter for PID";
						return true;
					} else {
						gazebo::common::PID pid(
								cfg.parameters[0],
								cfg.parameters[1],
								cfg.parameters[2]);
						ctrl->SetPositionPID(name, pid);
					}
					break;
				default:
					ROS_WARN("Unknown configuration type: %d for ID: %d",
							cfg.type, cfg.id);
					break;
			}
		}
		return true;
	}

	void GazeboDyretController::Pose(const dyret_common::Pose::ConstPtr& pose) {
		// NOTE: If method could affect 'UpdateJoints' must take mutex!
		std::lock_guard<std::mutex> guard(mutex);
		// Set position target for revolute joints
		if(pose->revolute.size() == 12) {
			for(int i = 0; i < 12; ++i) {
				if(!ctrl->SetPositionTarget(JOINT_NAMES[i], pose->revolute[i])) {
					ROS_WARN_STREAM("Could not set position '" <<
							pose->revolute[i] << "' for joint: " <<
							JOINT_NAMES[i]);
				}
			}
		} else if (pose->revolute.empty()) {
			// Do nothing by design, this arm is just to prevent
			// emitting warnings when revolute vector is empty
		} else {
			ROS_WARN("Unknown number of revolute joints, expected 12 was: %d",
					pose->revolute.size());
		}
		// Set position targets for prismatic joints
		// NOTE: prismatic joints move to negative values due to URDF definition
		// to enable same message as real-world we negate here:
		if(pose->prismatic.size() == 2) {
			ctrl->SetPositionTarget("dyret::fl_ext1", -pose->prismatic[0]);
			ctrl->SetPositionTarget("dyret::fr_ext1", -pose->prismatic[0]);
			ctrl->SetPositionTarget("dyret::bl_ext1", -pose->prismatic[0]);
			ctrl->SetPositionTarget("dyret::br_ext1", -pose->prismatic[0]);

			ctrl->SetPositionTarget("dyret::fl_ext2", -pose->prismatic[1]);
			ctrl->SetPositionTarget("dyret::fr_ext2", -pose->prismatic[1]);
			ctrl->SetPositionTarget("dyret::bl_ext2", -pose->prismatic[1]);
			ctrl->SetPositionTarget("dyret::br_ext2", -pose->prismatic[1]);
		} else if (pose->prismatic.size() == 8) {
			for(int i = 0; i < 8; ++i) {
				if(!ctrl->SetPositionTarget(EXT_NAMES[i], -pose->prismatic[i])) {
					ROS_WARN_STREAM("Could not set prismatic joint to '" <<
							-pose->prismatic[i] << "', joint: " <<
							EXT_NAMES[i]);
				}
			}
		} else if (pose->prismatic.empty()) {
			// Do nothing by design, this arm is just to prevent
			// emitting warnings when prismatic vector is empty
		} else {
			ROS_WARN("Unknown number of prismatic joints, expected 2 or 8 was: %d",
					pose->prismatic.size());
		}
	}

	void GazeboDyretController::UpdateJoints() {
		// If there are no subscribers we do not proceed
		if(state_pub.getNumSubscribers() == 0) {
			return;
		}

		auto currentTime = ctrl->GetLastUpdateTime();
		// If publishRate is above 0. and not enough time has passed
		// since last update we do nothing in this method
		if(publishRate > 0 && (currentTime - lastPublish).Double() < 1. / publishRate) {
			return;
		}
		// We wish to publish, but don't want to block Gazebo on ROS admin
		if(mutex.try_lock()) {
			auto joints     = ctrl->GetJoints();
			auto set_points = ctrl->GetPositions();
			// Create message to publish
			dyret_common::ServoStateArrayPtr state(new dyret_common::ServoStateArray);
			state->header.stamp = ros::Time::now();
			for(int i = 0; i < 12; ++i) {
				dyret_common::ServoState st;
				auto joint     = joints[JOINT_NAMES[i]];
				auto set_point = set_points[JOINT_NAMES[i]];
				st.position  = joint->Position(0);
				st.velocity  = joint->GetVelocity(0);
				st.current   = joint->GetForce(0);
				st.set_point = set_point;
				state->revolute[i] = st;
			}
			for(int i = 0; i < 8; ++i) {
				dyret_common::ServoState st;
				auto joint     = joints[EXT_NAMES[i]];
				auto set_point = set_points[EXT_NAMES[i]];
				// NOTE: The prismatic joints move from 0 -> negative 0.X
				// we multiply by -1 to give same output as real robot
				st.position  = std::abs(joint->Position(0));
				st.velocity  = joint->GetVelocity(0) * -1.0;
				st.current   = joint->GetForce(0) * -1.0;
				st.set_point = std::abs(set_point);
				state->prismatic[i] = st;
			}
			state_pub.publish(state);
			lastPublish = currentTime;
			mutex.unlock();
		}
	}
}
