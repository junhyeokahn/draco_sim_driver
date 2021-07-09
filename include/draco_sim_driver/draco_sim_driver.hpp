#pragma once

#include <apptronik_srvs/Float32.h>
#include <cortex_data_plugins/generic_command_data.hpp>
#include <cortex_data_plugins/generic_state_data.hpp>
#include <cortex_framework/plugin_base/controller_base.hpp>

#include <pnc/draco_pnc/draco_interface.hpp>

namespace aptk {
namespace ctrl {

class DracoSimDriver : public ctrl::ControllerBase {
public:
  DracoSimDriver() {

    draco_interface_ = new DracoInterface(true);
    draco_sensor_data_ = new DracoSensorData();
    draco_command_ = new DracoCommand();

    actuator_map_["neck_pitch"] = 0;
    actuator_map_["l_shoulder_fe"] = 1;
    actuator_map_["l_shoulder_aa"] = 2;
    actuator_map_["l_shoulder_ie"] = 3;
    actuator_map_["l_elbow_fe"] = 4;
    actuator_map_["l_wrist_ps"] = 5;
    actuator_map_["l_wrist_pitch"] = 6;
    actuator_map_["r_shoulder_fe"] = 7;
    actuator_map_["r_shoulder_aa"] = 8;
    actuator_map_["r_shoulder_ie"] = 9;
    actuator_map_["r_elbow_fe"] = 10;
    actuator_map_["r_wrist_ps"] = 11;
    actuator_map_["r_wrist_pitch"] = 12;
    actuator_map_["l_hip_ie"] = 13;
    actuator_map_["l_hip_aa"] = 14;
    actuator_map_["l_hip_fe"] = 15;
    actuator_map_["l_knee_fe"] = 16;
    actuator_map_["l_ankle_fe"] = 17;
    actuator_map_["l_ankle_ie"] = 18;
    actuator_map_["r_hip_ie"] = 19;
    actuator_map_["r_hip_aa"] = 20;
    actuator_map_["r_hip_fe"] = 21;
    actuator_map_["r_knee_fe"] = 22;
    actuator_map_["r_ankle_fe"] = 23;
    actuator_map_["r_ankle_ie"] = 24;

    dof_map_["rootJoint_rot_x"] = 0;
    dof_map_["rootJoint_rot_y"] = 1;
    dof_map_["rootJoint_rot_z"] = 2;
    dof_map_["rootJoint_pos_x"] = 3;
    dof_map_["rootJoint_pos_y"] = 4;
    dof_map_["rootJoint_pos_z"] = 5;
    dof_map_["neck_pitch"] = 6;
    dof_map_["l_shoulder_fe"] = 7;
    dof_map_["l_shoulder_aa"] = 8;
    dof_map_["l_shoulder_ie"] = 9;
    dof_map_["l_elbow_fe"] = 10;
    dof_map_["l_wrist_ps"] = 11;
    dof_map_["l_wrist_pitch"] = 12;
    dof_map_["r_shoulder_fe"] = 13;
    dof_map_["r_shoulder_aa"] = 14;
    dof_map_["r_shoulder_ie"] = 15;
    dof_map_["r_elbow_fe"] = 16;
    dof_map_["r_wrist_ps"] = 17;
    dof_map_["r_wrist_pitch"] = 18;
    dof_map_["l_hip_ie"] = 19;
    dof_map_["l_hip_aa"] = 20;
    dof_map_["l_hip_fe"] = 21;
    dof_map_["l_knee_fe_jp"] = 22;
    dof_map_["l_knee_fe"] = 23;
    dof_map_["l_ankle_fe"] = 24;
    dof_map_["l_ankle_ie"] = 25;
    dof_map_["r_hip_ie"] = 26;
    dof_map_["r_hip_aa"] = 27;
    dof_map_["r_hip_fe"] = 28;
    dof_map_["r_knee_fe_jp"] = 29;
    dof_map_["r_knee_fe"] = 30;
    dof_map_["r_ankle_fe"] = 31;
    dof_map_["r_ankle_ie"] = 32;
  };
  ~DracoSimDriver() {
    delete draco_interface_;
    delete draco_sensor_data_;
    delete draco_command_;
  };
  void initialize(boost::shared_ptr<ctrl::DriverBase> driver,
                  boost::shared_ptr<ctrl::StateDataBase> state_data_base,
                  boost::shared_ptr<ctrl::CommandDataBase> command_data_base) {
    driver_ = driver;

    model_ = driver_->sharedModel();
    skel_ = model_->getSkel();

    data_ = boost::dynamic_pointer_cast<GenericStateData>(state_data_base);
    command_ =
        boost::dynamic_pointer_cast<GenericCommandData>(command_data_base);

    command_data_base->initialize(driver_->rtNodeHandle(), driver_->logger());
    command_data_base->addDebugInterfaces();

    interrupt_handler_ = driver_->nodeHandle().advertiseService(
        "interrupt_handler", &DracoSimDriver::InterruptHandler, this);

    for (int i = 0; i < skel_->getNumJoints(); ++i) {
      dart::dynamics::JointPtr joint = skel_->getJoint(i);
      if (joint->getName() == "rootJoint") {
      } else if (joint->getType() != "WeldJoint") {
        joint_id_[joint->getName()] = joint;
      } else {
      }
    }

    for (int i = 0; i < skel_->getNumBodyNodes(); ++i) {
      dart::dynamics::BodyNodePtr bn = skel_->getBodyNode(i);
      link_id_[bn->getName()] = bn;
    }
  }

  // control loop
  void updateCommandData();
  void CopyData();
  void CopyCommand();

  // for debugging purpose
  void CopyBase();

private:
  boost::shared_ptr<ctrl::DriverBase> driver_;

  ctrl::CortexModelPtr model_;
  dart::dynamics::SkeletonPtr skel_;

  boost::shared_ptr<GenericStateData> data_;
  boost::shared_ptr<GenericCommandData> command_;

  DracoInterface *draco_interface_;
  DracoSensorData *draco_sensor_data_;
  DracoCommand *draco_command_;

  std::map<std::string, dart::dynamics::JointPtr> joint_id_;
  std::map<std::string, dart::dynamics::BodyNodePtr> link_id_;
  std::map<std::string, int> actuator_map_;
  std::map<std::string, int> dof_map_;

  ros::ServiceServer interrupt_handler_;

  bool InterruptHandler(apptronik_srvs::Float32::Request &req,
                        apptronik_srvs::Float32::Response &res);
};
} // namespace ctrl
} // namespace aptk
