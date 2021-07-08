#include <draco_sim_driver/draco_sim_driver.hpp>

namespace aptk {
namespace ctrl {
void DracoSimDriver::updateCommandData() {
  // copy data
  CopyData();
  CopyBase();
  // compute command
  draco_interface_->getCommand(draco_sensor_data_, draco_command_);
  // coppy command
  CopyCommand();
}

void DracoSimDriver::CopyData() {
  // copy imu data
  Eigen::Isometry3d imu_iso = link_id_["torso_imu"]->getTransform(
      dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
  draco_sensor_data_->imu_frame_iso.block(0, 0, 3, 3) = imu_iso.linear();
  draco_sensor_data_->imu_frame_iso.block(0, 3, 3, 1) = imu_iso.translation();
  draco_sensor_data_->imu_frame_vel = link_id_["torso_imu"]->getSpatialVelocity(
      dart::dynamics::Frame::World(), dart::dynamics::Frame::World());

  // copy encoder data
  for (std::map<std::string, dart::dynamics::JointPtr>::iterator it =
           joint_id_.begin();
       it != joint_id_.end(); it++) {
    if (it->first == "l_knee_fe" or it->first == "l_knee_fe_jp") {
      draco_sensor_data_->joint_positions["l_knee_fe_jd"] =
          joint_id_["l_knee_fe"]->getPosition(0) / 2.;
      draco_sensor_data_->joint_positions["l_knee_fe_jp"] =
          joint_id_["l_knee_fe"]->getPosition(0) / 2.;
      draco_sensor_data_->joint_velocities["l_knee_fe_jd"] =
          joint_id_["l_knee_fe"]->getVelocity(0) / 2.;
      draco_sensor_data_->joint_velocities["l_knee_fe_jp"] =
          joint_id_["l_knee_fe"]->getVelocity(0) / 2.;
    } else if (it->first == "r_knee_fe" or it->first == "r_knee_fe_jp") {
      draco_sensor_data_->joint_positions["r_knee_fe_jd"] =
          joint_id_["l_knee_fe"]->getPosition(0) / 2.;
      draco_sensor_data_->joint_positions["r_knee_fe_jp"] =
          joint_id_["l_knee_fe"]->getPosition(0) / 2.;
      draco_sensor_data_->joint_velocities["r_knee_fe_jd"] =
          joint_id_["r_knee_fe"]->getVelocity(0) / 2.;
      draco_sensor_data_->joint_velocities["r_knee_fe_jp"] =
          joint_id_["r_knee_fe"]->getVelocity(0) / 2.;
    } else {
      draco_sensor_data_->joint_positions[it->first] =
          it->second->getPosition(0);
      draco_sensor_data_->joint_velocities[it->first] =
          it->second->getVelocity(0);
    }
  }

} // namespace ctrl

void DracoSimDriver::CopyCommand() {
  // copy torques
  for (std::map<std::string, dart::dynamics::JointPtr>::iterator it =
           joint_id_.begin();
       it != joint_id_.end(); it++) {
    if (it->first == "l_knee_fe") {
      command_->eff_atv[actuator_map_[it->first]] =
          draco_command_->joint_torques["l_knee_fe_jd"];
    } else if (it->first == "l_knee_fe_jp") {
      // do nothing
    } else if (it->first == "r_knee_fe") {
      command_->eff_atv[actuator_map_[it->first]] =
          draco_command_->joint_torques["r_knee_fe_jd"];
    } else if (it->first == "r_knee_fe_jp") {
      // do nothing
    } else {
      command_->eff_atv[actuator_map_[it->first]] =
          draco_command_->joint_torques[it->first];
    }
  }
}

// TODO : for debugging purpose
void DracoSimDriver::CopyBase() {
  dart::dynamics::BodyNode *root_bn = skel_->getRootBodyNode();

  draco_sensor_data_->base_com_pos = root_bn->getCOM();
  Eigen::Quaternion<double> qt =
      Eigen::Quaternion<double>(root_bn->getWorldTransform().linear());
  draco_sensor_data_->base_com_quat << qt.w(), qt.x(), qt.y(), qt.z();

  draco_sensor_data_->base_com_ang_vel =
      root_bn->getCOMSpatialVelocity().head(3);
  draco_sensor_data_->base_com_lin_vel =
      root_bn->getCOMSpatialVelocity().tail(3);

  Eigen::MatrixXd rot_w_l = qt.toRotationMatrix();
  draco_sensor_data_->base_joint_pos =
      root_bn->getCOM() - rot_w_l * root_bn->getLocalCOM();
  draco_sensor_data_->base_joint_quat = draco_sensor_data_->base_com_quat;
  draco_sensor_data_->base_joint_ang_vel =
      root_bn->getSpatialVelocity().head(3);
  draco_sensor_data_->base_joint_lin_vel =
      root_bn->getSpatialVelocity().tail(3);
}
} // namespace ctrl
} // namespace aptk

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aptk::ctrl::DracoSimDriver, aptk::ctrl::ControllerBase)
