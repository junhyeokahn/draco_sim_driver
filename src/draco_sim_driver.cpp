#include <draco_sim_driver/draco_sim_driver.hpp>

namespace aptk {
namespace ctrl {
void DracoSimDriver::updateCommandData() {
  // copy data
  CopyData();
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
    if (it->first == "l_knee_fe") {
      draco_sensor_data_->joint_positions["l_knee_fe_jd"] =
          joint_id_["l_knee_fe"]->getPosition(0) -
          joint_id_["l_knee_fe_jp"]->getPosition(0);
    } else if (it->first == "r_knee_fe") {
      draco_sensor_data_->joint_positions["r_knee_fe_jd"] =
          joint_id_["r_knee_fe"]->getPosition(0) -
          joint_id_["r_knee_fe_jp"]->getPosition(0);
    } else {
      draco_sensor_data_->joint_positions[it->first] =
          it->second->getPosition(0);
    }
  }
}

void DracoSimDriver::CopyCommand() {
  // copy torques
  for (std::map<std::string, dart::dynamics::JointPtr>::iterator it =
           joint_id_.begin();
       it != joint_id_.end(); it++) {
    int idx = it->second->getIndexInSkeleton(0) - 6;
    if (it->first == "l_knee_fe") {
      // TODO : Double check this double up
      command_->eff_atv[idx] =
          2 * draco_command_->joint_torques["l_knee_fe_jd"];
    } else if (it->first == "l_knee_fe_jp") {
      // do nothing
    } else if (it->first == "r_knee_fe") {
      // TODO : Double check this double up
      command_->eff_atv[idx] =
          2 * draco_command_->joint_torques["r_knee_fe_jd"];
    } else if (it->first == "r_knee_fe_jp") {
      // do nothing
    } else {
      command_->eff_atv[idx] = draco_command_->joint_torques[it->first];
    }
  }
}

} // namespace ctrl
} // namespace aptk

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aptk::ctrl::DracoSimDriver, aptk::ctrl::ControllerBase)
