#include <draco_sim_driver/draco_sim_driver.hpp>

namespace aptk {
namespace ctrl {
void DracoSimDriver::updateCommandData() {

  // print out joint positions
  // std::cout << "pos dof" << std::endl;
  // std::cout << data_->pos_dof[5] << std::endl;
  // std::cout << data_->pos_dof.size() << std::endl;

  // set zero torque
  // std::cout << command_->pos_atv.size() << std::endl;
  // std::cout << "-------" << std::endl;
}

void DracoSimDriver::CopyData() {}

void DracoSimDriver::CopyCommand() {}

} // namespace ctrl
} // namespace aptk

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aptk::ctrl::DracoSimDriver, aptk::ctrl::ControllerBase)
