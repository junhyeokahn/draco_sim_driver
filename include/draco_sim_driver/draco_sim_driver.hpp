#pragma once

#include <cortex_data_plugins/generic_command_data.hpp>
#include <cortex_data_plugins/generic_state_data.hpp>
#include <cortex_framework/plugin_base/controller_base.hpp>

namespace aptk {
namespace ctrl {
class DracoSimDriver : public ctrl::ControllerBase {
public:
  DracoSimDriver(){};
  ~DracoSimDriver(){};
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
  }

  // control loop
  void updateCommandData();
  void CopyData();
  void CopyCommand();

private:
  boost::shared_ptr<ctrl::DriverBase> driver_;

  ctrl::CortexModelPtr model_;
  dart::dynamics::SkeletonPtr skel_;

  boost::shared_ptr<GenericStateData> data_;
  boost::shared_ptr<GenericCommandData> command_;
};
} // namespace ctrl
} // namespace aptk
