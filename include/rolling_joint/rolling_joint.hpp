#pragma once

#include <cortex_data_plugins/generic_command_data.hpp>
#include <cortex_data_plugins/generic_state_data.hpp>
#include <cortex_framework/plugin_base/controller_base.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

namespace aptk {
namespace ctrl {

class MyRollingJoint : public ctrl::ControllerBase {
public:
  MyRollingJoint(){};
  ~MyRollingJoint(){};
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

    for (int i = 0; i < skel_->getNumJoints(); ++i) {
      dart::dynamics::JointPtr joint = skel_->getJoint(i);
      if (joint->getName() == "rootJoint") {
      } else if (joint->getType() != "WeldJoint") {
        joint_id_[joint->getName()] = joint;
      } else {
      }
    }

    for (int i = 0; i < skel_->getNumDofs(); ++i) {
      dart::dynamics::DegreeOfFreedom *dof = skel_->getDof(i);
      std::cout << dof->getCoulombFriction() << std::endl;
      std::cout << dof->getDampingCoefficient() << std::endl;
      dof->setCoulombFriction(0.);
      dof->setDampingCoefficient(0.);
    }

    for (int i = 0; i < skel_->getNumBodyNodes(); ++i) {
      dart::dynamics::BodyNodePtr bn = skel_->getBodyNode(i);
      link_id_[bn->getName()] = bn;
    }

    std::string urdf_file = "/home/apptronik/catkin_ws/src/draco_3/models/"
                            "draco_3_model/urdf/rolling_joint2.urdf";
    dart::utils::DartLoader urdfLoader;
    pnc_skel_ = urdfLoader.parseSkeleton(urdf_file);
    for (int i = 0; i < pnc_skel_->getNumJoints(); ++i) {
      dart::dynamics::JointPtr joint = pnc_skel_->getJoint(i);
      if (joint->getName() == "rootJoint") {
        // n_floating = joint->getNumDofs();
      } else if (joint->getType() != "WeldJoint") {
        pnc_joint_id_[joint->getName()] = joint;
      } else {
      }
    }
    for (int i = 0; i < pnc_skel_->getNumBodyNodes(); ++i) {
      dart::dynamics::BodyNodePtr bn = pnc_skel_->getBodyNode(i);
      pnc_link_id_[bn->getName()] = bn;
    }
  }

  // control loop
  void updateCommandData();

private:
  boost::shared_ptr<ctrl::DriverBase> driver_;

  ctrl::CortexModelPtr model_;
  dart::dynamics::SkeletonPtr skel_;
  dart::dynamics::SkeletonPtr pnc_skel_;

  boost::shared_ptr<GenericStateData> data_;
  boost::shared_ptr<GenericCommandData> command_;

  std::map<std::string, dart::dynamics::JointPtr> joint_id_;
  std::map<std::string, dart::dynamics::BodyNodePtr> link_id_;

  std::map<std::string, dart::dynamics::JointPtr> pnc_joint_id_;
  std::map<std::string, dart::dynamics::BodyNodePtr> pnc_link_id_;

  int count_ = 0;
  double t = 0.;

  void PrintAtThisConfig(double q);
};
} // namespace ctrl
} // namespace aptk
