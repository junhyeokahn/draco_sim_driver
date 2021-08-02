#include <rolling_joint/rolling_joint.hpp>
#include <utils/util.hpp>

using namespace util;
namespace aptk {
namespace ctrl {
void MyRollingJoint::updateCommandData() {

  // int num_eval(100);
  // for (int i = 0; i < num_eval; ++i) {
  std::cout << "------------------------------------------------" << std::endl;
  std::cout << "count : " << count_ << std::endl;
  // std::cout << "i : " << i << std::endl;
  // PrintAtThisConfig(-M_PI / 2 + M_PI / num_eval * i);
  //}
  // exit(0);

  // command_->eff_atv.setZero();

  t = count_ * 0.001;
  double trq = 5 * sin(2 * t);
  command_->eff_atv[0] = 0.;

  double q(0.), qdot(0.);
  q = joint_id_["rolling_joint"]->getPosition(0);
  qdot = joint_id_["rolling_joint"]->getVelocity(0);

  SaveValue(q, "q");
  SaveValue(qdot, "qdot");
  SaveValue(trq, "trq");
  SaveValue(t, "t");

  count_ += 1;

  if (count_ == 50000) {
    exit(0);
  }
}

void MyRollingJoint::PrintAtThisConfig(double q) {
  // joint_id_["jb00"]->setPosition(0., -0.2);
  // joint_id_["jb00"]->setVelocity(0., 0.);
  // joint_id_["jb0"]->setPosition(0., -0.1);
  // joint_id_["jb0"]->setVelocity(0., 0.);
  // joint_id_["jb1"]->setPosition(0., -0.1);
  // joint_id_["jb1"]->setVelocity(0., 0.);
  joint_id_["rolling_joint"]->setPosition(0, q);
  joint_id_["rolling_joint"]->setVelocity(0, 0.);
  // joint_id_["ja1"]->setPosition(0., 0.2);
  // joint_id_["ja1"]->setVelocity(0., 0.);
  // joint_id_["ja2"]->setPosition(0., -0.4);
  // joint_id_["ja2"]->setVelocity(0., 0.);
  skel_->computeForwardKinematics();
  skel_->computeForwardDynamics();

  std::cout << "configuration : " << q << std::endl;
  std::cout << "ee pos : "
            << link_id_["ee"]->getWorldTransform().translation().transpose()
            << std::endl;
  // std::cout << "grav comp" << std::endl;
  // std::cout << "gravity comp command : " << skel_->getGravityForces()[6]
  //<< " , " << skel_->getGravityForces()[7] << " , "
  //<< skel_->getGravityForces()[8] << " , "
  //<< skel_->getGravityForces()[10] << ", "
  //<< skel_->getGravityForces()[11] << ", "
  //<< skel_->getGravityForces()[12] << std::endl;

  std::cout << "simdriver full gravity" << std::endl;
  std::cout << skel_->getGravityForces().transpose() << std::endl;

  // pnc
  // pnc_joint_id_["jb00"]->setPosition(0., -0.2);
  // pnc_joint_id_["jb00"]->setVelocity(0., 0.);
  // pnc_joint_id_["jb0"]->setPosition(0., -0.1);
  // pnc_joint_id_["jb0"]->setVelocity(0., 0.);
  // pnc_joint_id_["jb1"]->setPosition(0., -0.1);
  // pnc_joint_id_["jb1"]->setVelocity(0., 0.);
  // pnc_joint_id_["jp"]->setPosition(0, q / 2.);
  // pnc_joint_id_["jp"]->setVelocity(0, 0.);
  // pnc_joint_id_["jd"]->setPosition(0, q / 2.);
  // pnc_joint_id_["jd"]->setVelocity(0, 0.);
  // pnc_joint_id_["ja1"]->setPosition(0., 0.2);
  // pnc_joint_id_["ja1"]->setVelocity(0., 0.);
  // pnc_joint_id_["ja2"]->setPosition(0., -0.4);
  // pnc_joint_id_["ja2"]->setVelocity(0., 0.);
  // pnc_skel_->computeForwardKinematics();
  // pnc_skel_->computeForwardDynamics();
  // pnc_skel_->computeForwardKinematics();
  // pnc_skel_->computeForwardDynamics();

  // std::cout << "ee pos : "
  //<< pnc_link_id_["ee"]->getWorldTransform().translation().transpose()
  //<< std::endl;
  // std::cout << "grav comp" << std::endl;

  // std::cout << "pnc skel full gravity" << std::endl;
  // std::cout << pnc_skel_->getGravityForces().transpose() << std::endl;
}

} // namespace ctrl
} // namespace aptk

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aptk::ctrl::MyRollingJoint, aptk::ctrl::ControllerBase)
