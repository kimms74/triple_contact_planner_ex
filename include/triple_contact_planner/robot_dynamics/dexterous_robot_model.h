#pragma once

#include "triple_contact_planner/robot_dynamics/robot_dynamics_model.h"

namespace suhan_contact_planner
{

class DexterousRobotModel : public RobotDynamicsModel
{
public:
  bool isReachable(Eigen::Vector3d position) override
  {
    // ROS_INFO("%lf %lf %lf", position[0], position[1], position[2]);

    return (position.norm() < 100);
  }
  bool isPossibleContact(Eigen::Affine3d transform) override { return true; }

  Eigen::Vector3d getMaximumForce();
  Eigen::Vector3d getMaximumMoment();

  // for grasp contact
  Eigen::Matrix<double, 2, 6> getForceLimit() override
  {
    Eigen::Matrix<double, 2, 6> limit_matrix;
    for (int i = 0; i < 6; i++)
    {
      // limit_matrix(0, i) = 1;
      // limit_matrix(1, i) = -10;
      limit_matrix(0, i) = -20;
      limit_matrix(1, i) = -20;
    }
    return limit_matrix;
  }
};

}