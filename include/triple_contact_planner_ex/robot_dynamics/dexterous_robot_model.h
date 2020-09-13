#pragma once

#include "triple_contact_planner_ex/robot_dynamics/robot_dynamics_model.h"

namespace suhan_contact_planner
{

class DexterousRobotModel : public RobotDynamicsModel //RobotDynamicsModel의 자식 class
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
  Eigen::Matrix<double, 2, 6> getForceLimit() override  //부모 class에 있는 함수와 똑같은 이름의 함수를 알아보기 편하게 함수 뒤에 override써주는 것(앞에 virtual쓰는 것과 비슷)
  {
    Eigen::Matrix<double, 2, 6> limit_matrix; //Eigen::Matrix<scalar,rows,coloums>  //2x6 matrix
    // for (int i = 0; i < 6; i++)
    // {
      // // limit_matrix(0, i) = 1;
      // // limit_matrix(1, i) = -10;
      // limit_matrix(0, i) = -20;               //min:-20
      // limit_matrix(1, i) = -20;               //max:20
    // }
    for (int i = 0; i < 3; i++)
    {
      // limit_matrix(0, i) = 1;
      // limit_matrix(1, i) = -10;
      limit_matrix(0, i) = -40;               //min:-20
      limit_matrix(1, i) = -40;               //max:20
    }
    for (int i = 3; i < 6; i++)
    {
      // limit_matrix(0, i) = 1;
      // limit_matrix(1, i) = -10;
      limit_matrix(0, i) = 0;               //min:-20
      limit_matrix(1, i) = 0;               //max:20
    }
    return limit_matrix;
  }
};

}
