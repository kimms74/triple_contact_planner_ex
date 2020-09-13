#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>

namespace suhan_contact_planner
{

class RobotDynamicsModel
{
public:
  virtual bool isReachable(Eigen::Vector3d position)=0;       //virtual: 자식 class에서 이름이 같은 함수를 만들면 자식 class마다 다르게 작동하게 해준다
  virtual bool isPossibleContact(Eigen::Affine3d transform)=0;
  virtual Eigen::Matrix<double, 2, 6> getForceLimit()=0;
};

typedef std::shared_ptr<RobotDynamicsModel> RobotDynamicsModelPtr;

}
