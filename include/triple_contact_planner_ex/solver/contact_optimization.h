#pragma once

#include <limits>

#include <triple_contact_planner_ex/contact_model/contact_model.h>
#include <triple_contact_planner_ex/robot_dynamics/robot_dynamics_model.h>
#include <triple_contact_planner_ex/solver/constraint/constraint_equality.h>
#include <triple_contact_planner_ex/solver/constraint/constraint_inequality.h>
#include <triple_contact_planner_ex/solver/contact_optimization_solver.h>

namespace suhan_contact_planner               //headder 파일에서는 using namespace는 쓰지말자(icnlude하는 모든 파일에 영향을 미치므로)
{

class ContactOptimization
{
public:
  void setModel(ContactModelPtr model);       //contact_model.h
  void setRobot(RobotDynamicsModelPtr robot); //robot_dynamics_model.h
  bool solve();                               //contact_optimization.cpp에 정의돼 있음

private:
  ContactModelPtr model_;
  RobotDynamicsModelPtr robot_;
};

}                                     
