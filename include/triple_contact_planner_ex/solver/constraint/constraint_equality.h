#pragma once

#include <triple_contact_planner_ex/solver/constraint/constraint_base.h>

namespace suhan_contact_planner
{

class ConstraintEquality : public ConstraintBase
{
public:
  ConstraintEquality(const std::string &name = "");
  void setEqualityCondition(const Eigen::VectorXd & b);

  void printCondition() override;
};

} // namespace suhan_contact_planner

