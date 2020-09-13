#pragma once

#include <triple_contact_planner_ex/solver/constraint/constraint_base.h>

namespace suhan_contact_planner
{

class ConstraintInequality : public ConstraintBase
{
public:
  ConstraintInequality(const std::string &name = "");
  void setInequalityCondition(const Eigen::VectorXd & lb, const Eigen::VectorXd & ub);

  void printCondition() override;
};

} // namespace suhan_contact_planner
