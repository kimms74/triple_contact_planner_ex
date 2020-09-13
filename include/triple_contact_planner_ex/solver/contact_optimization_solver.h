#ifndef CONTACT_OPTIMIZATION_SOLVER_H
#define CONTACT_OPTIMIZATION_SOLVER_H

#include <qpOASES.hpp>
#include <Eigen/Dense>
#include <memory>
#include <triple_contact_planner_ex/contact_model/contact_model.h>
#include <triple_contact_planner_ex/solver/constraint/constraint_base.h>

namespace suhan_contact_planner
{

typedef Eigen::Matrix<double, -1, -1, Eigen::RowMajor> MatrixXdRow; //MatrixXdRow: 매트릭스에 row부터 값을 넣음


class ContactOptimizationSolver
{
public:
  ContactOptimizationSolver();
  void setContactNumber(int contact_number);
  void addConstraint(ConstraintBasePtr cb); //constraint_base.h
  bool solve(Eigen::VectorXd &result_force);

private:
  qpOASES::SQProblem qproblem_;

  Eigen::MatrixXd H_, A_; //runtime때 size정하는 matrix
  Eigen::VectorXd g_;     //rutime때 size정하는 vector

  MatrixXdRow  H_row_, A_row_;
  Eigen::VectorXd A_lower_bound_, A_upper_bound_, lower_bound_, upper_bound_;

  Eigen::VectorXd x_solved_;

  size_t contact_number_;


  std::vector<ConstraintBasePtr> constraints;

  bool hot_start_{false};

  void resize(int total_row);
};

} // namespace suhan_contact_planner

#endif // CONTACT_OPTIMIZATION_SOLVER_H
