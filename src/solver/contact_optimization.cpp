#include <triple_contact_planner/solver/contact_optimization.h>

namespace suhan_contact_planner
{

  Eigen::Matrix3d cross_skew(Eigen::Vector3d input)
  {
    Eigen::Matrix3d output;
    output.setZero();
    output(0, 1) = -input(2);
    output(1, 0) = input(2);
    output(0, 2) = input(1);
    output(2, 0) = -input(1);
    output(1, 2) = -input(0);
    output(2, 1) = input(0);
    return output;
  }

  void ContactOptimization::setModel(ContactModelPtr model)
  {
    model_ = model;
  }

  void ContactOptimization::setRobot(RobotDynamicsModelPtr robot)
  {
    robot_ = robot;
  }

  bool ContactOptimization::solve()
  {
    ContactOptimizationSolver solver;

    auto eq_constraint = std::make_shared<ConstraintEquality>();
    // eq_constraint.
    Eigen::MatrixXd A;
    Eigen::VectorXd b;

    // TODO: gravity

    const double contact_number = model_->getContactNumber();
    std::vector<ContactPtr> contacts;
    contacts = model_->getContactRobot();
    if (contact_number == 0)
      return false;
    A.setZero(6, contact_number * 6);
    b.setZero(6);
    // b.head<3>() = model_->getTransform().linear() *
    //               Eigen::Vector3d(0, 0, 9.8) * model_->getMass(); // TODO: Check this
    b.head<3>() =  Eigen::Vector3d(0, 0, 9.8) * model_->getMass(); // TODO: Check this
    // TODO: Momentum + b(3~5)
    for (size_t i = 0; i < contact_number; i++)
    {
      A.block<3, 3>(0, i * 6).setIdentity();
      A.block<3, 3>(3, i * 6) = cross_skew(contacts[i]->getContactTransform().translation());
      A.block<3, 3>(3, i * 6 + 3).setIdentity();
    }
    eq_constraint->setA(A);
    eq_constraint->setEqualityCondition(b);

    solver.addConstraint(eq_constraint);

    // every contact
    auto ineq_constraint = std::make_shared<ConstraintInequality>();

    Eigen::MatrixXd C_all;
    Eigen::VectorXd d_all;
    d_all.resize(36);

    C_all.setZero(12 * contact_number, 6 * contact_number);
    // d_all.setZero(contact_number * 12);

    Eigen::MatrixXd C_max[6];
    Eigen::VectorXd d_max[6];
    const auto &f_limit = robot_->getForceLimit();

    for (int i = 0; i < 6; i++)
    {
      C_max[i].setZero(2, 6);
      C_max[i](0, i) = 1;
      C_max[i](1, i) = -1;

      d_max[i].resize(2);
      d_max[i] = f_limit.col(i);
    }

    Eigen::MatrixXd C_i(12, 6), d_i(12, 1);

    for (int i = 0; i < 6; i++)
    {
      C_i.block<2, 6>(i * 2, 0) = C_max[i];
      d_i.block<2, 1>(i * 2, 0) = d_max[i];
    }

    int i = 0;
    for (auto &contact : contacts)
    {
      Eigen::Matrix<double, 6, 6> R_hat;
      Eigen::Matrix<double, 3, 3> R;
      R = contact->getContactTransform().linear().transpose();
      R_hat.setZero();
      R_hat.block<3, 3>(0, 0) = R;
      R_hat.block<3, 3>(3, 3) = R;

      C_all.block<12, 6>(i * 12, i * 6) = C_i * R_hat;
      d_all.segment<12>(i * 12) = d_i;

      i++;
    }
    
    ineq_constraint->setA(C_all);
    ineq_constraint->setOnlyLowerBound(d_all);
    // ineq_constraint->printCondition();
    // std::cout << std::endl;
    solver.addConstraint(ineq_constraint);
    solver.setContactNumber(model_->getContactNumber());
    Eigen::VectorXd result;
    if (solver.solve(result))
    {
      //auto &contacs = model_->getContactRobot();
      for (int i = 0; i < contacts.size(); i++)
      {
        // TODO: Force update!
        // TODO: Contact copy is needed!
        // std::cout << result.segment<6>(i * 6).transpose() << std::endl;
        contacts[i]->setContactForceTorque(result.segment<6>(i * 6));
        // std::cout << contacts[i]->getContactForceTorque().transpose() << std::endl;
      }
      return true;
    }
    return false;
  }

} // namespace suhan_contact_planner
