#include "triple_contact_planner/solver/contact_optimization_solver.h"


namespace suhan_contact_planner
{

ContactOptimizationSolver::ContactOptimizationSolver()
{}

void ContactOptimizationSolver::addConstraint(ConstraintBasePtr cb)
{
  constraints.push_back(cb);  //constraints는 vector<>
}

void ContactOptimizationSolver::setContactNumber(int contact_number)
{ contact_number_ = contact_number; }

bool ContactOptimizationSolver::solve(Eigen::VectorXd& result_force)
{
  size_t total_row = 0;
  for(auto & constraint : constraints)  
  {
    total_row += constraint->rows();
    assert(contact_number_ * 6 == constraint->cols());
  }

   resize(total_row);

  size_t A_row_index = 0;
  for(auto & constraint : constraints)
  {
    A_.block(A_row_index, 0, constraint->rows(), constraint->cols()) =  //block(i,j,p,q): block size (p,q), startng at (i,j) 출력 (i,j는 0부터 시작, p,q는 0일때 전체)
        constraint->getA();                                             //A_ 안에 eq와 ineq constraints 다 들어있다
    A_lower_bound_.segment(A_row_index, constraint->rows()) =           //segment(i,n): vector에서의 block(), Block containing n elements, starting at position i
        constraint->getLowerBound();
    A_upper_bound_.segment(A_row_index, constraint->rows()) =
        constraint->getUpperBound();
    A_row_index += constraint->rows();
  }

  int iter = 1000;
  int result;
  A_row_ = A_;
  // qproblem_.setPrintLevel(qpOASES::PL_DEBUG_ITER);
  
  if(!hot_start_)
  {
    result = qproblem_.init(H_row_.data(), g_.data(), A_row_.data(),    //.init(),hotstart(): returnvalue를 return
                   //lower_bound_.data(), upper_bound_.data(),          //returnvalue: enum으로 SUCCESSFUL_RETURN=0,...이다
                            NULL,NULL,
                   A_lower_bound_.data(), A_upper_bound_.data(), iter); //iter: mWSR으로 the maximum number of working set recalculations to be perfomed during the initioal homotopy
    hot_start_ = true;                                                  //H가 identity matrix일 경우에는 zero or identity matrix를 넣어주면 된다
  }
  else
  {
    result = qproblem_.hotstart(H_row_.data(), g_.data(), A_row_.data(),
                       //lower_bound_.data(), upper_bound_.data(),
                                NULL,NULL,
                       A_lower_bound_.data(), A_upper_bound_.data(), iter);
  }

//  std::cout << "A: " << std:: endl
//            << A_ << std::endl;

//  std::cout << "A_lb.\': " << std:: endl
//            << A_lower_bound_.transpose() << std::endl;

//  std::cout << "A_ub.\': " << std:: endl
//            << A_upper_bound_.transpose() << std::endl;

  if (result == qpOASES::SUCCESSFUL_RETURN)
  {
    qproblem_.getPrimalSolution(x_solved_.data());  //해 구하기
    result_force = x_solved_;
    //std::cout << x_solved_.transpose() << std::endl;
    qproblem_.reset();
    return true;
  }
  else
  {
    // RET_INIT_FAILED_INFEASIBILITY
    // std::cout << "qp solve failed: " << result << std::endl;
    return false;
  }
}

void ContactOptimizationSolver::resize(int total_row)
{
  A_.setZero(total_row, contact_number_ * 6);
  H_row_.setIdentity(contact_number_ * 6, contact_number_ * 6);
  A_lower_bound_.resize(total_row);
  A_upper_bound_.resize(total_row);
  g_.setZero(contact_number_ * 6);
  x_solved_.resize(contact_number_ * 6);
  qpOASES::Options options;

  options.setToDefault();                                                                               //option 초기화
  options.initialStatusBounds = qpOASES::ST_INACTIVE;                      //Initial status of bounds at first iteration
  options.printLevel          = qpOASES::PL_NONE;                                   //Print level
  options.enableRegularisation = qpOASES::BT_TRUE;                           //Specifies whether Hessian matrix shall be regularised in case semi-definiteness is detected
  options.enableEqualities = qpOASES::BT_TRUE;                                  //Specifies whether equalities shall be always treated as active constraints
  //qproblem_ = qpOASES::SQProblem(contact_number_ * 6, total_row,HST_IDENTITY);
  qproblem_ = qpOASES::SQProblem(contact_number_ * 6, total_row); //contact_nuber + s로 만들기
  qproblem_.setOptions(options);
}

} // namespace suhan_contact_planner
