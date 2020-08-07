#ifndef CONTACT_MODEL_H
#define CONTACT_MODEL_H

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "triple_contact_planner/contact/contact.h"

namespace suhan_contact_planner
{

/**                                             //Doxygen을 이용한 코드상의 주석을 통해 문서를 만들어내는 방법
 * @brief The ContactModel class                //class에 대한 주석: brief,details,author,date,version
 */
class ContactModel
{
public:
  ContactModel() : name_("") {}                 //변수 초기화
  ContactModel(std::string name) : name_(name) {}

  enum OperationDirection : int {DIR_Z=0, DIR_Y, DIR_X, DIR_YAW, DIR_PITCH, DIR_ROLL};  //enum: user-defined data types, DIR_Z=0이라고 지정해두면 나머지는 1씩 커지는 수가 해당 이름에 들어가는 값이 된다.
                                                                                        //class 안에서만 쓰이는 type일 경우 class안에 enum을 만들어 쓴다
  void contactWrenchOptimize();                                                         //다른 cpp 파일에 있는  함수를 쓰기위해 forward declaration한 것

  /**
   * @brief createContactSamples
   */
  virtual void createContactSamples(std::vector<ContactPtr> &contact_samples) = 0;      //ContactPtr: contatct.h
                                                                                        //virtual: base class에서 선언했지만 derived class에서 사용하면 derived class인 것처럼 행동한다
                                                                                        //즉, derived class마다 다른 성질을 가짐
  /**                                           //method에 대한 주석:brief,details,param,return,throws
   * @brief operation
   * @param dir Direction we want to move
   * @param delta
   * @return Whether it is possible operation.
   */
  virtual bool operate(OperationDirection dir, double delta_x, double delta_orientation) = 0;

  /**
   * @brief isSamePose
   * @param model An model to be compared
   * @param threshold_x Linear threshold
   * @param threshold_orientation Orientation threshold
   * @return
   */
  bool isSamePose(const ContactModel& model, double threshold_x, double threshold_orientation) const;

  virtual ContactPtr getBottomContact() = 0;

  inline const Eigen::Vector3d getPosition() const {return transform_.translation(); }
  inline const Eigen::Affine3d getTransform() const { return transform_; }


  inline size_t getContactNumberRobot() { return contact_robot_.size(); }
  inline size_t getContactNumberEnvironment() { return contact_environment_.size(); }
  inline size_t getContactNumber() { return getContactNumberEnvironment() + getContactNumberRobot(); }

  void setContactRobot(const std::vector<ContactPtr> & contact_robot) { contact_robot_ = contact_robot; }
  void setContactEnvironment(const std::vector<ContactPtr> & contact_environment) { contact_environment_ = contact_environment; }
  void setContactEnvironment(const ContactPtr contact_environment) { contact_environment_.clear();
                                                                     contact_environment_.push_back(contact_environment); }

  const std::vector<ContactPtr>& getContactRobot() const { return contact_robot_; }
  const std::vector<ContactPtr>& getContactEnvironment() const { return contact_environment_; }

  // const std::vector < ContactPtr > & getContactSamples() { return contact_samples_; }
  inline double getMass() { return mass_; }
  virtual double getBottomPositionZ() = 0;
  void setMass(double mass) { mass_ = mass; }
  void setGoalNode() { goal_node_ = true; }
  bool goalNode() { return goal_node_; }

  void copyContactEnvironment();
  void copyContactRobot();

  void printContacts();
  void printAbsoulteContactPositions();

protected:                                        //protected: derived class에서만 public처럼 사용가능
  std::string name_;
  Eigen::Affine3d transform_;
  Eigen::Vector3d centor_of_mass_ {0, 0, 0};

  // std::vector < ContactPtr > line_contact_samples_;
  // std::vector < ContactPtr > point_contact_samples_;
  //std::vector < ContactPtr > contact_samples_;
  std::vector<ContactPtr> contact_robot_;
  std::vector<ContactPtr> contact_environment_;
  
  bool goal_node_ {false};
  double mass_ {3.75};
};

typedef std::shared_ptr<ContactModel> ContactModelPtr;

}

#endif
