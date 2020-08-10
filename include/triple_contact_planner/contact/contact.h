#pragma once

#include <array>
#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>

namespace suhan_contact_planner
{

class Contact
{
public:
  enum class ContactRelation {CONTACT_OBJ_ENV, CONTACT_OBJ_ROBOT};                              //enum class:기존 enum은 다른 class에 동일한 이름의 data를 지니고 있으면 error를 발생시켰지만(전역변수처럼 작동), 
  enum class ContactDirection : int {DIR_Y = 0, DIR_X = 1};                                     //enum class를 쓰면 다른 것으로 인식해 만들 수 있게 해준다 
  enum class ContactState {CONTACT_FACE, CONTACT_LINE, CONTACT_POINT};

  Contact(ContactState contact_state = ContactState::CONTACT_POINT, 
          ContactRelation contact_relation = ContactRelation::CONTACT_OBJ_ROBOT) ;
  inline const ContactState getContactState() const {return contact_state_; }
  inline const Eigen::Affine3d & getContactTransform() { return transform_; }

  void setTransform(const Eigen::Affine3d& transform) { transform_ = transform; }
  void setContactState(ContactState state) { contact_state_ = state; }
  void setContactForceTorque(const Eigen::Matrix<double, 6, 1>& force_torque)
  { contact_force_torque_ = force_torque; }
  const Eigen::Matrix<double, 6, 1> & getContactForceTorque() { return contact_force_torque_; }
  void setContactLength(double x, double y)
  {
    contact_length_[0] = x;
    contact_length_[1] = y;
  }

  double getLineContactLength() const {
    return contact_length_[static_cast<int>(line_contact_direction_)];
  }
  const std::array<double, 2> & getContactLength() const { return contact_length_; }

  void setLineContactDirection(ContactDirection dir) { line_contact_direction_ = dir; }
  ContactDirection getLineContactDirection() { return line_contact_direction_; }

  void printContactState();

protected:
  ContactRelation contact_relation_;
  ContactState contact_state_;
  ContactDirection line_contact_direction_ {ContactDirection::DIR_X};
  std::array<double, 2> contact_length_ {{0.0, 0.0}}; ///< To calculate CoP

private:
  // position & rotation with regard to object frame
  Eigen::Affine3d transform_; ///< Contact position transform w.r.t object frame (\f$objH_c\f$)

  Eigen::Matrix<double, 6, 1> contact_force_torque_;
};

typedef std::shared_ptr<Contact> ContactPtr;  //typedef보단 using 권장

}
