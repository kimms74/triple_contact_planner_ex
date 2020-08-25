#include <iostream>
#include <fstream>
#include "triple_contact_planner/robot_dynamics/dexterous_robot_model.h"
#include "triple_contact_planner/solver/contact_optimization.h"
#include "triple_contact_planner/cont_reader.h"
#include "triple_contact_planner/contact_model/stefan_model.h"
#include <ros/ros.h>
#include <algorithm>
#include "triple_contact_planner/permutation.h"

#include <ros/package.h>
using namespace std;
using namespace suhan_contact_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "suhan_contact_planner_ex");
  ros::NodeHandle nh;
  std::string path = ros::package::getPath("triple_contact_planner");     //triple_contact_planner 위치 가져옴

  ContinuousGraspCandid grp_top;                                            //cont_reader.h
  grp_top.loadConfig(path + "/top.yaml");                                 //path로 triple_contact_planner있는 위치 가져오고 뒤에 /top.yaml해서 top.yaml을 불러오는 것
                                                                                                    //loadConfig:  top.yaml에서 lb,ub,ori,dist 정보를 tuple에 bound,rot,dist로 candids vector에 담고 candids를 candids_set vector에 담는다
  ContinuousGraspCandid grp_bottom;
  grp_bottom.loadConfig(path + "/bottom.yaml");

  ContactOptimization op;                                                                                               //contact_optimization.h
  RobotDynamicsModelPtr robot_model = make_shared<DexterousRobotModel>(); //robot_dynamics_model.h & dexterous_robot_model.h  
  op.setRobot(robot_model);                                                                                           //RobotDynamicsModelPtr은 shared_ptr로 만들어져 있기때문에 robot_model은 shared_ptr의 성격을 지니고 있다 
                                                                                                                                        //make_shared<>()를 통해 shared_ptr을 직접 초기화
  Eigen::Vector3d com;
  com << -0.40664, 0.12478, 0.18233;                                      //값 대입
  Eigen::Affine3d com_T;                                                  //affine3d: 4x4 matrix
  com_T.setIdentity();
  com_T.translation() = -com;                                             //center of mass  //계산쉽게 하려고 원점을 com으로 옮긴것

  std::vector<ContactPtr> contact_nodes_;
  contact_nodes_.resize(3);

  for (auto &p : contact_nodes_)
    p = make_shared<Contact>();

  std::vector<int> links;
  for (int i = 0; i < grp_bottom.candids_set.size(); i++)                               //candids_set:tuple을 지닌 vector를 지닌 vector
    links.push_back(i);

  ContactModelPtr model = make_shared<StefanModel>("hi");                 //contact model 이름을 hi로 설정하고 mass 3.75로 설정
  model->setMass(3.75);                                                                               //StefanModel: stefan 가구 정보를 지닌 class(contect_model class를 상속받음)

  vector<vector<int>> all_combination = get_combinations_from_vector<int>(links, 2);  //bottom links 중에 2개를 뽑아 조합을 만들어 벡터에 담아준다 ex) (1,3) (0,6) ... -> bottom은 로봇팔 두개로 잡기때문에 파지점을 랜덤으로 뽑아주는것?
  vector<vector<int>> tot_combination;
  for (auto combination : all_combination)      //auto &combination 안쓴 이유?
  {
    for (int i = 0; i < 7; i++)                 //왜 7개 일까? -> top은 로봇팔 하나로만 잡는데 파지점이 7군데라서?
    {
      combination.push_back(i);
      tot_combination.push_back(combination);   //combination에 0~6을 push_back한 것을 tot_combination에 넣음 -> 세 팔의 파지점을 정해주는 것?
      combination.pop_back();
    }
  }

  vector<vector<double>> ratio_combination;
  vector<double> test;
  DFS(ratio_combination, test);                 //test에 0.5를 3개를 넣어 ratio_combination에 넣어준다

  std::ofstream file;
  file.open(path + "/start_static_eq_grp.yaml");

  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
  int i = 0;
  tuple<Eigen::Affine3d, std::string, double> result[3];        //3군데의 로봇팔에 대한 result를 담음
  for (auto combination : tot_combination)                      //combination vector 자체는 어떤 part를 선택할지에 대한 조합을 지니고 있다
  {
    for (auto ratio_combi : ratio_combination)
    {
      for (int j = 0; j < grp_bottom.candids_set[combination[0]].size(); j++)       //combination vector 내에 0,1,2 가 있다, 거기에 해당하는 값은 각각 로봇팔이 파지하는 part_number를 나타낸다
      {                                                                                                                     //grp_bottom.candids_set[i].size(): i번째 part에 해당하는 candids의 크기, 즉 part i는 몇개의 파지점이 있는지를 알려준다
        result[0] = grp_bottom.getGrasp(combination[0], j, ratio_combi[0]);       //combination[i]: tot_combination안에 (1,2,6), (2,3,0) ...으로 다양한 조합의 3군데 part_num이 들어있는데, combination[0]는 첫번째 로봇팔이 선택할 part를 알려준다 
        contact_nodes_.at(0)->setTransform(com_T * get<0>(result[0]));            //combination[0]가 가진 index만큼을 for문을 돌리며 result로 만들어준다
                                                                                                                            //getGrasp를 하면 com을 중심으로 한 파지점의 정보를 알려준다
        for (int k = 0; k < grp_bottom.candids_set[combination[1]].size(); k++)
        {
          result[1] = grp_bottom.getGrasp(combination[1], k, ratio_combi[1]);   //combination[1]: 두번째 로봇팔이 선택한 part_num
          contact_nodes_.at(1)->setTransform(com_T * get<0>(result[1]));        //.at(): []와 같은 기능을 하지만 추가로 parameter가 array를 초과하는지도 확인해준다

          for (int m = 0; m < grp_top.candids_set[combination[2]].size(); m++)
          {
            result[2] = grp_top.getGrasp(combination[2], m, ratio_combi[2]);      //combination[2]: 세번째 로봇팔이 선택한 part_num
            contact_nodes_.at(2)->setTransform(com_T * get<0>(result[2]));      //setTransform: contact_nodes 내에 transform_으로 옮겨준다(transform_:4x4 matrix)
                                                                                                                          //com_T * get<0>(result[]): result[]를 com기준에서 표현하기 위해 바꾸는 것
            model->setContactRobot(contact_nodes_);
            op.setModel(model);
            if (op.solve())
            {
              bool write = true;
              for (int i = 0; i < 3; i++)
              {
                if (get<2>(result[i]) < 0.15 && ratio_combi[i] != 0.5)
                {
                  write = false;
                  break;
                }
              }
              if (write)
              {
                file << "- part num : [" << combination[0] + 1 << ", " << combination[1] + 1 << ", " << combination[2] + 1 << "]" << endl
                     << "  index : [" << j << ", " << k << ", " << m << "]" << endl
                     << "  ratio : [" << ratio_combi[0] << ", " << ratio_combi[1] << ", " << ratio_combi[2] << "]" << endl
                     << "  ori : [" << get<1>(result[0]) << ", " << get<1>(result[1]) << ", " << get<1>(result[2]) << "]" << endl;

                // offset
                // for (int t = 0; t < 3; t++)
                //   get<0>(result[t]).translation() += get<0>(result[t]).linear() * Eigen::Vector3d(0, 0, -0.103);

                Eigen::VectorXd t1(7), t2(7), t3(7);
                t1 << get<0>(result[0]).translation()+ get<0>(result[0]).linear()*Eigen::Vector3d(0, 0, -0.103), 
                      Eigen::Quaterniond(get<0>(result[0]).linear()).coeffs();
                t2 << get<0>(result[1]).translation()+ get<0>(result[1]).linear()*Eigen::Vector3d(0, 0, -0.103),
                      Eigen::Quaterniond(get<0>(result[1]).linear()).coeffs();
                t3 << get<0>(result[2]).translation()+ get<0>(result[2]).linear()*Eigen::Vector3d(0, 0, -0.103),
                      Eigen::Quaterniond(get<0>(result[2]).linear()).coeffs();
                // t3 << Eigen::Map<Eigen::VectorXd>( ( get<0>(result[2]).translation()+ get<0>(result[2]).linear()*Eigen::Vector3d(0, 0, -0.103)).data(), 3)  , 
                //                                   Eigen::Quaterniond(get<0>(result[2]).linear()).coeffs();

                // Eigen::VectorXd t1(Eigen::Map<Eigen::VectorXd>(get<0>(result[0]).matrix().data(), 16));
                // Eigen::VectorXd t2(Eigen::Map<Eigen::VectorXd>(get<0>(result[1]).matrix().data(), 16));
                // Eigen::VectorXd t3(Eigen::Map<Eigen::VectorXd>(get<0>(result[2]).matrix().data(), 16));
                file << "  transform1 : " << t1.transpose().format(CommaInitFmt) << endl
                     << "  transform2 : " << t2.transpose().format(CommaInitFmt) << endl
                     << "  transform3 : " << t3.transpose().format(CommaInitFmt) << endl;
                //  << "  transform1 : " << endl
                //  << get<0>(result[0]).matrix() << endl
                //  << "  transform2 : " << endl
                //  << get<0>(result[1]).matrix() << endl
                //  << "  transform3 : " << endl
                //  << get<0>(result[2]).matrix() << endl;
                // file << "      force 1 : " << contact_nodes_.at(0)->getContactForceTorque().transpose().format(CommaInitFmt) << endl
                //      << "      force 2 : " << contact_nodes_.at(1)->getContactForceTorque().transpose().format(CommaInitFmt) << endl
                //      << "      force 3: " << contact_nodes_.at(2)->getContactForceTorque().transpose().format(CommaInitFmt) << endl;
                i++;
              }

            }
          }
        }
      }
    }
  }
  std::cout << i << std::endl;
  file.close();
  return 0;
}
