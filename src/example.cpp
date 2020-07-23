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
  std::string path = ros::package::getPath("triple_contact_planner");

  ContinuousGraspCandid grp_top;
  grp_top.loadConfig(path + "/top.yaml");

  ContinuousGraspCandid grp_bottom;
  grp_bottom.loadConfig(path + "/bottom.yaml");

  ContactOptimization op;
  RobotDynamicsModelPtr robot_model = make_shared<DexterousRobotModel>();
  op.setRobot(robot_model);

  Eigen::Vector3d com;
  com << -0.40664, 0.12478, 0.18233;
  Eigen::Affine3d com_T;
  com_T.setIdentity();
  com_T.translation() = -com;

  std::vector<ContactPtr> contact_nodes_;
  contact_nodes_.resize(3);

  for (auto &p : contact_nodes_)
    p = make_shared<Contact>();

  std::vector<int> links;
  for (int i = 0; i < grp_bottom.candids_set.size(); i++)
    links.push_back(i);

  ContactModelPtr model = make_shared<StefanModel>("hi");
  model->setMass(3.75);

  vector<vector<int>> all_combination = get_combinations_from_vector<int>(links, 2);
  vector<vector<int>> tot_combination;
  for (auto combination : all_combination)
  {
    for (int i = 0; i < 7; i++)
    {
      combination.push_back(i);
      tot_combination.push_back(combination);
      combination.pop_back();
    }
  }

  vector<vector<double>> ratio_combination;
  vector<double> test;
  DFS(ratio_combination, test);
  
  std::ofstream file;
  file.open(path + "/static_eq_grp.yaml");

  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
  int i = 0;
  std::pair<Eigen::Affine3d, std::string> result1, result2, result3;
  for (auto combination : tot_combination)
  {
    for (auto ratio_combi : ratio_combination)
    {
      for (int j = 0; j < grp_bottom.candids_set[combination[0]].size(); j++)
      {
        result1 = grp_bottom.getGrasp(combination[0], j, ratio_combi[0]);
        contact_nodes_.at(0)->setTransform(com_T * result1.first);

        for (int k = 0; k < grp_bottom.candids_set[combination[1]].size(); k++)
        {
          result2 = grp_bottom.getGrasp(combination[1], k, ratio_combi[1]);
          contact_nodes_.at(1)->setTransform(com_T * result2.first);
          
          for (int m = 0; m < grp_top.candids_set[combination[2]].size(); m++)
          {
            result3 = grp_top.getGrasp(combination[2], m, ratio_combi[2]);
            contact_nodes_.at(2)->setTransform(com_T * result3.first);
        
            model->setContactRobot(contact_nodes_);
          
            model->copyContactRobot();
            op.setModel(model);
            if (op.solve())
            {
              file << "    - part num : [" << combination[0] << ", " << combination[1] << ", " << combination[2] << "]" << endl
                   << "      index : [" << j << ", " << k << ", " << m << "]" << endl
                   << "      ratio : [" << ratio_combi[0] << ", " << ratio_combi[1] << ", " << ratio_combi[2] << "]" << endl
                   << "        ori : [" << result1.second << ", " << result2.second << ", " << result3.second << "]" << endl;
              file << "      force 1 : " << contact_nodes_.at(0)->getContactForceTorque().transpose().format(CommaInitFmt) << endl
                   << "      force 2 : " << contact_nodes_.at(1)->getContactForceTorque().transpose().format(CommaInitFmt) << endl
                   << "      force 3: " << contact_nodes_.at(2)->getContactForceTorque().transpose().format(CommaInitFmt) << endl;
                i++;
            }  
            else
            {
              cout << "no" << endl;
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
