// #include <iostream>
// #include <fstream>
// #include <ros/ros.h>
// #include <algorithm>
// #include "triple_contact_planner/permutation.h"

// #include <map>
// #include <ros/package.h>

// #include <yaml-cpp/yaml.h>

// using namespace std;

// int main(int argc, char **argv)
// {
//   vector<string> bottom_contact = {"link0", "link1", "link2", "link3", "link4"};
//   vector<string> top_contact = {};
//   string start_dir = "plusZ";
//   pair<string, pair<vector<string>, vector<string>>> start_pos = (make_pair(start_dir,
//                                                                             make_pair(bottom_contact, top_contact)));

//   string file_name;
//   YAML::Node yamlnode;
//   yamlnode = YAML::LoadFile(file_name);
//   for (int i = 0; i < yamlnode["bottom"].size(); i++)
//   {
//     if
//       start_pos.front()
//   }
// }

// std::vector<std::pair<std::pair<Eigen::Vector3d, Eigen::Vector3d>, Eigen::Quaterniond>> candids;

// void loadConfig(std::string file_name)
// {
//   YAML::Node yamlnode;

//   yamlnode = YAML::LoadFile(file_name);
//   for (int i = 0; i < yamlnode["grasp_points"].size(); i++)
//   {
//     std::pair<std::pair<Eigen::Vector3d, Eigen::Vector3d>, Eigen::Quaterniond> pose;
//     pose.first.first = Eigen::Vector3d::Map(yamlnode["grasp_points"][i]["upper_bound"].as<std::vector<double>>().data());
//     pose.first.second = Eigen::Vector3d::Map(yamlnode["grasp_points"][i]["lower_bound"].as<std::vector<double>>().data());
//     pose.second.coeffs() = Eigen::Vector4d::Map(yamlnode["grasp_points"][i]["orientation"].as<std::vector<double>>().data());
//     candids.push_back(pose);
//   }
// }

// Eigen::Affine3d getGrasp(int index, double ratio)
// {
//   if (index >= candids.size())
//     throw std::out_of_range("index ");

//   const auto &ub = candids[index].first.first;
//   const auto &lb = candids[index].first.second;

//   const auto &quat = candids[index].second;

//   Eigen::Affine3d pose;
//   pose.setIdentity();
//   pose.translation() = (ub - lb) * ratio + lb;
//   pose.linear() = quat.matrix();

//   return pose;
// }