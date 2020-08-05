#pragma once
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <algorithm>
#include <tuple>
using namespace std;
struct ContinuousGraspCandid
{
  typedef tuple<std::pair<Eigen::Vector3d, Eigen::Vector3d>, std::pair<Eigen::Quaterniond, std::string>, double > grasp_tuple;
  std::vector<std::vector<grasp_tuple> > candids_set;
  void loadConfig(std::string file_name)
  {
    YAML::Node yamlnode;
    yamlnode = YAML::LoadFile(file_name);
    // std::vector<std::pair<std::pair<Eigen::Vector3d, Eigen::Vector3d>, std::pair<Eigen::Quaterniond, std::string> > > candids;

    for (int i = 0; i < yamlnode.size(); i++)
    {
      std::vector<grasp_tuple> candids;
      for (int j = 0; j < yamlnode["part" + std::to_string(i + 1)].size(); j++)
      {
        std::pair<Eigen::Vector3d, Eigen::Vector3d> bound;
        std::pair<Eigen::Quaterniond, std::string> rot;
        bound.first = Eigen::Vector3d::Map(yamlnode["part" + to_string(i + 1)][j]["upper_bound"].as<std::vector<double>>().data());
        bound.second = Eigen::Vector3d::Map(yamlnode["part" + to_string(i + 1)][j]["lower_bound"].as<std::vector<double>>().data());
        rot.first.coeffs() = Eigen::Vector4d::Map(yamlnode["part" + to_string(i + 1)][j]["orientation"].as<std::vector<double>>().data());
        rot.second = yamlnode["part" + to_string(i + 1)][j]["ori"].as<std::string>().data();
        double dist = yamlnode["part" + to_string(i + 1)][j]["distance"].as<double>();
        
        grasp_tuple pose = make_tuple(bound, rot, dist);
        candids.push_back(pose);
        // std::cout << get<0>(pose).first.transpose() << get<0>(pose).second.transpose() << endl
        //           << get<1>(pose).second << " " << get<2>(pose) << endl;
      }
      candids_set.push_back(candids);
      // candids.clear();
    }
  }

  tuple<Eigen::Affine3d, std::string, double> getGrasp(int part_num, int index, double ratio)
  // Eigen::Affine3d getGrasp(int part_num, int index, double ratio)
  {
    auto candids_ = candids_set[part_num];
    if (index >= candids_.size())
      throw std::out_of_range("index ");

    const auto &ub = get<0>(candids_[index]).first;
    const auto &lb = get<0>(candids_[index]).second;

    const auto &quat = get<1>(candids_[index]).first;
    const auto &ori = get<1>(candids_[index]).second;

    const auto &dist = get<2>(candids_[index]);

    Eigen::Affine3d pose;
    pose.setIdentity();
    pose.translation() = (ub - lb) * ratio + lb;
    pose.linear() = quat.matrix();
    
    // return pose;
    tuple<Eigen::Affine3d, std::string, double> result = make_tuple(pose, ori, dist);
    return result;
  }
};
