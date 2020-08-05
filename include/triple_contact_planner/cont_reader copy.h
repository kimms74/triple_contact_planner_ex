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
  typedef tuple<std::pair<Eigen::Vector3d, Eigen::Vector3d>, std::pair<Eigen::Quaterniond, std::string>, std::string > grasp_tuple;
  std::vector<std::vector<std::pair<std::pair<Eigen::Vector3d, Eigen::Vector3d>, 
                                    std::pair<Eigen::Quaterniond, std::string> > > > candids_set;
  void loadConfig(std::string file_name)
  {
    YAML::Node yamlnode;
    yamlnode = YAML::LoadFile(file_name);
    // std::vector<std::pair<std::pair<Eigen::Vector3d, Eigen::Vector3d>, std::pair<Eigen::Quaterniond, std::string> > > candids;

    for (int i = 0; i < yamlnode.size(); i++)
    {
      std::vector<std::pair<std::pair<Eigen::Vector3d, Eigen::Vector3d>, std::pair<Eigen::Quaterniond, std::string> > > candids;
      for (int j = 0; j < yamlnode["part" + std::to_string(i + 1)].size(); j++)
      {
        std::pair<std::pair<Eigen::Vector3d, Eigen::Vector3d>, std::pair<Eigen::Quaterniond, std::string> > pose;
        pose.first.first = Eigen::Vector3d::Map(yamlnode["part" + to_string(i + 1)][j]["upper_bound"].as<std::vector<double>>().data());
        pose.first.second = Eigen::Vector3d::Map(yamlnode["part" + to_string(i + 1)][j]["lower_bound"].as<std::vector<double>>().data());
        pose.second.first.coeffs() = Eigen::Vector4d::Map(yamlnode["part" + to_string(i + 1)][j]["orientation"].as<std::vector<double>>().data());
        pose.second.second = yamlnode["part" + to_string(i + 1)][j]["ori"].as<std::string>().data();
        candids.push_back(pose);
      }
      candids_set.push_back(candids);
      // candids.clear();
    }
  }

  std::pair<Eigen::Affine3d, std::string> getGrasp(int part_num, int index, double ratio)
  // Eigen::Affine3d getGrasp(int part_num, int index, double ratio)
  {
    auto candids_ = candids_set[part_num];
    if (index >= candids_.size())
      throw std::out_of_range("index ");

    const auto &ub = candids_[index].first.first;
    const auto &lb = candids_[index].first.second;
    const auto &quat = candids_[index].second.first;
    const auto &ori = candids_[index].second.second;

    Eigen::Affine3d pose;
    pose.setIdentity();
    pose.translation() = (ub - lb) * ratio + lb;
    pose.linear() = quat.matrix();
    
    // return pose;
    std::pair<Eigen::Affine3d, std::string> result;
    result.first = pose;
    result.second = ori;
    return result;
  }
};
