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
  typedef tuple<std::pair<Eigen::Vector3d, Eigen::Vector3d>, std::pair<Eigen::Quaterniond, std::string>, double > grasp_tuple;      //Eigen::Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
  std::vector<std::vector<grasp_tuple> > candids_set;                                                                               //Vector3d: 3x1 double 벡터
  void loadConfig(std::string file_name)                                                                                            //candids_set: matrix?
{                                                                                                                                   //std::tuple<>에서 <>안에 변수 타입들을 작성하는 것
  YAML::Node yamlnode;                                                                                                              //YAML::Node는 
    yamlnode = YAML::LoadFile(file_name);
    // std::vector<std::pair<std::pair<Eigen::Vector3d, Eigen::Vector3d>, std::pair<Eigen::Quaterniond, std::string> > > candids;

    for (int i = 0; i < yamlnode.size(); i++)
    {
      std::vector<grasp_tuple> candids;
      for (int j = 0; j < yamlnode["part" + std::to_string(i + 1)].size(); j++)                                                     //std::to_string: int를 string으로 변경
      {
        std::pair<Eigen::Vector3d, Eigen::Vector3d> bound;
        std::pair<Eigen::Quaterniond, std::string> rot;
        bound.first = Eigen::Vector3d::Map(yamlnode["part" + to_string(i + 1)][j]["upper_bound"].as<std::vector<double>>().data());       //yamlnode[][][],?  //vector<double>에서 < >는 class template을 의미
        bound.second = Eigen::Vector3d::Map(yamlnode["part" + to_string(i + 1)][j]["lower_bound"].as<std::vector<double>>().data());      //part0,1,2..은 파지점을 의미
        rot.first.coeffs() = Eigen::Vector4d::Map(yamlnode["part" + to_string(i + 1)][j]["orientation"].as<std::vector<double>>().data());//pair안에 든 것들은 .first, .second로 불러올 수 있다
        rot.second = yamlnode["part" + to_string(i + 1)][j]["ori"].as<std::string>().data();
        double dist = yamlnode["part" + to_string(i + 1)][j]["distance"].as<double>();
        
        grasp_tuple pose = make_tuple(bound, rot, dist);                                                                            //tuple인 pose에 bound, rot, dist를 저장
        candids.push_back(pose);                                                                                                    //vector인 candids에 pose 저장
        // std::cout << get<0>(pose).first.transpose() << get<0>(pose).second.transpose() << endl                                   //get<0>(pose): tuple인 pose에 저장된 첫번째를 가져와줌 
        //           << get<1>(pose).second << " " << get<2>(pose) << endl;                                                         //c++17부터는 auto[a,b,c]=pose를 이용해 tuple values를 쉽게 가져올 수 있다
      }                                                                                                                             //.first.transpose(): bound.first행렬을 transpose해준다
      candids_set.push_back(candids);                                                                                               //candids_set에 candids저장
      // candids.clear();
    }
  }

  tuple<Eigen::Affine3d, std::string, double> getGrasp(int part_num, int index, double ratio)                                       //part_num:부품,index:그 부품에서 어느 부분, ratio: 부분에서 0~1 사이
  // Eigen::Affine3d getGrasp(int part_num, int index, double ratio)                                                                //Affine3d: 4x4 matrix
{                                                                                                                                   //c++17부터는 auto getGrasp(int part_num, int index, double ratio)로 써도 된다
    auto candids_ = candids_set[part_num];
    if (index >= candids_.size())
      throw std::out_of_range("index ");

    const auto &ub = get<0>(candids_[index]).first;
    const auto &lb = get<0>(candids_[index]).second;

    const auto &quat = get<1>(candids_[index]).first;
    const auto &ori = get<1>(candids_[index]).second;

    const auto &dist = get<2>(candids_[index]);

    Eigen::Affine3d pose;                                                                                                           //Homogenous Transformation
    pose.setIdentity();
    pose.translation() = (ub - lb) * ratio + lb;                                                                                    //translation()은 translation 부분에 벡터를 넣어줌
    pose.linear() = quat.matrix();                                                                                                  //linear()은 rotation 부분에 행렬을 넣어줌, matrix():quaternion을 rotation 행렬로 변환
    
    // return pose;
    tuple<Eigen::Affine3d, std::string, double> result = make_tuple(pose, ori, dist);                                               //c++17부터는 result를 생성할 필요 없이 바로 return std::make_tuple(pose,ori,dist)하면 된다!
    return result;
  }
};
