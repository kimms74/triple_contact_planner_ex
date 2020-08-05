
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <algorithm>
#include <tuple>

#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include <ros/package.h>
#include <regrasp_constraint_planner/kinematics/grasping point.h>


using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    std::string name_ = "tripple_constraint_planning";
    ros::init(argc, argv, name_);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    std::vector<std::vector<Eigen::Affine3d>> tf_list;
    std::string path = ros::package::getPath("triple_contact_planner");

    YAML::Node yamlnode = YAML::LoadFile(ros::package::getPath("triple_contact_planner") + "/start_static_eq_grp.yaml");

    Sgrasping_point grp;
    for (int i = 0; i < yamlnode.size(); i++)
    {
        vector<Eigen::Affine3d> tf;
        Eigen::Matrix4d t[3];
        Eigen::Affine3d affine_t[3];
        
        for (int j = 0; j < 3; j++)
        {
            t[j] = Eigen::Matrix4d::Map(yamlnode[i]["transform" + to_string(j + 1)].as<std::vector<double>>().data());
            affine_t[j].matrix() = t[j];
            tf.push_back(grp.obj_start * affine_t[j]); // base to grasp point
        }
        tf_list.push_back(tf);
        // tf.clear();
    }
    vector<vector<Affine3d>> target_list;

    std::ofstream file;
    file.open(path + "/filter_start_static_eq_grp.yaml");

    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
    for (auto tf : tf_list)
    {
        vector<Affine3d> target;
        std::vector<double> panda_grp[3];
        int minElementIndex[3];
        for (int i = 0; i < tf.size(); i++)
        {
            panda_grp[0].push_back((grp.base_1st.translation() - tf[i].translation()).norm());
            panda_grp[1].push_back((grp.base_2nd.translation() - tf[i].translation()).norm());
            panda_grp[2].push_back((grp.base_3rd.translation() - tf[i].translation()).norm());
        }
        bool output = true;
        for (int i = 0; i < 3; i++)
        {
            if (*min_element(panda_grp[i].begin(), panda_grp[i].end()) > 0.8)
            {
                output = false;
                break;
            }
        }
        if (output)
        {
            Eigen::VectorXd t1(Eigen::Map<Eigen::VectorXd>(tf[0].matrix().data(), 16));
            Eigen::VectorXd t2(Eigen::Map<Eigen::VectorXd>(tf[1].matrix().data(), 16));
            Eigen::VectorXd t3(Eigen::Map<Eigen::VectorXd>(tf[2].matrix().data(), 16));
            file << "  transform1 : " << t1.transpose().format(CommaInitFmt) << endl
                 << "  transform2 : " << t2.transpose().format(CommaInitFmt) << endl
                 << "  transform3 : " << t3.transpose().format(CommaInitFmt) << endl;
        }
    }

    cout << target_list.size() << endl;
    std::shared_ptr<IKValidityChecker> IKvc = std::make_shared<IKValidityChecker>(grp);
    return 0;
}