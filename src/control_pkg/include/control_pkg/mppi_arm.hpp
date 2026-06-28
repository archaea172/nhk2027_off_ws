#pragma once

#include <Eigen/Dense>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <stdexcept>

struct ArmParams
{
    std::string urdf;
    Eigen::Vector3d max_link;
    Eigen::Vector3d min_link;

    std::string base_link;
    std::string end_link;
};

struct MppiArmParams
{
    double control_frequency; // hz
    double predict_resolution;
    int predict_horizon;
    int sample_num;
    Eigen::Matrix2d cov;
    double lambda;
    double gamma;

    ArmParams arm_parameters;
};

class MppiArmController
{
public:
    MppiArmController(const MppiArmParams& parameters);

// private:
    KDL::Frame predictArmPos(const Eigen::Vector3d& link_pos);

    const MppiArmParams parameters_;
    KDL::Chain chain_;
    int joint_count_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
};
