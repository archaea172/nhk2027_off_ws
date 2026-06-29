#include "mppi_arm.hpp"

MppiArmController::MppiArmController(const MppiArmParams& parameters)
: parameters_(parameters)
{
    /* kdl init begin */
    KDL::Tree tree;

    if (!kdl_parser::treeFromString(parameters_.arm_parameters.urdf, tree))
    {
        throw std::runtime_error("Failed to parse URDF into KDL tree.");
    }

    const std::string base_link = this->parameters_.arm_parameters.base_link;
    const std::string end_link = this->parameters_.arm_parameters.end_link;
    if (!tree.getChain(base_link, end_link, this->chain_))
    {
        throw std::runtime_error(("Failed to extract KDL chain from " + base_link + " to " + end_link).c_str());
    }

    this->joint_count_ = this->chain_.getNrOfJoints();
    if (0 == this->joint_count_)
    {
        throw std::runtime_error("No joints found in chain.");
    }
    if (3 != this->joint_count_)
    {
        throw std::runtime_error("Expected 3 joints in chain.");
    }

    this->fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(this->chain_);
    /* kdl init end */
    
    /* sampling end */
    Eigen::LLT<Eigen::Matrix3d> llt(this->parameters_.cov);
    this->L_ = llt.matrixL(); 
    /* sampling end */
}

Eigen::Matrix<double, 3, Eigen::Dynamic> MppiArmController::samplinglinkPos(const Eigen::Vector3d& now_link_pos)
{
    const int horizon = this->parameters_.predict_horizon;
    std::normal_distribution<double> normal(0.0, 1.0);

    Eigen::Matrix<double, 3, Eigen::Dynamic> z(3, horizon);

    z = Eigen::Matrix<double, 3, Eigen::Dynamic>::NullaryExpr(
        3, horizon,
        [&]() {
            return normal(this->rng_);
        }
    );

    Eigen::Matrix<double, 3, Eigen::Dynamic> control_array(3, horizon);
    control_array.noalias() = this->L_ * z;
    control_array.col(0) += now_link_pos;
    for (int i = 1; i < horizon; ++i)
    {
        control_array.col(i) += control_array.col(i - 1);
    }

    return control_array;
}

KDL::Frame MppiArmController::predictArmPos(const Eigen::Vector3d& link_pos)
{
    KDL::JntArray q_current(this->joint_count_);
    q_current(0) = link_pos(0);
    q_current(1) = link_pos(1);
    q_current(2) = link_pos(2);

    KDL::Frame current_pose;
    this->fk_solver_->JntToCart(q_current, current_pose);

    return current_pose;
}
