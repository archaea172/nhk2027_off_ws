#pragma once

#include <Eigen/Dense>
#include <cmath>

struct DistanceFieldMap
{
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_yaw = 0.0;
    int width = 0;
    int height = 0;
    Eigen::ArrayXXf distance;
};

struct Pose2D
{
    double x;
    double y;
    double theta;
};

struct particle
{
    Pose2D pose;
    double weight;
};

struct mclParameters
{
    double dt;
    double particle_num;
    double sigma_x;
    double sigma_y;
    double sigma_theta;
    DistanceFieldMap map;
};


class Mcl
{
public:
    Mcl();
    Pose2D calcPose();

private:
    std::vector<particle> updateParticles(
        std::vector<particle> now_particles,
        Eigen::Vector3d vel_feedback
    );

    Pose2D now_pose_;
    mclParameters parameters_;
    std::vector<particle> particles_;
};
