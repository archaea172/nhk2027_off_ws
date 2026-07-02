#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <random>

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
    Pose2D initial_pose;
    double dt;
    double particle_num;
    double odom_noise_tt;
    double odom_noise_rt;
    double odom_noise_tr;
    double odom_noise_rr;

    double odom_noise_initial_x;
    double odom_noise_initial_y;
    double odom_noise_initial_theta;
    DistanceFieldMap map;
};


class Mcl
{
public:
    Mcl(const mclParameters& parameters);
    // Pose2D calcPose();

// private:
    std::vector<particle> updateParticles(
        std::vector<particle> now_particles,
        Eigen::Vector3d vel_feedback
    );

    Pose2D now_pose_;
    mclParameters parameters_;
    std::vector<particle> particles_;

    std::mt19937 rng_{std::random_device{}()};
};
