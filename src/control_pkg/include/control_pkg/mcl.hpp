#pragma once

#include <Eigen/Dense>

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

struct mcl_parameters
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

private:
    Pose2D now_pose_;
};
