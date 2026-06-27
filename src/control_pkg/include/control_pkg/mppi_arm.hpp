#pragma once

#include <Eigen/Dense>

struct MppiArmParams
{
    double control_frequency; // hz
    double predict_resolution;
    int predict_horizon;
    int sample_num;
    Eigen::Matrix2d cov;
    double lambda;
    double gamma;
};

struct ArmParams
{
};


class MppiArmController
{

};