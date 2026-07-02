#include "mcl.hpp"

#include <chrono>
#include <iostream>

int main()
{
    mclParameters parameters;
    parameters.dt = 0.1;
    parameters.initial_pose.x = 0;
    parameters.initial_pose.y = 0;
    parameters.initial_pose.theta = 0;
    parameters.particle_num = 500;
    parameters.odom_noise_tt = 0.2;
    parameters.odom_noise_rt = 0.2;
    parameters.odom_noise_tr = 0.4;
    parameters.odom_noise_rr = 0.4;
    parameters.odom_noise_initial_x = 0.1;
    parameters.odom_noise_initial_y = 0.1;
    parameters.odom_noise_initial_theta = 0.3;
    
    Mcl mcl_test(parameters);
    Eigen::Vector3d vel_feedback;
    vel_feedback << 1, 1, 1;

    std::chrono::system_clock::time_point  start, end; // 型は auto で可
    start = std::chrono::system_clock::now(); // 計測開始時間

    mcl_test.updateParticles(mcl_test.particles_, vel_feedback);

    end = std::chrono::system_clock::now();  // 計測終了時間
    double elapsed = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << elapsed << std::endl;
}
