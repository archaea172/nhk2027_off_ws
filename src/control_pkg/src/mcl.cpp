#include "mcl.hpp"

Mcl::Mcl(const mclParameters& parameters)
: parameters_(parameters)
{
    this->now_pose_ = this->parameters_.initial_pose;

    std::normal_distribution<double> noise_x(0.0, this->parameters_.odom_noise_initial_x);
    std::normal_distribution<double> noise_y(0.0, this->parameters_.odom_noise_initial_y);
    std::normal_distribution<double> noise_theta(0.0, this->parameters_.odom_noise_initial_theta);

    this->now_pose_.x     += noise_x(this->rng_);
    this->now_pose_.y     += noise_y(this->rng_);
    this->now_pose_.theta += noise_theta(this->rng_);
}

std::vector<particle> Mcl::updateParticles(
    std::vector<particle> now_particles,
    Eigen::Vector3d vel_feedback
)
{
    std::vector<particle> next_particles;
    next_particles.reserve(this->parameters_.particle_num);

    const double dt     = this->parameters_.dt;
    const double dx_body = vel_feedback(0) * dt;
    const double dy_body = vel_feedback(1) * dt;
    const double dtheta  = vel_feedback(2) * dt;

    const double d_trans2 = dx_body * dx_body + dy_body * dy_body;
    const double d_theta2 = dtheta * dtheta;

    const double odom_noise_tt = this->parameters_.odom_noise_tt;
    const double odom_noise_rt = this->parameters_.odom_noise_rt;
    const double odom_noise_tr = this->parameters_.odom_noise_tr;
    const double odom_noise_rr = this->parameters_.odom_noise_rr;
    
    const double sigma_trans = std::sqrt(odom_noise_tt * d_trans2 + odom_noise_rt * d_theta2);
    const double sigma_theta = std::sqrt(odom_noise_tr * d_trans2 + odom_noise_rr * d_theta2);

    std::normal_distribution<double> noise_trans(0.0, sigma_trans);
    std::normal_distribution<double> noise_theta(0.0, sigma_theta);

    for (particle now_particle : now_particles)
    {
        const double cos_t = std::cos(now_particle.pose.theta);
        const double sin_t = std::sin(now_particle.pose.theta);

        const double noisy_dx = dx_body   + noise_trans(this->rng_);
        const double noisy_dy = dy_body   + noise_trans(this->rng_);
        const double noisy_theta = dtheta + noise_theta(this->rng_);

        particle next_particle;
        next_particle.pose.x = now_particle.pose.x + cos_t * noisy_dx - sin_t * noisy_dy;
        next_particle.pose.y = now_particle.pose.y + sin_t * noisy_dx + cos_t * noisy_dy;
        next_particle.pose.theta = now_particle.pose.theta + noisy_theta;
        next_particle.weight = now_particle.weight;
        next_particles.push_back(next_particle);
    }

    return next_particles;
}
