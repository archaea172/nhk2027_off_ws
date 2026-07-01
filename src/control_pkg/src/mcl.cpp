#include "mcl.hpp"

Mcl::Mcl(const mclParameters& parameters)
: parameters_(parameters)
{
    this->now_pose_ = this->parameters_.initial_pose;

    std::normal_distribution<double> noise_x(0.0, this->parameters_.sigma_x);
    std::normal_distribution<double> noise_y(0.0, this->parameters_.sigma_y);
    std::normal_distribution<double> noise_theta(0.0, this->parameters_.sigma_theta);

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
    double dt     = this->parameters_.dt;
    double dx     = vel_feedback(0) * dt;
    double dy     = vel_feedback(1) * dt;
    double dtheta = vel_feedback(2) * dt;

    std::normal_distribution<double> noise_x(0.0, this->parameters_.sigma_x);
    std::normal_distribution<double> noise_y(0.0, this->parameters_.sigma_y);
    std::normal_distribution<double> noise_theta(0.0, this->parameters_.sigma_theta);

    for (particle now_particle : now_particles)
    {
        particle next_particle;
        double cos_t = std::cos(now_particle.pose.theta);
        double sin_t = std::sin(now_particle.pose.theta);
    
        double noisy_dx = dx + noise_x(this->rng_);
        double noisy_dy = dy + noise_y(this->rng_);
        double noisy_dtheta = dtheta + noise_theta(this->rng_);

        next_particle.pose.x     = now_particle.pose.x     + noisy_dx * cos_t - noisy_dy * sin_t;
        next_particle.pose.y     = now_particle.pose.y     + noisy_dy * sin_t + noisy_dx * cos_t;
        next_particle.pose.theta = now_particle.pose.theta + noisy_dtheta;
        next_particle.weight = now_particle.weight;
        next_particles.push_back(next_particle);
    }

    return next_particles;
}
