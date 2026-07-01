#include "mcl.hpp"

Mcl::Mcl()
{

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

    for (particle now_particle : now_particles)
    {
        particle next_particle;
        double cos_t = std::cos(now_particle.pose.theta);
        double sin_t = std::sin(now_particle.pose.theta);

        next_particle.pose.x     = now_particle.pose.x     + dx * cos_t - dy * sin_t;
        next_particle.pose.y     = now_particle.pose.y     + dx * sin_t + dy * cos_t;
        next_particle.pose.theta = now_particle.pose.theta + dtheta;
        next_particle.weight = now_particle.weight;
        next_particles.push_back(next_particle);
    }

    return next_particles;
}
