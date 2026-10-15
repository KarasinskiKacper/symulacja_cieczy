#include "../include/physics_engine.hpp"

#include <physics_config.hpp>

#include <algorithm>
#include <cstddef>
#include <numbers>

namespace Physics {

PhysicsEngine& PhysicsEngine::instance() {
    static PhysicsEngine engine;
    return engine;
}

void PhysicsEngine::init_particles() {
    bool shift_row = false;
    for (float x = Config::BOX_LEFT + 0; x < Config::BOX_RIGHT - 0; x += Config::PARTICLE_SPACING) {

        for (float y = 100; y < 500; y += Config::PARTICLE_SPACING) {
            shift_row = !shift_row;
            const float offset = shift_row ? 1.f : 0;

            particles_.push_back({
                .position = glm::vec2(x + offset, y),
                .velocity = glm::vec2(0, 0),
                .force = glm::vec2(0, 0),
                .density = 0.0f,
                .pressure = 0.0f
            });
        }
    }
}

void PhysicsEngine::compute_density_pressure() {
    constexpr float poly6 = 4.0f / (std::numbers::pi_v<float> * Config::HSQ * Config::HSQ * Config::HSQ * Config::HSQ);

    for (auto& particle : particles_) {
        particle.density = 0.0f;
        for (const auto& neighbor : particles_) {
            glm::vec2 delta = neighbor.position - particle.position;

            if (const float distance_squared = glm::dot(delta, delta); distance_squared < Config::HSQ) {
                const float kernel_term = Config::HSQ - distance_squared;
                particle.density += Config::MASS * poly6 * kernel_term * kernel_term * kernel_term;
            }
        }
        particle.pressure = std::max(0.0f, Config::GAS_CONST * (particle.density - Config::REST_DENSITY));
    }
}

void PhysicsEngine::compute_forces() {
    constexpr float h_pow5 = Config::HSQ * Config::HSQ * Config::H;
    constexpr float spiky_gradient = -30.0f / (std::numbers::pi_v<float> * h_pow5);
    constexpr float viscosity_laplacian = 40.0f / (std::numbers::pi_v<float> * h_pow5);

    for (std::size_t i = 0; i < particles_.size(); ++i) {
        glm::vec2 pressure_force(0.0f);
        glm::vec2 viscosity_force(0.0f);
        glm::vec2 gravity_force = glm::vec2(0.0f, Config::GRAVITY) * particles_[i].density;

        for (std::size_t j = 0; j < particles_.size(); ++j) {
            if (i == j) continue;

            glm::vec2 delta = particles_[j].position - particles_[i].position;

            if (float distance = glm::length(delta); distance < Config::H && distance > 0.001f) {
                glm::vec2 direction = delta / distance;
                const float distance_term = Config::H - distance;

                // 1. Pressure force (spiky_gradient < 0, so the force pushes away from neighbor j)
                pressure_force += direction * Config::MASS * (particles_[i].pressure + particles_[j].pressure) / (2.0f * particles_[j].density) * spiky_gradient * (distance_term * distance_term);

                // 2. Viscosity force
                viscosity_force += Config::VISCOSITY * Config::MASS * (particles_[j].velocity - particles_[i].velocity) / particles_[j].density * viscosity_laplacian * distance_term;
            }
        }
        particles_[i].force = pressure_force + viscosity_force + gravity_force;
    }
}

void PhysicsEngine::update(const glm::vec2 mouse_position) {
    compute_density_pressure();
    compute_forces();

    // 1. Motion integration + mouse force
    for (auto& particle : particles_) {
        // Compute the distance from the particle to the mouse
        glm::vec2 direction_to_mouse = particle.position - mouse_position;
        const float distance_to_mouse = glm::length(direction_to_mouse);

        glm::vec2 mouse_force(0.0f);
        // If the mouse is close to the particle, generate a repulsive force
        if (distance_to_mouse < Config::MOUSE_RADIUS && distance_to_mouse > 0.001f) {
            glm::vec2 direction = direction_to_mouse / distance_to_mouse; // Unit vector from the mouse to the particle
            const float force_factor = (Config::MOUSE_RADIUS - distance_to_mouse) / Config::MOUSE_RADIUS; // The closer to the mouse, the stronger the repulsion
            mouse_force = direction * Config::MOUSE_FORCE * force_factor;
        }

        // Add the mouse force to the equation of motion (divided by density, like the other forces)
        particle.velocity += Config::DT * ((particle.force + mouse_force) / particle.density);
        particle.position += Config::DT * particle.velocity;
    }

    // Hard collisions between particles
    constexpr float min_distance = Config::PARTICLE_RADIUS * 2.0f;
    for (std::size_t i = 0; i < particles_.size(); ++i) {
        for (std::size_t j = i + 1; j < particles_.size(); ++j) {
            glm::vec2 delta = particles_[j].position - particles_[i].position;

            if (float distance = glm::length(delta); distance < min_distance && distance > 0.001f) {
                glm::vec2 normal = delta / distance;
                const float overlap = min_distance - distance;

                particles_[i].position -= normal * (overlap * 0.5f);
                particles_[j].position += normal * (overlap * 0.5f);

                glm::vec2 relative_velocity = particles_[j].velocity - particles_[i].velocity;

                if (
                    const float velocity_along_normal = glm::dot(relative_velocity, normal);
                    velocity_along_normal < 0
                ) {
                    const float impulse_scalar = -(1.0f + 0.2f) * velocity_along_normal / 2.0f;
                    const glm::vec2 impulse = impulse_scalar * normal;

                    particles_[i].velocity -= impulse;
                    particles_[j].velocity += impulse;
                }
            }
        }
    }

    for (auto& particle : particles_) {
        if (particle.position.x < Config::BOX_LEFT + Config::PARTICLE_RADIUS) {
            particle.position.x = Config::BOX_LEFT + Config::PARTICLE_RADIUS;
            particle.velocity.x *= Config::COLLISIONS_DAMPING;
        }
        if (particle.position.x > Config::BOX_RIGHT - Config::PARTICLE_RADIUS) {
            particle.position.x = Config::BOX_RIGHT - Config::PARTICLE_RADIUS;
            particle.velocity.x *= Config::COLLISIONS_DAMPING;
        }
        if (particle.position.y > Config::BOX_BOTTOM - Config::PARTICLE_RADIUS) {
            particle.position.y = Config::BOX_BOTTOM - Config::PARTICLE_RADIUS;
            particle.velocity.y *= Config::COLLISIONS_DAMPING;
        }
        if (particle.position.y < Config::BOX_TOP + Config::PARTICLE_RADIUS) {
            particle.position.y = Config::BOX_TOP + Config::PARTICLE_RADIUS;
            particle.velocity.y *= Config::COLLISIONS_DAMPING;
        }
    }
}

}
