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

            particles_.push_back(x + offset, y);
        }
    }
}

void PhysicsEngine::compute_density_pressure() {
    constexpr float poly6 = 4.0f / (std::numbers::pi_v<float> * Config::HSQ * Config::HSQ * Config::HSQ * Config::HSQ);

    const std::size_t count = particles_.size();
    const float* __restrict pos_x = particles_.pos_x.data();
    const float* __restrict pos_y = particles_.pos_y.data();
    float* __restrict density = particles_.density.data();
    float* __restrict pressure = particles_.pressure.data();

    for (std::size_t i = 0; i < count; ++i) {
        float density_sum = 0.0f;
        for (std::size_t j = 0; j < count; ++j) {
            const float dx = pos_x[j] - pos_x[i];
            const float dy = pos_y[j] - pos_y[i];

            if (const float distance_squared = dx * dx + dy * dy; distance_squared < Config::HSQ) {
                const float kernel_term = Config::HSQ - distance_squared;
                density_sum += Config::MASS * poly6 * kernel_term * kernel_term * kernel_term;
            }
        }
        density[i] = density_sum;
        pressure[i] = std::max(0.0f, Config::GAS_CONST * (density_sum - Config::REST_DENSITY));
    }
}

void PhysicsEngine::compute_forces() {
    constexpr float h_pow5 = Config::HSQ * Config::HSQ * Config::H;
    constexpr float spiky_gradient = -30.0f / (std::numbers::pi_v<float> * h_pow5);
    constexpr float viscosity_laplacian = 40.0f / (std::numbers::pi_v<float> * h_pow5);

    const std::size_t count = particles_.size();
    const float* __restrict pos_x = particles_.pos_x.data();
    const float* __restrict pos_y = particles_.pos_y.data();
    const float* __restrict vel_x = particles_.vel_x.data();
    const float* __restrict vel_y = particles_.vel_y.data();
    const float* __restrict density = particles_.density.data();
    const float* __restrict pressure = particles_.pressure.data();
    float* __restrict force_x = particles_.force_x.data();
    float* __restrict force_y = particles_.force_y.data();

    for (std::size_t i = 0; i < count; ++i) {
        const glm::vec2 position_i(pos_x[i], pos_y[i]);
        const glm::vec2 velocity_i(vel_x[i], vel_y[i]);

        glm::vec2 pressure_force(0.0f);
        glm::vec2 viscosity_force(0.0f);
        glm::vec2 gravity_force = glm::vec2(0.0f, Config::GRAVITY) * density[i];

        for (std::size_t j = 0; j < count; ++j) {
            if (i == j) continue;

            glm::vec2 delta = glm::vec2(pos_x[j], pos_y[j]) - position_i;

            if (float distance = glm::length(delta); distance < Config::H && distance > 0.001f) {
                glm::vec2 direction = delta / distance;
                const float distance_term = Config::H - distance;

                // 1. Pressure force (spiky_gradient < 0, so the force pushes away from neighbor j)
                pressure_force += direction * Config::MASS * (pressure[i] + pressure[j]) / (2.0f * density[j]) * spiky_gradient * (distance_term * distance_term);

                // 2. Viscosity force
                viscosity_force += Config::VISCOSITY * Config::MASS * (glm::vec2(vel_x[j], vel_y[j]) - velocity_i) / density[j] * viscosity_laplacian * distance_term;
            }
        }
        const glm::vec2 force = pressure_force + viscosity_force + gravity_force;
        force_x[i] = force.x;
        force_y[i] = force.y;
    }
}

void PhysicsEngine::update(const glm::vec2 mouse_position) {
    compute_density_pressure();
    compute_forces();

    const std::size_t count = particles_.size();
    float* __restrict pos_x = particles_.pos_x.data();
    float* __restrict pos_y = particles_.pos_y.data();
    float* __restrict vel_x = particles_.vel_x.data();
    float* __restrict vel_y = particles_.vel_y.data();
    const float* __restrict force_x = particles_.force_x.data();
    const float* __restrict force_y = particles_.force_y.data();
    const float* __restrict density = particles_.density.data();

    // 1. Motion integration + mouse force
    for (std::size_t i = 0; i < count; ++i) {
        glm::vec2 position(pos_x[i], pos_y[i]);
        glm::vec2 velocity(vel_x[i], vel_y[i]);

        // Compute the distance from the particle to the mouse
        glm::vec2 direction_to_mouse = position - mouse_position;
        const float distance_to_mouse = glm::length(direction_to_mouse);

        glm::vec2 mouse_force(0.0f);
        // If the mouse is close to the particle, generate a repulsive force
        if (distance_to_mouse < Config::MOUSE_RADIUS && distance_to_mouse > 0.001f) {
            glm::vec2 direction = direction_to_mouse / distance_to_mouse; // Unit vector from the mouse to the particle
            const float force_factor = (Config::MOUSE_RADIUS - distance_to_mouse) / Config::MOUSE_RADIUS; // The closer to the mouse, the stronger the repulsion
            mouse_force = direction * Config::MOUSE_FORCE * force_factor;
        }

        // Add the mouse force to the equation of motion (divided by density, like the other forces)
        velocity += Config::DT * ((glm::vec2(force_x[i], force_y[i]) + mouse_force) / density[i]);
        position += Config::DT * velocity;

        pos_x[i] = position.x;
        pos_y[i] = position.y;
        vel_x[i] = velocity.x;
        vel_y[i] = velocity.y;
    }

    // Hard collisions between particles
    constexpr float min_distance = Config::PARTICLE_RADIUS * 2.0f;
    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = i + 1; j < count; ++j) {
            glm::vec2 delta(pos_x[j] - pos_x[i], pos_y[j] - pos_y[i]);

            if (float distance = glm::length(delta); distance < min_distance && distance > 0.001f) {
                glm::vec2 normal = delta / distance;
                const float overlap = min_distance - distance;
                const glm::vec2 correction = normal * (overlap * 0.5f);

                pos_x[i] -= correction.x;
                pos_y[i] -= correction.y;
                pos_x[j] += correction.x;
                pos_y[j] += correction.y;

                glm::vec2 relative_velocity(vel_x[j] - vel_x[i], vel_y[j] - vel_y[i]);

                if (
                    const float velocity_along_normal = glm::dot(relative_velocity, normal);
                    velocity_along_normal < 0
                ) {
                    const float impulse_scalar = -(1.0f + 0.2f) * velocity_along_normal / 2.0f;
                    const glm::vec2 impulse = impulse_scalar * normal;

                    vel_x[i] -= impulse.x;
                    vel_y[i] -= impulse.y;
                    vel_x[j] += impulse.x;
                    vel_y[j] += impulse.y;
                }
            }
        }
    }

    for (std::size_t i = 0; i < count; ++i) {
        if (pos_x[i] < Config::BOX_LEFT + Config::PARTICLE_RADIUS) {
            pos_x[i] = Config::BOX_LEFT + Config::PARTICLE_RADIUS;
            vel_x[i] *= Config::COLLISIONS_DAMPING;
        }
        if (pos_x[i] > Config::BOX_RIGHT - Config::PARTICLE_RADIUS) {
            pos_x[i] = Config::BOX_RIGHT - Config::PARTICLE_RADIUS;
            vel_x[i] *= Config::COLLISIONS_DAMPING;
        }
        if (pos_y[i] > Config::BOX_BOTTOM - Config::PARTICLE_RADIUS) {
            pos_y[i] = Config::BOX_BOTTOM - Config::PARTICLE_RADIUS;
            vel_y[i] *= Config::COLLISIONS_DAMPING;
        }
        if (pos_y[i] < Config::BOX_TOP + Config::PARTICLE_RADIUS) {
            pos_y[i] = Config::BOX_TOP + Config::PARTICLE_RADIUS;
            vel_y[i] *= Config::COLLISIONS_DAMPING;
        }
    }
}

}
