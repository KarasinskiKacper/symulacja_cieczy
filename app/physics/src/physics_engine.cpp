#include "../include/physics_engine.hpp"

#include <physics_config.hpp>

#include <algorithm>
#include <cstddef>
#include <numbers>
#include <utility>

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

            current_.push_back(x + offset, y);
        }
    }

    const std::size_t count = current_.size();
    next_.resize(count);
    density_.resize(count);
    pressure_.resize(count);
}

void PhysicsEngine::compute_density_pressure() {
    constexpr float poly6 = 4.0f / (std::numbers::pi_v<float> * Config::HSQ * Config::HSQ * Config::HSQ * Config::HSQ);

    const std::size_t count = current_.size();
    const float* __restrict pos_x = current_.pos_x.data();
    const float* __restrict pos_y = current_.pos_y.data();
    float* __restrict density = density_.data();
    float* __restrict pressure = pressure_.data();

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

// Forces and integration in one pass: the force of particle i is needed only for its own integration.
// Reads current_ (viscosity needs the old velocities of neighbors), writes next_.
void PhysicsEngine::integrate(const glm::vec2 mouse_position) {
    constexpr float h_pow5 = Config::HSQ * Config::HSQ * Config::H;
    constexpr float spiky_gradient = -30.0f / (std::numbers::pi_v<float> * h_pow5);
    constexpr float viscosity_laplacian = 40.0f / (std::numbers::pi_v<float> * h_pow5);

    const std::size_t count = current_.size();
    const float* __restrict pos_x = current_.pos_x.data();
    const float* __restrict pos_y = current_.pos_y.data();
    const float* __restrict vel_x = current_.vel_x.data();
    const float* __restrict vel_y = current_.vel_y.data();
    const float* __restrict density = density_.data();
    const float* __restrict pressure = pressure_.data();
    float* __restrict next_pos_x = next_.pos_x.data();
    float* __restrict next_pos_y = next_.pos_y.data();
    float* __restrict next_vel_x = next_.vel_x.data();
    float* __restrict next_vel_y = next_.vel_y.data();

    for (std::size_t i = 0; i < count; ++i) {
        glm::vec2 position(pos_x[i], pos_y[i]);
        glm::vec2 velocity(vel_x[i], vel_y[i]);

        // 1. SPH forces from neighbors
        glm::vec2 pressure_force(0.0f);
        glm::vec2 viscosity_force(0.0f);
        glm::vec2 gravity_force = glm::vec2(0.0f, Config::GRAVITY) * density[i];

        for (std::size_t j = 0; j < count; ++j) {
            if (i == j) continue;

            glm::vec2 delta = glm::vec2(pos_x[j], pos_y[j]) - position;

            if (float distance = glm::length(delta); distance < Config::H && distance > 0.001f) {
                glm::vec2 direction = delta / distance;
                const float distance_term = Config::H - distance;

                // Pressure force (spiky_gradient < 0, so the force pushes away from neighbor j)
                pressure_force += direction * Config::MASS * (pressure[i] + pressure[j]) / (2.0f * density[j]) * spiky_gradient * (distance_term * distance_term);

                viscosity_force += Config::VISCOSITY * Config::MASS * (glm::vec2(vel_x[j], vel_y[j]) - velocity) / density[j] * viscosity_laplacian * distance_term;
            }
        }
        const glm::vec2 force = pressure_force + viscosity_force + gravity_force;

        // 2. Mouse force
        glm::vec2 direction_to_mouse = position - mouse_position;
        const float distance_to_mouse = glm::length(direction_to_mouse);

        glm::vec2 mouse_force(0.0f);
        // If the mouse is close to the particle, generate a repulsive force
        if (distance_to_mouse < Config::MOUSE_RADIUS && distance_to_mouse > 0.001f) {
            glm::vec2 direction = direction_to_mouse / distance_to_mouse; // Unit vector from the mouse to the particle
            const float force_factor = (Config::MOUSE_RADIUS - distance_to_mouse) / Config::MOUSE_RADIUS; // The closer to the mouse, the stronger the repulsion
            mouse_force = direction * Config::MOUSE_FORCE * force_factor;
        }

        // 3. Motion integration (the mouse force is divided by density, like the other forces)
        velocity += Config::DT * ((force + mouse_force) / density[i]);
        position += Config::DT * velocity;

        next_pos_x[i] = position.x;
        next_pos_y[i] = position.y;
        next_vel_x[i] = velocity.x;
        next_vel_y[i] = velocity.y;
    }
}

void PhysicsEngine::resolve_collisions() {
    constexpr float min_distance = Config::PARTICLE_RADIUS * 2.0f;

    const std::size_t count = current_.size();
    const float* __restrict pos_x = current_.pos_x.data();
    const float* __restrict pos_y = current_.pos_y.data();
    const float* __restrict vel_x = current_.vel_x.data();
    const float* __restrict vel_y = current_.vel_y.data();
    float* __restrict next_pos_x = next_.pos_x.data();
    float* __restrict next_pos_y = next_.pos_y.data();
    float* __restrict next_vel_x = next_.vel_x.data();
    float* __restrict next_vel_y = next_.vel_y.data();

    for (std::size_t i = 0; i < count; ++i) {
        const glm::vec2 position_i(pos_x[i], pos_y[i]);
        const glm::vec2 velocity_i(vel_x[i], vel_y[i]);

        // Jacobi: corrections from all neighbors are computed from the old state and summed.
        // Neighbor j computes the same pair with the opposite sign, so momentum is conserved.
        glm::vec2 position_correction(0.0f);
        glm::vec2 velocity_correction(0.0f);

        for (std::size_t j = 0; j < count; ++j) {
            if (i == j) continue;

            glm::vec2 delta = glm::vec2(pos_x[j], pos_y[j]) - position_i;

            if (float distance = glm::length(delta); distance < min_distance && distance > 0.001f) {
                glm::vec2 normal = delta / distance;
                const float overlap = min_distance - distance;

                position_correction -= normal * (overlap * 0.5f);

                glm::vec2 relative_velocity = glm::vec2(vel_x[j], vel_y[j]) - velocity_i;

                if (
                    const float velocity_along_normal = glm::dot(relative_velocity, normal);
                    velocity_along_normal < 0
                ) {
                    const float impulse_scalar = -(1.0f + 0.2f) * velocity_along_normal / 2.0f;
                    velocity_correction -= impulse_scalar * normal;
                }
            }
        }

        glm::vec2 position = position_i + Config::COLLISION_RELAXATION * position_correction;
        glm::vec2 velocity = velocity_i + velocity_correction;

        // Walls
        if (position.x < Config::BOX_LEFT + Config::PARTICLE_RADIUS) {
            position.x = Config::BOX_LEFT + Config::PARTICLE_RADIUS;
            velocity.x *= Config::COLLISIONS_DAMPING;
        }
        if (position.x > Config::BOX_RIGHT - Config::PARTICLE_RADIUS) {
            position.x = Config::BOX_RIGHT - Config::PARTICLE_RADIUS;
            velocity.x *= Config::COLLISIONS_DAMPING;
        }
        if (position.y > Config::BOX_BOTTOM - Config::PARTICLE_RADIUS) {
            position.y = Config::BOX_BOTTOM - Config::PARTICLE_RADIUS;
            velocity.y *= Config::COLLISIONS_DAMPING;
        }
        if (position.y < Config::BOX_TOP + Config::PARTICLE_RADIUS) {
            position.y = Config::BOX_TOP + Config::PARTICLE_RADIUS;
            velocity.y *= Config::COLLISIONS_DAMPING;
        }

        next_pos_x[i] = position.x;
        next_pos_y[i] = position.y;
        next_vel_x[i] = velocity.x;
        next_vel_y[i] = velocity.y;
    }
}

void PhysicsEngine::update(const glm::vec2 mouse_position) {
    compute_density_pressure();

    integrate(mouse_position);
    std::swap(current_, next_);

    resolve_collisions();
    std::swap(current_, next_);
}

}
