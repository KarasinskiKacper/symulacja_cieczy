#include "../include/physics_engine.hpp"

#include <physics_config.hpp>

#include <algorithm>
#include <cstddef>
#include <experimental/simd>
#include <numbers>
#include <utility>
#include <vector>

namespace Physics {

namespace stdx = std::experimental;
using floatv = stdx::native_simd<float>;

static_assert(ParticleData::PADDING % floatv::size() == 0);

PhysicsEngine& PhysicsEngine::instance() {
    static PhysicsEngine engine;
    return engine;
}

void PhysicsEngine::init_particles() {
    std::vector<glm::vec2> positions;

    bool shift_row = false;
    for (float x = Config::BOX_LEFT + 0; x < Config::BOX_RIGHT - 0; x += Config::PARTICLE_SPACING) {

        for (float y = 100; y < 500; y += Config::PARTICLE_SPACING) {
            shift_row = !shift_row;
            const float offset = shift_row ? 1.f : 0;

            positions.emplace_back(x + offset, y);
        }
    }

    const std::size_t count = positions.size();
    current_.reset(count);
    next_.reset(count);
    for (std::size_t i = 0; i < count; ++i) {
        current_.pos_x[i] = positions[i].x;
        current_.pos_y[i] = positions[i].y;
    }

    density_.assign(current_.padded_size(), 1.0f);
    pressure_.assign(current_.padded_size(), 0.0f);
}

void PhysicsEngine::compute_density_pressure() {
    constexpr float poly6 = 4.0f / (std::numbers::pi_v<float> * Config::HSQ * Config::HSQ * Config::HSQ * Config::HSQ);

    const std::size_t count = current_.size();
    const std::size_t padded_count = current_.padded_size();
    const float* __restrict pos_x = current_.pos_x.data();
    const float* __restrict pos_y = current_.pos_y.data();
    float* __restrict density = density_.data();
    float* __restrict pressure = pressure_.data();

    const floatv hsq = Config::HSQ;

    for (std::size_t i = 0; i < count; ++i) {
        const floatv xi = pos_x[i];
        const floatv yi = pos_y[i];
        floatv kernel_sum = 0.0f;

        for (std::size_t j = 0; j < padded_count; j += floatv::size()) {
            floatv xj;
            floatv yj;
            xj.copy_from(pos_x + j, stdx::vector_aligned);
            yj.copy_from(pos_y + j, stdx::vector_aligned);

            const floatv dx = xj - xi;
            const floatv dy = yj - yi;
            const floatv distance_squared = dx * dx + dy * dy;
            const floatv kernel_term = stdx::max(hsq - distance_squared, floatv(0.0f));

            kernel_sum += kernel_term * kernel_term * kernel_term;
        }

        const float density_sum = Config::MASS * poly6 * stdx::reduce(kernel_sum);
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

    constexpr float pressure_coefficient = Config::MASS * spiky_gradient * 0.5f;
    constexpr float viscosity_coefficient = Config::VISCOSITY * Config::MASS * viscosity_laplacian;

    const std::size_t count = current_.size();
    const std::size_t padded_count = current_.padded_size();
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

    const floatv h = Config::H;
    const floatv min_distance = 0.001f;

    for (std::size_t i = 0; i < count; ++i) {
        glm::vec2 position(pos_x[i], pos_y[i]);
        glm::vec2 velocity(vel_x[i], vel_y[i]);

        // 1. SPH forces from neighbors
        const floatv xi = pos_x[i];
        const floatv yi = pos_y[i];
        const floatv vxi = vel_x[i];
        const floatv vyi = vel_y[i];
        const floatv pi = pressure[i];
        floatv force_x = 0.0f;
        floatv force_y = 0.0f;

        for (std::size_t j = 0; j < padded_count; j += floatv::size()) {
            floatv xj;
            floatv yj;
            floatv vxj;
            floatv vyj;
            floatv pj;
            floatv dj;
            xj.copy_from(pos_x + j, stdx::vector_aligned);
            yj.copy_from(pos_y + j, stdx::vector_aligned);
            vxj.copy_from(vel_x + j, stdx::vector_aligned);
            vyj.copy_from(vel_y + j, stdx::vector_aligned);
            pj.copy_from(pressure + j, stdx::vector_aligned);
            dj.copy_from(density + j, stdx::vector_aligned);

            const floatv dx = xj - xi;
            const floatv dy = yj - yi;
            const floatv distance = stdx::sqrt(dx * dx + dy * dy);

            floatv distance_term = h - distance;
            stdx::where(!(distance < h && distance > min_distance), distance_term) = 0.0f;

            const floatv inverse_distance = 1.0f / stdx::max(distance, min_distance);
            const floatv inverse_density_j = 1.0f / dj;

            const floatv pressure_weight = pressure_coefficient * (pi + pj) * inverse_density_j * distance_term * distance_term * inverse_distance;
            const floatv viscosity_weight = viscosity_coefficient * inverse_density_j * distance_term;

            force_x += dx * pressure_weight + (vxj - vxi) * viscosity_weight;
            force_y += dy * pressure_weight + (vyj - vyi) * viscosity_weight;
        }

        const glm::vec2 gravity_force = glm::vec2(0.0f, Config::GRAVITY) * density[i];
        const glm::vec2 force = glm::vec2(stdx::reduce(force_x), stdx::reduce(force_y)) + gravity_force;

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
        glm::vec2 velocity = velocity_i + Config::COLLISION_RELAXATION * velocity_correction;

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
