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
    inverse_density_.assign(current_.padded_size(), 1.0f);
    pressure_.assign(current_.padded_size(), 0.0f);
}

void PhysicsEngine::compute_density_pressure() {
    constexpr float poly6 = 4.0f / (std::numbers::pi_v<float> * Config::HSQ * Config::HSQ * Config::HSQ * Config::HSQ);

    const std::size_t count = current_.size();
    const std::size_t padded_count = current_.padded_size();
    const float* __restrict pos_x = current_.pos_x.data();
    const float* __restrict pos_y = current_.pos_y.data();
    float* __restrict density = density_.data();
    float* __restrict inverse_density = inverse_density_.data();
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
        inverse_density[i] = 1.0f / density_sum;
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
    const float* __restrict inverse_density = inverse_density_.data();
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
            floatv inverse_density_j;
            xj.copy_from(pos_x + j, stdx::vector_aligned);
            yj.copy_from(pos_y + j, stdx::vector_aligned);
            vxj.copy_from(vel_x + j, stdx::vector_aligned);
            vyj.copy_from(vel_y + j, stdx::vector_aligned);
            pj.copy_from(pressure + j, stdx::vector_aligned);
            inverse_density_j.copy_from(inverse_density + j, stdx::vector_aligned);

            const floatv dx = xj - xi;
            const floatv dy = yj - yi;
            const floatv distance = stdx::sqrt(dx * dx + dy * dy);

            floatv distance_term = h - distance;
            stdx::where(!(distance < h && distance > min_distance), distance_term) = 0.0f;

            const floatv inverse_distance = 1.0f / stdx::max(distance, min_distance);

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

    constexpr float impulse_factor = (1.0f + 0.2f) / 2.0f;

    const std::size_t count = current_.size();
    const std::size_t padded_count = current_.padded_size();
    const float* __restrict pos_x = current_.pos_x.data();
    const float* __restrict pos_y = current_.pos_y.data();
    const float* __restrict vel_x = current_.vel_x.data();
    const float* __restrict vel_y = current_.vel_y.data();
    float* __restrict next_pos_x = next_.pos_x.data();
    float* __restrict next_pos_y = next_.pos_y.data();
    float* __restrict next_vel_x = next_.vel_x.data();
    float* __restrict next_vel_y = next_.vel_y.data();

    const floatv contact_distance = min_distance;
    const floatv epsilon = 0.001f;
    const floatv zero = 0.0f;

    for (std::size_t i = 0; i < count; ++i) {
        const glm::vec2 position_i(pos_x[i], pos_y[i]);
        const glm::vec2 velocity_i(vel_x[i], vel_y[i]);

        // Jacobi: corrections from all neighbors are computed from the old state and summed.
        // Neighbor j computes the same pair with the opposite sign, so momentum is conserved.
        const floatv xi = pos_x[i];
        const floatv yi = pos_y[i];
        const floatv vxi = vel_x[i];
        const floatv vyi = vel_y[i];
        floatv push_x = 0.0f;
        floatv push_y = 0.0f;
        floatv impulse_x = 0.0f;
        floatv impulse_y = 0.0f;

        for (std::size_t j = 0; j < padded_count; j += floatv::size()) {
            floatv xj;
            floatv yj;
            floatv vxj;
            floatv vyj;
            xj.copy_from(pos_x + j, stdx::vector_aligned);
            yj.copy_from(pos_y + j, stdx::vector_aligned);
            vxj.copy_from(vel_x + j, stdx::vector_aligned);
            vyj.copy_from(vel_y + j, stdx::vector_aligned);

            const floatv dx = xj - xi;
            const floatv dy = yj - yi;
            const floatv distance = stdx::sqrt(dx * dx + dy * dy);

            floatv inverse_distance = 1.0f / stdx::max(distance, epsilon);
            stdx::where(!(distance < contact_distance && distance > epsilon), inverse_distance) = 0.0f;

            const floatv normal_x = dx * inverse_distance;
            const floatv normal_y = dy * inverse_distance;
            const floatv overlap = contact_distance - distance;

            const floatv velocity_along_normal = (vxj - vxi) * normal_x + (vyj - vyi) * normal_y;
            const floatv approach = stdx::min(velocity_along_normal, zero);

            push_x += normal_x * overlap;
            push_y += normal_y * overlap;
            impulse_x += normal_x * approach;
            impulse_y += normal_y * approach;
        }

        const glm::vec2 position_correction = -0.5f * glm::vec2(stdx::reduce(push_x), stdx::reduce(push_y));
        const glm::vec2 velocity_correction = impulse_factor * glm::vec2(stdx::reduce(impulse_x), stdx::reduce(impulse_y));

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
