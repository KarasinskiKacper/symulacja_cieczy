#pragma once

#include "particle_data.hpp"

#include <glm/glm.hpp>
#include <cstddef>
#include <span>

namespace Physics {
    class PhysicsEngine {
    public:
        static PhysicsEngine& instance();

        PhysicsEngine(const PhysicsEngine&) = delete;
        PhysicsEngine& operator=(const PhysicsEngine&) = delete;

        void init_particles();
        void update(glm::vec2 mouse_position);

        [[nodiscard]] std::size_t particle_count() const { return particles_.size(); }
        [[nodiscard]] std::span<const float> positions_x() const { return particles_.pos_x; }
        [[nodiscard]] std::span<const float> positions_y() const { return particles_.pos_y; }

    private:
        PhysicsEngine() = default;
        ~PhysicsEngine() = default;

        void compute_density_pressure();
        void compute_forces();

        ParticleData particles_;
    };
}
