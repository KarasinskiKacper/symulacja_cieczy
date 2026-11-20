#pragma once

#include "frame_exchange.hpp"
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
        void write_frame(RenderFrame& frame) const;

        [[nodiscard]] std::size_t particle_count() const { return current_.size(); }
        [[nodiscard]] std::span<const float> positions_x() const { return std::span(current_.pos_x).first(current_.size()); }
        [[nodiscard]] std::span<const float> positions_y() const { return std::span(current_.pos_y).first(current_.size()); }

    private:
        static constexpr int CHUNK_SIZE = 64;
        static_assert(CHUNK_SIZE % ParticleData::PADDING == 0);

        PhysicsEngine() = default;
        ~PhysicsEngine() = default;

        void compute_density_pressure();
        void integrate(glm::vec2 mouse_position);
        void resolve_collisions();

        ParticleData current_;
        ParticleData next_;

        FloatArray density_;
        FloatArray inverse_density_;
        FloatArray pressure_;

        int thread_count_ = 1;
    };
}
