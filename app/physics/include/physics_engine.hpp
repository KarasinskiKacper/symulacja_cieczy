#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace Physics {
    struct Particle {
        glm::vec2 position;
        glm::vec2 velocity;
        glm::vec2 force;
        float density;
        float pressure;
    };

    class PhysicsEngine {
    public:
        static PhysicsEngine& instance();

        PhysicsEngine(const PhysicsEngine&) = delete;
        PhysicsEngine& operator=(const PhysicsEngine&) = delete;

        void init_particles();
        void update(glm::vec2 mouse_position);

        [[nodiscard]] const std::vector<Particle>& get_particles() const { return particles_; }

    private:
        PhysicsEngine() = default;
        ~PhysicsEngine() = default;

        void compute_density_pressure();
        void compute_forces();

        std::vector<Particle> particles_;
    };
}
