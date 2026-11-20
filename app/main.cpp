#include <physics.hpp>
#include <visualization.hpp>

#include <chrono>

int main() {
    auto& physics = Physics::PhysicsEngine::instance();
    auto& renderer = Visualization::Renderer::instance();

    physics.init_particles();

    while (renderer.is_open()) {
        renderer.process_events();

        const glm::vec2 mouse_position = renderer.get_mouse_position();

        auto physics_start = std::chrono::steady_clock::now();
        physics.update(mouse_position);
        const float physics_time = std::chrono::duration<float>(std::chrono::steady_clock::now() - physics_start).count();

        renderer.update_stats(physics_time, physics.particle_count());
        renderer.render(physics.positions_x(), physics.positions_y(), mouse_position);
    }

    return 0;
}
