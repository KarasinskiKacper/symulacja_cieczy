#include <physics.hpp>
#include <physics_config.hpp>
#include <visualization.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <stop_token>
#include <thread>

namespace {
    void run_physics(const std::stop_token& stop, Physics::PhysicsEngine& physics, Physics::FrameExchange& exchange,
                     const std::atomic<glm::vec2>& mouse_position) {
        using clock = std::chrono::steady_clock;
        constexpr auto period = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(Config::DT));

        std::uint64_t step = 0;
        auto next_step = clock::now();

        while (!stop.stop_requested()) {
            const auto physics_start = clock::now();
            physics.update(mouse_position.load(std::memory_order_relaxed));
            const float physics_time = std::chrono::duration<float>(clock::now() - physics_start).count();

            Physics::RenderFrame& frame = exchange.back();
            physics.write_frame(frame);
            frame.step = ++step;
            frame.physics_time = physics_time;
            exchange.publish();

            if constexpr (Config::PHYSICS_REAL_TIME) {
                next_step += period;
                if (const auto now = clock::now(); next_step < now) {
                    next_step = now;
                } else {
                    std::this_thread::sleep_until(next_step);
                }
            }
        }
    }
}

int main() {
    auto& physics = Physics::PhysicsEngine::instance();
    auto& renderer = Visualization::Renderer::instance();

    physics.init_particles();

    Physics::FrameExchange exchange;
    std::atomic<glm::vec2> mouse_position{renderer.get_mouse_position()};

    std::jthread physics_thread([&](const std::stop_token& stop) {
        run_physics(stop, physics, exchange, mouse_position);
    });

    while (renderer.is_open()) {
        renderer.process_events();

        const glm::vec2 current_mouse_position = renderer.get_mouse_position();
        mouse_position.store(current_mouse_position, std::memory_order_relaxed);

        const Physics::RenderFrame& frame = exchange.acquire();
        renderer.update_stats(frame.physics_time, frame.step, frame.x.size());
        renderer.render(frame.x, frame.y, current_mouse_position);
    }

    return 0;
}
