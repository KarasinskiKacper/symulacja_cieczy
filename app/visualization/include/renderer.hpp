#pragma once

#include <physics_engine.hpp>

#include <SFML/Graphics.hpp>
#include <glm/glm.hpp>

#include <cstddef>
#include <vector>

namespace Visualization {
    class Renderer {
    public:
        static Renderer& instance();

        Renderer(const Renderer&) = delete;
        Renderer& operator=(const Renderer&) = delete;

        bool is_open() const { return window_.isOpen(); }

        void process_events();

        glm::vec2 get_mouse_position() const;

        void update_stats(float physics_time, std::size_t particle_count);

        void render(const std::vector<Physics::Particle>& particles, glm::vec2 mouse_position);

    private:
        Renderer();
        ~Renderer() = default;

        sf::RenderWindow window_;
        sf::CircleShape particle_shape_;
        sf::RectangleShape container_;
        sf::CircleShape mouse_circle_;

        sf::Font font_;
        bool font_loaded_ = false;
        sf::Text stats_text_;

        sf::Clock stats_clock_;
        int frames_counted_ = 0;
        float physics_time_sum_ = 0.0f;
    };
}
