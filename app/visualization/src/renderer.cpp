#include "../include/renderer.hpp"

#include <graphics_config.hpp>
#include <physics_config.hpp>

#include <cstdio>

namespace Visualization {

Renderer& Renderer::instance() {
    static Renderer renderer;
    return renderer;
}

Renderer::Renderer()
    : window_(sf::VideoMode(Config::WINDOW_WIDTH, Config::WINDOW_HEIGHT), "Symulacja Cieczy 2D - SPH") {
    window_.setFramerateLimit(60);

    // Set up the particle shape
    particle_shape_.setRadius(Config::PARTICLE_RADIUS);
    particle_shape_.setFillColor(Config::PARTICLE_COLOR);
    particle_shape_.setOrigin(Config::PARTICLE_RADIUS, Config::PARTICLE_RADIUS);

    // Set up the visible container
    container_.setSize(sf::Vector2f(Config::BOX_RIGHT - Config::BOX_LEFT, Config::BOX_BOTTOM - Config::BOX_TOP));
    container_.setPosition(Config::BOX_LEFT, Config::BOX_TOP);
    container_.setFillColor(Config::CONTAINER_FILL_COLOR);
    container_.setOutlineThickness(3.0f);
    container_.setOutlineColor(Config::CONTAINER_OUTLINE_COLOR);

    // Mouse indicator (optional, for a nicer visual effect)
    mouse_circle_.setRadius(Config::MOUSE_RADIUS);
    mouse_circle_.setOrigin(Config::MOUSE_RADIUS, Config::MOUSE_RADIUS);
    mouse_circle_.setFillColor(Config::MOUSE_CIRCLE_FILL_COLOR);
    mouse_circle_.setOutlineThickness(1.0f);
    mouse_circle_.setOutlineColor(Config::MOUSE_CIRCLE_OUTLINE_COLOR);

    // FPS and physics step time counter in the top-left corner
    // If the font file is missing, SFML prints an error and the simulation keeps running without the counter
    font_loaded_ = font_.loadFromFile("assets/LiberationMono-Regular.ttf");
    stats_text_.setFont(font_);
    stats_text_.setCharacterSize(16);
    stats_text_.setFillColor(Config::STATS_TEXT_COLOR);
    stats_text_.setPosition(10.0f, 10.0f);
}

void Renderer::process_events() {
    sf::Event event;
    while (window_.pollEvent(event)) {
        if (event.type == sf::Event::Closed)
            window_.close();
    }
}

glm::vec2 Renderer::get_mouse_position() const {
    sf::Vector2i sf_mouse_position = sf::Mouse::getPosition(window_);
    return {static_cast<float>(sf_mouse_position.x), static_cast<float>(sf_mouse_position.y)};
}

void Renderer::update_stats(const float physics_time, const std::uint64_t physics_step, const std::size_t particle_count) {
    physics_time_sum_ += physics_time;
    ++frames_counted_;

    // Refresh stats every 0.5 s (averaged so the numbers do not jump around)
    if (
        const float stats_elapsed = stats_clock_.getElapsedTime().asSeconds();
        stats_elapsed >= 0.5f
    ) {
        char buffer[128];
        const std::uint64_t steps_counted = physics_step - last_physics_step_;
        std::snprintf(buffer, sizeof(buffer), "FPS: %.1f\nFizyka: %.2f ms\nKroki/s: %.1f\nCzasteczki: %zu",
                      static_cast<double>(static_cast<float>(frames_counted_) / stats_elapsed),
                      static_cast<double>(physics_time_sum_ / static_cast<float>(frames_counted_) * 1000.0f),
                      static_cast<double>(static_cast<float>(steps_counted) / stats_elapsed),
                      particle_count);
        stats_text_.setString(buffer);
        frames_counted_ = 0;
        physics_time_sum_ = 0.0f;
        last_physics_step_ = physics_step;
        stats_clock_.restart();
    }
}

void Renderer::render(const std::span<const float> positions_x, const std::span<const float> positions_y, const glm::vec2 mouse_position) {
    window_.clear(Config::BACKGROUND_COLOR);

    window_.draw(container_);

    // Draw particles
    for (std::size_t i = 0; i < positions_x.size(); ++i) {
        particle_shape_.setPosition(positions_x[i], positions_y[i]);
        window_.draw(particle_shape_);
    }

    mouse_circle_.setPosition(mouse_position.x, mouse_position.y);
    window_.draw(mouse_circle_);

    if (font_loaded_) window_.draw(stats_text_);

    window_.display();
}
}
