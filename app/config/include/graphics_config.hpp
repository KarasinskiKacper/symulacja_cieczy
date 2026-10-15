#pragma once

#include <SFML/Graphics/Color.hpp>

namespace Config {
    constexpr int WINDOW_WIDTH = 1200;
    constexpr int WINDOW_HEIGHT = 600;

    inline const sf::Color BACKGROUND_COLOR(20, 20, 20);
    inline const sf::Color PARTICLE_COLOR(50, 150, 255);
    inline const sf::Color CONTAINER_FILL_COLOR(0, 0, 0, 0); // Transparent
    inline const sf::Color CONTAINER_OUTLINE_COLOR(100, 100, 100);
    inline const sf::Color MOUSE_CIRCLE_FILL_COLOR(255, 255, 255, 15); // Very transparent white (alpha = 15)
    inline const sf::Color MOUSE_CIRCLE_OUTLINE_COLOR(255, 255, 255, 50);
    inline const sf::Color STATS_TEXT_COLOR(255, 255, 255);
}
