#pragma once

namespace Config {
    constexpr float BOX_LEFT = 400.0f;
    constexpr float BOX_RIGHT = 800.0f;
    constexpr float BOX_BOTTOM = 550.0f;
    constexpr float BOX_TOP = 50.0f;

    constexpr float GRAVITY = 9.81f * 100.0f;

    constexpr float PARTICLE_RADIUS = 4.0f;
    constexpr float PARTICLE_SPACING = PARTICLE_RADIUS  * 1.0f;
    constexpr float REST_DENSITY = 1.0f;
    constexpr float MASS = REST_DENSITY * PARTICLE_SPACING * PARTICLE_SPACING * 10;
    constexpr float GAS_CONST = 2000.0f;
    constexpr float H = 8.0f;
    constexpr float HSQ = H * H;
    constexpr float DT = 0.005f;
    constexpr float VISCOSITY = 5000.0f;
    constexpr float COLLISIONS_DAMPING = -0.3f;
    constexpr float COLLISION_RELAXATION = 0.25f;

    constexpr bool PHYSICS_REAL_TIME = true;

    // Modified simulation parameters for a "thin water" effect
    // constexpr float REST_DENSITY = 250.0f;
    // constexpr float GAS_CONST = 5000.0f;
    // constexpr float H = 16.0f;
    // constexpr float HSQ = H * H;
    // constexpr float MASS = 2.5f;
    // constexpr float DT = 0.005f;
    // constexpr float VISCOSITY = 5.0f;
    // constexpr float PARTICLE_RADIUS = 3.0f;

    constexpr float MOUSE_RADIUS = 75.0f;
    constexpr float MOUSE_FORCE = 800000.0f;
}
