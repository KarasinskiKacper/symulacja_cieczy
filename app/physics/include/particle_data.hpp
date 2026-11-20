#pragma once

#include "aligned_allocator.hpp"

#include <cstddef>
#include <vector>

namespace Physics {
    using FloatArray = std::vector<float, AlignedAllocator<float>>;

    // Particle state in SoA layout: every field is a separate, contiguous, 64 B aligned array.
    // Particle i is described by the i-th element of each array.
    struct ParticleData {
        FloatArray pos_x, pos_y;
        FloatArray vel_x, vel_y;
        FloatArray force_x, force_y;
        FloatArray density, pressure;

        [[nodiscard]] std::size_t size() const { return pos_x.size(); }

        void push_back(const float x, const float y) {
            pos_x.push_back(x);
            pos_y.push_back(y);
            vel_x.push_back(0.0f);
            vel_y.push_back(0.0f);
            force_x.push_back(0.0f);
            force_y.push_back(0.0f);
            density.push_back(0.0f);
            pressure.push_back(0.0f);
        }
    };
}
