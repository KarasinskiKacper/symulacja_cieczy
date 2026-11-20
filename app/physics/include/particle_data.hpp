#pragma once

#include "aligned_allocator.hpp"

#include <cstddef>
#include <vector>

namespace Physics {
    using FloatArray = std::vector<float, AlignedAllocator<float>>;

    struct ParticleData {
        FloatArray pos_x;
        FloatArray pos_y;
        FloatArray vel_x;
        FloatArray vel_y;

        [[nodiscard]] std::size_t size() const { return pos_x.size(); }

        void push_back(const float x, const float y) {
            pos_x.push_back(x);
            pos_y.push_back(y);
            vel_x.push_back(0.0f);
            vel_y.push_back(0.0f);
        }

        void resize(const std::size_t count) {
            pos_x.resize(count);
            pos_y.resize(count);
            vel_x.resize(count);
            vel_y.resize(count);
        }
    };
}
