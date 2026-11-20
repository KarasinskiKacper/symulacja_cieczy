#pragma once

#include "aligned_allocator.hpp"

#include <cstddef>
#include <vector>

namespace Physics {
    using FloatArray = std::vector<float, AlignedAllocator<float>>;

    struct ParticleData {
        static constexpr std::size_t PADDING = 16;
        static constexpr float DEAD_POSITION = 1e9f;

        FloatArray pos_x;
        FloatArray pos_y;
        FloatArray vel_x;
        FloatArray vel_y;
        std::size_t count = 0;

        [[nodiscard]] std::size_t size() const { return count; }
        [[nodiscard]] std::size_t padded_size() const { return pos_x.size(); }

        [[nodiscard]] static constexpr std::size_t padded(const std::size_t n) {
            return (n + PADDING - 1) / PADDING * PADDING;
        }

        void reset(const std::size_t new_count) {
            count = new_count;
            pos_x.assign(padded(new_count), DEAD_POSITION);
            pos_y.assign(padded(new_count), DEAD_POSITION);
            vel_x.assign(padded(new_count), 0.0f);
            vel_y.assign(padded(new_count), 0.0f);
        }
    };
}
