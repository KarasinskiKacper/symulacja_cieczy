#pragma once

#include "particle_data.hpp"

#include <array>
#include <atomic>
#include <cstdint>

namespace Physics {
    struct RenderFrame {
        FloatArray x;
        FloatArray y;
        std::uint64_t step = 0;
        float physics_time = 0.0f;
    };

    class FrameExchange {
    public:
        RenderFrame& physics() { return slots_[physics_]; }

        void publish() {
            const std::uint8_t old_middle = middle_.exchange(static_cast<std::uint8_t>(physics_ | NEW_FRAME), std::memory_order_acq_rel);
            physics_ = old_middle & INDEX_MASK;
        }

        const RenderFrame& acquire() {
            if (middle_.load(std::memory_order_relaxed) & NEW_FRAME) {
                const std::uint8_t old_middle = middle_.exchange(render_, std::memory_order_acq_rel);
                render_ = old_middle & INDEX_MASK;
            }
            return slots_[render_];
        }

    private:
        static constexpr std::uint8_t INDEX_MASK = 0b011;
        static constexpr std::uint8_t NEW_FRAME = 0b100;

        std::array<RenderFrame, 3> slots_;
        std::uint8_t physics_ = 0;
        std::uint8_t render_ = 1;
        std::atomic<std::uint8_t> middle_{2};
    };
}
