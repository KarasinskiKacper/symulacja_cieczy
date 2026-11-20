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
        RenderFrame& back() { return slots_[back_]; }

        void publish() {
            const std::uint8_t old_middle = middle_.exchange(static_cast<std::uint8_t>(back_ | NEW_FRAME), std::memory_order_acq_rel);
            back_ = old_middle & INDEX_MASK;
        }

        const RenderFrame& acquire() {
            if (middle_.load(std::memory_order_relaxed) & NEW_FRAME) {
                const std::uint8_t old_middle = middle_.exchange(front_, std::memory_order_acq_rel);
                front_ = old_middle & INDEX_MASK;
            }
            return slots_[front_];
        }

    private:
        static constexpr std::uint8_t INDEX_MASK = 0b011;
        static constexpr std::uint8_t NEW_FRAME = 0b100;

        std::array<RenderFrame, 3> slots_;
        std::uint8_t back_ = 0;
        std::uint8_t front_ = 1;
        std::atomic<std::uint8_t> middle_{2};
    };
}
