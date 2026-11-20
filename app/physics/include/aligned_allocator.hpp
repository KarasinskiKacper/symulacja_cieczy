#pragma once

#include <cstddef>
#include <limits>
#include <new>

namespace Physics {
    // Allocator for std::vector that places the buffer at an address aligned to Align bytes.
    // 64 B = one cache line, which also covers the 32 B required by aligned AVX2 loads.
    template <typename T, std::size_t Align = 64>
    struct AlignedAllocator {
        static_assert((Align & (Align - 1)) == 0, "Align must be a power of two");
        static_assert(Align >= alignof(T), "Align must not be weaker than the natural alignment of T");

        using value_type = T;

        // Needed because Align is a non-type template parameter, so std::allocator_traits cannot generate rebind itself
        template <typename U>
        struct rebind {
            using other = AlignedAllocator<U, Align>;
        };

        AlignedAllocator() noexcept = default;

        template <typename U>
        explicit constexpr AlignedAllocator(const AlignedAllocator<U, Align>&) noexcept {}

        [[nodiscard]] static T* allocate(const std::size_t count) {
            if (count > std::numeric_limits<std::size_t>::max() / sizeof(T)) throw std::bad_array_new_length();
            return static_cast<T*>(::operator new(count * sizeof(T), std::align_val_t{Align}));
        }

        static void deallocate(T* pointer, const std::size_t count) noexcept {
            // Must match the aligned overload used in allocate()
            ::operator delete(pointer, count * sizeof(T), std::align_val_t{Align});
        }
    };

    // The allocator is stateless, so any instance can free memory allocated by any other
    template <typename T, typename U, std::size_t Align>
    constexpr bool operator==(const AlignedAllocator<T, Align>&, const AlignedAllocator<U, Align>&) noexcept {
        return true;
    }
}
