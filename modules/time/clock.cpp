///
/// @copyright Copyright (C) 2021-2025, Gawei Xu
///
/// @author Gaowei Xu (gaowexu1991@gmail.com)
/// @date 2023-03-01
///

#include "modules/time/clock.h"

#include <cstdint>
#include <utility>

namespace perception
{
namespace object_fusion
{

// the clock needs to be set for different use-cases like simulation/reprocessing
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
Clock* Clock::clock_ = nullptr;

Clock* Clock::SetClock(Clock* const new_clock)
{
    return std::exchange(clock_, new_clock);
}

Clock* Clock::GetClock()
{
    return clock_;
}

bool Clock::IsClockNonNull()
{
    return clock_ != nullptr;
}

bool Clock::IsValid(const time_point& time_stamp)
{
    return time_stamp > time_point{std::chrono::seconds{0}};
}

}  // namespace object_fusion
}  // namespace perception
