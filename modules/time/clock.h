///
/// @copyright Copyright (C) 2021-2025, Gawei Xu
///
/// @author Gaowei Xu (gaowexu1991@gmail.com)
/// @date 2023-03-01
///

#ifndef MODULES_TIME_CLOCK_H
#define MODULES_TIME_CLOCK_H

#include <chrono>

namespace perception
{
namespace object_fusion
{

/// @brief sadp clock interface for time handling/manipulation in repo sadp.
class Clock
{
  public:
    /// @brief Type of a duration.
    using duration = std::chrono::nanoseconds;

    /// @brief Arithmetic type representing the number of ticks in the clock's duration.
    using rep = duration::rep;

    /// @brief A std::ratio type representing the tick period of the clock, in seconds.
    using period = duration::period;

    /// @brief Type of a point in time with respect to the epoch of GTM.
    using time_point = std::chrono::time_point<Clock, duration>;

    /// @brief Steady clock flag is always false as the local clock is synchronized with GTM by NTP or PTP.
    ///
    /// Note: A clock that can be adjusted backwards is not steady.
    static const bool is_steady = false;

    /// @brief Set a new clock to the mono-state.
    ///
    /// @param new_clock new Clock instance to be set to the class global.
    static Clock* SetClock(Clock* const new_clock);

    /// @brief Get the managed clock
    ///
    /// @pre Checks whether a clock was set before, so call SetClock beforehand.

    /// @return A pointer to the managed clock
    static Clock* GetClock();

    /// @brief Checks if non-null pointer is stored, i.e. whether GetClock() != nullptr.
    ///
    /// @return true if GetClock() stores a pointer, false otherwise.
    static bool IsClockNonNull();

    /// @brief Returns a time_point representing the current value of the clock.
    ///
    /// @note For fulfilment of stl-clock requirement
    ///
    /// @return A time point representing the current time.
    // Todo uncomment when transition is done
    // static time_point now() { return clock_->Now(); }

    /// @brief Returns true if time_stamp is greater than 0.
    static bool IsValid(const time_point& time_stamp);

    virtual ~Clock() = default;

    /// @brief to become a valid clock override Now method in derived classes
    ///
    /// @return the timepoint of now.
    virtual time_point Now() const = 0;

    /// @brief manipulate the now to facilitate reprocessing and simulation use-cases
    virtual void SetNow(const time_point& now) = 0;

  private:
    static Clock* clock_;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables) the clock needs to be set for
                           // different use-cases like simulation/reprocessing
};

}  // namespace object_fusion
}  // namespace perception

#endif  // MODULES_COMMON_TIME_CLOCK_H
