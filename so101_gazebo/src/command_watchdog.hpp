#pragma once

#include <cmath>

namespace so101 {
enum class TimeoutPolicy { MeasuredPosition, LastTarget };
enum class WatchdogEvent { None, Timeout, ClockReset };

// Simulation-time only: pausing must not unload a grasp. One timeout event
// per command episode; a genuinely new command starts another episode.
class CommandWatchdog {
public:
    WatchdogEvent update(double now)
    {
        if (!std::isfinite(now)) return WatchdogEvent::None;
        if (seen_time_ && now < last_update_) {
            reset(now);
            return WatchdogEvent::ClockReset;
        }
        seen_time_ = true;
        last_update_ = now;
        if (active_ && now - last_command_ > 1.0) {
            active_ = false;
            return WatchdogEvent::Timeout;
        }
        return WatchdogEvent::None;
    }

    void command(double now) { last_command_ = now; active_ = true; }
    void reset(double now)
    {
        active_ = false;
        seen_time_ = true;
        last_update_ = now;
        last_command_ = now;
    }

private:
    double last_update_{0.0};
    double last_command_{0.0};
    bool seen_time_{false};
    bool active_{false};
};

inline double timeoutTarget(TimeoutPolicy policy, double applied, double measured)
{
    return policy == TimeoutPolicy::LastTarget ? applied : measured;
}
} // namespace so101
