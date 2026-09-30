#include "../src/command_watchdog.hpp"
#include <cassert>
#include <limits>

int main()
{
    using namespace so101;
    CommandWatchdog watchdog;
    assert(watchdog.update(0.0) == WatchdogEvent::None);
    assert(watchdog.update(20.0) == WatchdogEvent::None); // No command yet.
    watchdog.command(20.0);
    for (int i = 0; i < 100; ++i) // A paused world has no watchdog advance.
        assert(watchdog.update(20.0) == WatchdogEvent::None);
    assert(watchdog.update(21.0) == WatchdogEvent::None);
    assert(watchdog.update(21.001) == WatchdogEvent::Timeout);
    assert(watchdog.update(141.0) == WatchdogEvent::None); // No repeated unload.
    assert(timeoutTarget(TimeoutPolicy::MeasuredPosition, -.15, .06) == .06);
    assert(timeoutTarget(TimeoutPolicy::LastTarget, -.15, .06) == -.15);
    assert(timeoutTarget(TimeoutPolicy::LastTarget, .60, .30) == .60); // Not auto-close.
    watchdog.command(141.0); // Recovery requires a new received command.
    assert(watchdog.update(142.001) == WatchdogEvent::Timeout);
    watchdog.command(142.001);
    assert(watchdog.update(0.0) == WatchdogEvent::ClockReset);
    assert(watchdog.update(120.0) == WatchdogEvent::None); // Old command disarmed.
    watchdog.command(120.0);
    watchdog.reset(0.0); // Explicit world/model reset, even if time did not change.
    assert(watchdog.update(2.0) == WatchdogEvent::None);
    assert(watchdog.update(std::numeric_limits<double>::quiet_NaN()) == WatchdogEvent::None);
}
