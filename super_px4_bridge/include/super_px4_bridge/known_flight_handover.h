#pragma once
#include <cmath>
#include <cstdint>
namespace super_px4_bridge {
// Dedicated local-SITL input lease. Rejected packets never renew freshness.
// A timeout/mode loss latches; resumption requires restarting this demo session.
class KnownFlightLease {
public:
    bool accept(uint64_t task, double stamp, double now, bool valid) {
        if (failed || !valid || !task || !std::isfinite(stamp) || !std::isfinite(now)
            || stamp<=0 || stamp>now+.02 || now-stamp>.3
            || (id && task!=id) || (id && stamp<=last_stamp)) return false;
        id=task; last_stamp=stamp; return true;
    }
    bool fresh(double now) {
        if (failed || !id || !std::isfinite(now) || now<last_stamp-.02 || now-last_stamp>.3) {
            if (id) failed=true;
            return false;
        }
        return true;
    }
    void fault() { failed=true; }
    uint64_t id{0}; bool failed{false};
private:
    double last_stamp{0};
};
}
