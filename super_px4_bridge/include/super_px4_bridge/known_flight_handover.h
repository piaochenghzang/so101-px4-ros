#pragma once
#include <cmath>
#include <cstdint>
namespace super_px4_bridge {
// Metadata is immutable for a local-SITL task. Once terminal hold is entered,
// transit requires an explicit, opt-in slow-carry service after verified grasp.
class KnownTaskBinding {
public:
    bool accept(uint64_t p,uint64_t t,uint64_t s,bool hold) {
        if (!p && !t && !s && !hold) return !plan;
        if (!p || !t || !s || (plan && (plan!=p || target!=t || scene!=s)) || (terminal && !hold && !carry)) return false;
        plan=p; target=t; scene=s; terminal=hold; return true;
    }
    bool beginCarry() { if (!terminal || !plan || carry) return false; carry=true; return true; }
    uint64_t plan{0},target{0},scene{0}; bool terminal{false},carry{false};
};
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
