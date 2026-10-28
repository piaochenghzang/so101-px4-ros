#include <super_px4_bridge/known_flight_handover.h>
#include <cassert>
#include <limits>
int main() {
    using super_px4_bridge::KnownFlightLease;
    KnownFlightLease a;
    assert(!a.accept(0,1,1,true));
    assert(!a.accept(1,0,1,true));
    assert(!a.accept(1,1.1,1,true));
    assert(!a.accept(1,.5,1,true));
    assert(!a.accept(1,1,1,false));
    assert(!a.accept(1,std::numeric_limits<double>::quiet_NaN(),1,true));
    assert(a.accept(1,1,1,true));
    assert(!a.accept(2,1.1,1.1,true));
    assert(!a.accept(1,1,1.1,true));
    assert(a.fresh(1.2));
    assert(!a.fresh(1.31));
    assert(!a.accept(1,1.4,1.4,true));
    assert(!a.accept(2,1.4,1.4,true));
    KnownFlightLease b;
    assert(b.accept(10,2,2,true));
    assert(b.accept(10,2.1,2.1,true));
    assert(b.fresh(2.2));
    b.fault(); assert(!b.fresh(2.2)); assert(!b.accept(10,2.3,2.3,true));
    KnownFlightLease c;
    assert(c.accept(3,3,3,true));
    assert(!c.fresh(2)); assert(c.failed);
    super_px4_bridge::KnownTaskBinding d;
    assert(!d.beginCarry());
    assert(d.accept(0,0,0,false)); assert(!d.accept(1,0,2,false));
    assert(d.accept(11,22,33,false)); assert(!d.accept(12,22,33,false));
    assert(!d.accept(11,23,33,false)); assert(!d.accept(11,22,34,false));
    assert(!d.accept(0,0,0,false)); assert(d.accept(11,22,33,true));
    assert(!d.accept(11,22,33,false)); assert(d.accept(11,22,33,true));
    assert(d.beginCarry());assert(!d.beginCarry());
    assert(d.accept(11,22,33,false));assert(!d.accept(12,22,33,false));
    assert(d.accept(11,22,33,true));
}
