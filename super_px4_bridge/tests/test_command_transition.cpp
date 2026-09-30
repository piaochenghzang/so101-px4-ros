#include <super_px4_bridge/command_transition.h>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace super_px4_bridge;

static void require(bool ok, const char *message)
{
    if (!ok) { throw std::runtime_error(message); }
}

int main()
{
    Motion start;
    start.position = Vec3{{1, 2, 1.65}};
    start.velocity = Vec3{{0.01, -0.02, 0.03}};
    const Vec3 goal{{1.05, 2.02, 1.65}};
    const auto first = settleToPoint(start, goal, 0, 1);
    const auto last = settleToPoint(start, goal, 1, 1);
    require(distance(first.position, start.position) < 1e-12, "settle position start");
    require(distance(first.velocity, start.velocity) < 1e-12, "settle velocity start");
    require(norm(first.acceleration) < 1e-12, "settle acceleration start");
    require(distance(last.position, goal) < 1e-12 && norm(last.velocity) < 1e-12 &&
            norm(last.acceleration) < 1e-12, "settle terminal rest");
    const auto entry = blendFromHold(goal, start, 0, 1);
    const auto tracking = blendFromHold(goal, start, 1, 1);
    require(distance(entry.position, goal) < 1e-12 && norm(entry.velocity) < 1e-12,
            "entry continuity");
    require(distance(tracking.position, start.position) < 1e-12 &&
            distance(tracking.velocity, start.velocity) < 1e-12, "entry end matches live");
    Motion terminal;
    terminal.position = goal;
    require(terminalEligible(start.position, terminal, goal, 0.35, 0.5, 0.05),
            "nearby projected endpoint accepted");
    require(!terminalEligible(Vec3{{10, 2, 1.65}}, terminal, goal, 0.35, 0.5, 0.05),
            "large goal projection rejected");
    terminal.velocity[0] = 0.2;
    require(!terminalEligible(start.position, terminal, goal, 0.35, 0.5, 0.05),
            "moving stream loss is not terminal");
    terminal.velocity[0] = std::numeric_limits<double>::quiet_NaN();
    require(!terminalEligible(start.position, terminal, goal, 0.35, 0.5, 0.05), "NaN rejected");
    YawLimiter yaw;
    require(!terminalTimelineComplete(9, 10), "early stream loss rejected even at rest");
    require(terminalTimelineComplete(10.01, 10), "observed command reaches planned end");
    require(!terminalTimelineComplete(10, std::numeric_limits<double>::quiet_NaN()),
            "invalid timeline rejected");
    yaw.reset(3.12, 0);
    double previous_yaw = 3.12, previous_rate = 0;
    for (int n = 1; n <= 1500; ++n) {
        const double desired = n < 500 ? -3.12 : (n < 1000 ? 0 : 2.8);
        const double value = yaw.update(desired, n * 0.02, 0.35, 0.5);
        require(std::fabs(wrapAngle(value - previous_yaw)) <= 0.35 * 0.02 + 1e-10,
                "yaw rate hard bound / angle wrapping");
        require(std::fabs(yaw.rate() - previous_rate) <= 0.5 * 0.02 + 1e-10,
                "yaw acceleration hard bound");
        previous_yaw = value;
        previous_rate = yaw.rate();
    }
    const double before = previous_yaw;
    require(yaw.update(0, 30, 0.35, 0.5) == before, "same sim time does not advance yaw");
    require(std::fabs(wrapAngle(yaw.update(0, 100, 0.35, 0.5) - before)) <= 0.35 * 0.05 + 1e-10,
            "no catch-up jump after pause");
    std::cout << "Transition endpoints, terminal guards and 1500 yaw rate/acceleration steps passed\n";
}
