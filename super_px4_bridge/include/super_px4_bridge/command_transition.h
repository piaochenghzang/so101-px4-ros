#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace super_px4_bridge {

using Vec3 = std::array<double, 3>;

struct Motion {
    Vec3 position{};
    Vec3 velocity{};
    Vec3 acceleration{};
};

inline double norm(const Vec3 &v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

inline double distance(const Vec3 &a, const Vec3 &b)
{
    return norm(Vec3{{a[0] - b[0], a[1] - b[1], a[2] - b[2]}});
}

inline bool finite(const Vec3 &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

inline double wrapAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

// C2 blend from a stationary hold into a moving planner command.
inline Motion blendFromHold(const Vec3 &hold, const Motion &live,
                            double elapsed, double duration)
{
    const double s = std::max(0.0, std::min(1.0, elapsed / duration));
    const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
    const double w = 10 * s3 - 15 * s4 + 6 * s5;
    const double dw = (30 * s2 - 60 * s3 + 30 * s4) / duration;
    const double ddw = (60 * s - 180 * s2 + 120 * s3) / (duration * duration);
    Motion out;
    for (int i = 0; i < 3; ++i) {
        const double d = live.position[i] - hold[i];
        out.position[i] = hold[i] + w * d;
        out.velocity[i] = dw * d + w * live.velocity[i];
        out.acceleration[i] = ddw * d + 2 * dw * live.velocity[i] + w * live.acceleration[i];
    }
    return out;
}

// Quintic Hermite stop: preserves initial position/velocity, ends at rest.
inline Motion settleToPoint(const Motion &start, const Vec3 &goal,
                            double elapsed, double duration)
{
    const double s = std::max(0.0, std::min(1.0, elapsed / duration));
    const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
    const double h0 = 1 - 10 * s3 + 15 * s4 - 6 * s5;
    const double h1 = s - 6 * s3 + 8 * s4 - 3 * s5;
    const double dh0 = -30 * s2 + 60 * s3 - 30 * s4;
    const double dh1 = 1 - 18 * s2 + 32 * s3 - 15 * s4;
    const double ddh0 = -60 * s + 180 * s2 - 120 * s3;
    const double ddh1 = -36 * s + 96 * s2 - 60 * s3;
    Motion out;
    for (int i = 0; i < 3; ++i) {
        const double d = start.position[i] - goal[i];
        out.position[i] = goal[i] + h0 * d + duration * h1 * start.velocity[i];
        out.velocity[i] = dh0 / duration * d + dh1 * start.velocity[i];
        out.acceleration[i] = ddh0 / (duration * duration) * d + ddh1 / duration * start.velocity[i];
    }
    return out;
}

inline bool terminalEligible(const Vec3 &requested, const Motion &terminal,
                             const Vec3 &actual, double max_projection,
                             double capture_radius, double terminal_speed)
{
    return finite(requested) && finite(terminal.position) && finite(terminal.velocity) &&
           finite(actual) && distance(requested, terminal.position) <= max_projection &&
           distance(actual, terminal.position) <= capture_radius &&
           norm(terminal.velocity) <= terminal_speed;
}

inline bool terminalTimelineComplete(double last_command_time, double planned_end_time)
{
    return std::isfinite(last_command_time) && std::isfinite(planned_end_time) &&
           planned_end_time > 0 && last_command_time >= planned_end_time;
}

class YawLimiter {
public:
    void reset(double yaw, double time)
    {
        yaw_ = wrapAngle(yaw);
        rate_ = 0.0;
        time_ = time;
        initialized_ = true;
    }

    double update(double target, double time, double max_rate, double max_acceleration)
    {
        if (!initialized_) {
            reset(target, time);
            return yaw_;
        }
        if (!std::isfinite(target) || !std::isfinite(time)) {
            return yaw_;
        }
        if (time <= time_) {
            if (time < time_) {
                time_ = time;
                rate_ = 0.0;
            }
            return yaw_;
        }
        // Never catch up a long pause in one command.
        const double dt = std::min(time - time_, 0.05);
        time_ = time;
        const double error = wrapAngle(target - yaw_);
        const double braking_rate = std::sqrt(2 * max_acceleration * std::fabs(error));
        const double desired = std::copysign(std::min(max_rate, braking_rate), error);
        rate_ += std::max(-max_acceleration * dt,
                         std::min(max_acceleration * dt, desired - rate_));
        rate_ = std::max(-max_rate, std::min(max_rate, rate_));
        yaw_ = wrapAngle(yaw_ + rate_ * dt);
        return yaw_;
    }

    double rate() const { return rate_; }

private:
    double yaw_{0.0};
    double rate_{0.0};
    double time_{0.0};
    bool initialized_{false};
};

} // namespace super_px4_bridge
