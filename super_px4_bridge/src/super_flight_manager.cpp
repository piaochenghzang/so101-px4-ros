#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <super_px4_bridge/command_transition.h>
#include <super_px4_bridge/known_flight_handover.h>
#include <super_px4_bridge/KnownFlightSetpoint.h>
#include <super_px4_bridge/KnownFlightStatus.h>
#include <std_msgs/String.h>


class SuperFlightManager
{
public:
    SuperFlightManager()
        : nh_(),
          pnh_("~")
    {
        loadParameters();

        state_sub_ =
            nh_.subscribe(
                state_topic_, 20,
                &SuperFlightManager::stateCallback,
                this);

        odom_sub_ =
            nh_.subscribe(
                odom_topic_, 50,
                &SuperFlightManager::odomCallback,
                this);

        cloud_sub_ =
            nh_.subscribe(
                cloud_topic_, 10,
                &SuperFlightManager::cloudCallback,
                this);

        user_goal_sub_ =
            nh_.subscribe(
                user_goal_topic_, 5,
                &SuperFlightManager::userGoalCallback,
                this);

        super_setpoint_sub_ =
            nh_.subscribe(
                super_setpoint_topic_, 50,
                &SuperFlightManager::superSetpointCallback,
                this);

        mavros_setpoint_pub_ =
            nh_.advertise<mavros_msgs::PositionTarget>(
                mavros_setpoint_topic_, 20);

        super_goal_pub_ =
            nh_.advertise<geometry_msgs::PoseStamped>(
                super_goal_topic_, 5);
        resolved_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/planning/resolved_goal", 1, true);
        flight_state_pub_=nh_.advertise<std_msgs::String>("/grasp_demo/flight_state",1,true);
        known_status_pub_=nh_.advertise<super_px4_bridge::KnownFlightStatus>("/grasp_demo/known_status",1);
        known_setpoint_sub_=nh_.subscribe("/grasp_demo/known_setpoint",20,
            &SuperFlightManager::knownSetpointCallback,this);
        trajectory_sub_ = nh_.subscribe("/planning_cmd/poly_traj", 50,
            &SuperFlightManager::trajectoryCallback, this);

        set_mode_client_ =
            nh_.serviceClient<mavros_msgs::SetMode>(
                "/mavros/set_mode");

        arming_client_ =
            nh_.serviceClient<mavros_msgs::CommandBool>(
                "/mavros/cmd/arming");

        const double timer_period =
            1.0 / std::max(2.0, publish_rate_);

        control_timer_ =
            nh_.createTimer(
                ros::Duration(timer_period),
                &SuperFlightManager::controlTimerCallback,
                this);

        ROS_INFO_STREAM(
            "SUPER flight manager started:"
            << "\n  user goal:       " << user_goal_topic_
            << "\n  SUPER goal:      " << super_goal_topic_
            << "\n  SUPER setpoint:  " << super_setpoint_topic_
            << "\n  MAVROS setpoint: " << mavros_setpoint_topic_
            << "\n  takeoff height:  " << takeoff_height_
            << "\n  publish rate:    " << publish_rate_);
    }

private:
    enum class FlightState
    {
        WAIT_FCU,
        PRESTREAM,
        OFFBOARD_ARM,
        TAKEOFF,
        MAP_WARMUP,
        HOVER_READY,
        WAIT_TRAJECTORY,
        TRACK_SUPER,
        SETTLE_HOVER,
        FINAL_HOVER,
        FAILSAFE_HOVER,
        TRACK_KNOWN
    };

    void loadParameters()
    {
        pnh_.param<std::string>(
            "state_topic",
            state_topic_,
            "/mavros/state");

        pnh_.param<std::string>(
            "odom_topic",
            odom_topic_,
            "/mavros/local_position/odom");

        pnh_.param<std::string>(
            "cloud_topic",
            cloud_topic_,
            "/velodyne_points");

        pnh_.param<std::string>(
            "user_goal_topic",
            user_goal_topic_,
            "/planning/user_goal");

        pnh_.param<std::string>(
            "super_goal_topic",
            super_goal_topic_,
            "/planning/click_goal");

        pnh_.param<std::string>(
            "super_setpoint_topic",
            super_setpoint_topic_,
            "/super/setpoint_raw/local");

        pnh_.param<std::string>(
            "mavros_setpoint_topic",
            mavros_setpoint_topic_,
            "/mavros/setpoint_raw/local");

        pnh_.param("takeoff_height",
                   takeoff_height_, 1.5);

        pnh_.param("publish_rate",
                   publish_rate_, 50.0);

        pnh_.param("prestream_time",
                   prestream_time_, 1.0);

        pnh_.param("hover_stable_time",
                   hover_stable_time_, 1.0);

        pnh_.param("map_warmup_time",
                   map_warmup_time_, 2.0);

        pnh_.param("super_cmd_timeout",
                   super_cmd_timeout_, 0.3);

        pnh_.param("planning_timeout",
                   planning_timeout_, 5.0);

        pnh_.param("goal_tolerance",
                   goal_tolerance_, 0.20);

        pnh_.param("altitude_tolerance",
                   altitude_tolerance_, 0.15);

        pnh_.param("velocity_tolerance",
                   velocity_tolerance_, 0.25);

        pnh_.param("odom_timeout",
                   odom_timeout_, 0.5);

        pnh_.param("cloud_timeout",
                   cloud_timeout_, 1.0);

        pnh_.param("auto_arm",
                   auto_arm_, true);
        pnh_.param("known_goal_mode", known_goal_mode_, false);
        if (known_goal_mode_ && !nh_.param<bool>("/use_sim_time", false)) {
            throw std::invalid_argument("known_goal_mode is simulation-only and requires use_sim_time");
        }
        pnh_.param("handover_time", handover_time_, 1.0);
        pnh_.param("settle_timeout", settle_timeout_, 6.0);
        pnh_.param("max_goal_projection", max_goal_projection_, 0.35);
        pnh_.param("terminal_capture_radius", terminal_capture_radius_, 0.5);
        pnh_.param("terminal_speed_tolerance", terminal_speed_tolerance_, 0.05);
        pnh_.param("max_source_age", max_source_age_, 0.3);
        pnh_.param("yaw_rate_limit", yaw_rate_limit_, 0.35);
        pnh_.param("yaw_acceleration_limit", yaw_acceleration_limit_, 0.5);
        if (!std::isfinite(handover_time_) || handover_time_ < 0.2 ||
            !std::isfinite(settle_timeout_) || settle_timeout_ <= handover_time_ ||
            !std::isfinite(max_goal_projection_) || max_goal_projection_ <= 0 ||
            !std::isfinite(terminal_capture_radius_) || terminal_capture_radius_ <= 0 ||
            !std::isfinite(terminal_speed_tolerance_) || terminal_speed_tolerance_ <= 0 ||
            !std::isfinite(max_source_age_) || max_source_age_ <= 0 ||
            !std::isfinite(yaw_rate_limit_) || yaw_rate_limit_ <= 0 ||
            !std::isfinite(yaw_acceleration_limit_) || yaw_acceleration_limit_ <= 0) {
            throw std::invalid_argument("invalid command handover / yaw limits");
        }
    }

    void stateCallback(
        const mavros_msgs::StateConstPtr &msg)
    {
        current_fcu_state_ = *msg;
        last_fcu_time_=ros::Time::now();
        have_fcu_state_ = true;
    }

    void odomCallback(
        const nav_msgs::OdometryConstPtr &msg)
    {
        current_odom_ = *msg;
        last_odom_time_ = ros::Time::now();
        have_odom_ = true;
    }

    void cloudCallback(
        const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        (void)msg;

        last_cloud_time_ = ros::Time::now();
        have_cloud_ = true;
    }

    void userGoalCallback(
        const geometry_msgs::PoseStampedConstPtr &msg)
    {
        if (known_goal_mode_) { ROS_WARN_THROTTLE(1,"Known-goal demo owns navigation; rejecting planner goal"); return; }
        if (!std::isfinite(msg->pose.position.x) ||
            !std::isfinite(msg->pose.position.y) ||
            !std::isfinite(msg->pose.position.z)) {
            ROS_ERROR("[FlightManager] Reject non-finite user goal.");
            return;
        }
        pending_goal_ = *msg;
        pending_goal_.header.frame_id = "world";
        pending_goal_.header.stamp = ros::Time::now();

        /*
         * RViz 的 2D Nav Goal 通常会给出 z=0。
         * 当前飞行测试自动将过低目标修正为巡航高度。
         */
        if (pending_goal_.pose.position.z < 0.5) {
            pending_goal_.pose.position.z =
                takeoff_height_;
        }

        if (!quaternionValid(
                pending_goal_.pose.orientation)) {

            pending_goal_.pose.orientation =
                yawToQuaternion(hold_yaw_);
        }

        have_pending_goal_ = true;

        ROS_INFO_STREAM(
            "[FlightManager] User goal received: "
            << pending_goal_.pose.position.x << ", "
            << pending_goal_.pose.position.y << ", "
            << pending_goal_.pose.position.z);

        if (flight_state_ != FlightState::HOVER_READY &&
            flight_state_ != FlightState::FINAL_HOVER) {

            ROS_WARN(
                "[FlightManager] Goal cached. "
                "It will be sent after stable hover.");
        }
    }

    void knownSetpointCallback(const super_px4_bridge::KnownFlightSetpointConstPtr &msg)
    {
        if (!known_goal_mode_ || known_lease_.failed) return;
        if (flight_state_!=FlightState::HOVER_READY && flight_state_!=FlightState::FINAL_HOVER
            && flight_state_!=FlightState::TRACK_KNOWN) return;
        const auto &p=msg->setpoint; const auto motion=motionOf(p);
        const uint16_t mask=mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        const bool valid=msg->header.frame_id=="map" && p.header.frame_id=="map"
            && p.header.stamp==msg->header.stamp &&
            p.coordinate_frame==mavros_msgs::PositionTarget::FRAME_LOCAL_NED && p.type_mask==mask
            && super_px4_bridge::finite(motion.position) && super_px4_bridge::finite(motion.velocity)
            && std::isfinite(p.yaw) && motion.position[2]>=.3 && motion.position[2]<=2.5
            && super_px4_bridge::distance(motion.position,positionOf(current_odom_.pose.pose.position))<=1.0
            && super_px4_bridge::distance(motion.velocity,super_px4_bridge::Vec3{{0,0,0}})<=1.0;
        auto binding=known_binding_;
        if (!known_lease_.accept(msg->task_id,msg->header.stamp.toSec(),ros::Time::now().toSec(),
            valid && binding.accept(msg->plan_id,msg->target_id,msg->scene_id,msg->terminal_hold))) return;
        known_binding_=binding;
        known_setpoint_=p;
        if (flight_state_!=FlightState::TRACK_KNOWN) transitionTo(FlightState::TRACK_KNOWN);
    }


    void superSetpointCallback(
        const mavros_msgs::PositionTargetConstPtr &msg)
    {
        if (known_goal_mode_) return;
        const ros::Time now = ros::Time::now();
        const auto motion = motionOf(*msg);
        const double age = (now - msg->header.stamp).toSec();
        if (!super_px4_bridge::finite(motion.position) ||
            !std::isfinite(msg->velocity.x) || !std::isfinite(msg->velocity.y) ||
            !std::isfinite(msg->velocity.z) || !std::isfinite(msg->yaw) ||
            msg->coordinate_frame != mavros_msgs::PositionTarget::FRAME_LOCAL_NED ||
            (msg->type_mask & (mavros_msgs::PositionTarget::IGNORE_PX |
                              mavros_msgs::PositionTarget::IGNORE_PY |
                              mavros_msgs::PositionTarget::IGNORE_PZ |
                              mavros_msgs::PositionTarget::IGNORE_VX |
                              mavros_msgs::PositionTarget::IGNORE_VY |
                              mavros_msgs::PositionTarget::IGNORE_VZ)) ||
            msg->header.stamp.isZero() || age < -0.05 || age > max_source_age_ ||
            (have_super_setpoint_ && msg->header.stamp < latest_super_setpoint_.header.stamp) ||
            (have_active_goal_ && msg->header.stamp < goal_sent_time_)) {
            ROS_WARN_THROTTLE(1.0, "[FlightManager] Reject invalid/stale SUPER command.");
            return;
        }
        latest_super_setpoint_ = *msg;

        /*
         * 第一阶段统一禁止加速度和 yaw rate。
         */
        latest_super_setpoint_.type_mask =
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        latest_super_setpoint_.acceleration_or_force.x = 0.0;
        latest_super_setpoint_.acceleration_or_force.y = 0.0;
        latest_super_setpoint_.acceleration_or_force.z = 0.0;
        latest_super_setpoint_.yaw_rate = 0.0;

        last_super_setpoint_time_ = ros::Time::now();
        have_super_setpoint_ = true;
    }

    void trajectoryCallback(const quadrotor_msgs::PolynomialTrajectoryConstPtr &msg)
    {
        if (!have_active_goal_ || msg->header.stamp < goal_sent_time_ ||
            msg->header.stamp < last_trajectory_stamp_ ||
            (ros::Time::now() - msg->header.stamp).toSec() > max_source_age_) {
            return;
        }
        if (msg->type & quadrotor_msgs::PolynomialTrajectory::EMER_STOP) {
            have_planned_end_ = false;
            return;
        }
        if (!(msg->type & quadrotor_msgs::PolynomialTrajectory::POSITION_TRAJ)) {
            return; // Heartbeats do not replace the committed trajectory boundary.
        }
        if (msg->piece_num_pos == 0 || msg->time_pos.size() != msg->piece_num_pos ||
            msg->start_WT_pos.isZero()) {
            have_planned_end_ = false;
            return;
        }
        double duration = 0;
        for (const double segment : msg->time_pos) {
            if (!std::isfinite(segment) || segment <= 0) {
                have_planned_end_ = false;
                return;
            }
            duration += segment;
        }
        if (!std::isfinite(duration) || duration > 300) {
            have_planned_end_ = false;
            return;
        }
        planned_end_time_ = msg->start_WT_pos + ros::Duration(duration);
        last_trajectory_stamp_ = msg->header.stamp;
        have_planned_end_ = true;
    }

    void controlTimerCallback(
        const ros::TimerEvent &)
    {
        const ros::Time now = ros::Time::now();
        if (known_goal_mode_) {
            super_px4_bridge::KnownFlightStatus s; s.header.stamp=now; s.header.frame_id="map";
            s.task_id=known_lease_.id; s.plan_id=known_binding_.plan;
            s.target_id=known_binding_.target; s.scene_id=known_binding_.scene;
            s.command_stamp=known_setpoint_.header.stamp; s.terminal_hold=known_binding_.terminal;
            s.lease_valid=flight_state_==FlightState::TRACK_KNOWN && known_lease_.fresh(now.toSec()) &&
                have_fcu_state_ && current_fcu_state_.connected && current_fcu_state_.armed &&
                current_fcu_state_.mode=="OFFBOARD" && now>=last_fcu_time_ &&
                (now-last_fcu_time_).toSec()<=1.2 && odomFresh(now);
            known_status_pub_.publish(s);
        }

        if (!have_fcu_state_ ||
            !current_fcu_state_.connected) {
            if (known_goal_mode_ && known_lease_.id) known_lease_.fault();

            printStateThrottled("waiting for FCU");
            return;
        }

        if (!odomFresh(now)) {
            if (known_goal_mode_ && known_lease_.id) known_lease_.fault();
            printStateThrottled("waiting for odometry");
            return;
        }

        if (flight_state_ == FlightState::WAIT_FCU) {
            if (!known_goal_mode_ && !cloudFresh(now)) {
                printStateThrottled("waiting for LiDAR");
                return;
            }

            setHoldFromCurrentOdometry();

            transitionTo(FlightState::PRESTREAM);
        }

        /*
         * TAKEOFF 之后如果 OFFBOARD 或 ARM 状态丢失，
         * 在继续发送安全设定点的同时尝试恢复。
         */
        if (flight_state_ != FlightState::WAIT_FCU &&
            flight_state_ != FlightState::PRESTREAM &&
            flight_state_ != FlightState::OFFBOARD_ARM) {
            if (known_goal_mode_ && (current_fcu_state_.mode!="OFFBOARD" || !current_fcu_state_.armed)) {
                known_lease_.fault();
                ROS_ERROR_THROTTLE(1,"Known-goal demo mode/arming lost; no automatic re-arm or mode override");
                return;
            }

            if (current_fcu_state_.mode != "OFFBOARD") {
                requestOffboard();
            }

            if (auto_arm_ && !current_fcu_state_.armed) {
                requestArm();
            }
        }

        switch (flight_state_) {
        case FlightState::WAIT_FCU:
            break;

        case FlightState::PRESTREAM:
            publishHoldSetpoint();

            if (stateElapsed(now) >= prestream_time_) {
                transitionTo(FlightState::OFFBOARD_ARM);
            }
            break;

        case FlightState::OFFBOARD_ARM:
            publishHoldSetpoint();

            if (current_fcu_state_.mode != "OFFBOARD") {
                requestOffboard();
                break;
            }

            if (auto_arm_ && !current_fcu_state_.armed) {
                requestArm();
                break;
            }

            hold_z_ = takeoff_height_;
            stable_since_ = ros::Time(0);

            transitionTo(FlightState::TAKEOFF);
            break;

        case FlightState::TAKEOFF:
            publishHoldSetpoint();

            if (takeoffStable()) {
                if (stable_since_.isZero()) {
                    stable_since_ = now;
                }

                if ((now - stable_since_).toSec() >=
                    hover_stable_time_) {

                    transitionTo(
                        FlightState::MAP_WARMUP);
                }

            } else {
                stable_since_ = ros::Time(0);
            }
            break;

        case FlightState::MAP_WARMUP:
            publishHoldSetpoint();

            if (!known_goal_mode_ && !cloudFresh(now)) {
                printStateThrottled(
                    "waiting for fresh LiDAR data");
                break;
            }

            if (stateElapsed(now) >= map_warmup_time_) {
                transitionTo(
                    FlightState::HOVER_READY);
            }
            break;

        case FlightState::HOVER_READY:
            publishHoldSetpoint();

            if (!known_goal_mode_ && have_pending_goal_) {
                sendPendingGoalToSuper();
            }
            break;

        case FlightState::WAIT_TRAJECTORY:
            publishHoldSetpoint();

            if (have_super_setpoint_ &&
                last_super_setpoint_time_ >
                    goal_sent_time_) {

                transitionTo(
                    FlightState::TRACK_SUPER);
                tracking_start_position_ = positionOf(last_published_setpoint_.position);
                publishSuperSetpoint(now);

            } else if (stateElapsed(now) >
                       planning_timeout_) {

                ROS_ERROR(
                    "[FlightManager] SUPER planning timeout.");

                have_active_goal_ = false;

                transitionTo(
                    FlightState::HOVER_READY);
            }
            break;

        case FlightState::TRACK_SUPER:
            if (superSetpointFresh(now)) {
                publishSuperSetpoint(now);
                break;
            }

            handleSuperCommandTimeout();
            break;

        case FlightState::FINAL_HOVER:
            publishHoldSetpoint();

            if (have_pending_goal_) {
                sendPendingGoalToSuper();
            }
            break;

        case FlightState::SETTLE_HOVER:
            publishSettlingSetpoint(now);
            break;

        case FlightState::FAILSAFE_HOVER:
            publishHoldSetpoint();

            ROS_ERROR_THROTTLE(
                2.0,
                "[FlightManager] FAILSAFE_HOVER: "
                "SUPER command lost before reaching goal.");
            break;
        case FlightState::TRACK_KNOWN:
            if (known_lease_.fresh(now.toSec())) publishCommand(known_setpoint_);
            else {
                setHoldFromCurrentOdometry();
                transitionTo(FlightState::FAILSAFE_HOVER);
                publishHoldSetpoint();
            }
            break;
        }
        std_msgs::String state_message;
        state_message.data=stateName(flight_state_); flight_state_pub_.publish(state_message);

        ROS_INFO_STREAM_THROTTLE(
            2.0,
            "[FlightManager] "
            << stateName(flight_state_)
            << " | connected="
            << current_fcu_state_.connected
            << " armed="
            << current_fcu_state_.armed
            << " mode="
            << current_fcu_state_.mode
            << " position=("
            << current_odom_.pose.pose.position.x << ", "
            << current_odom_.pose.pose.position.y << ", "
            << current_odom_.pose.pose.position.z << ")");
    }

    void sendPendingGoalToSuper()
    {
        active_goal_ = pending_goal_;
        active_goal_.header.stamp = ros::Time::now();
        active_goal_.header.frame_id = "world";

        have_active_goal_ = true;
        have_pending_goal_ = false;

        /*
         * 清除上一条轨迹状态，防止旧指令立即接管。
         */
        have_super_setpoint_ = false;
        have_planned_end_ = false;
        last_trajectory_stamp_ = ros::Time(0);

        goal_sent_time_ = ros::Time::now();

        super_goal_pub_.publish(active_goal_);

        ROS_INFO_STREAM(
            "[FlightManager] Goal sent to SUPER: "
            << active_goal_.pose.position.x << ", "
            << active_goal_.pose.position.y << ", "
            << active_goal_.pose.position.z);

        transitionTo(
            FlightState::WAIT_TRAJECTORY);
    }

    void handleSuperCommandTimeout()
    {
        const auto terminal = motionOf(latest_super_setpoint_);
        if (have_active_goal_ && have_super_setpoint_ && have_planned_end_ &&
            super_px4_bridge::terminalTimelineComplete(
                latest_super_setpoint_.header.stamp.toSec(), planned_end_time_.toSec()) &&
            super_px4_bridge::terminalEligible(
                positionOf(active_goal_.pose.position), terminal,
                positionOf(current_odom_.pose.pose.position), max_goal_projection_,
                terminal_capture_radius_, terminal_speed_tolerance_) &&
            super_px4_bridge::distance(motionOf(last_published_setpoint_).position,
                                      terminal.position) <= max_goal_projection_) {
            // Preserve the planner's actual endpoint, not the unsnapped request.
            hold_x_ = terminal.position[0];
            hold_y_ = terminal.position[1];
            hold_z_ = terminal.position[2];
            hold_yaw_ = latest_super_setpoint_.yaw;
            settle_start_ = motionOf(last_published_setpoint_);
            stable_since_ = ros::Time(0);
            ROS_INFO_STREAM("[FlightManager] Accept terminal: requested_z="
                            << active_goal_.pose.position.z << " resolved_z=" << hold_z_);
            transitionTo(FlightState::SETTLE_HOVER);
            publishSettlingSetpoint(ros::Time::now());
            return;
        }
        enterFailsafe("SUPER command lost away from a valid stationary endpoint");
    }

    void enterFailsafe(const char *reason)
    {
        setHoldFromCurrentOdometry();
        have_active_goal_ = false;
        ROS_ERROR_STREAM("[FlightManager] " << reason);
        transitionTo(FlightState::FAILSAFE_HOVER);
        publishHoldSetpoint();
    }

    void publishSettlingSetpoint(const ros::Time &now)
    {
        const double elapsed = stateElapsed(now);
        auto target = last_published_setpoint_;
        const super_px4_bridge::Vec3 endpoint{{hold_x_, hold_y_, hold_z_}};
        const auto sample = super_px4_bridge::settleToPoint(
            settle_start_, endpoint, elapsed, handover_time_);
        applyMotion(target, sample);
        target.type_mask = trackingMask();
        target.yaw = hold_yaw_;
        publishCommand(target);
        const bool stable = elapsed >= handover_time_ &&
            super_px4_bridge::distance(positionOf(current_odom_.pose.pose.position), endpoint)
                <= goal_tolerance_ && currentSpeed() <= velocity_tolerance_;
        if (!stable) {
            stable_since_ = ros::Time(0);
        } else if (stable_since_.isZero()) {
            stable_since_ = now;
        } else if ((now - stable_since_).toSec() >= hover_stable_time_) {
            geometry_msgs::PoseStamped resolved;
            resolved.header.stamp = now;
            resolved.header.frame_id = "world";
            resolved.pose.position.x = hold_x_;
            resolved.pose.position.y = hold_y_;
            resolved.pose.position.z = hold_z_;
            resolved.pose.orientation = yawToQuaternion(hold_yaw_);
            resolved_goal_pub_.publish(resolved);
            transitionTo(FlightState::FINAL_HOVER);
        }
        if (flight_state_ == FlightState::SETTLE_HOVER && elapsed > settle_timeout_) {
            enterFailsafe("terminal hold failed to settle in time");
        }
    }

    void publishSuperSetpoint(
        const ros::Time &now)
    {
        mavros_msgs::PositionTarget target =
            latest_super_setpoint_;

        target.header.stamp = now;
        target.coordinate_frame =
            mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        target.type_mask =
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        target.acceleration_or_force.x = 0.0;
        target.acceleration_or_force.y = 0.0;
        target.acceleration_or_force.z = 0.0;
        target.yaw_rate = 0.0;

        const double elapsed = stateElapsed(now);
        if (elapsed < handover_time_) {
            const auto blended = super_px4_bridge::blendFromHold(
                tracking_start_position_, motionOf(target), elapsed, handover_time_);
            applyMotion(target, blended);
        }
        publishCommand(target);
    }

    void publishHoldSetpoint()
    {
        mavros_msgs::PositionTarget target;

        target.header.stamp = ros::Time::now();
        target.coordinate_frame =
            mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        /*
         * 悬停和起飞阶段只使用 position + yaw。
         */
        target.type_mask =
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        target.position.x = hold_x_;
        target.position.y = hold_y_;
        target.position.z = hold_z_;

        target.velocity.x = 0.0;
        target.velocity.y = 0.0;
        target.velocity.z = 0.0;

        target.acceleration_or_force.x = 0.0;
        target.acceleration_or_force.y = 0.0;
        target.acceleration_or_force.z = 0.0;

        target.yaw = hold_yaw_;
        target.yaw_rate = 0.0;

        publishCommand(target);
    }

    static super_px4_bridge::Vec3 positionOf(const geometry_msgs::Point &p)
    {
        return super_px4_bridge::Vec3{{p.x, p.y, p.z}};
    }

    static super_px4_bridge::Motion motionOf(const mavros_msgs::PositionTarget &target)
    {
        super_px4_bridge::Motion motion;
        motion.position = positionOf(target.position);
        if (!(target.type_mask & mavros_msgs::PositionTarget::IGNORE_VX)) {
            motion.velocity[0] = target.velocity.x;
        }
        if (!(target.type_mask & mavros_msgs::PositionTarget::IGNORE_VY)) {
            motion.velocity[1] = target.velocity.y;
        }
        if (!(target.type_mask & mavros_msgs::PositionTarget::IGNORE_VZ)) {
            motion.velocity[2] = target.velocity.z;
        }
        return motion;
    }

    static unsigned trackingMask()
    {
        return mavros_msgs::PositionTarget::IGNORE_AFX |
               mavros_msgs::PositionTarget::IGNORE_AFY |
               mavros_msgs::PositionTarget::IGNORE_AFZ |
               mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    }

    static void applyMotion(mavros_msgs::PositionTarget &target,
                            const super_px4_bridge::Motion &motion)
    {
        target.position.x = motion.position[0];
        target.position.y = motion.position[1];
        target.position.z = motion.position[2];
        target.velocity.x = motion.velocity[0];
        target.velocity.y = motion.velocity[1];
        target.velocity.z = motion.velocity[2];
    }

    void publishCommand(mavros_msgs::PositionTarget target)
    {
        const ros::Time now = ros::Time::now();
        if (!have_published_setpoint_) {
            yaw_limiter_.reset(quaternionToYaw(current_odom_.pose.pose.orientation), now.toSec());
        }
        target.header.stamp = now;
        target.yaw = yaw_limiter_.update(target.yaw, now.toSec(),
                                         yaw_rate_limit_, yaw_acceleration_limit_);
        target.yaw_rate = 0.0; // yaw-rate feedforward remains masked out.
        last_published_setpoint_ = target;
        have_published_setpoint_ = true;
        mavros_setpoint_pub_.publish(target);
    }

    void setHoldFromCurrentOdometry()
    {
        hold_x_ =
            current_odom_.pose.pose.position.x;

        hold_y_ =
            current_odom_.pose.pose.position.y;

        hold_z_ =
            current_odom_.pose.pose.position.z;

        hold_yaw_ =
            quaternionToYaw(
                current_odom_.pose.pose.orientation);
    }

    bool takeoffStable() const
    {
        const double altitude_error =
            std::fabs(
                current_odom_.pose.pose.position.z -
                takeoff_height_);

        return altitude_error <= altitude_tolerance_ &&
               currentSpeed() <= velocity_tolerance_;
    }

    bool odomFresh(
        const ros::Time &now) const
    {
        return have_odom_ &&
               (now - last_odom_time_).toSec() <=
                   odom_timeout_;
    }

    bool cloudFresh(
        const ros::Time &now) const
    {
        return have_cloud_ &&
               (now - last_cloud_time_).toSec() <=
                   cloud_timeout_;
    }

    bool superSetpointFresh(
        const ros::Time &now) const
    {
        return have_super_setpoint_ &&
               (now - last_super_setpoint_time_).toSec()
                   <= super_cmd_timeout_;
    }

    double currentSpeed() const
    {
        const auto &v =
            current_odom_.twist.twist.linear;

        return std::sqrt(
            v.x * v.x +
            v.y * v.y +
            v.z * v.z);
    }

    double distanceToActiveGoal() const
    {
        if (!have_active_goal_) {
            return 1e9;
        }

        const auto &p =
            current_odom_.pose.pose.position;

        const auto &g =
            active_goal_.pose.position;

        const double dx = p.x - g.x;
        const double dy = p.y - g.y;
        const double dz = p.z - g.z;

        return std::sqrt(
            dx * dx +
            dy * dy +
            dz * dz);
    }

    double stateElapsed(
        const ros::Time &now) const
    {
        return (now - state_enter_time_).toSec();
    }

    void requestOffboard()
    {
        const ros::Time now = ros::Time::now();

        if (!last_mode_request_.isZero() &&
            (now - last_mode_request_).toSec() < 1.0) {
            return;
        }

        last_mode_request_ = now;

        mavros_msgs::SetMode mode_command;
        mode_command.request.base_mode = 0;
        mode_command.request.custom_mode = "OFFBOARD";

        if (set_mode_client_.call(mode_command) &&
            mode_command.response.mode_sent) {

            ROS_INFO(
                "[FlightManager] OFFBOARD request accepted.");
        } else {
            ROS_WARN(
                "[FlightManager] OFFBOARD request failed.");
        }
    }

    void requestArm()
    {
        const ros::Time now = ros::Time::now();

        if (!last_arm_request_.isZero() &&
            (now - last_arm_request_).toSec() < 1.0) {
            return;
        }

        last_arm_request_ = now;

        mavros_msgs::CommandBool arm_command;
        arm_command.request.value = true;

        if (arming_client_.call(arm_command) &&
            arm_command.response.success) {

            ROS_INFO(
                "[FlightManager] ARM request accepted.");
        } else {
            ROS_WARN(
                "[FlightManager] ARM request failed.");
        }
    }

    void transitionTo(
        FlightState next_state)
    {
        if (next_state == flight_state_) {
            return;
        }

        ROS_INFO_STREAM(
            "[FlightManager] "
            << stateName(flight_state_)
            << " -> "
            << stateName(next_state));

        flight_state_ = next_state;
        state_enter_time_ = ros::Time::now();
    }

    const char *stateName(
        FlightState state) const
    {
        switch (state) {
        case FlightState::WAIT_FCU:
            return "WAIT_FCU";
        case FlightState::PRESTREAM:
            return "PRESTREAM";
        case FlightState::OFFBOARD_ARM:
            return "OFFBOARD_ARM";
        case FlightState::TAKEOFF:
            return "TAKEOFF";
        case FlightState::MAP_WARMUP:
            return "MAP_WARMUP";
        case FlightState::HOVER_READY:
            return "HOVER_READY";
        case FlightState::WAIT_TRAJECTORY:
            return "WAIT_TRAJECTORY";
        case FlightState::TRACK_SUPER:
            return "TRACK_SUPER";
        case FlightState::SETTLE_HOVER:
            return "SETTLE_HOVER";
        case FlightState::FINAL_HOVER:
            return "FINAL_HOVER";
        case FlightState::FAILSAFE_HOVER:
            return "FAILSAFE_HOVER";
        case FlightState::TRACK_KNOWN:
            return "TRACK_KNOWN";
        }

        return "UNKNOWN";
    }

    void printStateThrottled(
        const std::string &message)
    {
        ROS_INFO_STREAM_THROTTLE(
            2.0,
            "[FlightManager] "
            << stateName(flight_state_)
            << ": " << message);
    }

    static bool quaternionValid(
        const geometry_msgs::Quaternion &q)
    {
        const double norm_squared =
            q.x * q.x +
            q.y * q.y +
            q.z * q.z +
            q.w * q.w;

        return std::isfinite(norm_squared) && norm_squared > 1e-6;
    }

    static double quaternionToYaw(
        const geometry_msgs::Quaternion &q)
    {
        const double sin_yaw =
            2.0 * (q.w * q.z + q.x * q.y);

        const double cos_yaw =
            1.0 -
            2.0 * (q.y * q.y + q.z * q.z);

        return std::atan2(sin_yaw, cos_yaw);
    }

    static geometry_msgs::Quaternion yawToQuaternion(
        double yaw)
    {
        geometry_msgs::Quaternion q;

        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw * 0.5);
        q.w = std::cos(yaw * 0.5);

        return q;
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber state_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber user_goal_sub_;
    ros::Subscriber super_setpoint_sub_;
    ros::Subscriber trajectory_sub_;
    ros::Subscriber known_setpoint_sub_;
    ros::Publisher flight_state_pub_;
    ros::Publisher known_status_pub_;
    ros::Time last_fcu_time_;
    bool known_goal_mode_{false};
    super_px4_bridge::KnownFlightLease known_lease_;
    super_px4_bridge::KnownTaskBinding known_binding_;
    mavros_msgs::PositionTarget known_setpoint_;

    ros::Publisher mavros_setpoint_pub_;
    ros::Publisher super_goal_pub_;
    ros::Publisher resolved_goal_pub_;

    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;

    ros::Timer control_timer_;

    mavros_msgs::State current_fcu_state_;
    nav_msgs::Odometry current_odom_;

    geometry_msgs::PoseStamped pending_goal_;
    geometry_msgs::PoseStamped active_goal_;

    mavros_msgs::PositionTarget latest_super_setpoint_;
    mavros_msgs::PositionTarget last_published_setpoint_;
    super_px4_bridge::Vec3 tracking_start_position_{};
    super_px4_bridge::Motion settle_start_;
    super_px4_bridge::YawLimiter yaw_limiter_;

    FlightState flight_state_{FlightState::WAIT_FCU};

    bool have_fcu_state_{false};
    bool have_odom_{false};
    bool have_cloud_{false};
    bool have_pending_goal_{false};
    bool have_active_goal_{false};
    bool have_super_setpoint_{false};
    bool have_published_setpoint_{false};
    bool have_planned_end_{false};

    bool auto_arm_{true};

    double takeoff_height_{1.5};
    double publish_rate_{50.0};
    double prestream_time_{1.0};
    double hover_stable_time_{1.0};
    double map_warmup_time_{2.0};
    double super_cmd_timeout_{1.0};
    double planning_timeout_{5.0};
    double goal_tolerance_{0.20};
    double altitude_tolerance_{0.15};
    double velocity_tolerance_{0.25};
    double odom_timeout_{0.5};
    double cloud_timeout_{1.0};
    double handover_time_{1.0};
    double settle_timeout_{6.0};
    double max_goal_projection_{0.35};
    double terminal_capture_radius_{0.5};
    double terminal_speed_tolerance_{0.05};
    double max_source_age_{0.3};
    double yaw_rate_limit_{0.35};
    double yaw_acceleration_limit_{0.5};

    double hold_x_{0.0};
    double hold_y_{0.0};
    double hold_z_{0.0};
    double hold_yaw_{0.0};

    ros::Time state_enter_time_;
    ros::Time stable_since_;
    ros::Time last_odom_time_;
    ros::Time last_cloud_time_;
    ros::Time last_super_setpoint_time_;
    ros::Time goal_sent_time_;
    ros::Time planned_end_time_;
    ros::Time last_trajectory_stamp_;
    ros::Time last_mode_request_;
    ros::Time last_arm_request_;

    std::string state_topic_;
    std::string odom_topic_;
    std::string cloud_topic_;
    std::string user_goal_topic_;
    std::string super_goal_topic_;
    std::string super_setpoint_topic_;
    std::string mavros_setpoint_topic_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "super_flight_manager");

    SuperFlightManager manager;

    ros::spin();
    return 0;
}
