#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <algorithm>
#include <cmath>
#include <functional>
#include "command_watchdog.hpp"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float64.h>

namespace gazebo
{

class JointPositionController : public ModelPlugin
{
public:
    JointPositionController()
    {
        gzmsg << "[JointPositionController] constructor\n";
    }

    ~JointPositionController() override
    {
        gzmsg << "[JointPositionController] destructor joint=" << joint_name_ << "\n";
        update_connection_.reset();
        running_ = false;
        queue_.disable();

        if (queue_thread_.joinable()) {
            if (queue_thread_.get_id() != std::this_thread::get_id()) {
                queue_thread_.join();
            } else {
                queue_thread_.detach();
            }
        }

        gz_command_sub_.reset();
        gz_status_pub_.reset();
        gz_node_.reset();
    }

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
        model_ = model;

        if (!model_) {
            gzerr << "[JointPositionController] Invalid model\n";
            return;
        }

        if (!sdf->HasElement("jointName")) {
            gzerr << "[JointPositionController] Missing <jointName>\n";
            return;
        }

        joint_name_ = sdf->Get<std::string>("jointName");
        control_source_ = sdf->HasElement("controlSource") ? sdf->Get<std::string>("controlSource") : "ros";
        if (control_source_ != "ros" && control_source_ != "gazebo") {
            gzerr << "Invalid controlSource (expected ros or gazebo)\n";
            return;
        }

        topic_name_ = sdf->HasElement("topic") ? sdf->Get<std::string>("topic") : joint_name_ + "/cmd_pos";

        const std::string robot_namespace = sdf->HasElement("robotNamespace") ? sdf->Get<std::string>("robotNamespace") : "/so101";

        const double p = sdf->HasElement("p") ? sdf->Get<double>("p") : 10.0;
        const double i = sdf->HasElement("i") ? sdf->Get<double>("i") : 0.0;
        const double d = sdf->HasElement("d") ? sdf->Get<double>("d") : 0.3;
        const double cmd_max = sdf->HasElement("cmdMax") ? sdf->Get<double>("cmdMax") : 3.0;

        if (!std::isfinite(p) || !std::isfinite(i) || !std::isfinite(d)
            || !std::isfinite(cmd_max) || p < 0.0 || i < 0.0 || d < 0.0 || cmd_max <= 0.0) {
            gzerr << "[SO101] PID gains and positive cmdMax must be finite\n";
            return;
        }
        const std::string timeout_policy = sdf->HasElement("commandTimeoutPolicy")
            ? sdf->Get<std::string>("commandTimeoutPolicy") : "measured_position";
        if (timeout_policy == "last_target" && hasJointSuffix(joint_name_, "gripper")) {
            timeout_policy_ = so101::TimeoutPolicy::LastTarget;
        } else if (timeout_policy != "measured_position") {
            gzerr << "[SO101] commandTimeoutPolicy must be measured_position; "
                  << "last_target is opt-in for gripper only\n";
            return;
        }

        joint_ = findJoint(model_, joint_name_, control_model_);

        if (!joint_ || !control_model_) {
            gzerr << "[JointPositionController] Cannot find joint: " << joint_name_ << "\n";
            return;
        }

        controller_ = control_model_->GetJointController();

        if (!controller_) {
            gzerr << "[JointPositionController] Cannot get controller for joint: " << joint_name_ << "\n";
            return;
        }

        scoped_name_ = joint_->GetScopedName();

        lower_limit_ = joint_->LowerLimit(0);
        upper_limit_ = joint_->UpperLimit(0);

        const bool has_initial_position = sdf->HasElement("initialPosition");

        initial_position_ = has_initial_position ? sdf->Get<double>("initialPosition") : joint_->Position(0);
        initial_position_ = clampTarget(initial_position_);
        initial_position_pending_ = has_initial_position;

        // Bound integral contribution; zero preserves legacy PD behavior.
        const double i_max = sdf->HasElement("iMax") ? sdf->Get<double>("iMax") : 0.0;
        if (!std::isfinite(i_max) || i_max < 0.0 || i_max > cmd_max) {
            gzerr << "[SO101] iMax must lie between zero and cmdMax\n";
            return;
        }
        common::PID pid(p, i, d, i_max, -i_max, cmd_max, -cmd_max);
        controller_->SetPositionPID(scoped_name_, pid);

        target_.store(initial_position_, std::memory_order_relaxed);
        applied_target_ = initial_position_;

        if (!has_initial_position) {
            controller_->SetPositionTarget(scoped_name_, initial_position_);
        }

        setupGazeboTransport(sdf);

        if (control_source_ == "ros" && ros::isInitialized()) {
            nh_.reset(new ros::NodeHandle(robot_namespace));
            nh_->setCallbackQueue(&queue_);

            sub_ = nh_->subscribe<std_msgs::Float64>(topic_name_, 1, &JointPositionController::commandCallback, this);

            queue_thread_ = std::thread(&JointPositionController::queueThread, this);

            gzmsg << "[JointPositionController] ROS enabled for joint " << joint_name_ << " topic=" << nh_->resolveName(topic_name_) << "\n";

        } else {
            gzmsg << "[JointPositionController] ROS not initialized, Gazebo Transport only for joint " << joint_name_ << "\n";
        }

        update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&JointPositionController::onUpdate, this));

        gzmsg << "[JointPositionController] joint=" << joint_name_ << " scoped=" << scoped_name_ << " initial=" << target_.load(std::memory_order_relaxed) << "\n";
        gzmsg << "[JointPositionController] GZ command topic=" << gz_command_topic_ << "\n";
        gzmsg << "[JointPositionController] GZ status topic=" << gz_status_topic_ << "\n";
        gzmsg << "[SO101] Timeout policy joint=" << joint_name_ << " policy=" << timeout_policy << "\n";
    }

    void Reset() override
    {
        // Perform controller mutations on the update thread, not in callbacks.
        reset_pending_.store(true, std::memory_order_release);
    }

private:
    void setupGazeboTransport(sdf::ElementPtr sdf)
    {
        gz_command_topic_ = sdf->HasElement("gzCommandTopic") ? sdf->Get<std::string>("gzCommandTopic") : "~/so101/" + joint_name_ + "/command";
        gz_status_topic_ = sdf->HasElement("gzStatusTopic") ? sdf->Get<std::string>("gzStatusTopic") : "~/so101/" + joint_name_ + "/status";

        gz_node_.reset(new transport::Node());
        gz_node_->Init(model_->GetWorld()->Name());

        if (control_source_ == "gazebo") {
            gz_command_sub_ = gz_node_->Subscribe(gz_command_topic_, &JointPositionController::gazeboCommandCallback, this);
        }
        gz_status_pub_ = gz_node_->Advertise<msgs::Vector2d>(gz_status_topic_, 10);
        // x: PID torque command, y: Joint::GetForce, z: target-angle error.
        gz_effort_pub_ = gz_node_->Advertise<msgs::Vector3d>(gz_status_topic_ + "/effort", 10);
    }

    void setTarget(double value)
    {
        if (!std::isfinite(value)) {
            return;
        }

        const double limited_target = clampTarget(value);

        if (limited_target != value) {
            gzdbg << "[JointPositionController] Command for " << joint_name_ << " limited from " << value << " to " << limited_target << "\n";
        }

        target_.store(limited_target, std::memory_order_relaxed);
        target_dirty_.store(true, std::memory_order_release);
    }

    void commandCallback(const std_msgs::Float64ConstPtr &msg)
    {
        if (!std::isfinite(msg->data)) {
            ROS_WARN_THROTTLE(1.0, "[JointPositionController] Ignoring non-finite command for %s", joint_name_.c_str());
            return;
        }

        const double limited_target = clampTarget(msg->data);

        if (limited_target != msg->data) {
            ROS_WARN_THROTTLE(1.0, "[JointPositionController] Command for %s limited to %.6f", joint_name_.c_str(), limited_target);
        }

        setTarget(msg->data);
    }

    void gazeboCommandCallback(ConstVector2dPtr &msg)
    {
        if (!msg) {
            return;
        }

        setTarget(msg->x());
    }

    double clampTarget(double value) const
    {
        if (std::isfinite(lower_limit_)) {
            value = std::max(value, lower_limit_);
        }

        if (std::isfinite(upper_limit_)) {
            value = std::min(value, upper_limit_);
        }

        return value;
    }

    void onUpdate()
    {
        if (!joint_ || !controller_) {
            return;
        }

        const double now = model_->GetWorld()->SimTime().Double();
        const auto watchdog_event = watchdog_.update(now);
        if (reset_pending_.exchange(false, std::memory_order_acq_rel)
            || watchdog_event == so101::WatchdogEvent::ClockReset) {
            watchdog_.reset(now);
            target_dirty_.store(false, std::memory_order_release);
            initial_position_pending_ = false;
            last_status_pub_time_ = common::Time(0, 0);
            auto pids = controller_->GetPositionPIDs();
            auto pid = pids.find(scoped_name_);
            if (pid != pids.end()) {
                pid->second.Reset();
                controller_->SetPositionPID(scoped_name_, pid->second);
            }
            const double measured = joint_->Position(0);
            if (std::isfinite(measured)) {
                applied_target_ = clampTarget(measured);
                target_.store(applied_target_, std::memory_order_relaxed);
                controller_->SetPositionTarget(scoped_name_, applied_target_);
            }
            gzmsg << "[SO101] Reset cleared old command/preload: " << joint_name_ << "\n";
            publishJointStatus();
            return;
        }

        bool received_command = false;
        if (initial_position_pending_) {
            joint_->SetPosition(0, initial_position_);
            target_.store(initial_position_, std::memory_order_relaxed);
            controller_->SetPositionTarget(scoped_name_, initial_position_);
            initial_position_pending_ = false;

        } else if (target_dirty_.exchange(false, std::memory_order_acq_rel)) {
            applied_target_ = target_.load(std::memory_order_relaxed);
            controller_->SetPositionTarget(scoped_name_, applied_target_);
            watchdog_.command(now);
            received_command = true;
        }

        if (control_source_ == "gazebo" && !received_command
            && watchdog_event == so101::WatchdogEvent::Timeout) {
            const double hold = so101::timeoutTarget(timeout_policy_, applied_target_, joint_->Position(0));
            if (std::isfinite(hold)) {
                applied_target_ = clampTarget(hold);
                controller_->SetPositionTarget(scoped_name_, applied_target_);
            }
            gzmsg << "[SO101] Command timeout: " << joint_name_
                  << (timeout_policy_ == so101::TimeoutPolicy::LastTarget ? " retaining bounded last target=" : " holding measured position=")
                  << applied_target_ << "\n";
        }

        publishJointStatus();
    }

    void publishJointStatus()
    {
        if (!gz_status_pub_ || !joint_ || !model_ || !model_->GetWorld()) {
            return;
        }

        const common::Time now = model_->GetWorld()->SimTime();

        if ((now - last_status_pub_time_).Double() < 0.02) {
            return;
        }

        last_status_pub_time_ = now;

        msgs::Vector2d status_msg;
        status_msg.set_x(joint_->Position(0));
        status_msg.set_y(joint_->GetVelocity(0));

        gz_status_pub_->Publish(status_msg);
        auto pids = controller_->GetPositionPIDs();
        auto pid = pids.find(scoped_name_);
        if (pid != pids.end()) {
            msgs::Vector3d effort;
            effort.set_x(pid->second.GetCmd());
            effort.set_y(joint_->GetForce(0));
            effort.set_z(applied_target_ - joint_->Position(0));
            gz_effort_pub_->Publish(effort);
        }
    }

    void queueThread()
    {
        while (running_ && ros::ok()) {
            queue_.callAvailable(ros::WallDuration(0.01));
        }
    }

    static bool hasJointSuffix(const std::string &actual_name, const std::string &requested_name)
    {
        if (actual_name == requested_name) {
            return true;
        }

        const std::string suffix = "::" + requested_name;

        return actual_name.size() >= suffix.size() && actual_name.compare(actual_name.size() - suffix.size(), suffix.size(), suffix) == 0;
    }

    static physics::JointPtr findJoint(const physics::ModelPtr &model, const std::string &requested_name, physics::ModelPtr &owner_model)
    {
        if (!model) {
            return {};
        }

        if (physics::JointPtr joint = model->GetJoint(requested_name)) {
            owner_model = model;
            return joint;
        }

        for (const auto &joint : model->GetJoints()) {
            if (!joint) {
                continue;
            }

            if (hasJointSuffix(joint->GetName(), requested_name) || hasJointSuffix(joint->GetScopedName(), requested_name)) {
                owner_model = model;
                return joint;
            }
        }

        for (const auto &nested_model : model->NestedModels()) {
            physics::JointPtr joint = findJoint(nested_model, requested_name, owner_model);

            if (joint) {
                return joint;
            }
        }

        return {};
    }

private:
    physics::ModelPtr model_;
    physics::JointPtr joint_;
    physics::JointControllerPtr controller_;
    physics::ModelPtr control_model_;

    std::unique_ptr<ros::NodeHandle> nh_;
    ros::Subscriber sub_;
    ros::CallbackQueue queue_;
    std::thread queue_thread_;

    transport::NodePtr gz_node_;
    transport::SubscriberPtr gz_command_sub_;
    transport::PublisherPtr gz_status_pub_;
    transport::PublisherPtr gz_effort_pub_;

    std::atomic<bool> running_{true};

    std::string control_source_;
    std::string joint_name_;
    std::string topic_name_;
    std::string scoped_name_;

    std::string gz_command_topic_;
    std::string gz_status_topic_;

    event::ConnectionPtr update_connection_;

    double lower_limit_{0.0};
    double upper_limit_{0.0};

    std::atomic<double> target_{0.0};
    std::atomic<bool> target_dirty_{false};

    double initial_position_{0.0};
    bool initial_position_pending_{false};

    double applied_target_{0.0};
    so101::CommandWatchdog watchdog_;
    so101::TimeoutPolicy timeout_policy_{so101::TimeoutPolicy::MeasuredPosition};
    std::atomic<bool> reset_pending_{false};
    common::Time last_status_pub_time_{0, 0};
};

GZ_REGISTER_MODEL_PLUGIN(JointPositionController)

} // namespace gazebo
