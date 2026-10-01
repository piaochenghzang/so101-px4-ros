// Read-only observer. Never sets a pose, joint target, velocity, force or PID.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <super_px4_bridge/CoordinateTruth.h>
#include <chrono>
#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace gazebo {
class CoordinateTruthPlugin : public WorldPlugin {
public:
    ~CoordinateTruthPlugin() override { connection_.reset(); reset_connection_.reset(); }
    void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override {
        if (!ros::isInitialized()) {
            gzerr << "[CoordinateTruth] Requires gazebo_ros API plugin; observation disabled.\n";
            return;
        }
        world_ = world;
        if (sdf->HasElement("vehicleName")) vehicle_ = sdf->Get<std::string>("vehicleName");
        if (sdf->HasElement("targetName")) target_ = sdf->Get<std::string>("targetName");
        instance_ = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
        node_.reset(new ros::NodeHandle());
        publisher_ = node_->advertise<super_px4_bridge::CoordinateTruth>("/grasp_audit/truth", 10);
        connection_ = event::Events::ConnectWorldUpdateEnd(std::bind(&CoordinateTruthPlugin::sample, this));
        reset_connection_ = event::Events::ConnectWorldReset([this]() { reset_requested_.store(true); });
        gzmsg << "[CoordinateTruth] Read-only physical-step snapshots, no actuation.\n";
    }
private:
    static geometry_msgs::Pose pose(const ignition::math::Pose3d &value) {
        geometry_msgs::Pose out;
        out.position.x=value.Pos().X(); out.position.y=value.Pos().Y(); out.position.z=value.Pos().Z();
        out.orientation.w=value.Rot().W(); out.orientation.x=value.Rot().X();
        out.orientation.y=value.Rot().Y(); out.orientation.z=value.Rot().Z();
        return out;
    }
    void sample() {
        const double now=world_->SimTime().Double();
        if (reset_requested_.exchange(false)) { ++epoch_; last_publish_=-1; identity_.clear(); }
        if (last_time_ >= 0 && now < last_time_) { ++epoch_; last_publish_=-1; identity_.clear(); }
        last_time_=now;
        if (last_publish_ >= 0 && now-last_publish_ < .02) return;
        last_publish_=now;
        super_px4_bridge::CoordinateTruth out;
        out.header.stamp.fromSec(now); out.header.frame_id="gazebo_world";
        out.source_instance=instance_; out.vehicle_name=vehicle_; out.target_name=target_;
        const auto model=world_->ModelByName(vehicle_);
        std::vector<uint64_t> identity;
        if (model) {
            out.vehicle_id=model->GetId();
            identity.push_back(out.vehicle_id);
            for (const auto &name : links_) {
                const auto link=model->GetLink(name);
                if (!link) { identity.push_back(0); continue; }
                out.link_names.push_back(name); out.link_ids.push_back(link->GetId());
                out.link_poses.push_back(pose(link->WorldPose())); identity.push_back(link->GetId());
            }
            // Static Joint::Position() is a cached scalar, not an ODE encoder.
            // Never read it or invent zero angles to pass an FK check.
            out.joint_measurement_valid=!model->IsStatic();
            if (out.joint_measurement_valid) for (const auto &name : joints_) {
                const auto joint=model->GetJoint("so101::"+name);
                if (!joint) continue;
                out.joint_names.push_back(name); out.joint_position.push_back(joint->Position(0));
                out.joint_velocity.push_back(joint->GetVelocity(0));
            }
        }
        const auto object=world_->ModelByName(target_);
        if (object) {
            out.target_present=true; out.target_id=object->GetId();
            out.target_pose=pose(object->WorldPose());
        }
        identity.push_back(out.target_id);
        if (!identity_.empty() && identity != identity_) ++epoch_;
        identity_=identity; out.world_epoch=epoch_;
        publisher_.publish(out);
    }
    physics::WorldPtr world_;
    event::ConnectionPtr connection_;
    event::ConnectionPtr reset_connection_;
    std::atomic<bool> reset_requested_{false};
    std::unique_ptr<ros::NodeHandle> node_;
    ros::Publisher publisher_;
    std::string vehicle_{"yhang550"}, target_{"test_cube"}, instance_;
    const std::vector<std::string> links_{"yhang550_base::base_link", "so101::base_link",
        "so101::gripper_link", "velodyne::link"};
    const std::vector<std::string> joints_{"shoulder_pan", "shoulder_lift", "elbow_flex",
        "wrist_flex", "wrist_roll", "gripper"};
    std::vector<uint64_t> identity_;
    uint64_t epoch_{1};
    double last_time_{-1}, last_publish_{-1};
};
GZ_REGISTER_WORLD_PLUGIN(CoordinateTruthPlugin)
}
