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
#include <gazebo/physics/ContactManager.hh>
#include <algorithm>
#include <cmath>

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
        approach_guard_=sdf->HasElement("approachGuard") && sdf->Get<bool>("approachGuard");
        if (approach_guard_) world_->Physics()->GetContactManager()->SetNeverDropContacts(true);
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
    static geometry_msgs::Vector3 vector(const ignition::math::Vector3d &v) {
        geometry_msgs::Vector3 out; out.x=v.X(); out.y=v.Y(); out.z=v.Z(); return out;
    }
    static bool bounds(physics::CollisionPtr c,ignition::math::Vector3d &lo,ignition::math::Vector3d &hi) {
        const auto box=c ? boost::dynamic_pointer_cast<physics::BoxShape>(c->GetShape()) : nullptr;
        if (!box) return false;
        const auto p=c->WorldPose(); const auto h=box->Size()*.5;
        if (!p.Pos().IsFinite() || !p.Rot().IsFinite() || !h.IsFinite()) return false;
        ignition::math::Vector3d e;
        for (int i=0;i<3;++i) { ignition::math::Vector3d a; a[i]=h[i];
            const auto r=p.Rot().RotateVector(a); for (int j=0;j<3;++j) e[j]+=std::abs(r[j]); }
        lo=p.Pos()-e; hi=p.Pos()+e; return true;
    }
    void approachGeometry(super_px4_bridge::CoordinateTruth &out,physics::ModelPtr model) {
        if (!approach_guard_ || !model) return;
        const auto cube=world_->ModelByName(target_), support=world_->ModelByName("cube_support");
        if (!cube || cube->IsStatic() || !support || !support->IsStatic()) return;
        for (const auto &m:world_->Models()) if (m!=model && m!=cube && m!=support &&
            m->GetName()!="ground_plane" && m->GetName()!="asphalt_plane") return;
        std::vector<physics::CollisionPtr> obstacles;
        for (const auto &m:{cube,support}) for (const auto &l:m->GetLinks())
            for (const auto &c:l->GetCollisions()) obstacles.push_back(c);
        if (obstacles.size()!=2) return;
        // The production empty world contains BOTH ground_plane and a 0.1 m
        // asphalt box. Include the box, do not pretend ground is always z=0.
        const auto asphalt=world_->ModelByName("asphalt_plane");
        if (asphalt) {
            if (!asphalt->IsStatic()) return;
            for (const auto &l:asphalt->GetLinks()) for (const auto &c:l->GetCollisions()) obstacles.push_back(c);
        }
        const auto ground=world_->ModelByName("ground_plane");
        if (!ground || !ground->IsStatic() || ground->WorldPose().Pos().Length()>1e-8 ||
            std::abs(ground->WorldPose().Rot().W())<.999999) return;
        const auto arm=model->GetLink("so101::base_link"); if (!arm) return;
        double clearance=100.; unsigned count=0;
        for (const auto &l:model->GetLinks()) {
            if (l->GetName().find("so101::")!=0) continue;
            for (const auto &c:l->GetCollisions()) {
                ignition::math::Vector3d lo,hi; if (!bounds(c,lo,hi)) return;
                for (int corner=0;corner<8;++corner) {
                    const ignition::math::Vector3d p((corner&1)?hi.X():lo.X(),(corner&2)?hi.Y():lo.Y(),(corner&4)?hi.Z():lo.Z());
                    if ((p-arm->WorldPose().Pos()).Length()>.55) return;
                }
                ++count; clearance=std::min(clearance,lo.Z()); // known flat ground z=0
                for (const auto &o:obstacles) {
                    ignition::math::Vector3d a,b; if (!bounds(o,a,b)) return;
                    double squared=0;
                    for (int i=0;i<3;++i) { const double gap=std::max({a[i]-hi[i],lo[i]-b[i],0.}); squared+=gap*gap; }
                    clearance=std::min(clearance,std::sqrt(squared));
                }
            }
        }
        if (count!=8 || !std::isfinite(clearance)) return;
        out.approach_geometry_valid=true; out.arm_obstacle_clearance=clearance;
        out.arm_external_contact=contact_seen_; contact_seen_=false;
    }
    void sample() {
        const double now=world_->SimTime().Double();
        if (approach_guard_) {
            const auto manager=world_->Physics()->GetContactManager();
            for (unsigned i=0;i<manager->GetContactCount();++i) {
                const auto c=manager->GetContact(i); if (!c || !c->collision1 || !c->collision2) continue;
                const auto a=c->collision1->GetScopedName(), b=c->collision2->GetScopedName();
                const bool arm_a=a.find(vehicle_+"::so101::")==0, arm_b=b.find(vehicle_+"::so101::")==0;
                const bool vehicle_a=a.find(vehicle_+"::")==0, vehicle_b=b.find(vehicle_+"::")==0;
                contact_seen_|=(arm_a && !vehicle_b) || (arm_b && !vehicle_a);
            }
        }
        if (reset_requested_.exchange(false)) { ++epoch_; last_publish_=-1; identity_.clear(); }
        if (last_time_ >= 0 && now < last_time_) { ++epoch_; last_publish_=-1; identity_.clear(); }
        last_time_=now;
        if (last_publish_ >= 0 && now-last_publish_ < .02) return;
        last_publish_=now;
        super_px4_bridge::CoordinateTruth out;
        out.header.stamp.fromSec(now); out.header.frame_id="gazebo_world";
        out.source_instance=instance_; out.vehicle_name=vehicle_; out.target_name=target_;
        const auto model=world_->ModelByName(vehicle_);
        approachGeometry(out,model);
        std::vector<uint64_t> identity;
        if (model) {
            out.vehicle_id=model->GetId();
            identity.push_back(out.vehicle_id);
            const auto body=model->GetLink("yhang550_base::base_link");
            if (body) {
                const auto v=body->WorldLinearVel(), w=body->WorldAngularVel();
                out.body_velocity_valid=v.IsFinite() && w.IsFinite();
                if (out.body_velocity_valid) {
                    out.body_linear_velocity_world.x=v.X(); out.body_linear_velocity_world.y=v.Y(); out.body_linear_velocity_world.z=v.Z();
                    out.body_angular_velocity_world.x=w.X(); out.body_angular_velocity_world.y=w.Y(); out.body_angular_velocity_world.z=w.Z();
                }
            }
            for (const auto &name : links_) {
                const auto link=model->GetLink(name);
                if (!link) { identity.push_back(0); continue; }
                out.link_names.push_back(name); out.link_ids.push_back(link->GetId());
                out.link_poses.push_back(pose(link->WorldPose())); identity.push_back(link->GetId());
                if (name=="so101::gripper_link") {
                    const auto v=link->WorldLinearVel(), w=link->WorldAngularVel();
                    out.gripper_velocity_valid=v.IsFinite() && w.IsFinite();
                    if (out.gripper_velocity_valid) {
                        out.gripper_linear_velocity_world=vector(v);
                        out.gripper_angular_velocity_world=vector(w);
                    }
                }
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
            // This fixture observes a single rigid cube. Missing/multi-link
            // targets stay invalid rather than inventing a zero velocity.
            const auto links=object->GetLinks();
            if (links.size()==1) {
                const auto offset=links[0]->WorldPose().Rot().RotateVectorReverse(
                    object->WorldPose().Pos()-links[0]->WorldPose().Pos());
                const auto v=links[0]->WorldLinearVel(offset), w=links[0]->WorldAngularVel();
                out.target_velocity_valid=v.IsFinite() && w.IsFinite();
                if (out.target_velocity_valid) {
                    out.target_linear_velocity_world=vector(v);
                    out.target_angular_velocity_world=vector(w);
                }
            }
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
    bool approach_guard_{false},contact_seen_{false};
    double last_time_{-1}, last_publish_{-1};
};
GZ_REGISTER_WORLD_PLUGIN(CoordinateTruthPlugin)
}
