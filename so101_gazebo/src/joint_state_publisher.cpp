#include <algorithm>
#include <cctype>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace gazebo
{

class JointStatePublisher : public ModelPlugin
{
public:
    ~JointStatePublisher() override
    {
        update_connection_.reset();
    }

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
        if (!ros::isInitialized()) {
            gzerr << "[SO101JointStatePublisher] ROS is not initialized\n";
            return;
        }

        if (!sdf->HasElement("jointName")) {
            gzerr << "[SO101JointStatePublisher] Missing <jointName>\n";
            return;
        }

        const std::string robot_namespace = sdf->HasElement("robotNamespace") ? sdf->Get<std::string>("robotNamespace") : "/so101";
        const double update_rate = sdf->HasElement("updateRate") ? sdf->Get<double>("updateRate") : 50.0;

        update_period_ = update_rate > 0.0 ? 1.0 / update_rate : 0.0;

        const std::string names = sdf->Get<std::string>("jointName");
        std::stringstream stream(names);
        std::string name;

        while (std::getline(stream, name, ',')) {
            name.erase(std::remove_if(name.begin(), name.end(), [](unsigned char value) { return std::isspace(value); }), name.end());

            if (name.empty()) {
                continue;
            }

            physics::JointPtr joint = findJoint(model, name);

            if (!joint) {
                gzerr << "[SO101JointStatePublisher] Cannot resolve joint: " << name << "\n";
                return;
            }

            joint_names_.push_back(name);
            joints_.push_back(joint);
        }

        if (joints_.size() != EXPECTED_JOINT_COUNT) {
            gzerr << "[SO101JointStatePublisher] Expected " << EXPECTED_JOINT_COUNT << " joints, got " << joints_.size() << "\n";
            return;
        }

        nh_.reset(new ros::NodeHandle(robot_namespace));
        publisher_ = nh_->advertise<sensor_msgs::JointState>("joint_states", 10);

        last_publish_time_ = model->GetWorld()->SimTime();

        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&JointStatePublisher::onUpdate, this, std::placeholders::_1));

        gzmsg << "[SO101JointStatePublisher] Publishing " << joints_.size()
              << " joints on " << nh_->resolveName("joint_states") << "\n";
    }

private:
    static constexpr std::size_t EXPECTED_JOINT_COUNT = 6;

    static bool hasJointSuffix(const std::string &actual_name, const std::string &requested_name)
    {
        if (actual_name == requested_name) {
            return true;
        }

        const std::string suffix = "::" + requested_name;

        return actual_name.size() >= suffix.size() &&
               actual_name.compare(actual_name.size() - suffix.size(), suffix.size(), suffix) == 0;
    }

    static physics::JointPtr findJoint(const physics::ModelPtr &model, const std::string &requested_name)
    {
        if (!model) {
            return {};
        }

        if (physics::JointPtr joint = model->GetJoint(requested_name)) {
            return joint;
        }

        for (const auto &joint : model->GetJoints()) {
            if (!joint) {
                continue;
            }

            if (hasJointSuffix(joint->GetName(), requested_name) ||
                hasJointSuffix(joint->GetScopedName(), requested_name)) {
                return joint;
            }
        }

        for (const auto &nested_model : model->NestedModels()) {
            if (physics::JointPtr joint = findJoint(nested_model, requested_name)) {
                return joint;
            }
        }

        return {};
    }

    void onUpdate(const common::UpdateInfo &info)
    {
        const double elapsed = (info.simTime - last_publish_time_).Double();

        // Gazebo Reset Simulation 后 simTime 会从大值重新回到 0。
        if (elapsed < 0.0) {
            last_publish_time_ = info.simTime;
            return;
        }

        if (update_period_ > 0.0 && elapsed < update_period_) {
            return;
        }

        last_publish_time_ = info.simTime;

        sensor_msgs::JointState message;
        // Stamp the Gazebo sample itself. ROS /clock delivery can lag a
        // physics update and must not give distinct samples the same time.
        message.header.stamp = ros::Time(info.simTime.sec, info.simTime.nsec);
        message.name = joint_names_;

        message.position.resize(joints_.size());
        message.velocity.resize(joints_.size());
        message.effort.resize(joints_.size());

        for (std::size_t index = 0; index < joints_.size(); ++index) {
            message.position[index] = joints_[index]->Position(0);
            message.velocity[index] = joints_[index]->GetVelocity(0);
            message.effort[index] = joints_[index]->GetForce(0);
        }

        publisher_.publish(message);
    }

private:
    std::unique_ptr<ros::NodeHandle> nh_;
    ros::Publisher publisher_;

    std::vector<std::string> joint_names_;
    std::vector<physics::JointPtr> joints_;

    event::ConnectionPtr update_connection_;

    common::Time last_publish_time_;
    double update_period_{0.02};
};

GZ_REGISTER_MODEL_PLUGIN(JointStatePublisher)

}  // namespace gazebo