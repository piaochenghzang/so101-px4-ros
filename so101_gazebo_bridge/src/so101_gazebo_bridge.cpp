#include <array>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

class SO101GazeboBridge
{
public:
    SO101GazeboBridge()
    {
        const std::array<std::string, 6> joint_topics {
            "shoulder_pan/cmd_pos",
            "shoulder_lift/cmd_pos",
            "elbow_flex/cmd_pos",
            "wrist_flex/cmd_pos",
            "wrist_roll/cmd_pos",
            "gripper/cmd_pos"
        };

        for (size_t i = 0; i < joint_topics.size(); ++i) {
            joint_pubs_[i] = nh_.advertise<std_msgs::Float64>(joint_topics[i], 1);
        }

        command_sub_ = nh_.subscribe("joint_command", 1, &SO101GazeboBridge::commandCallback, this);

        ROS_INFO("SO101 Gazebo bridge started");
        ROS_INFO("Input: /so101/joint_command");
    }

private:
    void commandCallback(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        if (msg->data.size() != joint_pubs_.size()) {
            ROS_WARN_THROTTLE(1.0, "Expected 6 joint values, received %zu", msg->data.size());
            return;
        }

        for (size_t i = 0; i < msg->data.size(); ++i) {
            if (!std::isfinite(msg->data[i])) {
                ROS_WARN_THROTTLE(1.0, "Invalid joint command at index %zu", i);
                return;
            }
        }

        for (size_t i = 0; i < joint_pubs_.size(); ++i) {
            std_msgs::Float64 command;
            command.data = msg->data[i];
            joint_pubs_[i].publish(command);
        }
    }

private:
    ros::NodeHandle nh_{"/so101"};
    ros::Subscriber command_sub_;
    std::array<ros::Publisher, 6> joint_pubs_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "so101_gazebo_bridge");

    SO101GazeboBridge bridge;

    ros::spin();
    return 0;
}
