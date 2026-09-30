#include <string>

#include <ros/ros.h>

#include <mavros_msgs/PositionTarget.h>
#include <quadrotor_msgs/PositionCommand.h>


class SuperMavrosBridge
{
public:
    SuperMavrosBridge()
        : nh_(),
          pnh_("~")
    {
        pnh_.param<std::string>(
            "pos_cmd_topic",
            pos_cmd_topic_,
            "/planning/pos_cmd");

        pnh_.param<std::string>(
            "setpoint_topic",
            setpoint_topic_,
            "/super/setpoint_raw/local");

        setpoint_pub_ =
            nh_.advertise<mavros_msgs::PositionTarget>(
                setpoint_topic_, 20);

        pos_cmd_sub_ =
            nh_.subscribe(
                pos_cmd_topic_,
                50,
                &SuperMavrosBridge::positionCommandCallback,
                this);

        ROS_INFO_STREAM(
            "SUPER MAVROS bridge started:"
            << "\n  input:  " << pos_cmd_topic_
            << "\n  output: " << setpoint_topic_);
    }

private:
    void positionCommandCallback(
        const quadrotor_msgs::PositionCommandConstPtr &msg)
    {
        mavros_msgs::PositionTarget target;

        // Preserve planner time so the manager can reject stale queued commands.
        target.header.stamp = msg->header.stamp;

        target.coordinate_frame =
            mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        /*
         * 使用 position + velocity + yaw。
         * 暂时屏蔽 acceleration 和 yaw_rate。
         */
        target.type_mask =
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        target.position.x = msg->position.x;
        target.position.y = msg->position.y;
        target.position.z = msg->position.z;

        target.velocity.x = msg->velocity.x;
        target.velocity.y = msg->velocity.y;
        target.velocity.z = msg->velocity.z;

        target.acceleration_or_force.x = 0.0;
        target.acceleration_or_force.y = 0.0;
        target.acceleration_or_force.z = 0.0;

        target.yaw = msg->yaw;
        target.yaw_rate = 0.0;

        setpoint_pub_.publish(target);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber pos_cmd_sub_;
    ros::Publisher setpoint_pub_;

    std::string pos_cmd_topic_;
    std::string setpoint_topic_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "super_mavros_bridge");

    SuperMavrosBridge bridge;

    ros::spin();
    return 0;
}
