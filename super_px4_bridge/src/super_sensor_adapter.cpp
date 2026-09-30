#include <cmath>
#include <mutex>
#include <string>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class SuperSensorAdapter
{
public:
    SuperSensorAdapter()
        : nh_(),
          pnh_("~")
    {
        pnh_.param<std::string>(
            "cloud_topic",
            cloud_topic_,
            "/velodyne_points");

        pnh_.param<std::string>(
            "odom_topic",
            odom_topic_,
            "/mavros/local_position/odom");

        pnh_.param<std::string>(
            "registered_cloud_topic",
            registered_cloud_topic_,
            "/cloud_registered");

        pnh_.param<std::string>(
            "registered_odom_topic",
            registered_odom_topic_,
            "/lidar_slam/odom");

        pnh_.param<std::string>(
            "world_frame",
            world_frame_,
            "world");

        pnh_.param<std::string>(
            "base_frame",
            base_frame_,
            "base_link");

        pnh_.param("lidar_x", lidar_x_, 0.0);
        pnh_.param("lidar_y", lidar_y_, 0.0);
        pnh_.param("lidar_z", lidar_z_, 0.16);

        pnh_.param("lidar_roll", lidar_roll_, 0.0);
        pnh_.param("lidar_pitch", lidar_pitch_, 0.0);
        pnh_.param("lidar_yaw", lidar_yaw_, 0.0);

        pnh_.param("max_odom_age", max_odom_age_, 0.10);

        cloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>(
                registered_cloud_topic_, 2);

        odom_pub_ =
            nh_.advertise<nav_msgs::Odometry>(
                registered_odom_topic_, 20);

        odom_sub_ =
            nh_.subscribe(
                odom_topic_, 50,
                &SuperSensorAdapter::odomCallback, this);

        cloud_sub_ =
            nh_.subscribe(
                cloud_topic_, 5,
                &SuperSensorAdapter::cloudCallback, this);

        tf2::Quaternion q_base_lidar;
        q_base_lidar.setRPY(
            lidar_roll_,
            lidar_pitch_,
            lidar_yaw_);

        q_base_lidar.normalize();

        transform_base_lidar_.setOrigin(
            tf2::Vector3(lidar_x_, lidar_y_, lidar_z_));

        transform_base_lidar_.setRotation(q_base_lidar);

        ROS_INFO_STREAM(
            "super_sensor_adapter started:"
            << "\n  cloud input:  " << cloud_topic_
            << "\n  odom input:   " << odom_topic_
            << "\n  cloud output: " << registered_cloud_topic_
            << "\n  odom output:  " << registered_odom_topic_
            << "\n  lidar xyz:    "
            << lidar_x_ << ", "
            << lidar_y_ << ", "
            << lidar_z_);
    }

private:
    void odomCallback(
        const nav_msgs::OdometryConstPtr &msg)
    {
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            latest_odom_ = *msg;
            have_odom_ = true;
        }

        nav_msgs::Odometry output = *msg;

        // MAVROS local map is used as the SUPER world frame.
        output.header.frame_id = world_frame_;
        output.child_frame_id = base_frame_;

        odom_pub_.publish(output);
    }

    void cloudCallback(
        const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        nav_msgs::Odometry odom;

        {
            std::lock_guard<std::mutex> lock(odom_mutex_);

            if (!have_odom_) {
                ROS_WARN_THROTTLE(
                    1.0,
                    "Waiting for odometry before transforming cloud");
                return;
            }

            odom = latest_odom_;
        }

        const double age =
            std::fabs((msg->header.stamp -
                       odom.header.stamp).toSec());

        if (age > max_odom_age_) {
            ROS_WARN_THROTTLE(
                1.0,
                "Cloud/odom timestamp difference is %.3f s",
                age);
        }

        const auto &pose = odom.pose.pose;

        tf2::Quaternion q_world_base(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);

        q_world_base.normalize();

        tf2::Transform transform_world_base;
        transform_world_base.setOrigin(
            tf2::Vector3(
                pose.position.x,
                pose.position.y,
                pose.position.z));

        transform_world_base.setRotation(q_world_base);

        const tf2::Transform transform_world_lidar =
            transform_world_base * transform_base_lidar_;

        geometry_msgs::TransformStamped transform_msg;
        transform_msg.header.stamp = msg->header.stamp;
        transform_msg.header.frame_id = world_frame_;
        transform_msg.child_frame_id = msg->header.frame_id;
        transform_msg.transform =
            tf2::toMsg(transform_world_lidar);

        sensor_msgs::PointCloud2 cloud_world;

        try {
            tf2::doTransform(
                *msg,
                cloud_world,
                transform_msg);
        } catch (const tf2::TransformException &exception) {
            ROS_ERROR_THROTTLE(
                1.0,
                "Point cloud transform failed: %s",
                exception.what());
            return;
        }

        cloud_world.header.stamp = msg->header.stamp;
        cloud_world.header.frame_id = world_frame_;

        cloud_pub_.publish(cloud_world);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber cloud_sub_;
    ros::Subscriber odom_sub_;

    ros::Publisher cloud_pub_;
    ros::Publisher odom_pub_;

    std::mutex odom_mutex_;
    nav_msgs::Odometry latest_odom_;
    bool have_odom_{false};

    tf2::Transform transform_base_lidar_;

    std::string cloud_topic_;
    std::string odom_topic_;
    std::string registered_cloud_topic_;
    std::string registered_odom_topic_;
    std::string world_frame_;
    std::string base_frame_;

    double lidar_x_;
    double lidar_y_;
    double lidar_z_;
    double lidar_roll_;
    double lidar_pitch_;
    double lidar_yaw_;
    double max_odom_age_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "super_sensor_adapter");

    SuperSensorAdapter adapter;

    ros::spin();
    return 0;
}
