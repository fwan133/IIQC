#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/MountControl.h>
#include <mavros_msgs/MountConfigure.h>
#include <gazebo_msgs/GetLinkState.h>
#include <std_msgs/Header.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "gimbal_control");
    ros::NodeHandle nh;

    ros::Publisher mountCnt = nh.advertise<mavros_msgs::MountControl>(
            "/typhoon_h480_0/mavros/mount_control/command", 1);

    ros::ServiceClient mountConfig = nh.serviceClient<mavros_msgs::MountConfigure>(
            "/typhoon_h480_0/mavros/mount_control/configure");

    ros::Rate rate(90);
    double gimbal_pitch = 90;
    double gimbal_yaw = 0.0;
    double gimbal_roll = 0.0;

    std_msgs::Header srvheader;
    srvheader.stamp = ros::Time::now();
    srvheader.frame_id = "map";

    geometry_msgs::PoseStamped cam_pose;
    ros::ServiceClient gazeboLinkstate = nh.serviceClient<gazebo_msgs::GetLinkState>(
            "gazebo/get_link_state");

    mavros_msgs::MountConfigure mount_configure_srv;
    mount_configure_srv.request.header = srvheader;
    mount_configure_srv.request.mode = 2;
    mount_configure_srv.request.stabilize_roll = 0;
    mount_configure_srv.request.stabilize_yaw = 0;
    mount_configure_srv.request.stabilize_pitch = 0;

    if (mountConfig.call(mount_configure_srv)) {
        ROS_INFO("Gimbal control");
    } else {
        ROS_ERROR("Failed to configure gimbal.");
        return 1;
    }

    while (ros::ok()) {
        mavros_msgs::MountControl msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.mode = 2;
        msg.pitch = gimbal_pitch;
        msg.roll = gimbal_roll;
        msg.yaw = gimbal_yaw;

        mountCnt.publish(msg);

        gazebo_msgs::GetLinkState link_state_srv;
        link_state_srv.request.link_name = "typhoon_h480_0::cgo3_camera_link";
        link_state_srv.request.reference_frame = "world";
        if (gazeboLinkstate.call(link_state_srv)) {
            cam_pose.header.stamp = ros::Time::now();
            cam_pose.pose = link_state_srv.response.link_state.pose;
            ROS_INFO("%f, %f, %f, %f, %f, %f, %f", link_state_srv.response.link_state.pose.position.x,
                     link_state_srv.response.link_state.pose.position.y,
                     link_state_srv.response.link_state.pose.position.z,
                     link_state_srv.response.link_state.pose.orientation.x,
                     link_state_srv.response.link_state.pose.orientation.y,
                     link_state_srv.response.link_state.pose.orientation.z,
                     link_state_srv.response.link_state.pose.orientation.w);
        } else {
            ROS_ERROR("Gazebo model state service call failed");
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
