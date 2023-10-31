//
// Created by feng on 3/07/23.
//

#ifndef DATA_QUALITY_CHECK_FLIGHTMISSIONMANAGER_H
#define DATA_QUALITY_CHECK_FLIGHTMISSIONMANAGER_H

#include <istream>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/MountControl.h>
#include <mavros_msgs/MountConfigure.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetLinkState.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "flight_control/waypoints_mission.h"
#include "pose6d.h"

class FlightMissionManager {
    struct GlobalPose {
        double latitude, longitude, altitude, heading, pitch, roll;
    };

public:
    class UAVState {
    public:
        UAVState() {};

        ~UAVState() {};

        const double timestamp() const { return time_stamp; };

        const std::string mode() const { return mode_; };

        const bool IsConnected() const { return is_connected; };

        const bool IsArmed() const { return is_armed; };

        const GlobalPose GetGlobalPose() const { return global_pose; };

        const Pose6d GetLocalPose() const { return local_pose; };

        const Pose6d GetGTPose() const { return gt_pose; };

        void UpdateMAVState(const mavros_msgs::State::ConstPtr &state) {
            time_stamp = state->header.stamp.toSec();
            is_connected = state->connected;
            mode_ = state->mode;
            is_armed = state->armed;
        }

        void UpdateGPSCoordinate(const sensor_msgs::NavSatFix::ConstPtr &global) {
            global_pose.latitude = global->latitude;
            global_pose.longitude = global->longitude;
        }

        void UpdateRelAlt(const std_msgs::Float64::ConstPtr &msg) {
            global_pose.altitude = msg->data;
        }

        void UpdateHeading(const std_msgs::Float64::ConstPtr &heading) {
            global_pose.heading = heading->data;
            global_pose.pitch = 0.0;
            global_pose.roll = 0.0;
        }

        void UpdateLocalPose(const geometry_msgs::PoseStamped::ConstPtr &local) {
            local_pose.setPosition(local->pose.position.x, local->pose.position.y, local->pose.position.z);
            local_pose.setOrientation(local->pose.orientation.w, local->pose.orientation.x, local->pose.orientation.y,
                                      local->pose.orientation.z);
        }

        void UpdateGTPose(const geometry_msgs::Pose &ground_truth_pose) {
            gt_pose.setPosition(ground_truth_pose.position.x, ground_truth_pose.position.y,
                                ground_truth_pose.position.z);
            gt_pose.setOrientation(ground_truth_pose.orientation.w, ground_truth_pose.orientation.x,
                                   ground_truth_pose.orientation.y, ground_truth_pose.orientation.z);
        }

    protected:
        double time_stamp;
        std::string mode_;
        bool is_armed = false;
        bool is_connected = false;
        FlightMissionManager::GlobalPose global_pose;
        Pose6d local_pose;
        Pose6d gt_pose;
    };

public:
    FlightMissionManager(std::string UAV_Name, std::string image_topic) {
        std::cout << "\nEstablishing the connection to: " << UAV_Name << std::endl;
        uav_name = UAV_Name;
        // UAV State Subscribers
        nh_status.setCallbackQueue(&uav_state_callback_queue);
        mav_state_sub = nh_status.subscribe(UAV_Name + "/mavros/state", 20, &FlightMissionManager::MAVStateCallback,
                                            this);
        global_pose_sub = nh_status.subscribe(UAV_Name + "/mavros/global_position/global", 20,
                                              &FlightMissionManager::GlobalPoseCallback, this);
        global_rel_alt_sub = nh_status.subscribe(UAV_Name + "/mavros/global_position/rel_alt", 20,
                                                 &FlightMissionManager::GlobalRelAltCallback, this);
        global_heading_sub = nh_status.subscribe(UAV_Name + "/mavros/global_position/compass_hdg", 20,
                                                 &FlightMissionManager::GlobalHeadingCallback, this);
        local_pose_sub = nh_status.subscribe(UAV_Name + "/mavros/local_position/pose", 20,
                                             &FlightMissionManager::LocalPoseCallback, this);
        ground_truth_sub = nh_status.subscribe("/gazebo/model_states", 20, &FlightMissionManager::GroundTruthCallback,
                                               this);
        colour_img_sub = nh_status.subscribe(image_topic, 20,
                                             &FlightMissionManager::ColourImgCallback, this);

        // New thread to continuously obtain the UAV status
        uav_state_update_thread = std::thread(&FlightMissionManager::UpdateUAVStateThread, this);

        // UAV mission subscribers
//        depth_img_sub = nh_mission.subscribe(UAV_Name + "/realsense/depth_camera/depth/image_raw", 10,
//                                             &FlightMissionManager::DepthImgCallback, this);
        // ROS Publisher
        setpoint_local_pub = nh_mission.advertise<mavros_msgs::PositionTarget>(UAV_Name + "/mavros/setpoint_raw/local",
                                                                               10);
        gimble_pub = nh_mission.advertise<mavros_msgs::MountControl>(UAV_Name + "/mavros/mount_control/command", 10);

        // ROS Client
        arming_client = nh_mission.serviceClient<mavros_msgs::CommandBool>(UAV_Name + "/mavros/cmd/arming");
        set_mode_client = nh_mission.serviceClient<mavros_msgs::SetMode>(UAV_Name + "/mavros/set_mode");
        mount_config_client = nh_mission.serviceClient<mavros_msgs::MountConfigure>(
                UAV_Name + "/mavros/mount_control/configure");
        gazebo_linkstate_client = nh_mission.serviceClient<gazebo_msgs::GetLinkState>("gazebo/get_link_state");

        // wait for FCU connection
        while (ros::ok() && !uav_state.IsConnected()) {
            ros::Duration(0.05).sleep();
        }
        std::cout << "Successfully Connected!" << std::endl;
    };

    ~FlightMissionManager() {
        if (uav_state_update_thread.joinable()) {
            uav_state_update_thread.join();
        }
    };

    void UpdateUAVStateThread() {
//        cv::namedWindow("UAV Video Stream", cv::WINDOW_NORMAL);

        while (ros::ok()) {
//            std::cout << "[Thread]: UAV State Update. The image status: " << img_stream.empty() << std::endl;

            // Execute callback_queue
            uav_state_callback_queue.callAvailable();

            // Show the image stream
//            if (!img_stream.empty()) {
//                cv::Mat resized_img;
//                cv::resize(img_stream, resized_img, cv::Size(1200, 800));
//
//                // Put text
//                std::stringstream local_pos_text;
//                local_pos_text << std::fixed << std::setprecision(3) << "UAV mode: " << uav_state.mode()
//                               << "; UAV_local_pose: x: " << uav_state.GetLocalPose().position.x() << "; y: "
//                               << uav_state.GetLocalPose().position.y() << "; z: "
//                               << uav_state.GetLocalPose().position.z() << ".";
//                cv::Point textPosition(50, 50);
//                cv::Scalar textColor(0, 0, 255);
//                int fontSize = 1;
//                int fontThickness = 2;
//                int fontFace = cv::FONT_HERSHEY_SIMPLEX;
//                cv::putText(resized_img, local_pos_text.str(), textPosition, fontFace, fontSize, textColor,
//                            fontThickness);
//
//                std::stringstream gt_pose_text;
//                cv::Point textPosition2(50, 100);
//                gt_pose_text << std::fixed << std::setprecision(3) << "UAV_gt_pose: x: "
//                             << uav_state.GetGTPose().position.x()
//                             << "; y: " << uav_state.GetGTPose().position.y() << "; z: "
//                             << uav_state.GetGTPose().position.z()
//                             << ".";
//                cv::putText(resized_img, gt_pose_text.str(), textPosition2, fontFace, fontSize, textColor,
//                            fontThickness);
//
//                cv::Point textPosition3(50, 150);
//                cv::putText(resized_img, std::to_string(update_times), textPosition3, fontFace, fontSize, textColor,
//                            fontThickness);
//                cv::imshow("UAV Video Stream", resized_img);
//            }

            ros::Duration(0.01).sleep(); // Update Rate 100 Hz.
        }
    };

    void MAVStateCallback(const mavros_msgs::State::ConstPtr &msg) {
//        std::cout << "[UAV State]: Basic State Received." << std::endl;
        uav_state.UpdateMAVState(msg);
    }

    void GlobalPoseCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
//        std::cout << "[UAV State]: Global Position Received." << std::endl;
        uav_state.UpdateGPSCoordinate(msg);
    }

    void GlobalRelAltCallback(const std_msgs::Float64::ConstPtr &msg) {
        uav_state.UpdateRelAlt(msg);
    }

    void GlobalHeadingCallback(const std_msgs::Float64::ConstPtr &msg) {
        uav_state.UpdateHeading(msg);
    };

    void LocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        uav_state.UpdateLocalPose(msg);
    }

    void GroundTruthCallback(const gazebo_msgs::ModelStatesConstPtr &msg) {
        int index;
        auto it = std::find(msg->name.begin(), msg->name.end(), uav_name);
        if (it != msg->name.end()) {
            // Calculate the index using std::distance
            index = std::distance(msg->name.begin(), it);
        } else {
            std::cout << "Ground truth not found." << std::endl;
            return;
        }
        uav_state.UpdateGTPose(msg->pose[index]);
    }

    void ColourImgCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // Convert the ROS image message to an OpenCV image
            img_stream = cv_bridge::toCvCopy(msg, "bgr8")->image;
//            std::cout << "[Image Stream State]: Success. Image stream received with size " << img_stream.at<cv::Vec3b>(10,10) << std::endl;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Error converting image: %s", e.what());
//            std::cout << "[Image Stream State]: Fail." << std::endl;
        }
        update_times++;
    }

    void DepthImgCallback(const sensor_msgs::ImageConstPtr &msg) {
        depth_img_ptr = msg;
    }

    void PrintState() const {
        std::cout << "\nThe status of the UAV at:" << uav_state.timestamp() << " [s]" << std::endl;
        std::cout << "Mode: " << this->uav_state.mode() << std::endl;
        std::cout << "Armed: " << (this->uav_state.IsArmed() ? "True" : "False") << std::endl;
        std::cout << std::fixed << std::setprecision(7) << "The global coordinate is: "
                  << this->uav_state.GetGlobalPose().latitude << ", " << this->uav_state.GetGlobalPose().longitude
                  << ", " << std::setprecision(3) << this->uav_state.GetGlobalPose().altitude << ", Heading: "
                  << this->uav_state.GetGlobalPose().heading << std::endl;
        std::cout << std::fixed << std::setprecision(3) << "The local coordinate is: "
                  << this->uav_state.GetLocalPose().position.x() << ", " << this->uav_state.GetLocalPose().position.y()
                  << ", "
                  << this->uav_state.GetLocalPose().position.z() << std::endl;
        std::cout << std::fixed << std::setprecision(3) << "The ground truth coordinate is: "
                  << this->uav_state.GetGTPose().position.x() << ", " << this->uav_state.GetGTPose().position.y()
                  << ", "
                  << this->uav_state.GetGTPose().position.z() << std::endl;
    }

    const CapturedFrameCollection ExecuteWaypointMission(const WaypointsMission &waypoints_mission, std::string link_name) {
        CapturedFrameCollection captured_frame_collection;

        // Apply for controlling gimble
        std_msgs::Header srvheader;
        srvheader.stamp = ros::Time::now();
        srvheader.frame_id = "map";
        mavros_msgs::MountConfigure mount_configure_srv;
        mount_configure_srv.request.header = srvheader;
        mount_configure_srv.request.mode = 2;
        mount_configure_srv.request.stabilize_roll = 0;
        mount_configure_srv.request.stabilize_yaw = 0;
        mount_configure_srv.request.stabilize_pitch = 0;

        if (mount_config_client.call(mount_configure_srv)) {
            ROS_INFO("Gimbal Control Enabled");
        } else {
            ROS_ERROR("Failed to configure gimbal.");
        }

        // Mission execution
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        int index = 0;
        double relative_distance = 2.0;
        ros::Time last_request = ros::Time::now();
        ros::Rate rate(30.0);

        // Preliminary pose publish
        mavros_msgs::PositionTarget pose;
        pose.type_mask = 0b100111111000;
        pose.coordinate_frame = 1;
        pose.position.x = uav_state.GetLocalPose().position.x();
        pose.position.y = uav_state.GetLocalPose().position.y();
        pose.position.z = uav_state.GetLocalPose().position.z();
        pose.yaw_rate = 1;
        pose.yaw = 0;
        for (int i = 100; ros::ok() && i > 0; --i) {
            setpoint_local_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }

        // Define the start pose
        Pose6d start_pose = uav_state.GetLocalPose();

        // Define the camera pose
        gazebo_msgs::GetLinkState link_state_srv;
        link_state_srv.request.link_name = uav_name + "::" + link_name;
        link_state_srv.request.reference_frame = "world";
        geometry_msgs::Pose camera_pose;

        // Define gimbal publish
        mavros_msgs::MountControl gimble_msg;
        gimble_msg.header.stamp = ros::Time::now();
        gimble_msg.header.frame_id = "map";
        gimble_msg.mode = 2;
        gimble_msg.pitch = 0;
        gimble_msg.roll = 0;
        gimble_msg.yaw = 0;

        while (ros::ok()) {
            // Execute the flight mission
            if (uav_state.mode() != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if (!uav_state.IsArmed() &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if (arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }

            // Preflight to 2m above the ground
            if (index == 0) {
                pose.header.stamp = ros::Time::now();
                pose.position.x = start_pose.position.x();
                pose.position.y = start_pose.position.y();
                pose.position.z = start_pose.position.z() + 2;
                if (abs(uav_state.GetLocalPose().position.z() - start_pose.position.z() - 2) < 0.2) {
                    index++;
                    ROS_INFO("The drone is flying to waypoint: %d ", index);
                    this->PrintState();
                }

            } else {  // Execute the mission
                relative_distance = sqrt(
                        pow(waypoints_mission.GetWaypointByIndex(index)->GetViewPoint().x() -
                            uav_state.GetLocalPose().position.x(), 2) +
                        pow(waypoints_mission.GetWaypointByIndex(index)->GetViewPoint().y() -
                            uav_state.GetLocalPose().position.y(), 2) +
                        pow(waypoints_mission.GetWaypointByIndex(index)->GetViewPoint().z() -
                            uav_state.GetLocalPose().position.z(), 2));

                pose.position.x = waypoints_mission.GetWaypointByIndex(index)->GetViewPoint().x();
                pose.position.y = waypoints_mission.GetWaypointByIndex(index)->GetViewPoint().y();
                pose.position.z = waypoints_mission.GetWaypointByIndex(index)->GetViewPoint().z();
                if (waypoints_mission.GetWaypointByIndex(index)->GetViewPoint().GetAngleUnit() == AngleUnit::RADIAN) {
                    pose.yaw = waypoints_mission.GetWaypointByIndex(index)->GetViewPoint().yaw();
                } else {
                    pose.yaw = waypoints_mission.GetWaypointByIndex(index)->GetViewPoint().yaw()*COEF_DEG2RAD;
                }


                // Update the gimble msg
                if (waypoints_mission.GetWaypointByIndex(index)->GetViewPoint().GetAngleUnit() == AngleUnit::RADIAN) {
                    gimble_msg.pitch = waypoints_mission.GetWaypointByIndex(index)->GetViewPoint().pitch() * COEF_RAD2DEG;
                    gimble_msg.roll = waypoints_mission.GetWaypointByIndex(index)->GetViewPoint().roll() * COEF_RAD2DEG;
                } else {
                    gimble_msg.pitch = waypoints_mission.GetWaypointByIndex(index)->GetViewPoint().pitch();
                    gimble_msg.roll = waypoints_mission.GetWaypointByIndex(index)->GetViewPoint().roll();
                }


                if (relative_distance < 0.2) {
                    // Execute image capture
                    std::cout << "Capturing image at waypoint:" << index << " at "
                              << uav_state.GetLocalPose().position.x() << ", "
                              << uav_state.GetLocalPose().position.y() << ", " << uav_state.GetLocalPose().position.z()
                              << ". Heading: "
                              << uav_state.GetGlobalPose().heading << std::endl;
                    CapturedFrame captured_frame(*waypoints_mission.GetWaypointByIndex(index), uav_state.timestamp());
                    captured_frame.SetGeoInfo(GPS, uav_state.GetGlobalPose().latitude,
                                              uav_state.GetGlobalPose().longitude, uav_state.GetGlobalPose().altitude,
                                              DEGREE, uav_state.GetGlobalPose().heading,
                                              gimble_msg.pitch);

                    // Obtain and Set up the GroundTruth
                    if (gazebo_linkstate_client.call(link_state_srv)) {
                        camera_pose = link_state_srv.response.link_state.pose;
                    } else {
                        ROS_ERROR("Gazebo model state service call failed");
                    }
                    captured_frame.SetGroudTruthPose(camera_pose);
                    captured_frame.SetColourImg(img_stream);
                    captured_frame_collection.AddFrame(captured_frame);

                    if (index != waypoints_mission.GetSize()) {
                        index++;
                        std::cout << "The drone is flying to waypoint: " << index << std::endl;
                    } else {
                        std::cout << "The drone has finished the flight mission" << std::endl;
                        break;
                    }
                }
            }

            pose.header.stamp = ros::Time::now();
//            std::cout << "[Local Pose Set]: " << pose.position << std::endl;
            setpoint_local_pub.publish(pose);
            gimble_pub.publish(gimble_msg);

            rate.sleep();
        }

        return captured_frame_collection;
    }

private:
    int update_times = 0;
    UAVState uav_state;
    std::string uav_name;
    cv::Mat img_stream;

    // UAV_Status_Subscriber
    std::thread uav_state_update_thread;
    ros::NodeHandle nh_status;
    ros::NodeHandle nh_mission;
    ros::CallbackQueue uav_state_callback_queue;
    ros::Subscriber mav_state_sub;
    ros::Subscriber global_pose_sub;
    ros::Subscriber global_rel_alt_sub;
    ros::Subscriber global_heading_sub;
    ros::Subscriber local_pose_sub;
    ros::Subscriber ground_truth_sub;

    ros::Subscriber colour_img_sub;
    ros::Subscriber depth_img_sub;
    ros::Publisher setpoint_local_pub;
    ros::Publisher gimble_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient mount_config_client;
    ros::ServiceClient gazebo_linkstate_client;

    sensor_msgs::ImageConstPtr depth_img_ptr;
};


#endif //DATA_QUALITY_CHECK_FLIGHTMISSIONMANAGER_H
