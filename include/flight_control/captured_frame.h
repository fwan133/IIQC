//
// Created by feng on 2/07/23.
//

#ifndef DATA_QUALITY_CHECK_CAPTURED_FRAME_H
#define DATA_QUALITY_CHECK_CAPTURED_FRAME_H

#include <opencv2/opencv.hpp>

#include "viewpoint.h"
#include "waypoint.h"
#include "pose6d.h"


class CapturedFrame {
public:
    CapturedFrame(const WayPoint &waypoint, const double &TimeStamp) {
        id = waypoint.GetId();
        semantic_label = waypoint.GetSemanticLabel();
        time_stamp = TimeStamp;
    };

    ~CapturedFrame() {};

    inline const int GetId () const {return id;}

    inline const int GetTimeStamp() const {return time_stamp;};

    inline const std::string GetSemanticLabel() const {return semantic_label;};

    inline const ViewPoint GetGeoCoordinate() const {return geo_info_from_sensor;};

    inline const Pose6d GetGroundTruthPose() const {return pose_ground_truth;};

    inline const Pose6d GetEstimatedPose() const {return pose_estimated;};

    void SetGeoInfo(CoordinateFrame coordinate_frame, double latitude, double longitude, double altitude, AngleUnit angle_unit, double heading, double pitch) {
        geo_info_from_sensor.UpdateAll(coordinate_frame,latitude,longitude,altitude,angle_unit,heading,pitch,0.0);
    };

    void SetGeoInfo(ViewPoint &viewpoint) {
        geo_info_from_sensor.UpdateAll(viewpoint);
    };

    void SetGroudTruthPose(const Pose6d &pose){
        pose_ground_truth=Pose6d(pose);
    }

    void SetImgPath(const std::string &img_path){CapturedFrame::colour_imag_path = img_path;}

    std::string GetImgPath()const{return colour_imag_path;}

    void SetColourImg(const cv::Mat &ColourImg){colour_img=ColourImg;};

    void SetDepthImg(const cv::Mat &DepthImg){depth_img=DepthImg;};

    void UpdateEstimatedPose(const Pose6d &pose6D){
        pose_estimated=pose6D;
    };

    void SetEstPose(Eigen::Isometry3d T_est){
        pose_estimated = Pose6d(T_est);
    }

    void SetEstPose(const Pose6d &pose){
        pose_estimated = pose;
    }

    inline const cv::Mat GetColourImage() const { return colour_img; };

    inline const cv::Mat GetDepthImage() const { return depth_img; };

    void CheckQuality() {};

protected:
    uint id;
    std::string semantic_label;
    double time_stamp;
    ViewPoint geo_info_from_sensor;
    Pose6d pose_ground_truth;
    Pose6d pose_estimated;
    cv::Mat colour_img;
    cv::Mat depth_img;
    std::string colour_imag_path;
};

#endif //DATA_QUALITY_CHECK_CAPTURED_FRAME_H
