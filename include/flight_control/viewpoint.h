//
// Created by feng on 2/07/23.
//

#ifndef DATA_QUALITY_CHECK_VIEWPOINT_H
#define DATA_QUALITY_CHECK_VIEWPOINT_H

#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Utils.h"

class ViewPoint {
public:
    ViewPoint() {};

    ViewPoint(CoordinateFrame coordinate_frame, float x, float y, float z, AngleUnit angle_unit, float yaw, float pitch,
              float roll) {
        coordinate_frame_ = coordinate_frame;
        position[0] = x;
        position[1] = y;
        position[2] = z;
        angle_unit_ = angle_unit;
        orientation[0] = yaw;
        orientation[1] = pitch;
        orientation[2] = roll;
    }

    ~ViewPoint() {};

    inline const CoordinateFrame GetCoordinateFrame() const { return coordinate_frame_; }

    inline const AngleUnit GetAngleUnit() const { return angle_unit_; }

    inline const std::string GetStrCoordinateFrame() const {
        switch (coordinate_frame_) {
            case 0:
                return "GPS";
                break;
            case 1:
                return "ENU";
                break;
            case 2:
                return "END";
                break;
            case 3:
                return "BRIDGE";
                break;
            case 4:
                return "LOCAL";
                break;
            default:
                return "Error";
        }
    }

    inline const std::string GetStrAngleUnit() const {
        switch (angle_unit_) {
            case 0:
                return "RADIAN";
                break;
            case 1:
                return "DEGREE";
                break;
            default:
                return "Error";
        }
    }

    inline const float x() const { return position[0]; };

    inline const float y() const { return position[1]; };

    inline const float z() const { return position[2]; };

    inline const float latitude() const { return position[0]; };

    inline const float longitude() const { return position[1]; };

    inline const float altitude() const { return position[2]; };

    inline const float yaw() const { return orientation[0]; };

    inline const float pitch() const { return orientation[1]; };

    inline const float roll() const { return orientation[2]; };

    Eigen::Isometry3d getEigenPose() const {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        if (coordinate_frame_!=CoordinateFrame::GPS){
            Eigen::Vector3d pos(position);
            Eigen::Vector3d ori(orientation);
            if (angle_unit_==AngleUnit::DEGREE) ori*=COEF_DEG2RAD;

            // Create rotation matrices for each axis
            Eigen::Matrix3d rotation_z, rotation_y, rotation_x;
            rotation_z = Eigen::AngleAxisd(ori.x(), Eigen::Vector3d::UnitZ());
            rotation_y = Eigen::AngleAxisd(ori.y(), Eigen::Vector3d::UnitY());
            rotation_x = Eigen::AngleAxisd(ori.z(), Eigen::Vector3d::UnitX());
            Eigen::Matrix3d rotation_matrix = rotation_z * rotation_y * rotation_x;

            // Formulate the transformation matrix
            T.prerotate(rotation_matrix);
            T.pretranslate(pos);
        }
        return T;
    }

    void RAD2DEGREE() {
        if (angle_unit_ == 0) {  // RADIAN=0, DEGREE=1
            orientation[0] *= COEF_RAD2DEG;
            orientation[1] *= COEF_RAD2DEG;
            orientation[2] *= COEF_RAD2DEG;
            angle_unit_ = AngleUnit::DEGREE;
        }
    }

    void DEGREE2RAD() {
        if (angle_unit_ == 1) {  // RADIAN=0, DEGREE=1
            orientation[0] *= COEF_DEG2RAD;
            orientation[1] *= COEF_DEG2RAD;
            orientation[2] *= COEF_DEG2RAD;
            angle_unit_ = AngleUnit::RADIAN;
        };
    };

    void UpdateAll(const ViewPoint &viewpoint){
        coordinate_frame_=viewpoint.GetCoordinateFrame();
        position[0]=viewpoint.x();
        position[1]=viewpoint.y();
        position[2]=viewpoint.z();
        angle_unit_=viewpoint.GetAngleUnit();
        orientation[0]=viewpoint.yaw();
        orientation[1]=viewpoint.pitch();
        orientation[2]=viewpoint.roll();
    }

    void UpdateAll(const CoordinateFrame &coordinate_frame, const Eigen::Vector3d &pos, const Eigen::Vector3d &ori,
                   const AngleUnit &angle_unit) {
        coordinate_frame_ = coordinate_frame;
        position[0] = pos.x();
        position[1] = pos.y();
        position[2] = pos.z();
        angle_unit_ = angle_unit;
        orientation[0] = ori.x();
        orientation[1] = ori.y();
        orientation[2] = ori.z();
    }

    void
    UpdateAll(CoordinateFrame coordinate_frame, float x, float y, float z, AngleUnit angle_unit, float yaw, float pitch,
              float roll) {
        coordinate_frame_ = coordinate_frame;
        position[0] = x;
        position[1] = y;
        position[2] = z;
        angle_unit_ = angle_unit;
        orientation[0] = yaw;
        orientation[1] = pitch;
        orientation[2] = roll;
    }

    void printInfo() {
        std::cout << "CS: " << GetStrCoordinateFrame() << ": " << position[0] << ", " << position[1] << ", " << position[2]
                  << ". " << GetStrAngleUnit() << ": " << orientation[0] << " (yaw), " << orientation[1] << " (pitch), " << orientation[2] << " (roll)."<< std::endl;
    }

protected:
    CoordinateFrame coordinate_frame_;
    AngleUnit angle_unit_;
    double position[3];  // The order is x, y, z.
    double orientation[3]; // The order is yaw, pitch roll, same as zyx;
};


#endif //DATA_QUALITY_CHECK_VIEWPOINT_H
