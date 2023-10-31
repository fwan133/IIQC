//
// Created by feng on 22/09/23.
//

#ifndef IIQC_COORDINATECONVERTER_H
#define IIQC_COORDINATECONVERTER_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "LocalCartesian.hpp"
#include "flight_control/viewpoint.h"

class CoordinateConverter {
public:
    CoordinateConverter(double lat0, double lon0, double alt0) {
        localCartesian.Reset(lat0, lon0, alt0);
        T_ENU_NED.linear() <<
                           0, 1, 0,
                1, 0, 0,
                0, 0, -1;

        // Invert the Down component to Up
        T_ENU_NED.translation() << 0, 0, 0;
    }

    const Eigen::Isometry3d obtainTransformMatFromVector(const double &x, const double &y, const double &z, const double &yaw,
                                                   const double &pitch, const double &roll) {
        Eigen::Isometry3d TransMat = Eigen::Isometry3d::Identity();
        TransMat.translation() = Eigen::Vector3d(x, y, z);
        TransMat.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
        return TransMat;
    }

    const Eigen::Isometry3d getPoseFromViewpoint(const ViewPoint &viewpoint) {
        Eigen::Isometry3d TransMat = Eigen::Isometry3d::Identity();
        TransMat.translation() = Eigen::Vector3d(viewpoint.x(), viewpoint.y(), viewpoint.z());
        TransMat.linear() = (Eigen::AngleAxisd(viewpoint.yaw(), Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(viewpoint.pitch(), Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(viewpoint.roll(), Eigen::Vector3d::UnitX())).toRotationMatrix();
        return TransMat;
    }

    const Eigen::Vector3d getOriPosition(const ViewPoint &viewpoint) {
        return Eigen::Vector3d(viewpoint.x(), viewpoint.y(), viewpoint.z());
    }

    const Eigen::Vector3d getOriOrientationInRad(const ViewPoint &viewpoint) {
        if (viewpoint.GetCoordinateFrame() == CoordinateFrame::GPS) {
            double yaw, pitch, roll;
            if (viewpoint.GetAngleUnit() == AngleUnit::DEGREE) {
                yaw = viewpoint.yaw() * COEF_DEG2RAD;
                pitch = viewpoint.pitch() * COEF_DEG2RAD;
                roll = viewpoint.roll() * COEF_DEG2RAD;
            } else {
                yaw = viewpoint.yaw();
                pitch = viewpoint.pitch();
                roll = viewpoint.roll();
            }
            return Eigen::Vector3d(yaw, pitch, roll);
        }else{
            return Eigen::Vector3d(0,0,0);
        }
    }

    Eigen::Isometry3d retrieveTransMatFromViewPoint(const ViewPoint &viewpoint) {
        if (viewpoint.GetCoordinateFrame() != CoordinateFrame::GPS) {
            double yaw, pitch, roll;
            if (viewpoint.GetAngleUnit() == AngleUnit::DEGREE) {
                yaw = viewpoint.yaw() * COEF_DEG2RAD;
                pitch = viewpoint.pitch() * COEF_DEG2RAD;
                roll = viewpoint.roll() * COEF_DEG2RAD;
            } else {
                yaw = viewpoint.yaw();
                pitch = viewpoint.pitch();
                roll = viewpoint.roll();
            }
            return obtainTransformMatFromVector(viewpoint.x(), viewpoint.y(), viewpoint.z(), yaw, pitch, roll);
        }
        return Eigen::Isometry3d::Identity();
    }

    const Eigen::Vector3d retrievePositionFromTransMat(const Eigen::Isometry3d &T) {
        return T.translation();
    }

    const Eigen::Vector3d retrieveOrientationFromTransMat(const Eigen::Isometry3d &T) {
        Eigen::Quaterniond quaternion(T.linear());
        double yaw, pitch, roll;
        yaw = atan2(2.0 * (quaternion.x() * quaternion.y() + quaternion.w() * quaternion.z()),
                    quaternion.w() * quaternion.w() + quaternion.x() * quaternion.x() -
                    quaternion.y() * quaternion.y() - quaternion.z() * quaternion.z());
        pitch = -asin(2.0 * (quaternion.x() * quaternion.z() - quaternion.w() * quaternion.y()));
        roll = atan2(2.0 * (quaternion.w() * quaternion.x() + quaternion.y() * quaternion.z()),
                     quaternion.w() * quaternion.w() - quaternion.x() * quaternion.x() -
                     quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z());
        return Eigen::Vector3d(yaw, pitch, roll);
    }

    void SetLocal2ENU(double x, double y, double z, double yaw, double pitch, double roll, AngleUnit angle_unit) {
        if (angle_unit == AngleUnit::DEGREE) {
            yaw = yaw * COEF_DEG2RAD;
            pitch = pitch * COEF_DEG2RAD;
            roll = roll * COEF_DEG2RAD;
        }
        T_ENU_LOCAL = obtainTransformMatFromVector(x, y, z, yaw, pitch, roll);
    }

    void SetBridge2ENU(double x, double y, double z, double yaw, double pitch, double roll, AngleUnit angle_unit) {
        if (angle_unit == AngleUnit::DEGREE) {
            yaw = yaw * COEF_DEG2RAD;
            pitch = pitch * COEF_DEG2RAD;
            roll = roll * COEF_DEG2RAD;
        }
        T_ENU_BRIDGE = obtainTransformMatFromVector(x, y, z, yaw, pitch, roll);
    }

    void ConvertBRIDGE2ENU(const ViewPoint &viewpoint_org, ViewPoint &viewpoint_new) {
        Eigen::Isometry3d P_bridge = retrieveTransMatFromViewPoint(viewpoint_org);
        Eigen::Isometry3d P_enu = T_ENU_BRIDGE * P_bridge;
        Eigen::Vector3d Position_enu = retrievePositionFromTransMat(P_enu);
        Eigen::Vector3d Orientation_enu = retrieveOrientationFromTransMat(P_enu);
        viewpoint_new.UpdateAll(CoordinateFrame::ENU, Position_enu, Orientation_enu, AngleUnit::RADIAN);
    }

    void ConvertBRIDGE2LOCAL(const ViewPoint &viewpoint_org, ViewPoint &viewpoint_new) {
        Eigen::Isometry3d P_bridge = retrieveTransMatFromViewPoint(viewpoint_org);
        Eigen::Isometry3d P_local = T_ENU_LOCAL.inverse() * T_ENU_BRIDGE * P_bridge;
        Eigen::Vector3d Position_local = retrievePositionFromTransMat(P_local);
        Eigen::Vector3d Orientation_local = retrieveOrientationFromTransMat(P_local);
        viewpoint_new.UpdateAll(CoordinateFrame::LOCAL, Position_local, Orientation_local, AngleUnit::RADIAN);
    }

    void ConvertBRIDGE2GPS(const ViewPoint &viewpoint_org, ViewPoint &viewpoint_new) {
        if (viewpoint_org.GetCoordinateFrame() == CoordinateFrame::BRIDGE) {
            Eigen::Isometry3d P_bridge = retrieveTransMatFromViewPoint(viewpoint_org);
            Eigen::Isometry3d P_enu = T_ENU_BRIDGE * P_bridge;
            Eigen::Isometry3d P_ned = T_ENU_NED.inverse() * P_enu;
            double Position_gps[3];
            localCartesian.Reverse(P_enu.translation().x(), P_enu.translation().y(), P_enu.translation().z(),
                                   Position_gps[0], Position_gps[1], Position_gps[2]);
            Eigen::Vector3d Orientation_gps = retrieveOrientationFromTransMat(P_ned);
            viewpoint_new.UpdateAll(CoordinateFrame::GPS,
                                    Eigen::Vector3d(Position_gps[0], Position_gps[1], Position_gps[2]), Orientation_gps,
                                    AngleUnit::RADIAN);
        } else {
            std::cout << "Error: Inaccurate reference coordinate frame!!!!" << std::endl;
        }
    }

    void ConvertNED2ENU(const ViewPoint &viewpoint_org, ViewPoint &viewpoint_new) {
        if (viewpoint_org.GetCoordinateFrame() == CoordinateFrame::NED) {
            Eigen::Isometry3d P_ned = retrieveTransMatFromViewPoint(viewpoint_org);
            Eigen::Isometry3d P_enu = T_ENU_NED * P_ned;
            Eigen::Vector3d Position_enu = retrievePositionFromTransMat(P_enu);
            Eigen::Vector3d Orientation_enu = retrieveOrientationFromTransMat(P_enu);
            viewpoint_new.UpdateAll(CoordinateFrame::ENU, Position_enu, Orientation_enu, AngleUnit::RADIAN);
        } else {
            std::cout << "Error: Inaccurate reference coordinate frame!!!!" << std::endl;
        }
    }

    void ConvertGPS2ENU(const ViewPoint &viewpoint_org, ViewPoint &viewpoint_new) {
        if (viewpoint_org.GetCoordinateFrame() == CoordinateFrame::GPS) {
            ViewPoint viewpoint_middle;
            // Calculate the XYZ in ENU
            double Position_enu[3];
            Eigen::Vector3d Position_gps = getOriPosition(viewpoint_org);
            localCartesian.Forward(Position_gps.x(), Position_gps.y(), Position_gps.z(), Position_enu[0],
                                   Position_enu[1], Position_enu[2]);
            double Orientation_enu[3];
            Eigen::Vector3d Orientation_ned = getOriOrientationInRad(viewpoint_org);
            Orientation_enu[0]=-Orientation_ned.x()+M_PI*2.5;
            Orientation_enu[1]=-Orientation_ned.y();
            Orientation_enu[2]=Orientation_ned.z();
            viewpoint_new.UpdateAll(CoordinateFrame::ENU, Position_enu[0], Position_enu[1], Position_enu[2], AngleUnit::RADIAN, Orientation_enu[0], Orientation_enu[1], Orientation_enu[2]);
        } else {
            std::cout << "Error: Inaccurate reference coordinate frame!!!!" << std::endl;
        }
    }


private:
    GeographicLib::LocalCartesian localCartesian;
    Eigen::Isometry3d T_ENU_LOCAL = Eigen::Isometry3d::Identity();    // Transform matrix from Local to ENU
    Eigen::Isometry3d T_ENU_BRIDGE = Eigen::Isometry3d::Identity();   // Transform matrix from UAVBoday to ENU
    Eigen::Isometry3d T_ENU_NED = Eigen::Isometry3d::Identity();    // Transform matrix from NED to ENU
};


#endif //IIQC_COORDINATECONVERTER_H
