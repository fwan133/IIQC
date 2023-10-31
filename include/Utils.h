//
// Created by feng on 29/06/23.
//

#ifndef DATA_QUALITY_CHECK_UTILS_H
#define DATA_QUALITY_CHECK_UTILS_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.570796326794896619
#endif

enum CoordinateFrame {
    GPS, ENU, NED, BRIDGE, LOCAL
};

enum AngleUnit {
    RADIAN = 0, DEGREE = 1
};

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <g2o/types/sba/types_six_dof_expmap.h>

#include "SfM/map.h"

class Utils {
public:
    Utils() {}

    virtual ~Utils() {}

    static Eigen::Isometry3d matToIsometry(const cv::Mat &R, const cv::Mat &t) {
        Eigen::Matrix3d rotationMatrix;
        cv::cv2eigen(R, rotationMatrix);
        Eigen::Vector3d tvec;
        cv::cv2eigen(t, tvec);

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        Eigen::AngleAxisd rvec(rotationMatrix);
        T.rotate(rvec);
        T.pretranslate(tvec);

        return std::move(T);
    };

    static cv::Mat IsometryToMatR(const Eigen::Isometry3d T) {
        // Extract the rotation matrix
        Eigen::Matrix3d rotationMatrix = T.rotation();
        // Convert Eigen::Matrix to cv::Mat
        cv::Mat cvRotationMatrix(3, 3, CV_64F);
        cv::eigen2cv(rotationMatrix, cvRotationMatrix);
        return cvRotationMatrix;
    }

    static cv::Mat IsometryToMatq(const Eigen::Isometry3d T) {
        Eigen::Quaterniond q(T.rotation());
        cv::Mat mat(1, 4, CV_64F);

        mat.at<double>(0, 0) = q.x();
        mat.at<double>(0, 1) = q.y();
        mat.at<double>(0, 2) = q.z();
        mat.at<double>(0, 3) = q.w();

        return mat;
    }

    static cv::Mat IsometryToMatt(const Eigen::Isometry3d T) {
        // Extract the translation vector
        Eigen::Vector3d translationVector = T.translation();

        cv::Mat cvTranslationVector(1, 3, CV_64F);
        cvTranslationVector.at<double>(0, 0) = translationVector.x();
        cvTranslationVector.at<double>(0, 1) = translationVector.y();
        cvTranslationVector.at<double>(0, 2) = translationVector.z();

        return cvTranslationVector;
    }


    static Eigen::Vector3d toVector3d(const cv::Vec3d &vec) {
        return Eigen::Vector3d(vec(0), vec(1), vec(2));
    }

    static cv::Vec3d toCvMat(const Eigen::Vector3d &vec) {
        return cv::Vec3d(vec.x(), vec.y(), vec.z());
    }

    static Eigen::Vector2d toVector2d(const cv::Point2f &kp) {
        return Eigen::Vector2d(kp.x, kp.y);
    }

    static g2o::SE3Quat toSE3Quat(const Eigen::Isometry3d &T) {
        Eigen::Matrix3d R = T.rotation();
        Eigen::Vector3d t = T.translation();
        return g2o::SE3Quat(R, t);
    }

    static Eigen::Isometry3d toIsometry3d(const g2o::SE3Quat &SE3Quat) {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.rotate(SE3Quat.rotation());
        T.pretranslate(SE3Quat.translation());
        return T;
    }

    static double calReproErrFrame(sfm::Map::Ptr map, sfm::Frame::Ptr frame) {
        std::vector<cv::Vec2f> kp;
        std::vector<cv::Vec3f> mapPoint;
        std::vector<int> kpIdx;
        std::vector<long> mapPointId;
        frame->getAssociatedMapPoints(kp, mapPoint, kpIdx, mapPointId);

//        std::cout << "[Frame ID]" << frame->getId() << "Kp number" << mapPoint.size() << "Camera " << frame->getTwc_est().matrix() <<std::endl;

        std::vector<cv::Vec2f> projectedPoints;
        cv::projectPoints(mapPoint, Utils::IsometryToMatR(frame->getTwc_est().inverse()),
                          Utils::IsometryToMatt(frame->getTwc_est().inverse()), sfm::Camera::instance()->K(),
                          sfm::Camera::instance()->distCoef(), projectedPoints);

        // Calculate the reprojection error
        double totalError = 0.0;
        for (size_t i = 0; i < projectedPoints.size(); ++i) {
            double error = cv::norm(kp[i] - projectedPoints[i]);
            totalError += error;
        }

        // Calculate the mean reprojection error
        double meanError = totalError / kp.size();

//        std::cout << "[ReproError]: For Frame " << meanError << "The calculated points are " << kp.size() << "\n";

        return meanError;
    }

    static double calReproErrorPoints(std::vector<cv::Vec2f> points2D, std::vector<cv::Vec3f> points3D,
                                      Eigen::Isometry3d T_w_c, std::vector<bool> inlierMask) {
        std::vector<cv::Vec2f> projectedPoints;
        cv::projectPoints(points3D, Utils::IsometryToMatR(T_w_c.inverse()),
                          Utils::IsometryToMatt(T_w_c.inverse()), sfm::Camera::instance()->K(),
                          sfm::Camera::instance()->distCoef(), projectedPoints);

        int number_inlier = 0;
        double totalError = 0.0;
        for (int i = 0; i < inlierMask.size(); ++i) {
            if (!inlierMask[i]) continue;
            number_inlier++;
            double error = cv::norm(points2D[i] - projectedPoints[i]);
            totalError += error;
        }

        double reproError = totalError / number_inlier;

//        std::cout << "[ReproError]: Post PnP " << reproError << "The calculated points are " << number_inlier << "\n";

        return reproError;
    }

    static double calReproErrMappoint(sfm::Map::Ptr map, sfm::MapPoint::Ptr mappoint) {
        std::vector<long> frameIds;
        std::vector<int> kpIds;
        mappoint->getTrack(frameIds, kpIds);

        double totalError = 0.0;
        for (int i = 0; i < frameIds.size(); i++) {
            cv::Point2f originalPoint, projectedPoint;
            projectedPoint = calProjectedPoint(map->getAllFrames().at(frameIds[i]), mappoint->getPos());
            originalPoint = map->getAllFrames().at(frameIds[i])->getKp().at(kpIds[i]).pt;
            double error = cv::norm(projectedPoint - originalPoint);
            totalError += error;
        }

        // Calculate the mean reprojection error
        double meanError = totalError / frameIds.size();
        return meanError;
    }

    static double calReproErrPoint(cv::Vec2f point2D, cv::Vec3f point3D, Eigen::Isometry3d T_w_c) {
        std::vector<cv::Point3f> objectPoints{point3D};
        std::vector<cv::Point2f> projectedPoints;

        cv::projectPoints(objectPoints, Utils::IsometryToMatR(T_w_c.inverse()),
                          Utils::IsometryToMatt(T_w_c.inverse()), sfm::Camera::instance()->K(),
                          sfm::Camera::instance()->distCoef(), projectedPoints);

        cv::Vec2f projectedPoint = projectedPoints[0];

        return cv::norm(point2D - projectedPoint);
    };

    static cv::Point2f calProjectedPoint(sfm::Frame::Ptr frame, cv::Vec3f point3d) {
        std::vector<cv::Point3f> objectPoints{point3d};
        std::vector<cv::Point2f> projectedPoints;

        cv::projectPoints(objectPoints, Utils::IsometryToMatR(frame->getTwc_est().inverse()),
                          Utils::IsometryToMatt(frame->getTwc_est().inverse()), sfm::Camera::instance()->K(),
                          sfm::Camera::instance()->distCoef(), projectedPoints);

        // Extract the projected 2D image point
        cv::Point2f projectedPoint = projectedPoints[0];
        return projectedPoint;
    }

    static bool checkDepthPositive(cv::Vec3f point3D, Eigen::Isometry3d T_w_c){
        Eigen::Vector3d point3_w(point3D[0], point3D[1], point3D[2]);
        Eigen::Vector3d point3_c = T_w_c.inverse()*point3_w;
        if (point3_c.z()>0){
            return true;
        }else{
            return false;
        }
    }
};


#ifndef COEF_DEG2RAD
#define COEF_DEG2RAD 0.01745329251994329575
#endif

#ifndef COEF_RAD2DEG
#define COEF_RAD2DEG 57.29577951308232087721
#endif

#endif //DATA_QUALITY_CHECK_UTILS_H
