//
// Created by feng on 8/10/23.
//

#ifndef IIQC_PNPSOLVER_H
#define IIQC_PNPSOLVER_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <memory>
#include <opencv4/opencv2/opencv.hpp>

#include "camera.h"
#include "Utils.h"

namespace sfm {

    class PnPSolver {
    public:
        typedef std::shared_ptr<PnPSolver> Ptr;

        bool solve(const std::vector<cv::Vec2f> &points2D, const std::vector<cv::Vec3f> &points3D, Eigen::Isometry3d &T, std::vector<bool> &inlierMask){
            try{
                assert(points2D.size() == points3D.size());

                cv::Mat K=Camera::instance()->K();
                cv::Mat r, t, inliers;
                cv::solvePnPRansac(points3D, points2D, K, cv::Mat(), r, t, false, maxIter, projError, confidence, inliers, method);

                int numInliers=inliers.rows;
                if(numInliers<minNumInliers) return false;

                inlierMask.clear();
                inlierMask.resize(points2D.size(), false);
                for(int i = 0; i < inliers.rows; ++i) {
                    int idx = inliers.at<int>(i, 0);
                    inlierMask[idx] = true;
                }

                cv::Mat R_tmp;
                cv::Rodrigues(r, R_tmp);
                Eigen::Matrix3d R;
                cv2eigen(R_tmp, R);
                Eigen::Vector3d tvec;
                cv2eigen(t, tvec);

                T = Eigen::Isometry3d::Identity();
                Eigen::AngleAxisd rvec(R);
                T.rotate(rvec);
                T.pretranslate(tvec);

                T = T.inverse();
                return true;
            } catch (const std::exception& error) {
                return false;
            }
        }

    private:
        int minNumInliers = 10;

        // some RANSAC parameters
        int maxIter = 200;
        float projError = 4.0;
        double confidence = 0.99;

        cv::SolvePnPMethod method = cv::SOLVEPNP_EPNP;
    };

}




#endif //IIQC_PNPSOLVER_H
