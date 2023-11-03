#ifndef IIQC_IQM_IMAGE_H
#define IIQC_IQM_IMAGE_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "pose6d.h"

#include "YoloInference.h"

class IQMImage {
    struct IntrinsicParas {
        double cx, cy, fx, fy;
    };

public:

    IQMImage(std::string img_path, int id){
        image_id = id;
        cv::Mat img = cv::imread(img_path);
        if(img.empty()) std::cerr << "Error: Unable to load image." << std::endl;
        color_image = img;
        cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);
        pose = Pose6d();
        blur_probability_map = cv::Mat(color_image.cols, color_image.rows, CV_32F);
        exposure_intensity_map = cv::Mat(color_image.cols, color_image.rows, CV_8UC1);
    }

    IQMImage(cv::Mat color_img, int id) {
        image_id = id;
        color_image = color_img;
        cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);
        pose = Pose6d();
        blur_probability_map = cv::Mat(color_image.cols, color_image.rows, CV_32F);
        exposure_intensity_map = cv::Mat(color_image.cols, color_image.rows, CV_8UC1);
    };

    ~IQMImage() {};

    bool checkValidity() {
        // Check pose
        if (!isPoseAssigned) {
            std::cout << "Error with " << image_id << ": the pose has not be assigned." << std::endl;
            return false;
        }
        // Check intrinsic parameters
        if (!isIntrinsicParasAssigned) {
            std::cout << "Error with " << image_id << ": the intrinsic parameters have not be assigned." << std::endl;
            return false;
        }
        // Check blur map
        if (!isBLurMapCalculated) {
            std::cout << "Error with " << image_id << ": the blur map has not be calculated." << std::endl;
            return false;
        }
        // Check exposure map
        if (!isExposureMapCalculated) {
            std::cout << "Error with " << image_id << ": the exposure intensity map has not be calculated."
                      << std::endl;
            return false;
        }
        return true;
    }

    void setIntrinsicParas(double cx, double cy, double fx, double fy) {
        intrinsic_paras.cx = cx;
        intrinsic_paras.cy = cy;
        intrinsic_paras.fx = fx;
        intrinsic_paras.fy = fy;
        isIntrinsicParasAssigned = true;
    };

    const IntrinsicParas getIntrinsicParas() const {
        if (isIntrinsicParasAssigned) { return intrinsic_paras; }
    }

    void setPose(const Pose6d pose_) {
        pose = pose_;
        isPoseAssigned = true;
    }

    void setPose(const float x, const float y, const float z, const float qx, const float qy, const float qz, const float qw){
        this->setPosition(x, y, z);
        this->setOrientation(qx, qy, qz, qw);
        isPoseAssigned = true;
    }

    void setPose(const float x, const float y, const float z, const float roll_degree, const float pitch_degree, const float yaw_degree){
        this->setPosition(x, y, z);
        this->setOrientation(roll_degree, pitch_degree, yaw_degree);
        isPoseAssigned = true;
    }

    inline const Pose6d getPose() const {
        return pose;
    }

    void setPosition(const float x, const float y, const float z) {
        pose.position.x() = x;
        pose.position.y() = y;
        pose.position.z() = z;
    }

    void setOrientation(const float qx, const float qy, const float qz, const float qw) {
        pose.orientation.x() = qx;
        pose.orientation.y() = qy;
        pose.orientation.z() = qz;
        pose.orientation.w() = qw;
    }

    void setOrientation(const float roll_degree, const float pitch_degree, const float yaw_degree){
        double pi=3.1415926;
        Eigen::Quaternion<float> q;
        q = Eigen::AngleAxisf(roll_degree/180*pi, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(pitch_degree/180*pi, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(yaw_degree/180*pi, Eigen::Vector3f::UnitZ());
        pose.orientation=Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
    }

    void extractROI(std::string model_path) {
        // classes.txt is a dummy placeholder
        mpInference = std::make_shared<Inference>(model_path, cv::Size(640, 640), "classes.txt", false);
        /// Load the model
        auto net = cv::dnn::readNet(model_path);

        /// Run yolo inference
        cv::Mat mask;
        std::vector<Detection> output = mpInference->runInference(color_image, mask);

        cv::Mat resized;
        cv::resize(mask, resized, cv::Size(1200, 800));

        cv::imshow("test yolo mask", resized);
        cv::waitKey(0);
        cv::destroyAllWindows();

        isROICalculated = true;
    }

    cv::Size imgSize() {
        return color_image.size();
    }

    void evaluateEIM(uint patch_size) {
        cv::boxFilter(gray_image, exposure_intensity_map, CV_8UC1, cv::Size(patch_size, patch_size));
        isExposureMapCalculated = true;
    }

    void evaluateBPM(uint patch_size) {
        // Calculate the laplacian
        cv::Mat laplacian;
        cv::Laplacian(gray_image, laplacian, CV_32F);

        // Obtain the global log
        cv::Scalar global_mean, global_stddev;
        cv::meanStdDev(color_image, global_mean, global_stddev);
        double global_blur_log = std::log10(global_stddev[1]);

        // Calculate the local std deviation of laplacian
        cv::Mat sqImg, meanImg, meanSqImg, varianceImg;
        cv::multiply(laplacian, laplacian, sqImg);
        // Mean of the squared image (E[X^2])
        cv::boxFilter(sqImg, meanSqImg, CV_32F, cv::Size(patch_size, patch_size));
        // Square of the mean image (E[X])^2
        cv::boxFilter(laplacian, meanImg, CV_32F, cv::Size(patch_size, patch_size));
        cv::multiply(meanImg, meanImg, meanImg);
        // Variance = E[X^2] - (E[X])^2
        varianceImg = meanSqImg - meanImg;
        // Take the square root to obtain standard deviation
        cv::Mat stdDevImg;
        cv::sqrt(varianceImg, stdDevImg);
        // Compute the logarithm (base 10)
        cv::Mat stdDevLogImg;
        cv::log(stdDevImg + 1e-6, stdDevLogImg);  // Small value added to avoid log(0)
        stdDevLogImg /= std::log(10.0);

        // Normalise the local map
        double local_max, local_min;
        cv::minMaxLoc(stdDevLogImg, &local_min, &local_max);

        if ((local_max - global_blur_log) > (global_blur_log - local_min)) {
            local_min = 2 * global_blur_log - local_max;
        } else {
            local_max = 2 * global_blur_log - local_min;
        }

        // Update the BPM
        blur_probability_map = (local_max - stdDevLogImg) / (local_max - local_min);
        isBLurMapCalculated = true;
    }


    Eigen::Vector3d backProjection(uint u, uint v, double depth = 1) {
        // Depth at the pixel location
        double &Z = depth;  // For instance, depth is 1 meter away

        // Back-project to 3D coordinates in Image Frame
        double X = (u - intrinsic_paras.cx) * Z / intrinsic_paras.fx;
        double Y = (v - intrinsic_paras.cy) * Z / intrinsic_paras.fy;

        // Transform to World Frame
        Eigen::Vector3d global_point = pose.cvt2TransformMatrix() * Eigen::Vector3d(X, Y, Z);

        return global_point;
    }

    const void showColorImage(cv::Size cv_size) const {
        cv::Mat resized;
        cv::resize(color_image,resized, cv_size);
        cv::imshow("Color Image", resized);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    const void showGrayImage(cv::Size cv_size) const {
        cv::Mat resized;
        cv::resize(gray_image,resized, cv_size);
        cv::imshow("Gray Image", resized);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    void showBPM(cv::Size cv_size) const {
        if (isBLurMapCalculated) {
            cv::Mat resized;
            cv::resize(blur_probability_map,resized, cv_size);
            cv::imshow("Blur Probability Image", resized);
            cv::waitKey(0);
            cv::destroyAllWindows();
        } else {
            std::cout << "Error: No valid blur map!" << std::endl;
        }

    }

    void showEIM(cv::Size cv_size) const {
        if (isExposureMapCalculated) {
            cv::Mat resized;
            cv::resize(exposure_intensity_map,resized, cv_size);
            cv::imshow("Exposure Intensity Image", resized);
            cv::waitKey(0);
            cv::destroyAllWindows();
        } else {
            std::cout << "Error: No valid exposure map!" << std::endl;
        }
    }

    float estimateSSD(float x, float y, float z) {
        // Transform to Image Frame
        Eigen::Vector3d point = pose.cvt2TransformMatrix().inverse() * Eigen::Vector3d(x, y, z);
        return point.z()*1000 / intrinsic_paras.fx;
    }

    inline const cv::Mat* getColorImg() const { return &color_image; }

    inline const cv::Mat* getBPM() const { return &blur_probability_map; }

    inline const cv::Mat* getEIM() const { return &exposure_intensity_map; }

    inline const int getImageID() const { return image_id; }

protected:
    int image_id;
    cv::Mat color_image;
    cv::Mat gray_image;
    Pose6d pose;
    std::vector<cv::Point> roi;
    cv::Mat blur_probability_map;
    cv::Mat exposure_intensity_map;
    IntrinsicParas intrinsic_paras;
    bool isIntrinsicParasAssigned = false;
    bool isPoseAssigned = false;
    bool isROICalculated = false;
    bool isBLurMapCalculated = false;
    bool isExposureMapCalculated = false;

    std::shared_ptr<Inference> mpInference = nullptr;
};


#endif //IIQC_IQM_IMAGE_H
