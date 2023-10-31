//
// Created by feng on 7/10/23.
//

#ifndef IIQC_FRAME_H
#define IIQC_FRAME_H

#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/xfeatures2d/nonfree.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/calib3d/calib3d.hpp>

#include "camera.h"
#include "SfM/mappoint.h"

namespace sfm {

// Frame contains everything about an image.
    class Frame {
    public:
        typedef std::shared_ptr<Frame> Ptr;
        enum class KeyPointType {
            SIFT, SURF, ORB
        };

        // ctor and factory function
        Frame(long id) : id(id) {}

        Frame(long id, const std::string &imageName, const cv::Mat &image) : id(id), imageName(imageName),
                                                                             image(image) {}

        virtual ~Frame() {}

        static Frame::Ptr createFrame(const std::string &imageName, const cv::Mat &image) {
            static int factory_id = 0;
            return Frame::Ptr(new Frame(factory_id++, imageName, image));
        };

        static Frame::Ptr createFrame(long id) {
            return Frame::Ptr(new Frame(id));
        }

        void detectKeypoint(KeyPointType type) {
            cv::Mat resizedImage = resizeImage();
            cv::Mat grayImg;
            cv::cvtColor(resizedImage, grayImg, cv::COLOR_BGRA2GRAY);


            std::vector<cv::KeyPoint> originalKeypoint;
            std::vector<cv::KeyPoint> selectedKeypoint;
            if (type == KeyPointType::SIFT) {
                cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
                sift->detect(grayImg, originalKeypoint);

                selectKeypoint(originalKeypoint, selectedKeypoint, maxNumFeatures);
                sift->compute(grayImg, selectedKeypoint, desp);
            } else if (type == KeyPointType::ORB) {
                cv::Ptr<cv::ORB> orb = cv::ORB::create();
                orb->detect(grayImg, originalKeypoint);

                selectKeypoint(originalKeypoint, selectedKeypoint, maxNumFeatures);
                orb->compute(grayImg, selectedKeypoint, desp);
            } else if (type == KeyPointType::SURF) {
                cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(400);
                surf->detect(grayImg, originalKeypoint);

                selectKeypoint(originalKeypoint, selectedKeypoint, maxNumFeatures);
                surf->compute(grayImg, selectedKeypoint, desp);
            } else {
                std::cerr << "Only SIFT, ORB and SURF are available" << std::endl;
            }

            restoreKeypoint(selectedKeypoint);

            // L1 root normalize descriptor
            for (size_t i = 0; i < desp.rows; ++i) {
                cv::Mat row = desp.rowRange(i, i + 1);
                const double norm_l1 = cv::norm(row, cv::NORM_L1);
                row /= norm_l1;
                cv::sqrt(row, row);
            }
        }

        void undistortKeypoint(const cv::Mat &K, const cv::Mat &distCoef) {
            if (distCoef.at<float>(0, 0) == 0.0) return;

            cv::Mat mat(kp.size(), 2, CV_32F);
            for (int i = 0; i < kp.size(); i++) {
                mat.at<float>(i, 0) = kp[i].pt.x;
                mat.at<float>(i, 1) = kp[i].pt.y;
            }

            mat = mat.reshape(2);
            cv::undistortPoints(mat, mat, K, distCoef, cv::Mat(), K);
            mat = mat.reshape(1);

            for (int i = 0; i < kp.size(); ++i) {
                kp[i].pt.x = mat.at<float>(i, 0);
                kp[i].pt.y = mat.at<float>(i, 1);
            }
        }

        void addNumRegister() { ++numRegister; }

        void visKeyPoints() const {
            cv::Mat outimg;
            cv::drawKeypoints(image, kp, outimg, cv::Scalar(0, 255, 0));
            cv::imshow(std::to_string(id), outimg);
            cv::waitKey(0);
            cv::destroyAllWindows();
        }

        void setMapPoint(int kptIdx, MapPoint::Ptr mapPoint) {
            mapPoints[kptIdx] = mapPoint;
        };

        bool hasMapPoint(int kptIdx) {
            if (mapPoints.find(kptIdx) != mapPoints.end() && mapPoints.at(kptIdx)->getValidity()) {
                return true;
            } else {
                return false;
            }
        }

        std::vector<long> getNeighborsId() const {
            std::unordered_set<long> neighborsId;
            for (auto &id_match: matches) {
                auto &match = id_match.second;
                for (auto frameId_kpId: match) {
                    long frameId = frameId_kpId.first;
                    neighborsId.insert(frameId);
                }
            }

            std::vector<long> neighbors(neighborsId.begin(), neighborsId.end());
            return move(neighbors);
        }

        // setter and getter
        void setMaxNumFeatures(int maxNumFeatures) { Frame::maxNumFeatures = maxNumFeatures; }

        void setMaxImageSize(int maxImageSize) { Frame::maxImageSize = maxImageSize; }

        const cv::Mat getImage() const { return image; }

        void setKp(const std::vector<cv::KeyPoint> &kp) { Frame::kp = kp; }

        const std::vector<cv::KeyPoint> &getKp() const { return kp; }

        const cv::Mat getDesp() const { return desp; }

        void setColor(const std::vector<cv::Vec3b> &color) { Frame::color = color; }

        const std::vector<cv::Vec3b> &getColor() const { return color; }

        long getId() const { return id; }

        const std::unordered_map<long, std::unordered_map<long, int>> &getMatches() const { return matches; }

        void clearMatches(){matches.clear();}

        int getNumRegister() const { return numRegister; }

        const Eigen::Isometry3d &getTwc_est() const { return Twc_est; }

        void setTwc_est(const Eigen::Isometry3d &twc_est) { Twc_est = twc_est; }

        const Eigen::Isometry3d &getTwc_gps() const { return Twc_gps; }

        void setTwc_gps(const Eigen::Isometry3d &twc_gps) { Twc_gps = twc_gps; }

        const Eigen::Isometry3d &getTwc_gt() const { return Twc_gt; }

        void setTwc_gt(const Eigen::Isometry3d &twc_gt) { Twc_gt = twc_gt; }

        void setRegistered(bool registered) { Frame::registered = registered; }

        bool isRegistered() const { return registered; }

        void setFixed(bool fixed) { Frame::fixed = fixed; }

        bool isFixed() const { return fixed; }

        const std::unordered_map<int, MapPoint::Ptr> getMapPoints() const { return mapPoints; }

        const std::unordered_map<int, MapPoint::Ptr> getValidMapPoints() {
            std::unordered_map<int, MapPoint::Ptr> valid_mappoints;
            for (auto &mappoint: mapPoints) {
                if (mappoint.second->getValidity()) {
                    valid_mappoints[mappoint.first] = mappoint.second;
                }
            }
            return valid_mappoints;
        }

        void setMapPoints(const std::unordered_map<int, MapPoint::Ptr> &mapPoints) { Frame::mapPoints = mapPoints; }

        void getAssociatedMapPoints(std::vector<cv::Vec2f> &keypoint, std::vector<cv::Vec3f> &mapPoint,
                                    std::vector<int> &kpIdx, std::vector<long> &mapPointId) {
            keypoint.clear();
            mapPoint.clear();
            kpIdx.clear();
            mapPointId.clear();

//        std::cout << "[Associated Mappoints]: " << mapPoints.size() << std::endl;
            int num = 0;
            for (auto &mappoint: mapPoints) {
//            std::cout << "No: " << num << std::endl;
                if (mappoint.second->getValidity()) {
                    num++;
                    kpIdx.push_back(mappoint.first);
                    keypoint.push_back(kp[mappoint.first].pt);
                    mapPoint.push_back(mappoint.second->getPos());
                    mapPointId.push_back(mappoint.second->getId());
                }
            }
        }

        friend void addMatch(Frame::Ptr frame1, Frame::Ptr frame2, const std::vector<cv::DMatch> &match) {
            for (auto &m: match) {
                int queryIdx = m.queryIdx;
                int trainIdx = m.trainIdx;
                frame1->matches[queryIdx][frame2->id] = trainIdx;
                frame2->matches[trainIdx][frame1->id] = queryIdx;
            }
        }

        friend void getMatch(Frame::Ptr frame1, Frame::Ptr frame2, std::vector<int> &kpIdx1, std::vector<int> &kpIdx2) {
            std::vector<int> keypointIdx1;
            std::vector<int> keypointIdx2;
            auto &id_matches = frame1->getMatches();
            int id2 = frame2->getId();
            for (auto &id_match: id_matches) {
                auto &match = id_match.second;
                if (match.find(id2) != match.end()) {
                    int idx1 = id_match.first;
                    int idx2 = match.at(id2);
                    keypointIdx1.push_back(idx1);
                    keypointIdx2.push_back(idx2);
                }
            }

            kpIdx1 = move(keypointIdx1);
            kpIdx2 = move(keypointIdx2);
        }

        friend bool isMatched(Frame::Ptr frame1, Frame::Ptr frame2) {
            auto &id_matches = frame1->getMatches();
            int id2 = frame2->getId();
            for (auto &id_match: id_matches) {
                auto &match = id_match.second;
                if (match.find(id2) != match.end()) {
                    return true;
                }
            }
            return false;
        }

    private:
        cv::Mat resizeImage() {
            if (maxImageSize < image.rows || maxImageSize < image.cols) {
                int width = image.cols;
                int height = image.rows;
                double scale = maxImageSize * 1.0 / std::max(width, height);

                int new_width = width * scale;
                int new_height = height * scale;

                scaleWidth = new_width * 1.0 / width;
                scaleHeight = new_height * 1.0 / height;

                cv::Mat resized;
                cv::resize(image, resized, cv::Size(new_width, new_height));
                return resized; //resized;
            } else {
                return image;
            }
        }

        void selectKeypoint(const std::vector<cv::KeyPoint> &src, std::vector<cv::KeyPoint> &dst, int maxFeatureSize) {
            if (maxFeatureSize > src.size()) {
                dst = src;
            } else {
                std::vector<std::pair<size_t, float>> scales;
                for (size_t i = 0; i < src.size(); ++i)
                    scales.emplace_back(i, src[i].size);

                std::partial_sort(scales.begin(), scales.begin() + maxFeatureSize,
                                  scales.end(),
                                  [](const std::pair<size_t, float> scale1, const std::pair<size_t, float> scale2) {
                                      return scale1.second > scale2.second;
                                  }
                );

                dst.reserve(maxFeatureSize);
                for (size_t i = 0; i < maxFeatureSize; ++i) {
                    dst.push_back(src[scales[i].first]);
                }
            }
        }

        void restoreKeypoint(const std::vector<cv::KeyPoint> &kpts) {
            const double inv_scale_x = 1.0 / scaleWidth;
            const double inv_scale_y = 1.0 / scaleHeight;
            const double inv_scale_xy = (inv_scale_x + inv_scale_y) / 2.0f;

            kp.resize(kpts.size());
            color.resize(kpts.size());

            for (size_t i = 0; i < kpts.size(); ++i) {
                kp[i].pt.x = kpts[i].pt.x * inv_scale_x;
                kp[i].pt.y = kpts[i].pt.y * inv_scale_y;
                kp[i].size = kpts[i].size * inv_scale_xy;
                kp[i].angle = kpts[i].angle;

                color[i] = image.at<cv::Vec3b>(kp[i].pt.y, kp[i].pt.x);
            }
        }

    private:
        // basic information
        long id;
        std::string imageName;

        // image information
        Eigen::Isometry3d Twc_est = Eigen::Isometry3d::Identity();           // Estimated pose from SfM
        Eigen::Isometry3d Twc_gps = Eigen::Isometry3d::Identity();           // Estimated pose from GPS
        Eigen::Isometry3d Twc_gt = Eigen::Isometry3d::Identity();            // Ground Truth pose from Gazebo

        cv::Mat image;
        cv::Mat desp;                                                       // descriptors
        std::vector<cv::KeyPoint> kp;                                       // keypoints
        std::vector<cv::Vec3b> color;                                       // keypoints color
        std::unordered_map<int, MapPoint::Ptr> mapPoints;                   // associated map points, nullptr if no association (use get())

        // match information
        // matches[idx] indicates which keypoints that keypoint idx matches on other pictures
        std::unordered_map<long, std::unordered_map<long, int>> matches;  // kptID1 : [frameID : kptID2]

        // feature extraction parameters
        int maxNumFeatures = 8000;
        int maxImageSize = 3200;
        double scaleWidth = 1.0;
        double scaleHeight = 1.0;

        // registration information
        int numRegister = 0;
        bool registered = false;
        bool fixed = false;    // If the frame is finally registered.
    };

}


#endif //IIQC_FRAME_H
