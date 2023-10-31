//
// Created by feng on 8/10/23.
//

#ifndef IIQC_MATCHER_H
#define IIQC_MATCHER_H

#include <vector>
#include <iostream>

#include <opencv4/opencv2/core/core.hpp>

#include "map.h"

namespace sfm {

// strategy class
    class Matcher {
    public:
        typedef std::shared_ptr<Matcher> Ptr;

        Matcher(){};
        virtual ~Matcher() {};

        Matcher &setRatioThresh(float ratioThresh) {
            Matcher::ratioThresh = ratioThresh;
            return *this;
        }

        Matcher &setDistanceThresh(float distanceThresh) {
            Matcher::distanceThresh = distanceThresh;
            return *this;
        }

        Matcher &setMinNumMatches(int minNumMatches) {
            Matcher::minNumMatches = minNumMatches;
            return *this;
        }

        Matcher &setCrossCheck(int crossCheck) {
            Matcher::crossCheck = crossCheck;
            return *this;
        }

        Matcher &setVisualize(bool visualize) {
            Matcher::visualize = visualize;
            return *this;
        }

        bool matchTwo(Frame::Ptr frame1, Frame::Ptr frame2){
            // Check if exit match
            if (isMatched(frame1,frame2)) return true;

            // Calculate the matches
            std::vector<cv::DMatch> matches;
            if(!getMatches(frame1, frame2, matches)){
                return false;
            };

            if (matches.size() < minNumMatches) {
                return false;
            }

            /// Add matches to the frame
            addMatch(frame1, frame2, matches);

            /// Print and Visualize
            std::cout << "[Match]: Success! Found " << std::setw(4) << matches.size()
                      << " features between Frame " + std::to_string(frame2->getId()) + " and " +
                         std::to_string(frame1->getId()) + "." << std::endl;


            if (visualize) {
                cv::Mat out_img;
                cv::drawMatches(frame1->getImage(), frame1->getKp(), frame2->getImage(), frame2->getKp(), matches, out_img, cv::Scalar(0, 0, 255),
                                cv::Scalar(0, 255, 0), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

                cv::Mat resized;
                cv::resize(out_img, resized, cv::Size(1600, 600));
                std::string window_name = "Matches between Frame " + std::to_string(frame2->getId()) + " and " + "Frame" + std::to_string(frame1->getId());
                cv::imshow(window_name, resized);
                cv::waitKey(0);
                cv::destroyAllWindows();
            };

            return true;
        }

        bool getMatches(Frame::Ptr frame1, Frame::Ptr frame2, std::vector<cv::DMatch> &finalMatches){
            // Calculate the matches
            cv::Mat desp1 = frame1->getDesp();
            cv::Mat desp2 = frame2->getDesp();

            std::vector<cv::DMatch> originalMatches;
            std::vector<cv::DMatch> filteredMatches;
            matchTwoImpl(desp1, desp2, originalMatches);

            if (originalMatches.size() < minNumMatches) {
                std::cout << "[Match]: Insufficient matches between " << frame1->getId() << " and " << frame2->getId() << ". Only " << originalMatches.size() << " matches are found." << std::endl;
                return false;
            }

            filterByMatchDistance(originalMatches, originalMatches, distanceThresh);

            std::vector<cv::KeyPoint> kp1 = frame1->getKp();
            std::vector<cv::KeyPoint> kp2 = frame2->getKp();

            filterByGeoConstraint(kp1, kp2, originalMatches, originalMatches);

            filteredMatches = originalMatches;

            finalMatches.clear();
            finalMatches = filteredMatches;

            return true;
        }

    private:
        void knnMatch(const cv::Mat &desp1, const cv::Mat &desp2, std::vector<cv::DMatch> &matches, float ratioThresh){
            cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
            std::vector<std::vector<cv::DMatch>> knn_matches;
            matcher->knnMatch(desp1, desp2, knn_matches, 2);

            for (size_t i = 0; i < knn_matches.size(); i++) {
                if (knn_matches[i][0].distance < ratioThresh * knn_matches[i][1].distance) {
                    matches.push_back(knn_matches[i][0]);
                }
            }
        }

        void filterByCrossCheck(const std::vector<cv::DMatch> &matches12, const std::vector<cv::DMatch> &matches21,
                                       std::vector<cv::DMatch> &matches){
            std::unordered_map<int, int> matchMap;
            for (size_t i = 0; i < matches21.size(); ++i) {
                int query_idx = matches21[i].queryIdx;
                int train_idx = matches21[i].trainIdx;
                matchMap[query_idx] = train_idx;
            }

            int good_matches = 0;
            for (size_t i = 0; i < matches12.size(); ++i) {

                int query_idx = matches12[i].queryIdx;
                int train_idx = matches12[i].trainIdx;

                if (matchMap[train_idx] == query_idx) {
                    good_matches += 1;
                    matches.push_back(matches12[i]);
                }
            }
        }

        void filterByMatchDistance(const std::vector<cv::DMatch> &matches, std::vector<cv::DMatch> &goodMatches,
                                          float maxDistance){
            std::vector<cv::DMatch> filteredMatches;
            for (size_t i = 0; i < matches.size(); ++i) {
                if (matches[i].distance > maxDistance)
                    continue;
                filteredMatches.push_back(matches[i]);
            }
            goodMatches = std::move(filteredMatches);
        }

        void filterByGeoConstraint(const std::vector<cv::KeyPoint> &kp1, const std::vector<cv::KeyPoint> &kp2,
                                          const std::vector<cv::DMatch> &matches, std::vector<cv::DMatch> &filteredMatches) {
            std::vector<cv::Point2f> pts1;
            std::vector<cv::Point2f> pts2;
            for (cv::DMatch match: matches) {
                int queryIdx = match.queryIdx;
                int trainIdx = match.trainIdx;
                pts1.push_back(kp1[queryIdx].pt);
                pts2.push_back(kp2[trainIdx].pt);
            }

            // note that the definition of inliers here is different from the definition in the cv::solvePnPRansac function
            cv::Mat inliers;
            cv::findFundamentalMat(pts1, pts2, inliers, cv::FM_RANSAC, 3, 0.99);

            std::vector<cv::DMatch> inlierMatches;
            for (int i = 0; i < inliers.rows; i++) {
                if (inliers.at<uchar>(i, 0) == 0) continue;
                inlierMatches.push_back(matches[i]);
            }
            filteredMatches = inlierMatches;
        }

        void matchTwoImpl(const cv::Mat &desp1, const cv::Mat &desp2, std::vector<cv::DMatch> &matches){
            if (crossCheck == 0) {
                knnMatch(desp1, desp2, matches, ratioThresh);
            } else if (crossCheck == 1) {
                std::vector<cv::DMatch> matches12;
                std::vector<cv::DMatch> matches21;

                knnMatch(desp1, desp2, matches12, ratioThresh);
                knnMatch(desp2, desp1, matches21, ratioThresh);

                filterByCrossCheck(matches12, matches21, matches);
            } else {
                std::cerr << "Cross check should be 0 or 1\n";
            }
        }

    private:
        float ratioThresh = 0.7;
        float distanceThresh = 0.7;
        int minNumMatches = 15;
        int crossCheck = 1;
        bool visualize = true;


    };
}


#endif //IIQC_MATCHER_H
