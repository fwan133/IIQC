//
// Created by feng on 8/10/23.
//

#ifndef IIQC_INITIALIZER_H
#define IIQC_INITIALIZER_H

#include <memory>
#include <vector>
#include <thread>

#include "SfM/frame.h"
#include "Utils.h"

namespace sfm {

    class Initializer {
    public:
        typedef std::shared_ptr<Initializer> Ptr;

        Initializer(int maxIter=200) : maxIter(maxIter) {}
        bool initialize(Frame::Ptr referenceFrame, Frame::Ptr currentFrame){
            // get matched keypoints
            frame1 = referenceFrame;
            frame2 = currentFrame;
            std::vector<int> kpIdx1,kpIdx2;
            getMatch(frame1, frame2, kpIdx1,kpIdx2);
            for(int i=0;i<kpIdx1.size();++i){
                kp1.push_back(frame1->getKp()[kpIdx1[i]].pt);
                kp2.push_back(frame2->getKp()[kpIdx2[i]].pt);
            }

            std::vector<bool> matchesInliersH, matchesInliersF;
            float SH, SF;
            cv::Mat H, F;

            std::thread threadH(&Initializer::findHomography, this, std::ref(matchesInliersH), std::ref(SH), std::ref(H));
            std::thread threadF(&Initializer::findFundamental, this, std::ref(matchesInliersF), std::ref(SF), std::ref(F));

            // wait until both threads have finished
            threadH.join();
            threadF.join();

            // compute ratio of scores
            float RH = SH / SF;

            // try to reconstruct from homography or fundamental depending on the score and ratio
            if (RH > 0.7 && SH > minNumInlier) {
                std::cout << "Calculate R t from Homography\n";
                return ReconstructH(H);
            } else if (SF > minNumInlier) {
                std::cout << "Calculate R t from Fundamental\n";
                return ReconstructF(F);
            } else {
                std::cerr << "No sufficient inliers\n";
                return false;
            }
        };

    private:
        void findHomography(std::vector<bool>& matchesInliers, float& score, cv::Mat &H21){
            cv::Mat inlierMask;
            H21 = cv::findHomography(kp1, kp2, cv::RANSAC, homographyError, inlierMask, maxIter, confidence);

            matchesInliers.resize(inlierMask.rows, false);
            score = 0;
            for (int i = 0; i < inlierMask.rows; ++i) {
                if (inlierMask.at<uchar>(i, 0) == 0) continue;

                matchesInliers[i] = true;
                ++score;
            }
        }

        void findFundamental(std::vector<bool>& matchesInliers, float& score, cv::Mat &F21){
            cv::Mat inlierMask;
            F21 = cv::findFundamentalMat(kp1, kp2, cv::FM_RANSAC, fundamentalError, confidence, inlierMask);

            matchesInliers.resize(inlierMask.rows, false);
            score = 0;
            for (int i = 0; i < inlierMask.rows; ++i) {
                if (inlierMask.at<uchar>(i, 0) == 0) continue;

                matchesInliers[i] = true;
                ++score;
            }
        }

        bool ReconstructH(const cv::Mat &H){
            cv::Mat K=Camera::instance()->K();
            std::vector<cv::Mat> Rs, ts;
            int solutions = cv::decomposeHomographyMat(H, K, Rs, ts, cv::noArray());

            // do chirality check
            for (int i = 0; i < solutions; i++) {
                cv::Mat R1 = cv::Mat::eye(3, 3, CV_64F);
                cv::Mat t1 = cv::Mat::zeros(3, 1, CV_64F);
                cv::Mat R2 = Rs[i];
                cv::Mat t2 = ts[i];

                if (hasPositiveDepth(R1, t1, R2, t2)) {
                    Eigen::Isometry3d Tcw = Utils::matToIsometry(R2, t2);
                    frame2->setTwc_est(Tcw.inverse());
                    break;
                }
            }

            return true;
        }

        bool ReconstructF(const cv::Mat &F){
            cv::Mat K=Camera::instance()->K();
            cv::Mat E = cv::findEssentialMat(kp1, kp2,K, cv::RANSAC, confidence,fundamentalError);
            cv::Mat R2, t2;
            cv::recoverPose(E, kp1, kp2, K, R2, t2); // already done chirality check

            Eigen::Isometry3d Tcw = Utils::matToIsometry(R2, t2);

            frame2->setTwc_est(Tcw.inverse());

            return true;
        }

        bool hasPositiveDepth(const cv::Mat& R1, const cv::Mat& t1, const cv::Mat& R2, const cv::Mat& t2){
            cv::Mat P1, P2;
            cv::Mat K=Camera::instance()->K();
            cv::hconcat(K * R1, K * t1, P1);
            cv::hconcat(K * R2, K * t2, P2);

            cv::Mat points4D;
            cv::triangulatePoints(P1, P2, kp1, kp2, points4D);

            for (int i = 0; i < points4D.rows; ++i) {
                float z = points4D.at<float>(4, i);
                if (z < 0) return false;
            }

            return true;
        }

    private:
        Frame::Ptr frame1;          // reference frame
        Frame::Ptr frame2;          // current frame
        std::vector<cv::Point2f> kp1;    // matched keypoints from Frame1
        std::vector<cv::Point2f> kp2;    // matched keypoints from Frame2

        // RANSAC parameters
        int maxIter=200;
        double confidence=0.99;
        int minNumInlier=100;
        double homographyError=4.0;  // maximum allowed reprojection error to treat a point pair as an inlier
        double fundamentalError=4.0;
    };

}



#endif //IIQC_INITIALIZER_H
