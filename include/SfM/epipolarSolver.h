#ifndef IIQC_EPIPOLARSOLVER_H
#define IIQC_EPIPOLARSOLVER_H

#include <thread>

#include "common.h"
#include "Utils.h"
#include "SfM/frame.h"


namespace sfm {

    class EpipolarSolver {
    public:
        typedef std::shared_ptr<EpipolarSolver> Ptr;

        EpipolarSolver(){}

        EpipolarSolver(int maxIter) : maxIter(maxIter) {}

        bool
        retrieveMotion(sfm::Frame::Ptr &referenceFrame, sfm::Frame::Ptr &currentFrame, Eigen::Isometry3d &T_ref_current) {
            /// Get matched keypoints
            sfm::Frame::Ptr frame1 = referenceFrame;
            sfm::Frame::Ptr frame2 = currentFrame;
            std::vector<cv::Point2f> kp1, kp2;
            std::vector<int> kpIdx1, kpIdx2;
            getMatch(frame1, frame2, kpIdx1, kpIdx2);
            for(int i=0;i<kpIdx1.size();++i){
                kp1.push_back(frame1->getKp()[kpIdx1[i]].pt);
                kp2.push_back(frame2->getKp()[kpIdx2[i]].pt);
            }

            /// Find Homography and Fundamental
            std::vector<bool> matchesInliersH, matchesInliersF;
            float SH, SF;
            cv::Mat H, F;

            std::thread threadH(&EpipolarSolver::findHomography, this, std::ref(kp1), std::ref(kp2), std::ref(matchesInliersH), std::ref(SH), std::ref(H));
            std::thread threadF(&EpipolarSolver::findFundamental, this, std::ref(kp1), std::ref(kp2), std::ref(matchesInliersF), std::ref(SF), std::ref(F));

            // wait until both threads have finished
            threadH.join();
            threadF.join();

            // compute ratio of scores
            float RH = SH / SF;

            /// Try to reconstruct from homography or fundamental depending on the score and ratio

            if (SF > minNumInlier) {
                return ReconstructF(kp1, kp2, T_ref_current);
            } else {
                std::cerr << "[Epipolar]: No sufficient inliers with " << SF << std::endl;
                return false;
            }

            /**
            if (RH > 0.7 && SH > minNumInlier) {
                std::cout << "Try to retrieve motion from Homography\n";
                return ReconstructH(kp1, kp2, H, T_ref_current);
            } else if (SF > minNumInlier) {
                std::cout << "Try to retrieve motion from Fundamental\n";
                return ReconstructF(kp1, kp2, T_ref_current);
            } else {
                std::cerr << "No sufficient inliers\n";
                return false;
            }
            **/

        }

    private:
        void findHomography(std::vector<cv::Point2f>& kp1, std::vector<cv::Point2f> &kp2, std::vector<bool>& matchesInliers, float& score, cv::Mat &H21){
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

        void findFundamental(std::vector<cv::Point2f>& kp1, std::vector<cv::Point2f> &kp2, std::vector<bool>& matchesInliers, float& score, cv::Mat &F21){
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

        bool ReconstructH(std::vector<cv::Point2f> kp1, std::vector<cv::Point2f> kp2, const cv::Mat &H, Eigen::Isometry3d &T){
            cv::Mat K=Camera::instance()->K();
            std::vector<cv::Mat> Rs, ts;
            int solutions = cv::decomposeHomographyMat(H, K, Rs, ts, cv::noArray());

            // do chirality check
            for (int i = 0; i < solutions; i++) {
                cv::Mat R1 = cv::Mat::eye(3, 3, CV_64F);
                cv::Mat t1 = cv::Mat::zeros(3, 1, CV_64F);
                cv::Mat R2 = Rs[i];
                cv::Mat t2 = ts[i];

                if (hasPositiveDepth(kp1, kp2, R1, t1, R2, t2)) {
                    T = Utils::matToIsometry(R2, t2).inverse();
                    return true;
                }
            }
            return false;
        }

        bool ReconstructF(std::vector<cv::Point2f> kp1, std::vector<cv::Point2f> kp2,  Eigen::Isometry3d &T){
            cv::Mat K=Camera::instance()->K();
            cv::Mat E = cv::findEssentialMat(kp1, kp2, K, cv::RANSAC, confidence,fundamentalError);
            cv::Mat R2, t2;
            cv::recoverPose(E, kp1, kp2, K, R2, t2); // already done chirality check
            T = Utils::matToIsometry(R2, t2).inverse();
            return true;
        }

        bool hasPositiveDepth(std::vector<cv::Point2f> kp1, std::vector<cv::Point2f> kp2, const cv::Mat& R1, const cv::Mat& t1, const cv::Mat& R2, const cv::Mat& t2){
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

        // RANSAC parameters
        int maxIter = 200;
        double confidence = 0.99;
        int minNumInlier = 50;
        double homographyError = 4.0;  // maximum allowed reprojection error to treat a point pair as an inlier
        double fundamentalError = 4.0;
    };

}


#endif //IIQC_EPIPOLARSOLVER_H
