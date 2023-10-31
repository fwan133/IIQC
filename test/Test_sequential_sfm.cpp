#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "flight_control/captured_frame_collection.h"
#include "config.h"
#include "SfM/track.h"
#include "SfM/frame.h"
#include "SfM/mappoint.h"
#include "SfM/map.h"
#include "SfM/matcher.h"
#include "SfM/epipolarSolver.h"
#include "SfM/triangulator.h"
#include "SfM/pnpSolver.h"
#include "SfM/optimizer.h"
#include "Viewer/viewer.h"

void render(sfm::Map::Ptr map) {
    Viewer::Ptr viewer = Viewer::Ptr(new Viewer());
    viewer->drawMap(map);
}

int main(int argc, char **argv) {
    /// Load image data and define parameters
    CapturedFrameCollection frame_collection("Pier",
                                             "/home/feng/Code/catkin_ros/src/IIQC/data/CapturedData/ComplexGirderBridge/Pier_new2");
    Eigen::Isometry3d T_body_c = Eigen::Isometry3d::Identity();
    T_body_c.matrix().topLeftCorner<3, 3>() << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    /// SfM
    Config::instance()->setParameterFile("/home/feng/Code/catkin_ros/src/IIQC/config/config.yaml");

    int maxNumFeatures = Config::instance()->get<int>("max_num_features");
    int maxImageSize = Config::instance()->get<int>("max_image_size");

    float ratio_thresh = Config::instance()->get<float>("ratio_thresh");
    float distance_thresh = Config::instance()->get<float>("distance_thresh");
    int min_num_matches = Config::instance()->get<int>("min_num_matches");
    int cross_check = Config::instance()->get<int>("cross_check");

    /// Create a new Map to store the data
    sfm::Map::Ptr map = std::make_shared<sfm::Map>();

    /// Create a matcher to obtain matches
    int overlap = 2;
    std::shared_ptr<sfm::Matcher> matcher(new sfm::Sequential(2));
    matcher->setRatioThresh(ratio_thresh).setDistanceThresh(distance_thresh).setMinNumMatches(
            min_num_matches).setCrossCheck(cross_check).setVisualize(false);

    /// Create a epipolarSolver
    sfm::EpipolarSolver::Ptr epipolarSolver = sfm::EpipolarSolver::Ptr(new sfm::EpipolarSolver);

    /// Create a triangulator
    sfm::Triangulator::Ptr triangulator = sfm::Triangulator::Ptr(new sfm::Triangulator);

    /// Create a PnPsolver
    sfm::PnPSolver::Ptr pnpSolver = sfm::PnPSolver::Ptr(new sfm::PnPSolver);

    /// Create a optimiser
    std::shared_ptr<sfm::Optimizer> optimizer = sfm::Optimizer::Ptr(new sfm::Optimizer());

    /// Create a viewer
    std::thread renderThread(&render, std::ref(map));

    /// Add image frames
    for (int i = 1; i <= frame_collection.GetSize(); i++) {
        std::cout << std::endl;
        sfm::Frame::Ptr current_frame = sfm::Frame::createFrame(
                frame_collection.getFrameByIndex(i).GetSemanticLabel() + std::to_string(i),
                frame_collection.getFrameByIndex(i).GetColourImage());
        current_frame->setMaxImageSize(maxImageSize);
        current_frame->setMaxNumFeatures(maxNumFeatures);
        current_frame->detectKeypoint(sfm::Frame::KeyPointType::SIFT);
        map->addFrame(current_frame);
        std::cout << "[Frame]: Frame " << current_frame->getId() << " added." << std::endl;

        /// Feature matching between all potential candidates
        // Match between the current and previous frame
//        {
//            // Define local variables
//            long previous_id = current_frame->getId() - 1;
//            sfm::Frame::Ptr previous_frame;
//            Eigen::Isometry3d rough_T_w_current;
//            double d_sum = 0, d_avg = 0;
//            int num_available = 0;
//
//            // Match to the previous frames
//            if (map->getAllFrames().find(previous_id) != map->getAllFrames().end()) {
//                previous_frame = map->getAllFrames().at(previous_id);
//                std::vector<cv::DMatch> matches;
//                bool is_matched = matcher->matchTwo(map, previous_frame->getId(), current_frame->getId(), matches);
//                if (is_matched) {
//                    addMatch(previous_frame, current_frame, matches);
//                    std::cout << "[Match]: Success! Found " << std::setw(4) << matches.size()
//                              << " features between Frame " + std::to_string(current_frame->getId()) + " and " +
//                                 std::to_string(previous_id) + "." << std::endl;
//
//                    // Retrieve rough_T_w_current
//                    std::vector<long> potential_matched_id;
//                    Eigen::Isometry3d T_previous_current;
//
//                    bool is_motion_retrieved = epipolarSolver->retrieveMotion(previous_frame, current_frame,
//                                                                              T_previous_current);
//                    if (is_motion_retrieved && previous_frame->isRegistered()) {
//                        rough_T_w_current = previous_frame->getTwc_est() * T_previous_current;
//                    } else {
//                        std::cout
//                                << "[Epipolar] Fail. No R and t retrieved from fundamental matrix OR Not registered for the reference frame.\n";
//                        return false;
//                    }
//
//                } else {
//                    std::cout << "[Match]: Fail! No features found between Frame " +
//                                 std::to_string(current_frame->getId()) + " and " + std::to_string(previous_id) + "."
//                              << std::endl;
//                    return false;
//                }
//            }
//
////            std::cout << "[Rough Register]: The rough pose of the current frame is " << rough_T_w_current.matrix() << std::endl;
//
//            // Obtain the search range
//            for (auto &frame: map->getAllFrames()) {
//                long sequent_frame_id = frame.second->getId() + 1;
//                if (map->getAllFrames().find(sequent_frame_id) != map->getAllFrames().end() &&
//                    map->getAllFrames().at(sequent_frame_id)->isRegistered()) {
//                    sfm::Frame::Ptr first_frame = frame.second;
//                    sfm::Frame::Ptr second_frame = map->getAllFrames().at(sequent_frame_id);
//                    Eigen::Isometry3d T_1_2 = first_frame->getTwc_est().inverse() * second_frame->getTwc_est();
//                    d_sum += T_1_2.translation().norm();
//                    num_available++;
//                }
//            }
//            if (num_available != 0) {
//                d_avg = d_sum / num_available;
//            }
//
//            std::vector<long> potential_frame_ids;
//            for (auto &frame: map->getAllFrames()) {
//                if (frame.second->isRegistered()) {
//                    Eigen::Isometry3d relative_pose = frame.second->getTwc_est().inverse() * rough_T_w_current;
//                    Eigen::AngleAxisd relative_angle_axis(relative_pose.linear());
////                    std::cout << "[Angle]: " << relative_angle_axis.angle()*COEF_RAD2DEG << std::endl;
//                    if (relative_pose.translation().norm() <= 2.5 * d_avg &&
//                        relative_angle_axis.angle() * COEF_RAD2DEG < 100) {
//                        potential_frame_ids.push_back(frame.second->getId());
//                    }
//                }
//            }
//
//            // Match potential frames
//            int num_additional= 1;
//            for (auto &frame_id:potential_frame_ids){
//                if (frame_id != (current_frame->getId()-1)){
//                    sfm::Frame::Ptr reference_frame = map->getAllFrames().at(frame_id);
//                    std::vector<cv::DMatch> matches;
//                    bool is_matched = matcher->matchTwo(map, reference_frame->getId(), current_frame->getId(), matches);
//                    if (is_matched) {
//                        addMatch(reference_frame, current_frame, matches);
//                        std::cout << "[Match]: Success! Found " << std::setw(4) << matches.size()
//                                  << " features between Frame " + std::to_string(current_frame->getId()) + " and " +
//                                     std::to_string(reference_frame->getId()) + "." << std::endl;
//                        num_additional ++;
//                    }
//                }
//            }
//
//            std::cout << "[Multiple Match]: Found " << potential_frame_ids.size() << " potential candidates. The total number of matched frames are " << num_additional << "." << std::endl;
//        }

        /// Matching strategy 2

        for (int i = 1; i <= 2; i++) {
            long reference_frame_id = current_frame->getId()-i;
            if (map->getAllFrames().find(reference_frame_id) != map->getAllFrames().end()){
                sfm::Frame::Ptr reference_frame = map->getAllFrames().at(reference_frame_id);
                std::vector<cv::DMatch> matches;
                bool is_matched = matcher->matchTwo(map, reference_frame->getId(), current_frame->getId(), matches);
                if (is_matched) {
                    addMatch(reference_frame, current_frame, matches);
                    std::cout << "[Match]: Success! Found " << std::setw(4) << matches.size()
                              << " features between Frame " + std::to_string(current_frame->getId()) + " and " +
                                 std::to_string(reference_frame->getId()) + "." << std::endl;
                }
            }
        }

        /// Process the first frame
        if (current_frame->getId() == 0) {
            current_frame->setTwc_est(
                    frame_collection.getFrameByIndex(i).GetGroundTruthPose().cvt2TransformMatrix() * T_body_c);
            current_frame->setRegistered(true);
            std::cout << "[Register]: Success." << std::endl;
            continue;
        }

        /// Process the second frame ---- Initialise
        if (current_frame->getId() == 1) {
            // Retrieve relative motion
            sfm::Frame::Ptr first_frame = map->getAllFrames().at(0);
            Eigen::Isometry3d T_first_current;
            bool is_motion_retrieved = epipolarSolver->retrieveMotion(first_frame, current_frame, T_first_current);

            if (is_motion_retrieved && first_frame->isRegistered()) {
                current_frame->setTwc_est(first_frame->getTwc_est() * T_first_current);

//                current_frame->setTwc_est(frame_collection.getFrameByIndex(i).GetGroundTruthPose().cvt2TransformMatrix()*T_body_c);

                current_frame->setRegistered(true);
                std::cout << "[Epipolar]: Successed." << std::endl;
            } else {
                std::cout
                        << "[Epipolar] Fail. No R and t retrieved from fundamental matrix OR Not registered for the first frame."
                        << std::endl;
                return -1;
            }

            // Triangulate mappoint
            std::vector<cv::Point3f> points3d;
            triangulator->triangulateTwoFrames(first_frame, current_frame, points3d);
            std::vector<int> kpIdx1, kpIdx2;
            getMatch(first_frame, current_frame, kpIdx1, kpIdx2);

            for (int i = 0; i < points3d.size(); ++i) {
                sfm::Track track;
                track.addElement(first_frame->getId(), kpIdx1[i]);
                track.addElement(current_frame->getId(), kpIdx2[i]);
                map->insertMapPoint(points3d[i], track);
            }

            // BA for optimise the scale
            std::cout << "[Initialise]: Success. The number of initialised map points is " << points3d.size() << "."
                      << std::endl;
            optimizer->globalBA(map);
        }

        /// Incremental SfM --------- For the frames after first two
        if (current_frame->getId() >= 2) {
            // Registration
            std::vector<cv::Vec2f> kp;
            std::vector<cv::Vec3f> mapPoint;
            std::vector<int> kpIdx;
            std::vector<long> mapPointId;
            map->getMatchToMap(current_frame, kp, mapPoint, kpIdx, mapPointId);
            Eigen::Isometry3d T_w_c;
            std::vector<bool> inlierMask;
            bool isRegisteredByPnP = pnpSolver->solve(kp, mapPoint, T_w_c, inlierMask);
            if (isRegisteredByPnP) {
                std::cout << "[PnP] Succeeded! Frame " << current_frame->getId() << ". Find matched map points "
                          << inlierMask.size() << "." << std::endl;
                // update frame
                current_frame->setRegistered(true);
                current_frame->setTwc_est(T_w_c);

//                current_frame->setTwc_est(frame_collection.getFrameByIndex(i).GetGroundTruthPose().cvt2TransformMatrix()*T_body_c);

                // update track
                for (int i = 0; i < inlierMask.size(); ++i) {
                    if (!inlierMask[i]) continue;
                    map->addObservation(mapPointId[i], current_frame->getId(), kpIdx[i]);
                }
            } else {
                std::cout << "[PnP] Fail! Trying Epipolar." << std::endl;
                sfm::Frame::Ptr previous_frame = map->getAllFrames().at(current_frame->getId() - 1);
                Eigen::Isometry3d T_previous_current;
                bool is_motion_retrieved = epipolarSolver->retrieveMotion(previous_frame, current_frame,
                                                                          T_previous_current);
                if (is_motion_retrieved && previous_frame->isRegistered()) {
                    T_w_c = previous_frame->getTwc_est() * T_previous_current;
                    std::cout << "[Epipolar]: Success. " << std::endl;
                } else {
                    std::cout
                            << "[Epipolar] Fail. No R and t retrieved from fundamental matrix OR Not registered for the reference frame."
                            << std::endl;
                    return false;
                }
            }
            current_frame->setRegistered(true);
            current_frame->setTwc_est(T_w_c);
            if (!isRegisteredByPnP) optimizer->localBA(map, current_frame);
            std::cout << "[Register]: Success." << std::endl;

            /// Triangulate new points
            std::vector<std::vector<sfm::Map::CorrData>> data;
            map->getMatchToNeighbor(current_frame, data);

            int new_point_num = 0;
            for (auto &corrData: data) {   // for each point
                if (data.size() < 2) continue;

                sfm::Track track;
                std::vector<cv::Mat> Ps;
                std::vector<cv::Point2f> points2d;
                for (auto singleViewData: corrData) {    // for each view
                    track.addElement(singleViewData.frameId, singleViewData.kptIdx);
                    Ps.push_back(singleViewData.P);
                    points2d.push_back(singleViewData.kpt);
                }

                cv::Point3f pt3d;
                triangulator->triangulateMultiviews(points2d, Ps, pt3d);

                // update map
                map->insertMapPoint(pt3d, track);
                new_point_num++;
            }
            std::cout << "[Triangulate]: Success. The number of new triangulated points is " << new_point_num
                      << std::endl;

            if (i % 5 == 0) {
                optimizer->localBA(map, current_frame);
            }

        }

    }
    /// Global BA
    optimizer->globalBA(map);

    /// Visualise the results
    renderThread.join();
    return 0;
}