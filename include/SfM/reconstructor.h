#ifndef IIQC_SFM_RECONSTRUCTOR_H
#define IIQC_SFM_RECONSTRUCTOR_H

#include <Eigen/Geometry>

#include "config.h"

#include "SfM/frame.h"
#include "SfM/map.h"
#include "SfM/matcher.h"
#include "SfM/pnpSolver.h"
#include "SfM/epipolarSolver.h"
#include "SfM/triangulator.h"
#include "SfM/optimizer.h"
#include "ICP/icp_solver.h"


namespace sfm {
    class Reconstructor {
    public:
        typedef std::shared_ptr<Reconstructor> Ptr;

        Reconstructor() {
            map = Map::Ptr(new Map());
            matcher = Matcher::Ptr(new Matcher());
            epipolar_solver = EpipolarSolver::Ptr(new EpipolarSolver());
            pnp_solver = PnPSolver::Ptr(new PnPSolver());
            triangulator = Triangulator::Ptr(new Triangulator());
            optimizer = Optimizer::Ptr(new Optimizer());
            icp_solver = ICPSolver::Ptr(new ICPSolver());
        };

        ~Reconstructor() {}

        void setFeatureDetector(int max_image_size, int max_num_features, Frame::KeyPointType keypoint_type) {
            Reconstructor::max_image_size = max_image_size;
            Reconstructor::max_num_features = max_num_features;
            Reconstructor::keypoint_type = keypoint_type;
        }

        void
        setMatcher(int match_type, int sequential_overlap, int max_overlap, float ratio_thresh, float distance_thresh,
                   int min_num_matches, int cross_check, bool visualize) {
            matcher->setRatioThresh(ratio_thresh).setDistanceThresh(distance_thresh).setMinNumMatches(
                    min_num_matches).setCrossCheck(cross_check).setVisualize(visualize);
            Reconstructor::match_type = match_type;
            Reconstructor::sequential_overlap = sequential_overlap;
            Reconstructor::max_overlap = max_overlap;
        }

        void setICPSolver(std::string ground_truth_path, bool visualised) {
            icp_solver->setICPSolver(ground_truth_path, visualised);
        }

        Frame::Ptr getCurrentFrame() const { return current_frame; }

        Map::Ptr getMap() { return map; };

        void addNewImg(const std::string &img_name, Eigen::Isometry3d &T_gps) {
            /// Add new frame and do feature detection and matching
            cv::Mat inputImg = cv::imread(img_name);
            addImage(img_name, inputImg, T_gps);

            /// Register the coming image
            while (!registerAndTriangulate(T_gps)) {  // When the registration fails.
                current_frame->clearMatches();
                current_frame->setRegistered(false);
                current_frame->setFixed(false);

                /// Align to GPS
                align2GPS();

                /// Align to ground truth point cloud
                std::unordered_map<long, MapPoint::Ptr> unfixed_valid_mappoint = map->getUnfixedAndValidMapPoints();

                if (unfixed_valid_mappoint.size() ==
                    0) { // This means the new coming second images failed to match with the first frame in the new map
                    current_frame->setFixed(true);
                } else {
                    // Obtain source point cloud
                    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);;
                    for (auto &id_mappoint: unfixed_valid_mappoint) {
                        pcl::PointXYZ point;
                        point.x = id_mappoint.second->getPos()[0];
                        point.y = id_mappoint.second->getPos()[1];
                        point.z = id_mappoint.second->getPos()[2];
                        src_cloud->points.push_back(point);
                    }

                    // Calculate the Transformation Matrix
                    pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud;
//                    std::cout << "[ICP]: Registering unfixed points " << src_cloud->points.size() << std::endl;

                    align2GroundTruthCloud(src_cloud, registered_cloud);
                }
            }
        }

        void finalRefine() {
            align2GPS();
            align2Cloud();
        }

        bool registerAndTriangulate(Eigen::Isometry3d &T_gps) {
            int num_unfixed_frames = map->getUnfixedFrames().size();
//            std::cout << "[Unfixed Frame]: The number of unfixed frame " << num_unfixed_frames << std::endl;
            if (num_unfixed_frames == 1) {   // First frame for a new map block
                current_frame->setTwc_est(T_gps);
                current_frame->clearMatches();
                current_frame->setRegistered(true);
                return true;
            } else if (num_unfixed_frames == 2) {
                return initialise(true);
            } else {
                if (registration()) {
                    triangulate();
                    if (num_unfixed_frames % 2 == 0) localOptimisation();
                    if (num_unfixed_frames % 5 == 0) globalOptimisation();
                    return true;
                } else {}
                return false;
            }
        }

        bool addImage(const std::string &img_name, const cv::Mat &img, const Eigen::Isometry3d T_gps) {
            if (img.empty()) {
                std::cout << "[Error]: The added image is empty.\n";
                return false;
            } else {
                Frame::Ptr frame = sfm::Frame::createFrame(img_name, img);
                frame->setMaxImageSize(max_image_size);
                frame->setMaxNumFeatures(max_num_features);
                frame->detectKeypoint(keypoint_type);
                frame->setTwc_gps(T_gps);
                map->addFrame(frame);
                std::cout << "\n[Frame]: Frame " << frame->getId() << " added." << std::endl;
                current_frame = frame;
            }

            // Add matches to the previous frame
            matchFrames();

            return true;
        }

        bool initialise(bool if_scale) {
            // Pre check
//            if (current_frame->getId() != 1) {
//                std::cout << "[Initialise]: Fail. The initilisation is only allowed for the second frame." << std::endl;
//                return false;
//            }

            // Start Initialise
            Frame::Ptr first_frame = map->getAllFrames().at(current_frame->getId() - 1);
            Frame::Ptr second_frame = current_frame;

            // Retrieve rough_T_w_current
            Eigen::Isometry3d T_first_second;
            bool is_motion_retrieved = epipolar_solver->retrieveMotion(first_frame, second_frame, T_first_second);
            if (is_motion_retrieved && first_frame->isRegistered()) {
                second_frame->setTwc_est(first_frame->getTwc_est() * T_first_second);
                second_frame->setRegistered(true);
            } else {
                std::cout << "[Initialise] Fail. Insufficient matches for initilisation.\n";
                return false;
            }

            // Initilise the scale
            if (if_scale) {
                align2GPS();
            }

            // Triangulation
            {
                std::vector<cv::Point3f> points3d;
                triangulator->triangulateTwoFrames(first_frame, second_frame, points3d);
                std::vector<int> kpIdx1, kpIdx2;
                getMatch(first_frame, second_frame, kpIdx1, kpIdx2);

                for (int i = 0; i < points3d.size(); ++i) {
                    /// If the triangulated point is valid
                    sfm::Track track;
                    track.addElement(first_frame->getId(), kpIdx1[i]);
                    track.addElement(current_frame->getId(), kpIdx2[i]);
                    long id = map->insertMapPoint(points3d[i], track);
                    MapPoint::Ptr mapPoint = map->getAllMapPoints().at(id);
                    double error_inserted_mappoint = Utils::calReproErrMappoint(map, mapPoint);
                    if (error_inserted_mappoint > reprojection_error_post_tri) {
                        map->getAllMapPoints().at(id)->setValidity(false);
                    }
                }
            }

            // Minimise reprojection error
            double repro_error;
//            std::cout << "The number unfixed frames is " << map->getUnfixedFrames().size() << ". The number unfixed mappoints is "<< map->getUnfixedMapPoints().size() <<". The number unfixed and valid mappoints is " << map->getUnfixedAndValidMapPoints().size() << std::endl;
            optimizer->initialise(map, first_frame, second_frame, repro_error);
            std::cout << "[Initialisation]: Success. A total of " << map->getUnfixedAndValidMapPoints().size()
                      << " mappoints are triangulated with reproject error of " << repro_error << "." << std::endl;

            return true;
        }

        bool registration() {
//            std::cout << "[Registration] Start.\n";
            std::vector<cv::Vec2f> kp;
            std::vector<cv::Vec3f> mapPoint;
            std::vector<int> kpIdx;
            std::vector<long> mapPointId;
            map->getMatchToMap(current_frame, kp, mapPoint, kpIdx, mapPointId);
            Eigen::Isometry3d T_w_c;
            std::vector<bool> inlierMask;
            std::string registration_type;
            double reproError;

            /// PnP Solver
            {
                bool isRegisteredByPnP = pnp_solver->solve(kp, mapPoint, T_w_c, inlierMask);

                /// Calculate Reprojection Error
                if (isRegisteredByPnP) {
                    reproError = Utils::calReproErrorPoints(kp, mapPoint, T_w_c, inlierMask);
                }

                if (isRegisteredByPnP && reproError <= 10) {
                    registration_type = "PnP";
                } else {
                    std::cout << "[Registration] PnP Fail. Insufficent matches.\n";
                    return false;
                }
            }

            // update frame
            current_frame->setRegistered(true);
            current_frame->setTwc_est(T_w_c);
            // update track
            int num_inlier = 0;
            for (int i = 0; i < inlierMask.size(); ++i) {
                if (!inlierMask[i]) continue;
                map->addObservation(mapPointId[i], current_frame->getId(), kpIdx[i]);
                num_inlier++;
            }
            // Calculate reprojection error
            reproError = Utils::calReproErrFrame(map, current_frame);

            if (reproError>1){
                localOptimisation();
                reproError = Utils::calReproErrFrame(map, current_frame);
            }

            std::cout << "[Registration] Success with " << registration_type << ". The reprojection error is "
                      << reproError
                      << " for valid observation of " << num_inlier << "/" << inlierMask.size() << " mappoints."
                      << std::endl;
            return true;
        }

        bool triangulate() {
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

                // Skip if the depth of the triangulated point is false
                if (!Utils::checkDepthPositive(pt3d, current_frame->getTwc_est())) continue;

                // update map
                map->insertMapPoint(pt3d, track);
                new_point_num++;
            }

            // Reject the outlier
            int invalid_num = 0;
            std::unordered_map<int, MapPoint::Ptr> frame_mappoints = current_frame->getValidMapPoints();
            for (auto &element: frame_mappoints) {
                MapPoint::Ptr tem_mappoint = element.second;
                double error = Utils::calReproErrMappoint(map, tem_mappoint);
                if (error >= reprojection_error_post_tri) {
                    tem_mappoint->setValidity(false);
                    invalid_num++;
                }
            }

            if (invalid_num > (0.8 * new_point_num)) {
                std::cout << "[Triangulate]: Fail. New triangulated map points: " << new_point_num
                          << ". The invalid map points of this frame: " << invalid_num << "." << std::endl;
                return false;
            } else {
                std::cout << "[Triangulate]: Success. New triangulated map points: " << new_point_num
                          << ". The invalid map points of this frame: " << invalid_num << "." << std::endl;
                return true;
            }
        }

        void localOptimisation() {
            optimizer->localBA(map, current_frame);
        }

        void globalOptimisation() {
            optimizer->globalBA(map);
        }

        bool matchFrames() {
            //
            switch (match_type) {
                case 0: // Sequential Match
                    for (int i = 1; i <= std::min(sequential_overlap, max_overlap); i++) {
                        long reference_frame_id = current_frame->getId() - i;
                        if (map->getAllFrames().find(reference_frame_id) != map->getAllFrames().end() &&
                            !map->getAllFrames().find(reference_frame_id)->second->isFixed()) {
                            sfm::Frame::Ptr reference_frame = map->getAllFrames().at(reference_frame_id);
                            matcher->matchTwo(reference_frame, current_frame);
                        }
                    }
                case 1: // Match with all potential candidates


                default:;
            }
            return true;
        }

        /**
         * align2GPS is used to register the frames to the GPS data. It can be used for one, two, or even more frames.
         */
        void align2GPS() {
            /// Obtain the frames to be aligned.
            std::unordered_map<long, Frame::Ptr> tem_id_frames;
            for (auto id_frame: map->getAllFrames()) {
                if (!id_frame.second->isFixed() && id_frame.second->isRegistered()) {
                    tem_id_frames[id_frame.first] = id_frame.second;
                }
            }

            /// Calculate the SIM3 transformation matrix
            Eigen::Isometry3d T_gps_est;
            double scale_factor;
            register2GPS(tem_id_frames, T_gps_est, scale_factor);

            /// Applied the SIM#
            map->applySIM3(T_gps_est, scale_factor, false);

            Eigen::Matrix4d SIM3 = Eigen::Matrix4d::Identity();
            SIM3.block<3, 3>(0, 0) = scale_factor * T_gps_est.linear();
            SIM3.block<3, 1>(0, 3) = T_gps_est.translation();
            std::cout << "[Align2GPS]: Success. The scaling factor is " << scale_factor << ".\n";
//            std::cout << "The SIM3 transformation matrix is " << SIM3 << std::endl;
        }

        void align2Cloud() {
            Eigen::Isometry3d T_gt_est;
            double scale_factor;

            // Obtain the src_cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud = getSrcCloud();
            pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud;
            align2GroundTruthCloud(src_cloud, registered_cloud);
        }

        void align2GroundTruthCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr &registered_cloud) {
            Eigen::Isometry3d T_gt_est;
            double scale_factor;
            icp_solver->registerWithScaleGICP(src_cloud, registered_cloud, T_gt_est, scale_factor);

            std::cout << "[Align2Cloud]: Success with " << registered_cloud->size() << "/" << src_cloud->size()
                      << " points. The scaling factor is " << scale_factor << ". The SIM3 transformation matrix is \n"
                      << T_gt_est.matrix() << std::endl;
            map->applySIM3(T_gt_est, scale_factor, true);
        }

    protected:
        pcl::PointCloud<pcl::PointXYZ>::Ptr getSrcCloud() {
            std::unordered_map<long, MapPoint::Ptr> unfixed_valid_mappoint = map->getUnfixedAndValidMapPoints();

            pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);;
            for (auto &id_mappoint: unfixed_valid_mappoint) {
                pcl::PointXYZ point;
                point.x = id_mappoint.second->getPos()[0];
                point.y = id_mappoint.second->getPos()[1];
                point.z = id_mappoint.second->getPos()[2];
                src_cloud->points.push_back(point);
            }

            return src_cloud;
        }

    private:
        bool match2previous() {
            return true;
        }

        void register2GPS(std::unordered_map<long, Frame::Ptr> id_frames, Eigen::Isometry3d &T_gps_local,
                          double &scale_factor) {
            int size = id_frames.size();
            T_gps_local = Eigen::Isometry3d::Identity();

            switch (size) {
                case 0: {
                    std::cerr << "[Error]: No valid frames when registering to GPS poses";
                    break;
                }
                case 1: {
                    Frame::Ptr frame = id_frames.begin()->second;
                    scale_factor = 1;
                    T_gps_local = frame->getTwc_gps().inverse() * frame->getTwc_est();
                    break;
                }
                case 2: {
                    std::unordered_map<long, Frame::Ptr>::iterator it = id_frames.begin();
                    Frame::Ptr first_frame = it->second;
                    std::advance(it, 1);
                    Frame::Ptr second_frame = it->second;

                    /// Calculate the scale
                    Eigen::Isometry3d T_1_2_gps = first_frame->getTwc_gps().inverse() * second_frame->getTwc_gps();
                    Eigen::Isometry3d T_1_2_est = first_frame->getTwc_est().inverse() * second_frame->getTwc_est();
                    scale_factor = T_1_2_gps.translation().norm() / T_1_2_est.translation().norm();

                    /// Calculate the rotation matrix
                    Eigen::Isometry3d T_gps_est_1 = first_frame->getTwc_gps().inverse() * first_frame->getTwc_est();
                    Eigen::Matrix3d RotationMatrix = T_gps_est_1.linear();
                    T_gps_local.prerotate(RotationMatrix);

                    /// Calculate the translation vector
                    Eigen::Vector3d pos_gps_1 = first_frame->getTwc_gps().translation();
                    Eigen::Vector3d pos_est_1 = first_frame->getTwc_est().translation();
                    Eigen::Vector3d TranslationVector =
                            pos_gps_1 - scale_factor * RotationMatrix * pos_est_1;
                    T_gps_local.pretranslate(TranslationVector);
                    break;
                }
                default: {
                    std::vector<Vector3d> source;
                    std::vector<Vector3d> target;
                    for (auto &id_frame: id_frames) {
                        source.push_back(id_frame.second->getTwc_est().translation());
                        target.push_back(id_frame.second->getTwc_gps().translation());
                    }
                    Umeyama(source, target, T_gps_local, scale_factor);
                }
            }
        }

        Eigen::Matrix4d
        solveSIM3(const std::vector<Eigen::Isometry3d> &Ts_source, const std::vector<Eigen::Isometry3d> &Ts_target) {
            /// Data check
            if (Ts_source.size() != Ts_target.size()) {
                throw std::runtime_error("[Error]: The size of input source and target is not the same.");
            }

            /// Fundamental data
            std::vector<Eigen::Quaterniond> qs_source_target;
            std::vector<Eigen::Vector3d> ts_source;
            std::vector<Eigen::Vector3d> ts_target;
            Eigen::Vector3d source_centroid(0, 0, 0);
            Eigen::Vector3d target_centroid(0, 0, 0);

            int num_poses = Ts_source.size();
            for (int i = 0; i < num_poses; i++) {
                qs_source_target.push_back(Eigen::Quaterniond((Ts_source[i].inverse() * Ts_target[i]).linear()));
                ts_source.push_back(Ts_source[i].translation());
                ts_target.push_back(Ts_target[i].translation());
                source_centroid += Ts_source[i].translation();
                target_centroid += Ts_target[i].translation();
            }

            source_centroid /= num_poses;
            target_centroid /= num_poses;

            /// Calculate the rotation
            Eigen::Matrix3d RotationMatrix;

            Eigen::Quaterniond averageQuaternion(0.0, 0.0, 0.0, 0.0);
            for (const Eigen::Quaterniond &q: qs_source_target) {
                averageQuaternion.coeffs() += q.coeffs();
            }
            averageQuaternion.coeffs() /= static_cast<double>(qs_source_target.size());

            RotationMatrix = averageQuaternion.toRotationMatrix();

            /// Calculate the scale
            double scaling_factor;

            Eigen::MatrixXd centered_source(3, num_poses);
            Eigen::MatrixXd centered_target(3, num_poses);
            for (int i = 0; i < num_poses; i++) {
                centered_source.col(i) = ts_source[i] - source_centroid;
                centered_target.col(i) = ts_target[i] - target_centroid;
            }

            if (num_poses == 1) {
                scaling_factor = 1;
            } else {
                scaling_factor = (centered_target.norm() / centered_source.norm());
            }

            /// Calculate the translation
            Eigen::Vector3d TranslationVector = target_centroid - scaling_factor * RotationMatrix * source_centroid;

            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) = scaling_factor * RotationMatrix;
            T.block<3, 1>(0, 3) = TranslationVector;

            return T;
        }

        void Umeyama(const std::vector<Vector3d> &source, const std::vector<Vector3d> &target, Eigen::Isometry3d &T,
                     double &scale) {
            // Number of points
            int num_points = source.size();

            // Calculate the centroids of source and target points
            Vector3d source_centroid(0, 0, 0);
            Vector3d target_centroid(0, 0, 0);

            for (int i = 0; i < num_points; i++) {
                source_centroid += source[i];
                target_centroid += target[i];
            }

            source_centroid /= num_points;
            target_centroid /= num_points;

            // Calculate the centered source and target points
            Eigen::MatrixXd centered_source(3, num_points);
            Eigen::MatrixXd centered_target(3, num_points);

            for (int i = 0; i < num_points; i++) {
                centered_source.col(i) = source[i] - source_centroid;
                centered_target.col(i) = target[i] - target_centroid;
            }

            // Compute the cross-covariance matrix
            Matrix3d cov_matrix = centered_source * centered_target.transpose();

            // Calculate the Singular Value Decomposition (SVD)
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);

            // Calculate the optimal rotation matrix
            Matrix3d optimal_rotation = svd.matrixV() * svd.matrixU().transpose();

            // Calculate the optimal scaling factor
            double scaling_factor = (centered_target.norm() / centered_source.norm());
            scale = scaling_factor;

            // Calculate the optimal translation vector
            Vector3d optimal_translation = target_centroid - scaling_factor * optimal_rotation * source_centroid;

            // Build the Rigid transformation
            T = Eigen::Isometry3d::Identity();
            T.prerotate(optimal_rotation);
            T.pretranslate(optimal_translation);
        }

    private:
        Map::Ptr map;
        Matcher::Ptr matcher;
        EpipolarSolver::Ptr epipolar_solver;
        PnPSolver::Ptr pnp_solver;
        Triangulator::Ptr triangulator;
        Optimizer::Ptr optimizer;
        Frame::Ptr current_frame;
        ICPSolver::Ptr icp_solver;
        std::thread render_thread;


        // Keypoint detection parametres
        int max_image_size = 3000;
        int max_num_features = 8000;
        Frame::KeyPointType keypoint_type = Frame::KeyPointType::SIFT;

        // Match strategies
        int match_type = 0;    // 0 for Sequential; 1 for all potential candidates.
        int sequential_overlap = 3;
        int max_overlap = 5;
        double reprojection_error_post_tri = 4;
    };

};


#endif //IIQC_SFM_RECONSTRUCTOR_H