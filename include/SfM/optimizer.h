//
// Created by feng on 8/10/23.
//

#ifndef IIQC_OPTIMIZER_H
#define IIQC_OPTIMIZER_H

#include "common.h"
#include "Utils.h"

#include "SfM/frame.h"
#include "SfM/mappoint.h"
#include "SfM/map.h"

namespace sfm {

    class Optimizer {
    public:
        typedef std::shared_ptr<Optimizer> Ptr;
        typedef g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType> LinearSolver;

        Optimizer() {
            std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(
                    new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>);
            std::unique_ptr<g2o::BlockSolver_6_3> blockSolver(new g2o::BlockSolver_6_3(std::move(linearSolver)));
            solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

            float fx = Camera::instance()->getFx();
            float cx = Camera::instance()->getCx();
            float cy = Camera::instance()->getCy();
            camera = new g2o::CameraParameters(fx, Eigen::Vector2d(cx, cy), 0);
            camera->setId(0);

            robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");
        }

        void globalBA(Map::Ptr map) {
            const auto &mapPoints = map->getUnfixedAndValidMapPoints();
            const auto &frames = map->getUnfixedFrames();

            // only get all registered frames
            std::unordered_map<long, Frame::Ptr> registeredFrames;
            for (auto &frame: frames) {
                if (!frame.second->isRegistered()) continue;
                registeredFrames[frame.first] = frame.second;
            }

            bundleAdjustment(registeredFrames, mapPoints);
            recoverOptimizedData(map, registeredFrames, mapPoints);
            std::cout << "[Bundle Adjustment]: Global BA successfully." << std::endl;
        }

        void localBA(Map::Ptr map, Frame::Ptr currentFrame) {
            // get covisible neighbor frames (not all connected neighbors) from track
            std::unordered_map<long, Frame::Ptr> frames;
            frames.insert({currentFrame->getId(), currentFrame});

            bool exceedMaxNum = false;
            const std::unordered_map<int, MapPoint::Ptr> &currentMapPoints = currentFrame->getValidMapPoints();
            for (auto &point: currentMapPoints) {
                if (point.second == nullptr) continue;
                if (exceedMaxNum) break;

                const auto &trackElements = point.second->getTrack().getElements();
                for (auto element: trackElements) {
                    long neighborFrameId = element.first;
                    frames.insert({neighborFrameId, map->getAllFrames().at(neighborFrameId)});

                    if (frames.size() > maxLocalBAFramesNum) {
                        exceedMaxNum = true;
                        break;
                    }
                }
            }

            // get local map points seen in these local frames
            std::unordered_map<long, MapPoint::Ptr> localMapPoints;
            for (auto it = frames.begin(); it != frames.end(); ++it) {
                const auto &mapPoints = it->second->getValidMapPoints();
                for (auto &point: mapPoints) {
                    if (point.second.get() == nullptr) continue;
                    localMapPoints[point.second->getId()] = point.second;
                }
            }

            bundleAdjustment(frames, localMapPoints);
            recoverOptimizedData(map, frames, localMapPoints);
            std::cout << "[Bundle Adjustment]: Local BA successfully." << std::endl;
        }

        bool
        framePoseOptimisation(std::vector<cv::Vec2f> &points2D, std::vector<cv::Vec3f> &points3D,
                              Eigen::Isometry3d &T_initial, Eigen::Isometry3d &T_final, double &repro_error, std::vector<bool> &inlierMask) {
            inlierMask.clear();
            try {
                assert(points2D.size() == points3D.size());

                reset();

                // Set frame vertices
                auto *vSE3 = new g2o::VertexSE3Expmap();  // Pose of current frame
                vSE3->setId(0);
                vSE3->setEstimate(Utils::toSE3Quat(T_initial));
                vSE3->setFixed(false);
                optimizer->addVertex(vSE3);

                // Set 3d points and edge
                int index = 1;
                for (int i = 0; i < points3D.size(); i++) {
                    auto p2d = points2D[i];
                    auto p3d = points3D[i];

                    // Set 3d point
                    auto *vPoint3d = new g2o::VertexPointXYZ();
                    vPoint3d->setId(index);
                    vPoint3d->setEstimate(Utils::toVector3d(p3d));
                    vPoint3d->setMarginalized(true);
                    vPoint3d->setFixed(true);
                    optimizer->addVertex(vPoint3d);

                    // Set Edge
                    auto *edge = new g2o::EdgeProjectXYZ2UV();
                    edge->setId(index);
                    edge->setVertex(0, dynamic_cast<g2o::VertexPointXYZ *> (optimizer->vertex(index)));
                    edge->setVertex(1, vSE3);
                    edge->setMeasurement(Utils::toVector2d(p2d));
                    edge->setInformation(Eigen::Matrix2d::Identity());
                    edge->setParameterId(0, 0);
                    edge->setRobustKernel(robustKernel);
                    optimizer->addEdge(edge);

                    index++;
                }

                optimizer->initializeOptimization();
                optimizer->optimize(iteration);

                /// Results output
                // Prior to optimise
                double error_prior = calReproErr(points2D, points3D, T_initial);
//                std::cout << "[Optimise]: Prior: The frame pose is p:" << Utils::IsometryToMatt(T_initial)
//                          << ", q: " << Utils::IsometryToMatq(T_initial) << ". The reprojection error is "
//                          << error_prior << "." << std::endl;

                optimizer->initializeOptimization();
                optimizer->optimize(iteration);

                auto *v = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer->vertex(0));
                g2o::SE3Quat SE3quat = v->estimate();
                T_final = Utils::toIsometry3d(SE3quat).inverse();
                repro_error = calReproErr(points2D, points3D, T_final);

//                std::cout << "[Optimise]: Post: The frame pose is p:" << Utils::IsometryToMatt(T_final)
//                          << ", q: " << Utils::IsometryToMatq(T_final) << ". The reprojection error is "
//                          << repro_error << "." << std::endl;

                for (int i = 0; i < points3D.size(); i++){
                    if (calReproErrPoint(points2D[i], points3D[i], T_final) < projError){
                        inlierMask.push_back(true);
                    }else{
                        inlierMask.push_back(false);
                    }
                }

                return true;
            }
            catch (const std::exception &error) {
                return false;
            }

//            // Set frame vertices
//            auto *vSE3 = new g2o::VertexSE3Expmap();  // Pose of current frame
//            vSE3->setId(0);
//            vSE3->setEstimate(Utils::toSE3Quat(current_frame->getTwc_est().inverse()));
//            vSE3->setFixed(false);
//            optimizer->addVertex(vSE3);
//
//            // Set point vertices and edges
//            int index = 1;
//            for (const auto &id_points: mappoints) {
//                long kpId = id_points.first;
//                MapPoint::Ptr point = id_points.second;
//                // Set 3d point
//                auto *vPoint3d = new g2o::VertexPointXYZ();
//                vPoint3d->setId(index);
//                vPoint3d->setEstimate(Utils::toVector3d(point->getPos()));
//                vPoint3d->setMarginalized(true);
//                vPoint3d->setFixed(true);
//                optimizer->addVertex(vPoint3d);
//
//                // Set Edge
//                auto *edge = new g2o::EdgeProjectXYZ2UV();
//                edge->setId(index);
//                edge->setVertex(0, dynamic_cast<g2o::VertexPointXYZ *> (optimizer->vertex(index)));
//                edge->setVertex(1, vSE3);
//                edge->setMeasurement(Utils::toVector2d(current_frame->getKp()[kpId].pt));
//                edge->setInformation(Eigen::Matrix2d::Identity());
//                edge->setParameterId(0, 0);
//                edge->setRobustKernel(robustKernel);
//                optimizer->addEdge(edge);
//
//                index++;
//            }

            // Set edge
//            int index = 1;
//            for (const auto&id_points: mappoints){
//                long kpId = id_points.first;
//                MapPoint::Ptr point = id_points.second;
//                auto *edge = new g2o::EdgeSE3ProjectXYZOnlyPose();
//                edge->setId(index);
//                edge->setVertex(0, vSE3);
//                edge->setMeasurement(Utils::toVector2d(current_frame->getKp()[kpId].pt));
//                edge->setInformation(Eigen::Matrix2d::Identity());
//                edge->setParameterId(0, 0);
//                edge->setRobustKernel(robustKernel);
//                edge->Xw=Utils::toVector3d(point->getPos());
//                optimizer->addEdge(edge);
//            }

            // Results output
//            auto *edge = new g2o::EdgeSE3ProjectXYZOnlyPose();

//            /// Results output
//            std::cout << "\n[Optimise]: Frame Pose optimisation.\n";
//            // Prior to optimise
//            double error_prior = calReproErrFrame(map, current_frame);
//            std::cout << "[Optimise]: Prior: The frame pose is p:" << Utils::IsometryToMatt(current_frame->getTwc_est())
//                      << ", q: " << Utils::IsometryToMatq(current_frame->getTwc_est()) << ". The reprojection error is "
//                      << error_prior << "." << std::endl;
//
//            optimizer->initializeOptimization();
//            optimizer->optimize(iteration);
//            // Post optimise
//            auto *v = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer->vertex(0));
//            g2o::SE3Quat SE3quat = v->estimate();
//            current_frame->setTwc_est(Utils::toIsometry3d(SE3quat).inverse());
//            double error_post = calReproErrFrame(map, current_frame);
//            std::cout << "[Optimise]: Prior: The frame pose is p:" << Utils::IsometryToMatt(current_frame->getTwc_est())
//                      << ", q: " << Utils::IsometryToMatq(current_frame->getTwc_est()) << ". The reprojection error is "
//                      << error_post << "." << std::endl;
        }

        bool initialise(Map::Ptr &map, Frame::Ptr &first_frame, Frame::Ptr &second_frame, double &repro_error) {
            const auto &mapPoints = map->getUnfixedAndValidMapPoints();

            // only get all registered frames
            std::unordered_map<long, Frame::Ptr> registeredFrames;
            if (first_frame->isRegistered()) registeredFrames[0] = first_frame;
            if (first_frame->isRegistered()) registeredFrames[1] = second_frame;

            bundleAdjustment(registeredFrames, mapPoints);
            recoverOptimizedData(map, registeredFrames, mapPoints);
            // Post optimisation
            repro_error = calReproErrMap(map);
            return true;
        }

    private:
        void reset() {
            optimizer = new g2o::SparseOptimizer;
            optimizer->setAlgorithm(solver);
            optimizer->setVerbose(false);
            optimizer->addParameter(camera);
        }

        void bundleAdjustment(const std::unordered_map<long, Frame::Ptr> &frames,
                              const std::unordered_map<long, MapPoint::Ptr> &mappoints) {
            // reset all BA data
            reset();

            // set frame vertices
            maxFrameId = -1;
            for (auto &id_frame: frames) {
                long frameId = id_frame.first;
                Frame::Ptr frame = id_frame.second;
                maxFrameId = std::max(maxFrameId, frameId);

                auto *v = new g2o::VertexSE3Expmap;
                v->setId(frameId);
                v->setEstimate(Utils::toSE3Quat(frame->getTwc_est().inverse()));
                optimizer->addVertex(v);

                if (frameId == 0) {
                    v->setFixed(true);
                }
            }

            // set map point vertices
            for (auto &id_points: mappoints) {
                long pointId = id_points.first;
                MapPoint::Ptr point = id_points.second;

                auto *v = new g2o::VertexPointXYZ();
                long vertexId = maxFrameId + pointId + 1;   // in order not to conflict with the frame ID
                v->setId(vertexId);
                v->setMarginalized(true);
                v->setEstimate(Utils::toVector3d(point->getPos()));
                optimizer->addVertex(v);

                const auto &tracks = point->getTrack();

                // set edges
                for (auto &track: tracks.getElements()) {
                    long frameId = track.first;
                    int kpId = track.second;
                    if (frames.find(frameId) == frames.end()) continue;

                    auto *edge = new g2o::EdgeProjectXYZ2UV();

                    edge->setVertex(0, optimizer->vertex(vertexId));
                    edge->setVertex(1, optimizer->vertex(frameId));

                    Eigen::Matrix<double, 2, 1> obs;
                    auto kp = frames.at(frameId)->getKp()[kpId].pt;
                    obs << kp.x, kp.y;
                    edge->setMeasurement(obs);
                    edge->setInformation(Eigen::Matrix2d::Identity());
                    edge->setRobustKernel(robustKernel);
                    edge->setParameterId(0, 0);
                    optimizer->addEdge(edge);
                }
            }

            // optimize!
//            std::cout << "[Optimise]: Start optimization\n";
            optimizer->initializeOptimization();
            optimizer->optimize(iteration);
        }

        void recoverOptimizedData(const Map::Ptr map, const std::unordered_map<long, Frame::Ptr> &frames,
                                  const std::unordered_map<long, MapPoint::Ptr> &mappoints) {
            // frames
            for (auto &id_frame: frames) {
                long frameId = id_frame.first;
                Frame::Ptr frame = id_frame.second;

                auto *v = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer->vertex(frameId));
                g2o::SE3Quat SE3quat = v->estimate();
                frame->setTwc_est(Utils::toIsometry3d(SE3quat).inverse()); // fixme: may be inverse?
            }

            // map points
            int count_outlier = 0;
            for (auto &id_points: mappoints) {
                long pointId = id_points.first;
                MapPoint::Ptr point = id_points.second;

                long vertexId = maxFrameId + pointId + 1;
                auto *v = dynamic_cast<g2o::VertexPointXYZ *>(optimizer->vertex(vertexId));
                point->setPos(Utils::toCvMat(v->estimate()));
                v->setId(vertexId);

                /// Reject outlier mappoints
                double error = calReproErrMappoint(map, point);
                if (error > projError) {
                    point->setValidity(false);
                    count_outlier++;
                };
            }
//            std::cout << "[Optimise]: The number of outlier points after optimisation are " << count_outlier << ".\n";
        }

        double calReproErr(std::vector<cv::Vec2f> &points2D, const std::vector<cv::Vec3f> &points3D,
                           Eigen::Isometry3d &T_w_c) {

            std::vector<cv::Vec2f> projectedPoints2D;
            cv::projectPoints(points3D, Utils::IsometryToMatR(T_w_c.inverse()),
                              Utils::IsometryToMatt(T_w_c.inverse()), Camera::instance()->K(),
                              Camera::instance()->distCoef(), projectedPoints2D);

            double totalError = 0.0;
            for (size_t i = 0; i < projectedPoints2D.size(); ++i) {
                double error = cv::norm(points2D[i] - projectedPoints2D[i]);
                totalError += error;
            }
            return totalError / points2D.size();
        }

        double calReproErrMap(Map::Ptr map) {
            std::unordered_map<long, Frame::Ptr> frames = map->getUnfixedFrames();
            double totalError = 0.0;
            for (auto &frame: frames) {
                double error = Utils::calReproErrFrame(map,frame.second);
                totalError += error;
//                std::cout << "[Optimise]: " << "Reprojection error of Frame " << frame.first << " is " << error << ","
//                          << std::endl;
            }
            return totalError / frames.size();
        }

        double calReproErrMappoint(Map::Ptr map, MapPoint::Ptr mappoint) {
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

        double calReproErrFrame(Map::Ptr map, Frame::Ptr frame) {
            std::vector<cv::Vec2f> kp;
            std::vector<cv::Vec3f> mapPoint;
            std::vector<int> kpIdx;
            std::vector<long> mapPointId;
            frame->getAssociatedMapPoints(kp, mapPoint, kpIdx, mapPointId);

            std::vector<cv::Vec2f> projectedPoints;
            cv::projectPoints(mapPoint, Utils::IsometryToMatR(frame->getTwc_est().inverse()),
                              Utils::IsometryToMatt(frame->getTwc_est().inverse()), Camera::instance()->K(),
                              Camera::instance()->distCoef(), projectedPoints);

            // Calculate the reprojection error
            double totalError = 0.0;
            for (size_t i = 0; i < projectedPoints.size(); ++i) {
                double error = cv::norm(kp[i] - projectedPoints[i]);
                totalError += error;
            }

            // Calculate the mean reprojection error
            double meanError = totalError / kp.size();
            return meanError;
        }

        cv::Point2f calProjectedPoint(Frame::Ptr frame, cv::Vec3f point3d) {
            std::vector<cv::Point3f> objectPoints{point3d};
            std::vector<cv::Point2f> projectedPoints;

            cv::projectPoints(objectPoints, Utils::IsometryToMatR(frame->getTwc_est().inverse()),
                              Utils::IsometryToMatt(frame->getTwc_est().inverse()), Camera::instance()->K(),
                              Camera::instance()->distCoef(), projectedPoints);

            // Extract the projected 2D image point
            cv::Point2f projectedPoint = projectedPoints[0];
            return projectedPoint;
        }

        double calReproErrPoint(cv::Vec2f &point2D, cv::Vec3f &point3D, Eigen::Isometry3d &T_w_c) {
            std::vector<cv::Point3f> objectPoints{point3D};
            std::vector<cv::Point2f> projectedPoints;

            cv::projectPoints(objectPoints, Utils::IsometryToMatR(T_w_c.inverse()),
                              Utils::IsometryToMatt(T_w_c.inverse()), Camera::instance()->K(),
                              Camera::instance()->distCoef(), projectedPoints);

            cv::Vec2f projectedPoint = projectedPoints[0];

            return cv::norm(point2D - projectedPoint);
        };


    private:
        int iteration = 30;
        int maxLocalBAFramesNum = 5;                    // max number of frames in local BA
        float projError = 2.0;                          // allowed maximum reprojection error

        g2o::SparseOptimizer *optimizer = nullptr;      // g2o optimizer
        g2o::RobustKernel *robustKernel;                // robust kernel
        g2o::OptimizationAlgorithmLevenberg *solver;
        g2o::CameraParameters *camera;

        long maxFrameId = -1;
    };

}

#endif //IIQC_OPTIMIZER_H
