#ifndef IIQC_MAP_H
#define IIQC_MAP_H

#include <fstream>

#include "SfM/camera.h"
#include "SfM/frame.h"
#include "SfM/mappoint.h"
#include "SfM/track.h"
#include "Utils.h"


namespace sfm {

    class Map {
    public:
        typedef std::shared_ptr<Map> Ptr;

        struct CorrData {
            long frameId;
            int kptIdx;
            cv::Mat P;
            cv::Vec2f kpt;
        };

    public:
        Map() { camera = Camera::Ptr(new Camera); }

        void addFrame(const Frame::Ptr &frame) { frames[frame->getId()] = frame; }

        long insertMapPoint(const cv::Vec3f &pos, const Track &track) {
            // average the colors of multiple points on a track
            int sumB = 0, sumG = 0, sumR = 0;
            for (auto &trackItem: track.getElements()) {
                long frameId = trackItem.first;
                int kpIdx = trackItem.second;
                Frame::Ptr frame = frames[frameId];
                cv::Vec3b color = frame->getColor()[kpIdx];

                sumB += color(0);
                sumG += color(1);
                sumR += color(2);
            }

            cv::Vec3b color(static_cast<uchar>(sumB / track.length()),
                            static_cast<uchar>(sumG / track.length()),
                            static_cast<uchar>(sumR / track.length()));

            // insert map point
            MapPoint::Ptr mapPoint = MapPoint::createMapPoint(pos, color);
            mapPoint->setTrack(track);
            mapPoint->setValidity(true);
            mapPoints[mapPoint->getId()] = mapPoint;

            // set frame associated information
            for (auto &trackItem: track.getElements()) {
                long frameId = trackItem.first;
                int kpIdx = trackItem.second;
                Frame::Ptr frame = frames[frameId];
                frame->setMapPoint(kpIdx, mapPoint);
            }
            return mapPoint->getId();
        }

        void rejectMapPoint(MapPoint::Ptr mappoint) {
            // average the colors of multiple points on a track
            mappoint->setValidity(false);
        }

        void addObservation(long mapPointId, long frameId, int kpIdx) {
            assert(hasMapPoint(mapPointId));
            assert(mapPoints.at(mapPointId)->getValidity());

            Frame::Ptr newFrame = frames[frameId];
            MapPoint::Ptr mapPoint = mapPoints[mapPointId];

            // update color
            cv::Vec3b oldColor = mapPoint->getColor();
            cv::Vec3b color = newFrame->getColor()[kpIdx];
            int length = mapPoint->getTrack().length();

            int oldB = oldColor(0), oldG = oldColor(1), oldR = oldColor(2);
            int newB = color(0), newG = color(1), newR = color(2);
            cv::Vec3b newColor(
                    static_cast<unsigned char>((oldB * length + newB) / (length + 1)),
                    static_cast<unsigned char>((oldG * length + newG) / (length + 1)),
                    static_cast<unsigned char>((oldR * length + newR) / (length + 1)));
            mapPoint->setColor(newColor);

            // update track
            mapPoint->getTrack().addElement(frameId, kpIdx);
            newFrame->setMapPoint(kpIdx, mapPoint);
        }

        bool hasMapPoint(long id) {
            return mapPoints.find(id) != mapPoints.end();
        }

        bool isTwoViewObservation(Frame::Ptr frame, int kpId) {
            auto id_matches = frame->getMatches();
            auto mathces = id_matches.at(kpId);
            if (mathces.size() != 1) return false;

            long otherFrameId = mathces.begin()->first;
            int otherKptId = mathces.begin()->second;
            Frame::Ptr otherFrame = frames[otherFrameId];
            return otherFrame->getMatches().at(otherKptId).size() == 1;
        }

        // setter and getter
        const std::unordered_map<long, Frame::Ptr> &getAllFrames() const { return frames; }

        const std::unordered_map<long, Frame::Ptr> getUnfixedFrames() const {
            std::unordered_map<long, Frame::Ptr> unfixed_frames;
            for (auto &frameId_framePtr: frames) {
                if (!frameId_framePtr.second->isFixed()) {
                    unfixed_frames[frameId_framePtr.first] = frameId_framePtr.second;
                }
            }
            return unfixed_frames;
        }

        const std::unordered_map<long, MapPoint::Ptr> &getAllMapPoints() const { return mapPoints; }

        const std::unordered_map<long, MapPoint::Ptr> getValidMapPoints() const {
            std::unordered_map<long, MapPoint::Ptr> valid_mappoint;
            for (auto &element: mapPoints) {
                if (element.second->getValidity()) {
                    valid_mappoint[element.first] = element.second;
                }
            }
            return valid_mappoint;
        }

        const std::unordered_map<long, MapPoint::Ptr> getUnfixedMapPoints() const {
            std::unordered_map<long, MapPoint::Ptr> unfixed_mappoint;
            for (auto &id_mappoint: mapPoints) {
                if (!id_mappoint.second->isFixed()) {
                    unfixed_mappoint[id_mappoint.first] = id_mappoint.second;
                }
            }
            return unfixed_mappoint;
        }

        const std::unordered_map<long, MapPoint::Ptr> getUnfixedAndValidMapPoints() const {
            std::unordered_map<long, MapPoint::Ptr> unfixed_mappoint;
            for (auto &id_mappoint: mapPoints) {
                if (!id_mappoint.second->isFixed() && id_mappoint.second->getValidity()) {
                    unfixed_mappoint[id_mappoint.first] = id_mappoint.second;
                }
            }
            return unfixed_mappoint;
        }

        int getValidMapPointsNum() const {
            int num = 0;
            for (auto &element: mapPoints) {
                if (element.second->getValidity()) num++;
            }
            return num;
        }

        // best frame selection
        std::vector<Frame::Ptr> findFirstInitialImage() const {
            std::vector<Frame::Ptr> candidates;
            for (auto &frame: frames) {
                Frame::Ptr pFrame = frame.second;
                if (pFrame->getNumRegister() > 0) continue; // skip Frames that failed to initialize

                candidates.push_back(pFrame);
            }

            std::sort(candidates.begin(), candidates.end(), [](Frame::Ptr frame1, Frame::Ptr frame2) {
                return frame1->getMatches().size() > frame2->getMatches().size();
            });

            return candidates;
        }

        std::vector<Frame::Ptr> findSecondInitialImage(Frame::Ptr firstFrame) {

            // find images that are connected to the first seed image and have
            // not been registered before in other reconstructions
            std::unordered_map<long, int> id_numMatches;
            for (auto &match: firstFrame->getMatches()) {
                auto id_KpIdx = match.second;
                for (auto &it: id_KpIdx) {
                    long id = it.first;
                    if (frames[id]->getNumRegister() > 0) continue; // skip Frames that failed to initialize

                    id_numMatches[id] += 1;
                }
            }

            std::vector<std::pair<long, int>> id_numMatches_vec;
            for (auto &id_numMatch: id_numMatches) {
                long id = id_numMatch.first;
                int numMatch = id_numMatch.second;
                id_numMatches_vec.push_back({id, numMatch});
            }

            // more correspondences are preferred when sorting
            sort(id_numMatches_vec.begin(), id_numMatches_vec.end(),
                 [](std::pair<long, int> elem1, std::pair<long, int> elem2) {
                     return elem1.second > elem2.second;
                 });

            // extract Frames in sorted order
            std::vector<Frame::Ptr> candidates;
            for (auto &id_numMatch: id_numMatches_vec) {
                long id = id_numMatch.first;
                candidates.push_back(frames[id]);
            }

            return move(candidates);
        }

        std::vector<Frame::Ptr> findNextRegisterImage() {
            std::vector<std::pair<long, int>> id_scores;
            for (auto &frame: frames) {
                Frame::Ptr pFrame = frame.second;
                long id = pFrame->getId();

                // skip if already registered
                if (pFrame->isRegistered()) continue;

                std::vector<long> neighborsId = pFrame->getNeighborsId();
                std::vector<long> registeredNeighborsId;
                for (auto neighborId: neighborsId) {
                    if (frames[neighborId]->isRegistered()) registeredNeighborsId.push_back(neighborId);
                }

                // skip if neighbors are all not registered
                if (registeredNeighborsId.empty()) continue;

                id_scores.push_back({id, registeredNeighborsId.size()});
            }

            sort(id_scores.begin(), id_scores.end(),
                 [](const std::pair<long, int> &id_scores1, const std::pair<long, int> &id_scores2) {
                     return id_scores1.second > id_scores2.second;
                 });

            std::vector<Frame::Ptr> candidates;
            for (auto &id_score: id_scores) {
                long id = id_score.first;
                candidates.push_back(frames[id]);
            }
            return move(candidates);
        }

        void getMatchToMap(Frame::Ptr frame, std::vector<cv::Vec2f> &keypoint, std::vector<cv::Vec3f> &mapPoint,
                           std::vector<int> &kpIdx, std::vector<long> &mapPointId) {
            keypoint.clear();
            mapPoint.clear();
            kpIdx.clear();
            mapPointId.clear();
            auto matches = frame->getMatches();

            // search all neighbors
            for (auto &id_match: matches) {
                auto &match = id_match.second;
                for (auto &frameId_kptId: match) {
                    long frameId = frameId_kptId.first;
                    int kptId = frameId_kptId.second;

                    // find the map points that are already in the map
                    if (frames[frameId]->hasMapPoint(kptId)) {
                        kpIdx.push_back(id_match.first);
                        keypoint.push_back(frame->getKp()[id_match.first].pt);

                        auto mappoint = frames[frameId]->getMapPoints().at(kptId);
                        mapPointId.push_back(mappoint->getId());
                        mapPoint.push_back(mappoint->getPos());
                        break;
                    }
                }
            }
        }

        void getMatchToNeighbor(Frame::Ptr frame, std::vector<std::vector<CorrData>> &data) {
            data.clear();

            // search all neighbors
            auto matches = frame->getMatches();
            for (auto &id_match: matches) {  // for each keypoint
                long kptId = id_match.first;
                auto &match = id_match.second;

                // skip keypoint if already have associated mappoint
                if (frame->hasMapPoint(kptId)) { continue; }

                // skip if it is a two-view track (not very useful in reconstruction)
//                if (isTwoViewObservation(frame, kptId)) continue;

                std::vector<CorrData> dataTemp;
                for (auto &frameId_kptId: match) {   // for each neighbor frame
                    long otherFrameId = frameId_kptId.first;
                    int otherKptId = frameId_kptId.second;

                    // skip if neighbor frame is not registered
                    bool isRegistered = frames[otherFrameId]->isRegistered();
                    if (!isRegistered) continue;

                    // skip neighbor keypoint if it already has associated mappoint
                    // ******** New Associated Point *********
                    if (frames[otherFrameId]->hasMapPoint(otherKptId)) {
                        addObservation(frames[otherFrameId]->getMapPoints().at(otherKptId)->getId(), frame->getId(),
                                       kptId);
                        continue;
                    }
                    // *********** End ************

                    cv::Mat P;
                    cv::eigen2cv(frames[otherFrameId]->getTwc_est().inverse().matrix(), P);
                    cv::Mat K = Camera::instance()->K();
                    P = K * P.rowRange(0, 3); // P=K[R|t]

                    CorrData corrData;
                    corrData.frameId = otherFrameId;
                    corrData.kptIdx = otherKptId;
                    corrData.P = P;
                    corrData.kpt = frames[otherFrameId]->getKp()[otherKptId].pt;
                    dataTemp.push_back(corrData);
                }


                // add current frame itself for multiview triangulation
                if (!dataTemp.empty()) {
                    cv::Mat P;
                    cv::eigen2cv(frame->getTwc_est().inverse().matrix(), P);
                    cv::Mat K = Camera::instance()->K();
                    P = K * P.rowRange(0, 3); // P=K[R|t]

                    CorrData corrData;
                    corrData.frameId = frame->getId();
                    corrData.kptIdx = kptId;
                    corrData.P = P;
                    corrData.kpt = frame->getKp()[kptId].pt;
                    dataTemp.push_back(corrData);

                    data.push_back(dataTemp);
                }
            }
        }


        void applySIM3(Eigen::Isometry3d &T_gps_local, double &scalar, bool if_final){
            Eigen::Matrix3d rotation_matrix = T_gps_local.rotation();
            Eigen::Vector3d translate_vector = T_gps_local.translation();

            /// Update the mappoints
            for (auto &id_mappoint: mapPoints) {
                if (id_mappoint.second->isFixed()) continue;
                MapPoint::Ptr mappoint = id_mappoint.second;
                Eigen::Vector3d oldpos(mappoint->getPos()[0], mappoint->getPos()[1], mappoint->getPos()[2]);
                Eigen::Vector3d newpos = scalar * rotation_matrix * oldpos + translate_vector;
                mappoint->setPos(cv::Vec3d(newpos.x(), newpos.y(), newpos.z()));
                if(if_final) mappoint->setFixed(true);
            }

            /// Update the frames pose
            for (auto &id_frame: frames) {
                if (id_frame.second->isFixed() || !id_frame.second->isRegistered()) continue;  // Only update the unfixed and registered frames
                Frame::Ptr frame = id_frame.second;
                Eigen::Isometry3d oldpose = frame->getTwc_est();
                Eigen::Isometry3d newpose = Eigen::Isometry3d::Identity();
                newpose.linear() = rotation_matrix * oldpose.rotation();
                newpose.translation() = scalar * rotation_matrix * oldpose.translation() + translate_vector;
                frame->setTwc_est(newpose);
                if(if_final) frame->setFixed(true);
            }
        }

        // save results in certain format
        void writePLYBinary(const std::string &path) {
            std::ofstream file;
            // trunc: delete the file if it already exists
            file.open(path.c_str(), std::ios::trunc);

            file << "ply" << std::endl;
            file << "format binary_little_endian 1.0" << std::endl;
            file << "element vertex " << mapPoints.size() << std::endl;
            file << "property float x" << std::endl;
            file << "property float y" << std::endl;
            file << "property float z" << std::endl;
            file << "property uchar red" << std::endl;
            file << "property uchar green" << std::endl;
            file << "property uchar blue" << std::endl;
            file << "end_header" << std::endl;

            for (auto &pts3d: mapPoints) {
                MapPoint::Ptr point = pts3d.second;
                cv::Vec3f pos = point->getPos();
                cv::Vec3b color = point->getColor();
                std::swap(color(0), color(2));    // opencv store color in B, G, R order

                file.write((char *) pos.val, sizeof(float) * 3);
                file.write((char *) color.val, sizeof(uchar) * 3);
            }

            file.close();
            std::cout << "[Data Saving]: Ply point cloud saved." << std::endl;
        };

        void writePoses(const std::string &path){

        }

    private:
        std::unordered_map<long, MapPoint::Ptr> mapPoints;           // all landmarks
        std::unordered_map<long, Frame::Ptr> frames;                 // all frames (registered and non-registered)
        Camera::Ptr camera;
    };

}


#endif //IIQC_MAP_H
