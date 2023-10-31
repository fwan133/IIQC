//
// Created by feng on 7/10/23.
//

#ifndef IIQC_MAPPOINT_H
#define IIQC_MAPPOINT_H

#include <memory>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>

#include "SfM/track.h"
#include "SfM/frame.h"

namespace sfm {

    class MapPoint {
    public:
        typedef std::shared_ptr<MapPoint> Ptr;

        MapPoint(long id, const cv::Vec3f &pos, const cv::Vec3b &color) : id(id), pos(pos), color(color) {}

        ~MapPoint() {};

        static MapPoint::Ptr createMapPoint(const cv::Vec3f &pos, const cv::Vec3b &color) {
            static long factory_id = 0;
            return MapPoint::Ptr(new MapPoint(factory_id++, pos, color));
        }

        // setter and getter
        void setTrack(const Track &track) { MapPoint::track = track; }

        Track &getTrack() { return track; }

        void getTrack(std::vector<long> &frameIds, std::vector<int> &kpIds) {
            std::unordered_map<long, int> elements = track.getElements();
            for (auto &element: elements) {
                frameIds.push_back(element.first);
                kpIds.push_back(element.second);
            }
        }

        long getId() const { return id; }

        const cv::Vec3f &getPos() const { return pos; }

        void setPos(const cv::Vec3f &pos) { MapPoint::pos = pos; }

        const cv::Vec3b &getColor() const { return color; }

        void setColor(const cv::Vec3b &color) { MapPoint::color = color; }

        void setValidity(bool validity) { MapPoint::validity = validity; }

        bool getValidity() { return validity; }

        void setFixed(bool fixed) { MapPoint::fixed = fixed; }

        bool isFixed() { return fixed; }

    private:
        long id;
        cv::Vec3f pos;                                        // position in world
        cv::Vec3b color;                                      // b, g, r
        bool validity = true;                                 // If this point is valid with minor mistake
        Track track;                                          // frameID_keypointIdx
        bool fixed = false;                                 // If this map point has been registered.
    };

}


#endif //IIQC_MAPPOINT_H
