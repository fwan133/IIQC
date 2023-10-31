
#ifndef IIQC_CAMERA_H
#define IIQC_CAMERA_H

#include <memory>

#include "config.h"

namespace sfm {

    class Camera {
    public:
        typedef std::shared_ptr<Camera> Ptr;

        Camera(){
            fx = Config::instance()->get<float>("camera.fx");
            fy = Config::instance()->get<float>("camera.fy");
            cx = Config::instance()->get<float>("camera.cx");
            cy = Config::instance()->get<float>("camera.cy");

            k1 = Config::instance()->get<float>("camera.k1");
            k2 = Config::instance()->get<float>("camera.k2");
            p1 = Config::instance()->get<float>("camera.p1");
            p2 = Config::instance()->get<float>("camera.p2");
        };
        static Camera::Ptr instance(){
            static std::shared_ptr<Camera> camera = nullptr;
            if (camera == nullptr)
                camera = std::shared_ptr<Camera>(new Camera);

            return camera;
        };

        // getter
        cv::Mat K() const {
            cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                    0, fy, cy,
                    0, 0, 1);
            return K;
        };
        cv::Mat distCoef() const{
            cv::Mat distCoef = (cv::Mat_<double>(4, 1) << k1, k2, p1, p2);
            return distCoef;
        };

        float getCx() const { return cx; }
        float getCy() const { return cy; }
        float getFx() const { return fx; }
        float getFy() const { return fy; }

    private:
        float cx, cy, fx, fy;       // camera intrinsics
        float k1, k2, p1, p2;       // distortion
    };


}



#endif //IIQC_CAMERA_H
