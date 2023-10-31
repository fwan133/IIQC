
#ifndef IIQC_VIEWER_H
#define IIQC_VIEWER_H

#include <memory>
#include <mutex>

#include <pangolin/pangolin.h>

#include "common.h"
#include "SfM/map.h"

class Viewer {
public:
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer() {}

    void setParameters(float map_scale, float camera_scale, float point_scale, int window_width, int window_height, float viewpoint_x, float viewpoint_y, float viewpoint_z) {
        Viewer::map_scale = map_scale;
        Viewer::camera_scale = camera_scale;
        Viewer::point_scale = point_scale;
        Viewer::window_width = window_width;
        Viewer::window_height = window_height;
        Viewer::mViewpointX = viewpoint_x;
        Viewer::mViewpointY = viewpoint_y;
        Viewer::mViewpointZ = viewpoint_z;
    }

    void drawMap(sfm::Map::Ptr map) {
        // Create a new window
        pangolin::CreateWindowAndBind(window_title, window_width, window_height);
        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);
        // Issue specific OpenGl
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // Define Camera Render Object (for view)
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(window_width, window_height, mViewpointF, mViewpointF, window_width / 2,
                                           window_height / 2, 0.1f, 1000),
                pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, 0.0, 1.0));
        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));

        while (!pangolin::ShouldQuit()) {
            // Clear the buffer
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);
            // Change the background
            glClearColor(0.95f, 0.95f, 0.95f, 1.0f);
            // Draw global coordinate
            drawCoordinate();
            // Draw all frames
            drawFrames(map->getAllFrames());
            // Draw all mappoints
            drawMappoint(map->getValidMapPoints());

            /// Finish this frame
            pangolin::FinishFrame();
        }
    }

    void drawCameraPose(Eigen::Isometry3d &T_w_c) {
        // Create a new window
        pangolin::CreateWindowAndBind(window_title, window_width, window_height);
        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);
        // Issue specific OpenGl
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // Define Camera Render Object (for view)
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(window_width, window_height, mViewpointF, mViewpointF, window_width / 2,
                                           window_height / 2, 0.1f, 1000),
                pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, 0.0, 1.0));
        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));

        while (!pangolin::ShouldQuit()) {
            // Clear the buffer
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);
            // Change the background
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            // Draw global coordinate
            drawCoordinate();
            // Draw all frames
            drawCameraFrame(T_w_c);

            /// Finish this frame
            pangolin::FinishFrame();
        }
    }

protected:
    void drawCoordinate() {
        glLineWidth(3);
        glBegin(GL_LINES);
        //x
        glColor3f(1.0, 0.0, 0.0);
        glVertex3d(0, 0, 0);
        glVertex3d(map_scale, 0, 0);
        //y
        glColor3f(0.0, 1.0, 0.0);
        glVertex3d(0, 0, 0);
        glVertex3d(0, map_scale, 0);
        //y
        glColor3f(0.0, 0.0, 1.0);
        glVertex3d(0, 0, 0);
        glVertex3d(0, 0, map_scale);
        glEnd();
    }

    void drawFrames(std::unordered_map<long, sfm::Frame::Ptr> frames){
        for (auto &id_frame : frames){
            sfm::Frame::Ptr frame = id_frame.second;
            if (frame->isRegistered()){
                drawCameraFrame(frame->getTwc_est());
            }
        }
    }

    void drawMappoint(std::unordered_map<long, sfm::MapPoint::Ptr> mappoints){
        for (auto &id_mappoint:mappoints){
            sfm::MapPoint::Ptr mappoint = id_mappoint.second;
            drawPoint(mappoint->getPos()[0],mappoint->getPos()[1],mappoint->getPos()[2], mappoint->getColor()[0], mappoint->getColor()[1], mappoint->getColor()[2]);
        }
    }

    void drawCameraFrame(Eigen::Isometry3d frame_pose) {
        // Define parameters
        float camera_linewidth = 2.0f;
        float camera_size = 0.5f * camera_scale;
        float alxe_length = 0.4f * camera_scale;

        glPushMatrix();
        glMultMatrixd(frame_pose.matrix().data());

        // Draw the camera body
        const float w = camera_size;
        const float h = 0.75 * camera_size;
        const float z = 0.75 * camera_size;

        glLineWidth(camera_linewidth);
        glBegin(GL_LINES);

        // Draw camera shape
//        glColor3f(151/255.0, 172/255.0, 164/255.0);
        glColor3f(0.0, 1.0, 1.0);
        glVertex3d(0, 0, 0);
        glVertex3d(w, h, z);
        glVertex3d(0, 0, 0);
        glVertex3d(w, -h, z);
        glVertex3d(0, 0, 0);
        glVertex3d(-w, -h, z);
        glVertex3d(0, 0, 0);
        glVertex3d(-w, h, z);
        glVertex3d(w, h, z);
        glVertex3d(w, -h, z);
        glVertex3d(-w, h, z);
        glVertex3d(-w, -h, z);
        glVertex3d(-w, h, z);
        glVertex3d(w, h, z);
        glVertex3d(-w, -h, z);
        glVertex3d(w, -h, z);

        // Draw camera coordinate
        //x
        glColor3f(1.0, 0.0, 0.0);
        glVertex3d(0, 0, 0);
        glVertex3d(alxe_length, 0, 0);
        //y
        glColor3f(0.0, 1.0, 0.0);
        glVertex3d(0, 0, 0);
        glVertex3d(0, alxe_length, 0);
        //y
        glColor3f(0.0, 0.0, 1.0);
        glVertex3d(0, 0, 0);
        glVertex3d(0, 0, alxe_length);

        glEnd();
        glPopMatrix();
    }

    void drawPoint(float x, float y, float z, uint r, uint g, uint b){
        glPointSize(point_scale * 5.0f);
        glBegin(GL_POINTS);
        glColor3f(r/255.0, g/255.0, b/255.0);
        glVertex3f(x, y, z);
        glEnd();
    }

private:
    // Foundamentatal Parameters
    float map_scale = 1;
    float camera_scale = 1;
    float point_scale = 1;
    std::string window_title = "Viewer";
    int window_width = 1024;
    int window_height = 768;
    float mViewpointX = 10.0;
    float mViewpointY = -10.0;
    float mViewpointZ = 10.0;
    float mViewpointF = 500.0f;

    // Visibility Parameters
};


#endif //IIQC_VIEWER_H
