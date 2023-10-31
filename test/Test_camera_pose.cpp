#include <iostream>

#include <pangolin/pangolin.h>

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

int main(int argc, char **argv) {
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

    Eigen::Isometry3d T_w_c = Eigen::Isometry3d::Identity();
    Eigen::Vector3d t(0, 0, 1);
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 0, 1, 0,
            0, 0, -1,
            -1, 0, 0;
    Eigen::Matrix3d inverse_matrix = rotation_matrix.inverse();

    T_w_c.prerotate(inverse_matrix);
    T_w_c.pretranslate(t);

    while (!pangolin::ShouldQuit()) {
        // Clear the buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        // Draw global coordinate
        drawCoordinate();
        // Draw all frames
        drawCameraFrame(T_w_c);

        /// Finish this frame
        pangolin::FinishFrame();
    }
}
