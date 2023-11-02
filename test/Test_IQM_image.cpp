#include "IQM_image/IQM_image.h"

int main(int argc, char **argv) {
    // Load the octomap
    std::string filename = "/home/feng/Code/catkin_ros/src/IIQC/data/CapturedData/Simulation/GirderBridge/girder_bridge_pier/3_colour.jpg";
    std::string model_path = "/home/feng/Code/catkin_ros/src/IIQC/data/segmentation_model/bridge_seg_m2.onnx";
    cv::Mat color_img = cv::imread(filename);

    IQMImage IQM_image(color_img, 1);
    IQM_image.extractROI(model_path);
    IQM_image.evaluateBPM(17);
    IQM_image.evaluateEIM(17);
    IQM_image.showColorImage(cv::Size(1200, 800));
    IQM_image.showGrayImage(cv::Size(1200, 800));
    IQM_image.showBPM(cv::Size(1200, 800));
    IQM_image.showEIM(cv::Size(1200, 800));

    return 0;
}
