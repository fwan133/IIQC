#include "flight_control/captured_frame_collection.h"

int main(int argc, char** argv){
    CapturedFrameCollection frame_collection("Pier","/home/feng/Code/catkin_ros/src/IIQC/data/CapturedData/ComplexGirderBridge/Pier");
    for (int index=1; index <=frame_collection.GetSize();index++){
        CapturedFrame frame = frame_collection.getFrameByIndex(index);
        cv::Mat resize_img;
        cv::resize(frame.GetColourImage(),resize_img,cv::Size(900,600),0,0);
        cv::imshow("Image", resize_img);
        cv::waitKey(500);
    }
    cv::destroyAllWindows();
    return 0;
}