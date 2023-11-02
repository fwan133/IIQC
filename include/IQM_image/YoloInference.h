#pragma once
#ifndef INFERENCE_H
#define INFERENCE_H


// Cpp native
#include <fstream>
#include <vector>
#include <string>
#include <random>

// OpenCV / DNN / Inference
#include "opencv2/opencv.hpp"
//#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>



struct Detection
{
    int class_id{0};
    std::string className{};
    float confidence{0.0};
    cv::Scalar color{};
    cv::Rect box{};

    cv::Mat boxMask;    //mask in the box, accelerate and save space
};

class SemanticPoint {

public:
    SemanticPoint(const cv::Point2d& point, const int& class_id) : x(point.x), y(point.y), id(class_id){}
    SemanticPoint(const int& x, const int& y, const int& class_id) : x(x), y(y), id(class_id){}

    int x = -1;
    int y = -1;
    int id = -1;

};

class TargetContainer {

public:
    TargetContainer(const cv::MatSize size){
        mvSemanticPoints.reserve(size[0] * size[1]);
    }

    void Add(const SemanticPoint& point){
        std::pair<int, int> temp_pair(point.x, point.y);
        mvSemanticPoints.push_back(point);
        SemanticPoint* pp = &(mvSemanticPoints.back());
        mmReverseIndex.insert(std::make_pair(temp_pair, pp));
    }

    bool Find(const int x, const int y){
        return mmReverseIndex.count(std::make_pair(x, y));
    }

    std::vector<SemanticPoint> GetVector(){
        return mvSemanticPoints;
    }

    friend std::ostream & operator<<(std::ostream& out, const TargetContainer& point_container);

private:
    std::map<std::pair<int, int>, SemanticPoint*> mmReverseIndex; // for quick searching
    std::vector<SemanticPoint> mvSemanticPoints;
};




class Inference
{
public:
    Inference(const std::string &onnxModelPath, const cv::Size &modelInputShape = {640, 640}, const std::string &classesTxtFile = "", const bool &runWithCuda = true);
    std::vector<Detection> runInference(const cv::Mat &input, cv::Mat &mask);
    void GetMask(const cv::Mat& maskProposals, const cv::Mat& output02, Detection& detects, const cv::Vec4d& params, const cv::MatSize& sz);

    bool CheckParams(int netHeight, int netWidth, const int* netStride, int strideSize);
    
    std::vector<std::string> GetClasses(){
        return classes;
    }
    

    //void GetMask(const cv::Mat& maskProposals, const cv::Mat& maskProtos, std::vector<OutputSeg>& output, const MaskParams& maskParams);
    //void GetMask(const cv::Mat& maskProposals, const cv::Mat& maskProtos, OutputSeg& output, const MaskParams& maskParams);

private:
    void loadClassesFromFile();
    void loadOnnxNetwork();


    // convert input image to fit in the size of 640 * 640;
    void LetterBox(const cv::Mat& image, cv::Mat& outImage,
        cv::Vec4d& params, //[ratio_x,ratio_y,dw,dh]
        const cv::Size& newShape = cv::Size(640, 640),
        bool autoShape = false,
        bool scaleFill = false,
        bool scaleUp = true,
        int stride = 32,
        const cv::Scalar& color = cv::Scalar(114, 114, 114));

    cv::Mat formatToSquare(const cv::Mat &source);
    //void Preprocessing(const cv::Mat& srcImg, cv::Mat& outImg, cv::Vec4d& params);
    

    std::string modelPath{};
    std::string classesPath{};
    bool cudaEnabled{};

    //std::vector<std::string> classes{"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};
    std::vector<std::string> classes{"bridge"};

    cv::Size2f modelShape{}; // 640*640

    float modelConfidenceThreshold {0.25};
    //float modelScoreThreshold      {0.45};
    float modelScoreThreshold      {0.70};
    float modelNMSThreshold        {0.50};

    bool letterBoxForSquare = true;

    int mnSegChannels = 32;
    int mnSegWidth = 160;
    int mnSegHeight = 160;
    int mnNetWidth = 640;
    int mnNetHeight = 640;
    float mfMaskThreshold = 0.5;

    cv::dnn::Net net;
};

#include "IQM_image/YoloInference.cpp"

#endif