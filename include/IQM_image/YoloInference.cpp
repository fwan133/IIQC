#include "IQM_image/YoloInference.h"

using namespace cv;
using namespace std;

std::ostream& operator<< (std::ostream& os, TargetContainer& point_container) {
    std::vector<SemanticPoint> v = point_container.GetVector();
    for(std::vector<SemanticPoint>::iterator it = v.begin(); it != v.end(); it++){
        os << "[" << it->x << "," << it->y << "," << it->id << "]" << std::endl;
    }
    return os;
} 

Inference::Inference(const std::string &onnxModelPath, const cv::Size &modelInputShape, const std::string &classesTxtFile, const bool &runWithCuda)
{
    modelPath = onnxModelPath;
    modelShape = modelInputShape;
    classesPath = classesTxtFile;
    cudaEnabled = runWithCuda;

    loadOnnxNetwork();
    // loadClassesFromFile(); The classes are hard-coded for this example
}

std::vector<Detection> Inference::runInference(const cv::Mat &input, cv::Mat &mask)
{
    /* cv::Mat modelInput = input;
    if (letterBoxForSquare && modelShape.width == modelShape.height)
        modelInput = formatToSquare(modelInput); */

    // do we need to resize the input to be fit with 640 * 640 using Preprocessing()?
    cv::Vec4d params;
    cv::Mat modelInput;
    LetterBox(input, modelInput, params, modelShape);

    cv::Mat blob;
    cv::dnn::blobFromImage(modelInput, blob, 1.0/255.0, modelShape, cv::Scalar(), true, false);
    net.setInput(blob);

    //std::cout << "blob dimension: " << blob.dims << std::endl;

    // loop over the dimensions of blob
    /* for(int i = 0; i < blob.dims; i++){
        std::cout << "size of each dimension in blob: " << blob.size[i] << std::endl;
    } */
    //std::cout << "size of each dimension in blob: " << blob.size << std::endl;

    /* for(int i = 0; i < blob.size[0]; i++){
        // loop over images
        //const int size[3] = {blob.size[2], blob.size[3], 3};
        //cv::Mat img(3, size, CV_32FC(3), blob.ptr<float>(i));
        std::vector<cv::Mat> channels;
        for (int c = 0; c < blob.size[1]; c++){
            cv::Mat img(blob.size[2], blob.size[3], CV_32F, blob.ptr<float>(i, c));
            //cv::imshow("img", img);
            //cv::waitKey(-1);
            //std::cout << "blob-img channel: " << img.channels() << " dimensions: " << img.dims << " rows: " << img.rows << " cols: " << img.cols << std::endl;
            //std::cout << "blob-img size[0]: " << img.size[0] << " blob-img size[1]: " << img.size[1] << std::endl;
            //std::cout << "blob-img depth: " << img.depth() << std::endl; // is actually cv_32f
            channels.push_back(img);
        }
        cv::Mat out;
        cv::merge(channels, out);
        //cv::Mat img(blob.size[2], blob.size[3], CV_32FC3, blob.ptr<float>(i,0));
        //cv::imshow("img", out);
        //cv::waitKey(-1);
    } */

    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    int rows = outputs[0].size[1];
    int dimensions = outputs[0].size[2];

    //std::cout << "output debug----------------------" << std::endl;
    //std::cout << "output size: " << outputs.size() << std::endl;
    //std::cout << "output dimension: " << outputs[0].dims << std::endl;
    //std::cout << "output batchSize: " << outputs[0].size[0] << std::endl;
    //std::cout << "class nums: " << classes.size() << std::endl;

    bool yolov8 = false;
    // yolov5 has an output of shape (batchSize, 25200, 85) (Num classes + box[x,y,w,h] + confidence[c])
    // yolov8 has an output of shape (batchSize, 84,  8400) (Num classes + box[x,y,w,h])
    if (dimensions > rows) // Check if the shape[2] is more than shape[1] (yolov8)
    {
        yolov8 = true;
        rows = outputs[0].size[2];
        dimensions = outputs[0].size[1]; // should be 4 + 80 + 32 = 116

        outputs[0] = outputs[0].reshape(1, dimensions);
        cv::transpose(outputs[0], outputs[0]);
    }
    float *data = (float *)outputs[0].data;

    //std::cout << "new output dimensions: " << outputs[0].dims << std::endl;
    //std::cout << "new output rows: " << outputs[0].rows << std::endl; // 8400
    //std::cout << "new output cols: " << outputs[0].cols << std::endl; // 116

    //float x_factor = modelInput.cols / modelShape.width;
    //float y_factor = modelInput.rows / modelShape.height;

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<float>> picked_proposals; // vector for segmentation; should be nms later

    for (int i = 0; i < rows; ++i)
    {
        if (yolov8)
        {
            /* float *classes_scores = data+4;

            cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double maxClassScore;

            minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

            //int net_width = classes.size() + 4 + _segChannels;

            if (maxClassScore > modelScoreThreshold)
            {
                confidences.push_back(maxClassScore);
                class_ids.push_back(class_id.x);
                
                //std::cout << "class id: " << class_id << std::endl;
                // get the proto of the mask
                std::vector<float> temp_proto(data + 4 + classes.size(), data + dimensions);
                picked_proposals.push_back(temp_proto);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];

                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);

                int width = int(w * x_factor);
                int height = int(h * y_factor);

                boxes.push_back(cv::Rect(left, top, width, height));
            } */


            float *classes_scores = data+4;

            cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double maxClassScore;

            minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

            //int net_width = classes.size() + 4 + _segChannels;

            if (maxClassScore > modelScoreThreshold && (class_id.x == 2 || class_id.x == 0 || class_id.x == 1))
            {
                confidences.push_back(maxClassScore);
                class_ids.push_back(class_id.x);
                
                //std::cout << "class id: " << class_id << std::endl;
                // get the proto of the mask
                std::vector<float> temp_proto(data + 4 + classes.size(), data + dimensions);
                picked_proposals.push_back(temp_proto);

                float x = (data[0] - params[2]) / params[0];
                float y = (data[1] - params[3]) / params[1];
                float w = data[2] / params[0];
                float h = data[3] / params[1];

                /* int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);

                int width = int(w * x_factor);
                int height = int(h * y_factor);

                boxes.push_back(cv::Rect(left, top, width, height)); */
                int left = MAX(int(x - 0.5 * w + 0.5), 0);
                int top = MAX(int(y - 0.5 * h + 0.5), 0);
                boxes.push_back(Rect(left, top, int(w + 0.5), int(h + 0.5)));

            }


        }
        else // yolov5
        {
            /* float confidence = data[4];

            if (confidence >= modelConfidenceThreshold)
            {
                float *classes_scores = data+5;

                cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
                cv::Point class_id;
                double max_class_score;

                minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

                if (max_class_score > modelScoreThreshold)
                {
                    confidences.push_back(confidence);
                    class_ids.push_back(class_id.x);

                    float x = data[0];
                    float y = data[1];
                    float w = data[2];
                    float h = data[3];

                    int left = int((x - 0.5 * w) * x_factor);
                    int top = int((y - 0.5 * h) * y_factor);

                    int width = int(w * x_factor);
                    int height = int(h * y_factor);

                    boxes.push_back(cv::Rect(left, top, width, height));
                }
            } */
        }

        data += dimensions;
    }

    //std::vector<std::vector<float>> temp_mask_proposals;
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

    cv::Rect holeImgRect(0, 0, input.cols, input.rows);

    std::vector<Detection> detections{};
    cv::Mat result_mask = cv::Mat::zeros(input.size(), 0);

    for (unsigned long i = 0; i < nms_result.size(); ++i)
    {
        int idx = nms_result[i];

        Detection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(100, 255);
        result.color = cv::Scalar(dis(gen),
                                  dis(gen),
                                  dis(gen));

        result.className = classes[result.class_id];
        result.box = boxes[idx] & holeImgRect;
        //temp_mask_proposals.push_back(picked_proposals[idx]);

        cv::Mat temp_mask(picked_proposals[idx]);
        GetMask(temp_mask.t(), outputs[1], result, params, input.size);

        detections.push_back(result);

        result_mask = result_mask | result.boxMask;
    }

    mask = result_mask;
    /* for(int i = 0; i < temp_mask_proposals.size(); i++){
        cv::Mat temp_mask(temp_mask_proposals[i]);
        GetMask(temp_mask.t(), outputs[1], detections[i], params, input.size);
    } */

    return detections;
}

void Inference::loadClassesFromFile()
{
    std::ifstream inputFile(classesPath);
    if (inputFile.is_open())
    {
        std::string classLine;
        while (std::getline(inputFile, classLine))
            classes.push_back(classLine);
        inputFile.close();
    }
}

void Inference::loadOnnxNetwork()
{
    net = cv::dnn::readNetFromONNX(modelPath);
    if (cudaEnabled)
    {
        std::cout << "\nRunning on CUDA" << std::endl;
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    }
    else
    {
        std::cout << "\nRunning on CPU" << std::endl;
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
}

cv::Mat Inference::formatToSquare(const cv::Mat &source)
{
    int col = source.cols;
    int row = source.rows;
    int _max = MAX(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
    return result;
}

/* void Inference::Preprocessing(const cv::Mat& srcImg, cv::Mat& outImg, cv::Vec4d& params){
    
    cv::Size2f input_size = modelShape;
    
    Mat temp_img = srcImg;
    Vec4d temp_param = { 1,1,0,0 };
    if (temp_img.size() != input_size) {
        Mat borderImg;
        LetterBox(temp_img, borderImg, temp_param, input_size, false, false, true, 32);
        
        outImg = borderImg;
        params = temp_param;
        //outSrcImgs.push_back(borderImg);
        //params.push_back(temp_param);
    }
    else {
        outImg = borderImg;
        params = temp_param;
        //outSrcImgs.push_back(temp_img);
        //params.push_back(temp_param);
    }

} */


bool CheckParams(int netHeight, int netWidth, const int* netStride, int strideSize) {
    if (netHeight % netStride[strideSize - 1] != 0 || netWidth % netStride[strideSize - 1] != 0)
    {
        cout << "Error:_netHeight and _netWidth must be multiple of max stride " << netStride[strideSize - 1] << "!" << endl;
        return false;
    }
    return true;
}
 
void Inference::LetterBox(const cv::Mat& image, cv::Mat& outImage, cv::Vec4d& params, const cv::Size& newShape,
    bool autoShape, bool scaleFill, bool scaleUp, int stride, const cv::Scalar& color)
{
    if (false) {
        int maxLen = MAX(image.rows, image.cols);
        outImage = Mat::zeros(Size(maxLen, maxLen), CV_8UC3);
        image.copyTo(outImage(Rect(0, 0, image.cols, image.rows)));
        params[0] = 1;
        params[1] = 1;
        params[3] = 0;
        params[2] = 0;
    }
 
    cv::Size shape = image.size();
    float r = std::min((float)newShape.height / (float)shape.height,
        (float)newShape.width / (float)shape.width);
    if (!scaleUp)
        r = std::min(r, 1.0f);
 
    float ratio[2]{ r, r };
    int new_un_pad[2] = { (int)std::round((float)shape.width * r),(int)std::round((float)shape.height * r) };
 
    auto dw = (float)(newShape.width - new_un_pad[0]);
    auto dh = (float)(newShape.height - new_un_pad[1]);
 
    if (autoShape)
    {
        dw = (float)((int)dw % stride);
        dh = (float)((int)dh % stride);
    }
    else if (scaleFill)
    {
        dw = 0.0f;
        dh = 0.0f;
        new_un_pad[0] = newShape.width;
        new_un_pad[1] = newShape.height;
        ratio[0] = (float)newShape.width / (float)shape.width;
        ratio[1] = (float)newShape.height / (float)shape.height;
    }
 
    dw /= 2.0f;
    dh /= 2.0f;
 
    if (shape.width != new_un_pad[0] && shape.height != new_un_pad[1])
    {
        cv::resize(image, outImage, cv::Size(new_un_pad[0], new_un_pad[1]));
    }
    else {
        outImage = image.clone();
    }
 
    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));
    params[0] = ratio[0];
    params[1] = ratio[1];
    params[2] = left;
    params[3] = top;
    cv::copyMakeBorder(outImage, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, color);
}

void Inference::GetMask(const cv::Mat& maskProposals, const cv::Mat& output02, Detection& detects, const cv::Vec4d& params, const cv::MatSize& sz){
    //cv::Mat protos = output02.reshape(0, { mnSegChannels, mnSegWidth * mnSegHeight });
    int seg_channels = mnSegChannels;
    int net_width = mnNetWidth;
    int seg_width = mnSegWidth;
    int net_height = mnNetHeight;
    int seg_height = mnSegHeight;
    float mask_threshold = mfMaskThreshold;
    //Vec4f params = maskParams.params;
    //Size src_img_shape = maskParams.srcImgShape;
 
    Rect temp_rect = detects.box;
    //crop from mask_protos
    int rang_x = floor((temp_rect.x * params[0] + params[2]) / net_width * seg_width);
    int rang_y = floor((temp_rect.y * params[1] + params[3]) / net_height * seg_height);
    int rang_w = ceil(((temp_rect.x + temp_rect.width) * params[0] + params[2]) / net_width * seg_width) - rang_x;
    int rang_h = ceil(((temp_rect.y + temp_rect.height) * params[1] + params[3]) / net_height * seg_height) - rang_y;
 
    
    rang_w = MAX(rang_w, 1);
    rang_h = MAX(rang_h, 1);
    if (rang_x + rang_w > seg_width) {
        if (seg_width - rang_x > 0)
            rang_w = seg_width - rang_x;
        else
            rang_x -= 1;
    }
    if (rang_y + rang_h > seg_height) {
        if (seg_height - rang_y > 0)
            rang_h = seg_height - rang_y;
        else
            rang_y -= 1;
    }
 
    vector<Range> roi_rangs;
    roi_rangs.push_back(Range(0, 1));
    roi_rangs.push_back(Range::all());
    roi_rangs.push_back(Range(rang_y, rang_h + rang_y));
    roi_rangs.push_back(Range(rang_x, rang_w + rang_x));
 
    //crop
    Mat temp_mask_protos = output02(roi_rangs).clone();
    Mat protos = temp_mask_protos.reshape(0, { seg_channels,rang_w * rang_h });
    Mat matmul_res = (maskProposals * protos).t();
    Mat masks_feature = matmul_res.reshape(1, { rang_h,rang_w });
    Mat dest, mask;
 
    //sigmoid
    cv::exp(-masks_feature, dest);
    dest = 1.0 / (1.0 + dest);
 
    int left = max(0, static_cast<int>(floor((net_width / seg_width * rang_x - params[2]) / params[0])));
    int top = max(0, static_cast<int>(floor((net_height / seg_height * rang_y - params[3]) / params[1])));
    int width = ceil(net_width / seg_width * rang_w / params[0]);
    int height = ceil(net_height / seg_height * rang_h / params[1]);
 
    cv::resize(dest, mask, Size(width, height), INTER_NEAREST);
    mask = mask(temp_rect - Point(left, top)) > mask_threshold;

    //std::cout << "Mask top left: " << temp_rect.tl() << " and " << Point(left, top) << " channel: " << mask.channels() << " size: " << mask.size << std::endl; 

    cv::copyMakeBorder(mask, detects.boxMask, top, max(sz[0] - top - mask.rows, 0), left, max(sz[1] - left - mask.cols, 0), cv::BORDER_CONSTANT, 0);

    //detects.boxMask = mask;

    //extract all the pixel with value == 255
    //int h = mask.rows;
    //int w = mask.cols;
    /* if(mask.isContinuous()){
        h = 1;
        w = w * mask.rows * mask.channels();
    } */
    /* for(int i = 0; i < h; i++){
        uchar* pt = mask.ptr<uchar>(i);
        for(int j = 0; j < w; j++){
            if(*pt > 0){
                SemanticPoint semPoint(j + left, i + top, detects.class_id);
                pTargetPoints->Add(semPoint);
            }
            pt++;
        }
    } */

}
 
/* void GetMask(const cv::Mat& maskProposals, const cv::Mat& maskProtos, std::vector<OutputSeg>& output, const MaskParams& maskParams) {
    //cout << maskProtos.size << endl;
 
    int seg_channels = maskParams.segChannels;
    int net_width = maskParams.netWidth;
    int seg_width = maskParams.segWidth;
    int net_height = maskParams.netHeight;
    int seg_height = maskParams.segHeight;
    float mask_threshold = maskParams.maskThreshold;
    Vec4f params = maskParams.params;
    Size src_img_shape = maskParams.srcImgShape;
 
    Mat protos = maskProtos.reshape(0, { seg_channels,seg_width * seg_height });
 
    Mat matmul_res = (maskProposals * protos).t();
    Mat masks = matmul_res.reshape(output.size(), { seg_width,seg_height });
    vector<Mat> maskChannels;
    split(masks, maskChannels);
    for (int i = 0; i < output.size(); ++i) {
        Mat dest, mask;
        //sigmoid
        cv::exp(-maskChannels[i], dest);
        dest = 1.0 / (1.0 + dest);
 
        Rect roi(int(params[2] / net_width * seg_width), int(params[3] / net_height * seg_height), int(seg_width - params[2] / 2), int(seg_height - params[3] / 2));
        dest = dest(roi);
        resize(dest, mask, src_img_shape, INTER_NEAREST);
 
        //crop
        Rect temp_rect = output[i].box;
        mask = mask(temp_rect) > mask_threshold;
        output[i].boxMask = mask;
    }
}
 
void GetMask2(const Mat& maskProposals, const Mat& mask_protos, OutputSeg& output, const MaskParams& maskParams) {
    int seg_channels = maskParams.segChannels;
    int net_width = maskParams.netWidth;
    int seg_width = maskParams.segWidth;
    int net_height = maskParams.netHeight;
    int seg_height = maskParams.segHeight;
    float mask_threshold = maskParams.maskThreshold;
    Vec4f params = maskParams.params;
    Size src_img_shape = maskParams.srcImgShape;
 
    Rect temp_rect = output.box;
    //crop from mask_protos
    int rang_x = floor((temp_rect.x * params[0] + params[2]) / net_width * seg_width);
    int rang_y = floor((temp_rect.y * params[1] + params[3]) / net_height * seg_height);
    int rang_w = ceil(((temp_rect.x + temp_rect.width) * params[0] + params[2]) / net_width * seg_width) - rang_x;
    int rang_h = ceil(((temp_rect.y + temp_rect.height) * params[1] + params[3]) / net_height * seg_height) - rang_y;
 
    //如果下面的 mask_protos(roi_rangs).clone()位置报错，说明你的output.box数据不对，或者矩形框就1个像素的，开启下面的注释部分防止报错。
    rang_w = MAX(rang_w, 1);
    rang_h = MAX(rang_h, 1);
    if (rang_x + rang_w > seg_width) {
        if (seg_width - rang_x > 0)
            rang_w = seg_width - rang_x;
        else
            rang_x -= 1;
    }
    if (rang_y + rang_h > seg_height) {
        if (seg_height - rang_y > 0)
            rang_h = seg_height - rang_y;
        else
            rang_y -= 1;
    }
 
    vector<Range> roi_rangs;
    roi_rangs.push_back(Range(0, 1));
    roi_rangs.push_back(Range::all());
    roi_rangs.push_back(Range(rang_y, rang_h + rang_y));
    roi_rangs.push_back(Range(rang_x, rang_w + rang_x));
 
    //crop
    Mat temp_mask_protos = mask_protos(roi_rangs).clone();
    Mat protos = temp_mask_protos.reshape(0, { seg_channels,rang_w * rang_h });
    Mat matmul_res = (maskProposals * protos).t();
    Mat masks_feature = matmul_res.reshape(1, { rang_h,rang_w });
    Mat dest, mask;
 
    //sigmoid
    cv::exp(-masks_feature, dest);
    dest = 1.0 / (1.0 + dest);
 
    int left = floor((net_width / seg_width * rang_x - params[2]) / params[0]);
    int top = floor((net_height / seg_height * rang_y - params[3]) / params[1]);
    int width = ceil(net_width / seg_width * rang_w / params[0]);
    int height = ceil(net_height / seg_height * rang_h / params[1]);
 
    resize(dest, mask, Size(width, height), INTER_NEAREST);
    mask = mask(temp_rect - Point(left, top)) > mask_threshold;
    output.boxMask = mask;
 
} */
 
/* void DrawPred(Mat& img, vector<Detection> result, std::vector<std::string> classNames, vector<Scalar> color) {
    Mat mask = img.clone();
    for (int i = 0; i < result.size(); i++) {
        int left, top;
        left = result[i].box.x;
        top = result[i].box.y;
        int color_num = i;
        rectangle(img, result[i].box, color[result[i].class_id], 2, 8);
        if(result[i].boxMask.rows&& result[i].boxMask.cols>0)
            mask(result[i].box).setTo(color[result[i].class_id], result[i].boxMask);
        string label = classNames[result[i].class_id] + ":" + to_string(result[i].confidence);
        int baseLine;
        Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        top = max(top, labelSize.height);
        //rectangle(frame, Point(left, top - int(1.5 * labelSize.height)), Point(left + int(1.5 * labelSize.width), top + baseLine), Scalar(0, 255, 0), FILLED);
        putText(img, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 1, color[result[i].class_id], 2);
    }
    addWeighted(img, 0.5, mask, 0.5, 0, img); //add mask to src
//    imshow("1", img);
    //imwrite("out.bmp", img);
//    waitKey();
    //destroyAllWindows();
 
} */
