#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

cv::Mat image;
std::vector<cv::Point> polygonPoints;
bool drawing = false;

double calculateVariance(const cv::Mat& src) {
    cv::Scalar mean, stddev;
    cv::meanStdDev(src, mean, stddev);
    return stddev.val[0] * stddev.val[0];
}

bool isRegionBlurred(const cv::Mat& image, const std::vector<cv::Point>& polygon, double threshold) {
    // Create a binary mask for the polygon region
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::fillConvexPoly(mask, polygon.data(), polygon.size(), 255);

    // Convert the image to grayscale
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

    // Extract the region of interest using the mask
    cv::Mat roi;
    grayImage.copyTo(roi, mask);

    // Calculate the variance of pixel intensities in the ROI
    double variance = calculateVariance(roi);

    // Compare the variance with the threshold
    return variance < threshold;
}

void drawPolygon() {
    if (polygonPoints.size() > 1) {
        // Draw the polygon on a copy of the original image (so the original image is not modified)
        cv::Mat imageCopy = image.clone();
        const cv::Point* pts = polygonPoints.data();
        int npts = static_cast<int>(polygonPoints.size());
        cv::polylines(imageCopy, &pts, &npts, 1, true, cv::Scalar(0, 255, 0), 2);

        cv::imshow("Image with Polygon", imageCopy);
    }
}

void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        drawing = true;
        polygonPoints.push_back(cv::Point(x, y));
        drawPolygon();
    } else if (event == cv::EVENT_LBUTTONUP) {
        drawing = false;
    } else if (event == cv::EVENT_MOUSEMOVE && drawing) {
        polygonPoints.push_back(cv::Point(x, y));
        drawPolygon();
    }
}

int main() {
    // Load the image
    std::string imagePath = "/home/feng/Code/catkin_ros/src/IIQC/data/images/train_001.JPG";
    image = cv::imread(imagePath);

    if (image.empty()) {
        std::cerr << "Failed to load image." << std::endl;
        return 1;
    }

    cv::imshow("Image", image);

    // Set up the mouse callback to handle polygon drawing
    cv::setMouseCallback("Image", onMouse);

    // Wait for a key press
    cv::waitKey(0);

    // Set the threshold for detecting blur
    // You may need to adjust this value based on your images and requirements
    double threshold = 100.0;

    // Check if the region is blurred
    bool result = isRegionBlurred(image, polygonPoints, threshold);

    if (result) {
        std::cout << "The region is blurred." << std::endl;
    } else {
        std::cout << "The region is sharp." << std::endl;
    }

    cv::destroyAllWindows();

    return 0;
}
