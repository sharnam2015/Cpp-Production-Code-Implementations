#include <iostream>
#include <vector>
#include<cstdint>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>


//Running the OpenCV FAST feature detector on an image and storing the output image with drawn keypoints

int main()
{
    // Image path
    std::string image_path = "C:/Users/sharn/Documents/gitcode/repodata/cbimrn.jpg";

    // Converting to Grayscale
    
    cv::Mat color_img = cv::imread(image_path,cv::IMREAD_COLOR);
    if (color_img.empty()) {
        std::cerr << "Error: could not load image at '" << image_path << "'" << std::endl;
        return -1;
    }

    //Converting to gray scale for FAST
    cv::Mat gray;
    cvtColor(color_img, gray, cv::COLOR_BGR2GRAY);

    //FAST  parameters
    uint32_t threshold = 30U;
    bool nonmaxSuppression = true;
    auto detector = cv::FastFeatureDetector::create(
        threshold,
        nonmaxSuppression,
        cv::FastFeatureDetector::TYPE_9_16
    );

    //Detecting Keypoints
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(gray, keypoints);

    //Drawing keypoints on image
    cv::Mat img_with_keypoints;
    cv::drawKeypoints(
        color_img,
        keypoints,
        img_with_keypoints,
        cv::Scalar(255,0,0),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    );

    
    cv::imshow("FAST Keypoints", img_with_keypoints);
    cv::waitKey(0);
    cv::imwrite("fast_keypointscbimrn.jpg", img_with_keypoints);

    return 0;
}
