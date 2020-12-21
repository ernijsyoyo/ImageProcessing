#include <memory>
#include <opencv2/opencv.hpp>
#pragma once

class Utilities {
private:

public:
    Utilities();
    ~Utilities();
    
    static void HelloWorld();
    /// Return the 
    static cv::Mat AddTwoFrames(cv::Mat frame1, cv::Mat frame2);
    // Get int value of pixel at position and convert to float
    static float GetPixelValue(cv::Mat frame, int x, int y);
    
    static void SetPixelValue(cv::Mat frame, int x, int y, float value);
    static void IncreaseInt(std::shared_ptr<float> input);
    static void DecreaseInt(std::shared_ptr<float> input);
};