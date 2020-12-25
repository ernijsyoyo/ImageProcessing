#include <iostream>
#include <stdio.h>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <Utilities.hpp>
#include <Algorithms.hpp>

int main(int argc, char** argv )
{
    std::cout << "Program startup initiated" << std::endl;

    // Start the webcam capping
    cv::VideoCapture vidCap;
    if (!vidCap.open(0))
    {
        std::cout << "returning " << std::endl;
        return 0;
    }

	// Placeholders for frames
    cv::Mat input;
    cv::Mat prev_input;
    cv::Mat output;
    cv::namedWindow("Gray", cv::WINDOW_AUTOSIZE);

    // Control buttons
    auto userInput = std::make_shared<float>(0.92f);
    auto algorithmSwitch = std::make_shared<int>(1);

    // Algorithms
    auto thresholdStrategy = new ThresholdStrategy;
    auto motionStrategy = new MotionStrategy;
    auto exampleStrategy = new ExampleStrategy;
    auto lowPass = new LowpassFilter;
    auto simpleConvolution = new SimpleConvolution;
    auto sobelEdgeDetection= new SobelEdgeDetection;
    auto morphOps= new MorphologicalOperations;
    auto medianFilter= new MedianFilter;
    auto adaptiveThreshold= new AdaptiveThreshold;;
    Context *context = new Context(adaptiveThreshold);
    
    std::cout << "Stream starting" << std::endl;
    while (true) {
        // Convert webcam to grayscale
        vidCap >> input;
        cv::cvtColor(input, input, cv::COLOR_BGR2GRAY); // conv color to gray

        // Process strategy and display result
        auto processedInput = context->ProcessStrategy(input , userInput);
        

        if( processedInput.empty() ) break; // end of video stream

        // Show original and processed side by side
        cv::imshow("Gray", processedInput);
        
        // Algorithm control and loop break
        if (cv::waitKey(5) == 113) break; // stop capturing by pressing ESC
        if (cv::waitKey(5) == 48) context->set_strategy(exampleStrategy); // key 0
        if (cv::waitKey(5) == 49) context->set_strategy(thresholdStrategy); // key 1
        if (cv::waitKey(5) == 50) context->set_strategy(motionStrategy); // key 2
        if (cv::waitKey(5) == 51) context->set_strategy(lowPass); // key 3
        if (cv::waitKey(5) == 52) context->set_strategy(simpleConvolution); // key 4
        if (cv::waitKey(5) == 53) context->set_strategy(sobelEdgeDetection); // key 5
        if (cv::waitKey(5) == 54) context->set_strategy(morphOps); // key 6
        if (cv::waitKey(5) == 55) context->set_strategy(medianFilter); // key 7
        if (cv::waitKey(5) == 56) context->set_strategy(adaptiveThreshold); // key 8
        if (cv::waitKey(5) == 120) Utilities::IncreaseFloat(userInput); // key z
        if (cv::waitKey(5) == 122) Utilities::DecreaseFloat(userInput); // key x
    }
    
    vidCap.release();
    cv::destroyAllWindows();
    return 0;
}
