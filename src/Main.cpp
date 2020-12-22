#include <iostream>
#include <stdio.h>
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
    cv::namedWindow("Gray", cv::WINDOW_AUTOSIZE);

    // Control buttons
    auto userInput = std::make_shared<float>(0.25f);
    auto algorithmSwitch = std::make_shared<int>(1);

    // Algorithms
    auto thresholdStrategy = new ThresholdStrategy;
    auto motionStrategy = new MotionStrategy;
    auto exampleStrategy = new ExampleStrategy;
    auto lowPass = new LowpassFilter;
    auto simpleConvolution = new SimpleConvolution;
    Context *context = new Context(simpleConvolution);
    
    std::cout << "Stream starting" << std::endl;
    while (true) {
        // Convert webcam to grayscale
        vidCap >> input;
        cv::cvtColor(input, input, cv::COLOR_BGR2GRAY); // conv color to gray

        // Process strategy and display result
        auto output = context->ProcessStrategy(input, userInput);
        if( output.empty() ) break; // end of video stream
        cv::imshow("Gray", output);
        
        // Algorithm control and loop break
        if (cv::waitKey(5) == 113) break; // stop capturing by pressing ESC
        if (cv::waitKey(5) == 48) context->set_strategy(exampleStrategy); // key 0
        if (cv::waitKey(5) == 49) context->set_strategy(thresholdStrategy); // key 1
        if (cv::waitKey(5) == 50) context->set_strategy(motionStrategy); // key 2
        if (cv::waitKey(5) == 51) context->set_strategy(lowPass); // key 3
        if (cv::waitKey(5) == 52) context->set_strategy(simpleConvolution); // key 4
        if (cv::waitKey(5) == 120) Utilities::IncreaseFloat(userInput); // key z
        if (cv::waitKey(5) == 122) Utilities::DecreaseFloat(userInput); // key x
    }
    
    vidCap.release();
    cv::destroyAllWindows();
    return 0;
}
