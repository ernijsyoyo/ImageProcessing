#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <Utilities.hpp>

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

    auto threshold = std::make_shared<float>(0.25f);
    
	// Placeholders for frames
    cv::Mat input;
    cv::Mat prev_input;

    std::cout << "Stream started" << std::endl;
    while (true) {
        prev_input = input;
        vidCap >> input;
        cv::cvtColor(input, input, cv::COLOR_BGR2GRAY); // conv color to gray

        for (size_t i = 1; i < input.rows; i++) {
            for (size_t j = 1; j < input.cols; j ++) {
                
                auto pix = Utilities::GetPixelValue(input, i, j);

                if ( pix < *threshold.get() ) {
                    Utilities::SetPixelValue(input, i, j, 0.f);
                }
                else {
                    Utilities::SetPixelValue(input, i, j, 1.f);
                }
            }
        }

        if( input.empty() ) break; // end of video stream
        cv::namedWindow("Gray", cv::WINDOW_AUTOSIZE);
        cv::imshow("Gray", input);

        if( cv::waitKey(10) == 120 ) Utilities::IncreaseInt(threshold);
        if( cv::waitKey(10) == 122 ) Utilities::DecreaseInt(threshold);
        if( cv::waitKey(10) == 27 ) break; // stop capturing by pressing ESC
    }
    
    vidCap.release();
    cv::destroyAllWindows();
    return 0;
}
