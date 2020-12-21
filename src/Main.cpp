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

    int threshold = 100;

		// Placeholders for frames
    cv::Mat frame;
    cv::Mat gray;

    std::cout << "Stream started" << std::endl;
    while (true) {
        vidCap >> frame;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        for (size_t i = 1; i < gray.rows - 1; i++) {
            for (size_t j = 1; j < gray.cols; j ++) {
                /* Thresholding */
                auto pix = gray.at<uchar>(i, j);

                if (pix < threshold) {
                    gray.at<uchar>(i, j) = 0;
                }
                else {
                    gray.at<uchar>(i, j) = 255;
                }
            }
        }

        if( gray.empty() ) break; // end of video stream
        cv::namedWindow("Gray", cv::WINDOW_AUTOSIZE);
        cv::imshow("Gray", gray);

        if( cv::waitKey(10) == 120 ) { // Button X pressed
            threshold++; // stop capturing by pressing ESC
            std::cout << "Variable " << threshold << std::endl;
        }
        if( cv::waitKey(10) == 122 ) { // Button Z pressed
            threshold--; // stop capturing by pressing ESC
            std::cout << "Variable " << threshold << std::endl;
        }
        if( cv::waitKey(10) == 27 ) break; // stop capturing by pressing ESC
    }

    vidCap.release();
    cv::destroyAllWindows();

    return 0;
}
