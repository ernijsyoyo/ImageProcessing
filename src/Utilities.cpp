#include <Utilities.hpp>
#include <iostream>

Utilities::Utilities(){
    std::cout << "Constructor of Utilities" << std::endl;
}

Utilities::~Utilities(){
    /* Nix */
}

void Utilities::HelloWorld(){
    std::cout << " Hello World " << std::endl;
}

cv::Mat Utilities::AddTwoFrames(cv::Mat frame1, cv::Mat frame2) {
    cv::Mat output;
    cv::addWeighted(frame1, 1.0, frame2, 1.0, 0.0, output);
    return output;
}

float Utilities::GetPixelValue(cv::Mat frame, int x, int y) {
    auto output = static_cast<float>( (int)frame.at<uchar>(x, y) );
    return output / 255.f;
}

void Utilities::SetPixelValue(cv::Mat frame, int x, int y, float value) {
    auto setTo = value * 255.f;
    auto intValue = static_cast<size_t>(setTo);
    frame.at<uchar>(x, y) = intValue;
}

void Utilities::IncreaseInt(std::shared_ptr<float> input){
    std::cout << "Increasing input " << *input.get() << std::endl;
    *input += 0.01f;
    std::cout << "Increased input " << *input.get() << std::endl;
}

void Utilities::DecreaseInt(std::shared_ptr<float> input){
    std::cout << "Decreasing input " << *input.get() << std::endl;
    *input -= 0.01f;
    std::cout << "Decrease input " << *input.get() << std::endl;
}