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

float Utilities::GetPixelValue(cv::Mat frame, int row, int column) {
    auto output = static_cast<float>( (int)frame.at<uchar>(row, column) );
    return output / 255.f;
}

void Utilities::SetPixelValue(cv::Mat frame, int row, int column, float value) {
    if (value > 1.0f) value = 1.0f;
    
    auto setTo = value * 255.f;
    auto intValue = static_cast<size_t>(setTo + 0.5f);
    frame.at<uchar>(row, column) = intValue;
}

void Utilities::IncreaseInt(std::shared_ptr<int> input){
    *input += 1;
    std::cout << "Increased input " << *input.get() << std::endl;
}

void Utilities::DecreaseInt(std::shared_ptr<int> input){
    *input -= 1;
    std::cout << "Decrease input " << *input.get() << std::endl;
}

void Utilities::IncreaseFloat(std::shared_ptr<float> input){
    *input += 0.01f;
    std::cout << "Increased input " << *input.get() << std::endl;
}

void Utilities::DecreaseFloat(std::shared_ptr<float> input){
    *input -= 0.01f;
    std::cout << "Decrease input " << *input.get() << std::endl;
}