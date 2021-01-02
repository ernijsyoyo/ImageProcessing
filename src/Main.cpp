#include <iostream>
#include <stdio.h>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>
#include <Utilities.hpp>
#include <Algorithms.hpp>

#include <opencv4/opencv2/dnn.hpp>  
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
using namespace cv;

int main(int argc, const char *argv[]) {
    RunAlgorithms::Run();
    return 0;
}