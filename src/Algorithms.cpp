#include <Algorithms.hpp>

Context::Context(AlgorithmStrategy *algStrat) {
  std::cout << "Initializing with algorithm: " << algStrat->AlgorithmName << std::endl;
  strategy_ = algStrat;
}

Context::~Context() {
  delete this->strategy_;
}

void Context::set_strategy(AlgorithmStrategy *strategy) {
  strategy_ = strategy;
  std::cout << "Algorithm set to: " << strategy->AlgorithmName << std::endl;
}

cv::Mat Context::ProcessStrategy(cv::Mat frame, std::shared_ptr<float> userInput) {
  cv::Mat result = this->strategy_->Process(frame, userInput);
  return result;
}

ThresholdStrategy::ThresholdStrategy(){
  AlgorithmName = "Thresholding";
}

cv::Mat ThresholdStrategy::Process(cv::Mat frame, std::shared_ptr<float> variable) {
  assert(variable != nullptr);

  auto rows = frame.rows;
  auto columns = frame.cols;
  float threshold = static_cast<float>(*variable.get());

  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < columns; j++) {
      if(Utilities::GetPixelValue(frame, i, j) < threshold ){
        Utilities::SetPixelValue(frame, i, j, 0.0f);
      }
      else {
        Utilities::SetPixelValue(frame, i, j, 1.0f);
      }
    }
  }
  return frame;
}

MotionStrategy::MotionStrategy(){
  AlgorithmName = "Motion Algorithm";
}

cv::Mat MotionStrategy::Process(cv::Mat frame, std::shared_ptr<float> variable) {
  assert(variable != nullptr);
  auto output = cv::Mat(480, 640, CV_8UC1);
  if (prev_Frame.empty()) {
    frame.copyTo(prev_Frame);
    return frame;
  }    
  
  auto rows = frame.rows;
  auto columns = frame.cols;

  // Subtract this frame from previous frame and set absolute diff
  // alternatively: cv::absdiff(frame, prev_Frame, output);
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < columns; j++) {
      auto diff = fabs(Utilities::GetPixelValue(frame, i, j) - Utilities::GetPixelValue(prev_Frame, i, j));
      Utilities::SetPixelValue(output, i, j, diff);
    }
  }

  frame.copyTo(prev_Frame); // save previous frame
  return output;
}

LowpassFilter::LowpassFilter(){
  AlgorithmName = "Lowpass Filter";
}
cv::Mat LowpassFilter::Process(cv::Mat frame, std::shared_ptr<float> variable) {
  assert(variable != nullptr);

  auto output = cv::Mat(480, 640, CV_8UC1);
  auto rows = frame.rows;
  auto columns = frame.cols;
  float fLowPassRC = static_cast<float>(*variable.get());

  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < columns; j++) {
      auto framePix = Utilities::GetPixelValue(frame, i, j);
      auto outputPix = Utilities::GetPixelValue(output, i, j);
      float dPixel = framePix - outputPix; 
      dPixel *= fLowPassRC;
      auto outputValue = dPixel + outputPix;
      Utilities::SetPixelValue(output, i, j, outputValue);
    }
  }

  /* Code */
  return output;
}

SimpleConvolution::SimpleConvolution(){
  AlgorithmName = "Simple Convolution";
}

cv::Mat SimpleConvolution::Process(cv::Mat frame, std::shared_ptr<float> variable) {
  assert(variable != nullptr);

  auto output = cv::Mat(480, 640, CV_8UC1);
  auto rows = frame.rows;
  auto columns = frame.cols;
  float threshold = static_cast<float>(*variable.get());

  if (cv::waitKey(5) == 110) {
    pConvoKernel = kernel_blur; // key n
    std::cout << "Blur" << std::endl;
  }
  if (cv::waitKey(5) == 109) {
    pConvoKernel = kernel_sharp; // key m
    std::cout << "Sharp" << std::endl;
  } 

  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < columns; j++) {
      float fSum = 0.0f;

      for (int n = -1; n < +2; n++) {
        for (int m = -1; m < +2; m++) {
          auto kernelCoeff = pConvoKernel[(m + 1) * 3 + (n + 1)] * threshold;
          auto pixelValue = Utilities::GetPixelValue(frame, i + n, j + m);
          auto outputPixel = pixelValue * kernelCoeff;
          fSum += outputPixel;
        }
      }
      Utilities::SetPixelValue(output, i, j, fSum);
    }
  }
  return output;
}

SobelEdgeDetection::SobelEdgeDetection(){
  AlgorithmName = "Sobel Edge Detection";
}

cv::Mat SobelEdgeDetection::Process(cv::Mat frame, std::shared_ptr<float> variable) {
  assert(variable != nullptr);

  auto output = cv::Mat(480, 640, CV_8UC1);
  auto rows = frame.rows;
  auto columns = frame.cols;
  float threshold = static_cast<float>(*variable.get());

  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < columns; j++) {
      float fSumH = 0.0f;
      float fSumV = 0.0f;

      for (int n = -1; n < +2; n++) {
        for (int m = -1; m < +2; m++) {
          auto kernelCoeffH = kernel_sobel_h[(m + 1) * 3 + (n + 1)];
          auto kernelCoeffV = kernel_sobel_v[(m + 1) * 3 + (n + 1)];
          auto pixelValue = Utilities::GetPixelValue(frame, i + n, j + m);
          fSumH += pixelValue * kernelCoeffH;
          fSumV += pixelValue * kernelCoeffV;
        }
      }
      Utilities::SetPixelValue(output, i, j, fabs(fSumH + fSumV) / 2);
    }
  }
  return output;
}

MorphologicalOperations::MorphologicalOperations(){
  AlgorithmName = "Morphological Operations";
}

cv::Mat MorphologicalOperations::Process(cv::Mat frame, std::shared_ptr<float> variable) {
  assert(variable != nullptr);

  auto output = cv::Mat(480, 640, CV_8UC1);
  auto activity = cv::Mat(480, 640, CV_8UC1);
  auto rows = frame.rows;
  auto columns = frame.cols;
  float threshold = static_cast<float>(*variable.get());
  int cvThresh = static_cast<int>(threshold * 255);

  /* Code */
  cv::threshold(frame, activity, cvThresh, 255, 0); // threshold to value 75
  activity.copyTo(output);
  int morphCount = 3;
  // Dilation
  // 
  // for (size_t n = 0; n < morphCount; n++) {
  //   for (int i = 0; i < rows; i++) {
  //     for (int j = 0; j < columns; j++) {
  //       if (Utilities::GetPixelValue(activity, i, j) == 1) {

  //         Utilities::SetPixelValue(output, i-1, j, 1.0f);
  //         Utilities::SetPixelValue(output, i+1, j, 1.0f);

  //         Utilities::SetPixelValue(output, i, j-1, 1.0f);
  //         Utilities::SetPixelValue(output, i, j+1, 1.0f);

  //         Utilities::SetPixelValue(output, i-1, j-1, 1.0f);
  //         Utilities::SetPixelValue(output, i+1, j+1, 1.0f);
  //         Utilities::SetPixelValue(output, i-1, j+1, 1.0f);
  //         Utilities::SetPixelValue(output, i+1, j-1, 1.0f);
  //       }
  //     }
  //   }
  // }
  // return output;
  

  // Erosion
  // for (int i = 0; i < rows; i++) {
  //   for (int j = 0; j < columns; j++) {
  //     float sum =  Utilities::GetPixelValue(activity, i, j) +
  //           Utilities::GetPixelValue(activity, i-1, j) +
  //           Utilities::GetPixelValue(activity, i+1, j) +

  //           Utilities::GetPixelValue(activity, i, j-1) +
  //           Utilities::GetPixelValue(activity, i, j+1) +

  //           Utilities::GetPixelValue(activity, i-1, j-1) +
  //           Utilities::GetPixelValue(activity, i+1, j+1) +
  //           Utilities::GetPixelValue(activity, i-1, j+1) +
  //           Utilities::GetPixelValue(activity, i+1, j-1);

  //     auto equalToOne = Utilities::GetPixelValue(activity, i, j) == 1.0f;
  //     auto lessThan = sum < 8.0f;
  //     if (equalToOne && lessThan) {
  //       Utilities::SetPixelValue(output, i, j, 0.0f);
  //     }
  //   }
  // }
  // return output;

  // Edge
  
    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < columns; j++) {
        float sum = Utilities::GetPixelValue(activity, i-1, j) +
              Utilities::GetPixelValue(activity, i+1, j) +

              Utilities::GetPixelValue(activity, i, j-1) +
              Utilities::GetPixelValue(activity, i, j+1) +

              Utilities::GetPixelValue(activity, i-1, j-1) +
              Utilities::GetPixelValue(activity, i+1, j+1) +
              Utilities::GetPixelValue(activity, i-1, j+1) +
              Utilities::GetPixelValue(activity, i+1, j-1);

        auto equalToOne = Utilities::GetPixelValue(activity, i, j) == 1.0f;
        auto lessThan = sum == 8.0f;
        if (equalToOne && lessThan) {
          Utilities::SetPixelValue(output, i, j, 0.0f);
        }
      }
    }
  return output;
}

MedianFilter::MedianFilter(){
  AlgorithmName = "Median Filter";
}

cv::Mat MedianFilter::Process(cv::Mat frame, std::shared_ptr<float> variable) {
    assert(variable != nullptr);

    auto output = cv::Mat(480, 640, CV_8UC1);
    auto rows = frame.rows;
    auto columns = frame.cols;
    float threshold = static_cast<float>(*variable.get());

    /* Code */
    for (size_t i = 0; i < rows; i++) {
      for (size_t j = 0; j < columns; j++) {
        std::vector<float> v;

        for (int n = -2; n < +3; n++) {
          for (int m = -2; m < +3; m++) {
            v.push_back(Utilities::GetPixelValue(frame, i + n, j + m));
          }
        }

        std::sort(v.begin(), v.end(), std::greater<float>());
        Utilities::SetPixelValue(output, i, j, v[12]);             
      }
    }
    std::cout << "Returning output" << std::endl;
    return output;
}

AdaptiveThreshold::AdaptiveThreshold(){
  AlgorithmName = "Adaptive Threshold";
}
cv::Mat AdaptiveThreshold::Process(cv::Mat frame, std::shared_ptr<float> variable) {
  assert(variable != nullptr);

  auto output = cv::Mat(480, 640, CV_8UC1);
  auto rows = frame.rows;
  auto columns = frame.cols;
  float threshold = static_cast<float>(*variable.get());
  // cv::adaptiveThreshold(frame, output, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);
  // return output;
  /* Code */
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < columns; j++) {
      float regionSum = 0.0f;
      for (int n = -2; n < +3; n++) {
        for (int m = -2; m < +3; m++) {
          regionSum += Utilities::GetPixelValue(frame, i + n, j + m);
        }
      }
      
      regionSum /= 25.0f;
      float outputPixVal = Utilities::GetPixelValue(frame, i, j) > (regionSum * threshold) ? 1.0f : 0.0f;
      Utilities::SetPixelValue(output, i, j, outputPixVal);
    }
  }
  return output;
}