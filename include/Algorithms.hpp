#ifndef IMGPROC_INCL_ALGORITHMS_H
#define IMGPROC_INCL_ALGORITHMS_H

#include <opencv2/opencv.hpp>
#include <Utilities.hpp>


/**
 * The Strategy interface declares operations common to all supported versions
 * of some algorithm.
 *
 * The Context uses this interface to call the algorithm defined by Concrete
 * Strategies.
 */

class AlgorithmStrategy {
public:
  std::string AlgorithmName; 
  virtual ~AlgorithmStrategy() {}
  virtual cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable = nullptr) = 0;
};

/**
 * The Context defines the interface of interest to clients.
 */

class Context {
  /**
   * @var Strategy The Context maintains a reference to one of the Strategy
   * objects. The Context does not know the concrete class of a strategy. It
   * should work with all strategies via the Strategy interface.
   */
private:
  AlgorithmStrategy *strategy_;
  std::shared_ptr<float> m_Variable;

  /**std::vector<std::string>{"a", "e", "c", "b", "d"}
   * Usually, the Context accepts a strategy through the constructor, but also
   * provides a setter to change it at runtime.
   */
public:
  Context(AlgorithmStrategy *algStrat = nullptr);
  ~Context();

  /**
   * Usually, the Context allows replacing a Strategy object at runtime.
   */
  void set_strategy(AlgorithmStrategy *strategy);
  /**
   * The Context delegates some work to the Strategy object instead of
   * implementing +multiple versions of the algorithm on its own.
   */
  cv::Mat ProcessStrategy(cv::Mat frame, std::shared_ptr<float> userInput = nullptr);
};

/**
 * Concrete Strategies implement the algorithm while following the base Strategy
 * interface. The interface makes them interchangeable in the Context.
 */
class ExampleStrategy : public AlgorithmStrategy {
public:
  ExampleStrategy() {
    AlgorithmName = "Example Algorithm";
  }
  cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override {
    assert(variable != nullptr);

    auto output = cv::Mat(480, 640, CV_8UC1);
    auto rows = frame.rows;
    auto columns = frame.cols;
    float threshold = static_cast<float>(*variable.get());

    /* Code */
    frame.copyTo(output);   
    return output;
  }
};

/**
 * Image binarization - if a pixel is below a certain threshold, then make its value 0
 * If above a certain threshold, then make value 1
 */
class ThresholdStrategy : public AlgorithmStrategy {
public:
  ThresholdStrategy();
  cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override;
};

/**
 * Calculate a difference between previous and current frame, if both frames are the same,
 * then the output is small. If both frames are quite different then the output is large.
 * Useful for detecting motion
 */
class MotionStrategy : public AlgorithmStrategy {
private:
  cv::Mat prev_Frame;

public:
  MotionStrategy();
  cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override;
};

/**
 * Simple RC Filter for removing noise
 * P = (I - P) * temporal constant
 * Where:
 * P - Pixel
 * I - Input
 * Temporal constant - user defined, control the amount of noise reduced
 */
class LowpassFilter : public AlgorithmStrategy {
public:
  LowpassFilter();
  cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override;
};

/**
 * Example of a simple convolution
 * 
 * TODO: Experiment with FFTs
 */
class SimpleConvolution : public AlgorithmStrategy {
public:
  float *pConvoKernel = kernel_sharp;
  float kernel_blur[9] = {
    0.0f,   0.125f, 0.0f,
    0.125f, 0.5f,   0.125f,
    0.0f,   0.125f, 0.0f
  };
  float kernel_sharp[9] = {
    0.0f,   -1.0f,  0.0f,
    -1.0f,  -5.0f,  -1.0f,
    0.0f,   -1.0f,  0.0f
  };

  SimpleConvolution();
  cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override;    
};

/**
 * Detect edges by convolving outputs of the two kernels
 */
class SobelEdgeDetection : public AlgorithmStrategy {
public:
  float kernel_sobel_h[9] = {
    -1.0f,  -2.0f, -1.0f,
    0.0f,   0.0f,  0.0f,
    1.0f,   2.0f,  1.0f
  };
  float kernel_sobel_v[9] = {
    -1.0f,   0.0f,  1.0f,
    -2.0f,   0.0f,  2.0f,
    -1.0f,   0.0f,  1.0f
  };

  SobelEdgeDetection();
  cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override;
};

/**
 * CHange the shape of objects in image in binary (thresholded) domain
 * 1. Erosion - Removing a single pixel from edges (shrinking), useful to remove spurious pixels
 * 2. Dilation - Add a pixel to edges
 * Combine erosion with dilation to remove spatial noise
 * 
 * 3. Edge detection, similar to erosion
 *  If a pixel has all 8 neighbours high, then set itself to low
 * 
 * 
 */
class MorphologicalOperations : public AlgorithmStrategy {
public:
  MorphologicalOperations();
  cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override;
};

/**
 * Assume image information is blurred out and we have transient-like outliers (rain, snow)
 * Looking at pixels 5x5 neighbourhood and collect all 25 pixels in a list
 * Sort the list from smallest to biggest and set the median(middle) value
 * as our output. This will remove the transient outliers
 */
class MedianFilter : public AlgorithmStrategy {
public:
  MedianFilter();
  cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override;
};

/**
 * Concrete Strategies implement the algorithm while following the base Strategy
 * interface. The interface makes them interchangeable in the Context.
 */
class AdaptiveThreshold : public AlgorithmStrategy {
public:
  AdaptiveThreshold();
  cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override;
};

class RunAlgorithms{

public:
  static int ChangeStrategy(){
    std::cout << "Change Algorithm" << std::endl;
    std::cout << "0 - Direct Output" << std::endl;
    std::cout << "1 - Thresholding" << std::endl;
    std::cout << "2 - Motion Detect" << std::endl;
    std::cout << "3 - LowPass RC filter (Noise Removal)" << std::endl;
    std::cout << "4 - Simple Convolution" << std::endl;
    std::cout << "5 - Sobel Edge Detection" << std::endl;
    std::cout << "6 - Morphological Operation" << std::endl;
    std::cout << "7 - Median Filter" << std::endl;
    std::cout << "8 - Adaptive Threshold" << std::endl;
    std::cout << "9 - Exit program" << std::endl;

    int input = 0;
    std::cin >> input;
    return input;
  }
  static void Run(){
    // Start the webcam capping
  cv::VideoCapture vidCap;
  if (!vidCap.open(0))
  {
    std::cout << "returning " << std::endl;
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
  auto lowPass = new LowpassFilter;
  auto simpleConvolution = new SimpleConvolution;
  auto sobelEdgeDetection= new SobelEdgeDetection;
  auto morphOps= new MorphologicalOperations;
  auto medianFilter= new MedianFilter;
  auto adaptiveThreshold= new AdaptiveThreshold;
  auto exampleStrategy = new ExampleStrategy;
  Context *context = new Context(exampleStrategy);
  
  std::cout << "Stream starting" << std::endl;
  while (true) {
    // Capture frame and convert to grayscale
    vidCap >> input;
    cv::cvtColor(input, input, cv::COLOR_BGR2GRAY); // conv color to gray

    // Process strategy and display result
    auto processedInput = context->ProcessStrategy(input , userInput);
    if( processedInput.empty() ) break; // end of video stream

    // Show original and processed side by side
    cv::imshow("Gray", processedInput);
    
    // Algorithm control and loop break
    if (cv::waitKey(5) == 113){ // button Q
      auto strategy = ChangeStrategy();
      switch (strategy) {
      case 1:
        context->set_strategy(thresholdStrategy);
        break;
      case 2:
        context->set_strategy(motionStrategy);
        break;
      case 3:
        context->set_strategy(lowPass);
        break;
      case 4:
        context->set_strategy(simpleConvolution);
        break;
      case 5:
        context->set_strategy(sobelEdgeDetection);
        break;
      case 6:
        context->set_strategy(morphOps);
        break;
      case 7:
        context->set_strategy(medianFilter);
        break;
      case 8:
        context->set_strategy(adaptiveThreshold);
        break;
      case 0:
        context->set_strategy(exampleStrategy);
        break;
      case 9: // exit the program
        return exit(0);
      }
    }
    if (cv::waitKey(5) == 120) Utilities::IncreaseFloat(userInput); // key z
    if (cv::waitKey(5) == 122) Utilities::DecreaseFloat(userInput); // key x
  } // end of processing loop
  
  vidCap.release();
  cv::destroyAllWindows();
  }
};

#endif