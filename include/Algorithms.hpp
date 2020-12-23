#include <opencv2/opencv.hpp>
#include <Utilities.hpp>
#pragma once

#define GETPIX

/**
 * The Strategy interface declares operations common to all supported versions
 * of some algorithm.
 *
 * The Context uses this interface to call the algorithm defined by Concrete
 * Strategies.
 */

class AlgorithmStrategy {
public:
    virtual ~AlgorithmStrategy() {}
    virtual cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable = nullptr) = 0;
    std::string AlgorithmName; 
};

/**
 * The Context defines the interface of interest to clients.
 */

class Context
{
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
    Context(AlgorithmStrategy *algStrat = nullptr) {
        std::cout << "Initializing with algorithm: " << algStrat->AlgorithmName << std::endl;
        strategy_ = algStrat;
    }
    ~Context() {
        delete this->strategy_;
    }

    /**
     * Usually, the Context allows replacing a Strategy object at runtime.
     */
    void set_strategy(AlgorithmStrategy *strategy) {
        strategy_ = strategy;
        std::cout << "Algorithm set to: " << strategy->AlgorithmName << std::endl;
    }

    /**
     * The Context delegates some work to the Strategy object instead of
     * implementing +multiple versions of the algorithm on its own.
     */
    cv::Mat ProcessStrategy(cv::Mat frame, std::shared_ptr<float> userInput = nullptr) {
        cv::Mat result = this->strategy_->Process(frame, userInput);
        return result;
    }
};

/**
 * Concrete Strategies implement the algorithm while following the base Strategy
 * interface. The interface makes them interchangeable in the Context.
 */
class ThresholdStrategy : public AlgorithmStrategy {
public:

    ThresholdStrategy(){
        AlgorithmName = "Thresholding";
    }

    cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override {
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
};

/**
 * Concrete Strategies implement the algorithm while following the base Strategy
 * interface. The interface makes them interchangeable in the Context.
 */
class MotionStrategy : public AlgorithmStrategy {
private:
    cv::Mat prev_Frame;

public:
    MotionStrategy(){
        AlgorithmName = "Motion Algorithm";
    }
    cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override {
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
};

/**
 * Concrete Strategies implement the algorithm while following the base Strategy
 * interface. The interface makes them interchangeable in the Context.
 */
class LowpassFilter : public AlgorithmStrategy {
public:
    LowpassFilter(){
        AlgorithmName = "Lowpass Filter";
    }
    cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override {
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
};

/**
 * Concrete Strategies implement the algorithm while following the base Strategy
 * interface. The interface makes them interchangeable in the Context.
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



    SimpleConvolution(){
        AlgorithmName = "Simple Convolution";
    }

    cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override {
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
};

/**
 * Concrete Strategies implement the algorithm while following the base Strategy
 * interface. The interface makes them interchangeable in the Context.
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


    SobelEdgeDetection(){
        AlgorithmName = "Sobel Edge Detection";
    }

    cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override {
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
};

/**
 * Concrete Strategies implement the algorithm while following the base Strategy
 * interface. The interface makes them interchangeable in the Context.
 */
class MorphologicalOperations : public AlgorithmStrategy {
public:
    MorphologicalOperations(){
        AlgorithmName = "Morphological Operations";
    }
    cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override {
        assert(variable != nullptr);

        auto output = cv::Mat(480, 640, CV_8UC1);
        auto rows = frame.rows;
        auto columns = frame.cols;
        float threshold = static_cast<float>(*variable.get());
        int cvThresh = static_cast<int>(threshold * 255);

        /* Code */
        cv::threshold(frame, output, cvThresh, 255, 0);

        // Dilation
        size_t morphCount = 1;
        for (size_t n = 0; n < morphCount; n++) {
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < columns; j++) {
                    if (Utilities::GetPixelValue(frame, i, j) == 1) {

                        Utilities::SetPixelValue(output, i, j, 1.0f);
                        Utilities::SetPixelValue(output, i-1, j, 1.0f);
                        Utilities::SetPixelValue(output, i+1, j, 1.0f);

                        Utilities::SetPixelValue(output, i, j-1, 1.0f);
                        Utilities::SetPixelValue(output, i, j+1, 1.0f);

                        Utilities::SetPixelValue(output, i-1, j-1, 1.0f);
                        Utilities::SetPixelValue(output, i+1, j+1, 1.0f);
                        Utilities::SetPixelValue(output, i-1, j+1, 1.0f);
                        Utilities::SetPixelValue(output, i+1, j-2, 1.0f);
                    }
                }
            }
        }

        return output;
    }
};

/**
 * Concrete Strategies implement the algorithm while following the base Strategy
 * interface. The interface makes them interchangeable in the Context.
 */
class ExampleStrategy : public AlgorithmStrategy {
public:
    ExampleStrategy(){
        AlgorithmName = "Example Algorithm";
    }
    cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override {
        assert(variable != nullptr);

        auto output = cv::Mat(480, 640, CV_8UC1);
        auto rows = frame.rows;
        auto columns = frame.cols;
        float threshold = static_cast<float>(*variable.get());

        /* Code */
        // copy input to output
        frame.copyTo(output);

        return output;
    }
};