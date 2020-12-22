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
    int counter = 1;

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
class ExampleStrategy : public AlgorithmStrategy {
public:
    ExampleStrategy(){
        AlgorithmName = "Example Algorithm";
    }
    cv::Mat Process(cv::Mat frame, std::shared_ptr<float> variable=nullptr) override {
        assert(variable != nullptr);

        auto rows = frame.rows;
        auto columns = frame.cols;
        float threshold = static_cast<float>(*variable.get());

        /* Code */
        return frame;
    }
};