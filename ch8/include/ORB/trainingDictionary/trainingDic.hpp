#ifndef ORB_TRAININGDICTIONARY_HPP_
#define ORB_TRAININGDICTIONARY_HPP_

#include "ORB/common_include.h"
#include <DBoW3/DBoW3.h>
// #include <DBoW3/>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <glog/logging.h>
#include "ORB/tools/tic_toc.hpp"

namespace ORB {
    class TraingDIC{
        public:
            TraingDIC(const std::string& dataSetPath, const std::string& outDICPath, int Depth = 5, int Branch = 10);
            TraingDIC() = default;

            void Run();

            // 作业
            void UsingDictionary(const std::string& dicFilePath, const std::string& imgPath);
        private:
            void ReadDataSet(const std::string& dataSetPath, std::vector<cv::String>& ImagePath);

            void DetectDescriptors(const cv::String& image_path, cv::Mat& descriptor);

            void DetectDescriptors(const cv::String& imagePath, std::vector<cv::KeyPoint>& vKeyPoints, cv::Mat& descriptor);

            void traingDictionary(const std::vector<cv::Mat> vdescriptors);

        private:
            int k, d;
            std::string dataSetPath;
            std::string outputPath;
            std::vector<cv::String> vImagePath;
    };
}

#endif