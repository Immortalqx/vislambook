#ifndef ORB_ORBFEATURE_HPP_
#define ORB_ORBFEATURE_HPP_

#include "ORB/parameter.hpp"
#include <opencv2/opencv.hpp>
#include "ORB/ExtractorNode.hpp"

namespace ORB {
    class ORBFeature {
    public:
        ORBFeature(const std::string &image_path, const std::string &config_path);

        //1、 提取ORB特征点,使用opencv函数实现
        void ExtractORB();

        //2、 图像去畸变
        void UndistortImage();

        //3、 ORB-SLAM2源码 ExtractORB()
        void ExtractORB(std::vector<cv::KeyPoint> &_keypoints, cv::Mat &_descriptors);

    private:
        // 设置参数
        void setPara();

        // 构建图像金字塔
        void ComputePyramid();

        // 使用四叉树的方式计算每层图像的特征点并进行分配
        void ComputeKeyPointsQuadtree(std::vector<std::vector<cv::KeyPoint>> &allKeypoints);

        // 计算描述子
        void computeDescriptors(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors,
                                const std::vector<cv::Point> &pattern);

    private:
        void computeOrbDescriptor(const cv::KeyPoint &kpt, const cv::Mat &img, const cv::Point *pattern, uchar *desc);

        std::vector<cv::KeyPoint>
        DistributeQuadtree(const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
                           const int &maxX, const int &minY, const int &maxY, const int &N, const int &level);

        void
        computeOrientation(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, const std::vector<int> &umax);

    private:
        float IC_Angle(const cv::Mat &image, cv::Point2f pt, const std::vector<int> &u_max);

    private:
        std::shared_ptr<Parameter> camera_ptr;
        cv::Mat image;

        std::vector<double> mvScaleFactor, mvInvScaleFactor;
        std::vector<double> mvLevelSigma2, mvInvLevelSigma2;
        std::vector<cv::Mat> mvImagePyramid;
        std::vector<int> mnFeaturesPerLevel;
        std::vector<int> umax;

        static int HALF_PATCH_SIZE;
        std::vector<cv::Point> pattern;

        static int EDGE_THRESHOLD;
        static int PATCH_SIZE;
    };
}

#endif