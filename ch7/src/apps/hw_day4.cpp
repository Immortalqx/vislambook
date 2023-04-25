#include "ORB/global_defination/global_defination.h"
#include "ORB/ORBFeature.hpp"

using namespace ORB;

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;

    std::string config_path = WORK_SPACE_PATH + "/config/camera_para.yaml";
    std::string image_path = WORK_SPACE_PATH + "/image/distorted.png";

    std::shared_ptr<ORBFeature> orb_feature_ptr = std::make_shared<ORBFeature>(image_path, config_path);

    std::vector<cv::KeyPoint> vKeypoints;
    cv::Mat descriptor;
    orb_feature_ptr->ExtractORB(vKeypoints, descriptor);

    cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    LOG(INFO) << "vKeypoints size " << vKeypoints.size();
    LOG(INFO) << "descriptor size " << descriptor.rows;

    cv::Mat outImage;
    cv::drawKeypoints(image, vKeypoints, outImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("ORB-SLAM2 Result", outImage);

    orb_feature_ptr->ExtractORB();
    cv::waitKey();
    return 0;
}
