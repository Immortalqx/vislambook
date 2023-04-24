#include "ORB/parameter.hpp"

namespace ORB {
    BaseParameter::BaseParameter(const std::string config_file) {
        file_path = config_file;
        ReadPara();
    }

    void BaseParameter::ReadPara() {
        if (1) {
            // 使用OpenCV函数读取配置文件
            cv::FileStorage file(file_path, cv::FileStorage::READ);
            if (!file.isOpened()) {
                LOG(ERROR) << "文件打开失败，请检查文件路径";
                exit(-1);
            }
            fx = file["Camera.fx"];
            fy = file["Camera.fy"];
            cx = file["Camera.cx"];
            cy = file["Camera.cy"];

            k1 = file["Camera.k1"];
            k2 = file["Camera.k2"];
            p1 = file["Camera.p1"];
            p2 = file["Camera.p2"];
            file.release();
        } else {
            // 使用yaml文件读取配置文件
            YAML::Node camera_node = YAML::LoadFile(file_path);
            fx = camera_node["Camera.fx"].as<double>();
            fy = camera_node["Camera.fy"].as<double>();
            cx = camera_node["Camera.cx"].as<double>();
            cy = camera_node["Camera.cy"].as<double>();

            k1 = camera_node["Camera.k1"].as<double>();
            k2 = camera_node["Camera.k2"].as<double>();
            p1 = camera_node["Camera.p1"].as<double>();
            p2 = camera_node["Camera.p2"].as<double>();
        }
    }

    Parameter::Parameter(const std::string config_file) : BaseParameter(config_file) {
        ReadPara();
        ReadOtherPara();
    }

    void Parameter::ReadOtherPara() {
        if (1) {
            cv::FileStorage file(file_path, cv::FileStorage::READ);
            if (!file.isOpened()) {
                LOG(ERROR) << "文件打开失败，请检查文件路径";
                exit(-1);
            }
            nFeatures = file["Features"];
            scalseFactor = file["scaleFactor"];
            nLevels = file["nLevels"];
            iniThFAST = file["iniThFAST"];
            minThFAST = file["minThFAST"];
        } else {
            YAML::Node node = YAML::LoadFile(file_path);
            nFeatures = node["Features"].as<int>();
            scalseFactor = node["scaleFactor"].as<double>();
            nLevels = node["nLevels"].as<int>();
            iniThFAST = node["iniThFAST"].as<int>();
            minThFAST = node["minThFAST"].as<int>();
        }
    }
}