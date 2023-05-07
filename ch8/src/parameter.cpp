#include "ORB/parameter.hpp"

namespace ORB {
    Parameter::Parameter(const std::string config_file)
    {
        file_path = config_file;
        ReadPara();
    }
    void Parameter::ReadPara()
    {
        if (1)
        {
            // 使用OpenCV函数读取配置文件
            cv::FileStorage file(file_path, cv::FileStorage::READ);
            if (!file.isOpened())
            {
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

            nFeatures = file["Features"];
            scalseFactor = file["scaleFactor"];
            nLevels = file["nLevels"];
            iniThFAST = file["iniThFAST"];
            minThFAST = file["minThFAST"];

            file.release();
        }
        else {
            // 使用yaml文件读取配置文件
            YAML::Node slam_node = YAML::LoadFile(file_path);
            fx = slam_node["Camera.fx"].as<double>();
            fy = slam_node["Camera.fy"].as<double>();
            cx = slam_node["Camera.cx"].as<double>();
            cy = slam_node["Camera.cy"].as<double>();

            k1 = slam_node["Camera.k1"].as<double>();
            k2 = slam_node["Camera.k2"].as<double>();
            p1 = slam_node["Camera.p1"].as<double>();
            p2 = slam_node["Camera.p2"].as<double>();

            nFeatures = slam_node["Features"].as<int>();
            scalseFactor = slam_node["scaleFactor"].as<double>();
            nLevels = slam_node["nLevels"].as<int>();
            iniThFAST = slam_node["iniThFAST"].as<int>();
            minThFAST = slam_node["minThFAST"].as<int>();
        }
    }
}