#ifndef ORB_PARAMETER_HPP_
#define ORB_PARAMETER_HPP_

#include "common_include.h"

namespace ORB {
    class BaseParameter {
    public:
        BaseParameter(const std::string config_file);

        void ReadPara();

    public:
        std::string file_path;
        double fx, fy, cx, cy;  // 相机内参
        double k1, k2, p1, p2;  // 相机畸变参数
    };

    class Parameter : public BaseParameter {
    public:
        Parameter(const std::string config_file);

        void ReadOtherPara();

    public:
        int nFeatures;
        double scalseFactor;
        int nLevels;
        int iniThFAST;
        int minThFAST;
    };

}


#endif