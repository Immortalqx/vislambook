#include<stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>

#include<Eigen/Core>

using namespace std;
std::vector<double> picture_stamp_;//图片时间戳
std::string picture_stamp_path = "../picture_data/picture_stamp.txt";//路径需根据自身情况调整


std::string imu_data_path = "../imu_data/dataset-corridor4_512.txt";//路径需根据自身情况调整
struct ImuData
{
    double stamp;
    Eigen::Vector3d acc_;
    Eigen::Vector3d omega_;
};
std::vector<ImuData> imudata_v_;

struct MeasureGroup
{
    double picture_next_stamp;
    double picture_cur_stamp;
    std::vector<Eigen::Vector3d> acc_group;
    std::vector<Eigen::Vector3d> omega_group;
    std::vector<double> delta_t;
    int imu_data_size;
};
std::vector<MeasureGroup> MeasureGroups_;

int ComputeMeasurements(const std::vector<double> &picture_stamps, const std::vector<ImuData> &imudatas)
{
    int first_imu_id = 0;

    for (size_t i = 0; i < picture_stamps.size() - 1; i++)
    {
        MeasureGroup preintegration_data_temp;//单独一个预积分量所包含的数据
        //根据前后两帧图像的时间戳获取相应的imu数据
        std::vector<ImuData> imudata_tmp;//改变量防止已经查询过的重复查询
        for (size_t j = first_imu_id; j < imudatas.size(); j++)
        {
            first_imu_id++;
            if (imudatas[j].stamp < picture_stamps[i])//时间不符合要求，不存入
            {
                continue;
            } else if (imudatas[j].stamp < picture_stamps[i + 1])//时间符合要求，存入
            {
                imudata_tmp.push_back(imudatas[j]);
            } else
            {
                imudata_tmp.push_back(imudatas[j]);//最后一个符合时间要求的imu数据
                first_imu_id--;
                break;
            }
        }

        //计算加速度和速度
        int n = imudata_tmp.size() - 1;
        for (size_t k = 0; k < n; k++)
        {
            Eigen::Vector3d acc, w;
            double t_step;

            if ((k == 0) && (k < (n - 1)))//注意第一帧imu数据的处理
            {
                double t = imudata_tmp[k + 1].stamp - imudata_tmp[k].stamp;//单帧imu时间间隔
                double t_picture = imudata_tmp[k].stamp - picture_stamps[i];//第一帧imu数据到图像间隔

                //*******************补全下面代码*********************//
                acc = (imudata_tmp[k].acc_ + imudata_tmp[k + 1].acc_ -
                       (imudata_tmp[k + 1].acc_ - imudata_tmp[k].acc_) * (t_picture / t)) * 0.5f;
                w = (imudata_tmp[k].omega_ + imudata_tmp[k + 1].omega_ -
                     (imudata_tmp[k + 1].omega_ - imudata_tmp[k].omega_) * (t_picture / t)) * 0.5f;
                t_step = imudata_tmp[k + 1].stamp - picture_stamps[i + 1];
            } else if (k < (n - 1))
            {
                //*******************补全下面代码*********************//
                acc = (imudata_tmp[k].acc_ + imudata_tmp[k + 1].acc_) * 0.5f;
                w = (imudata_tmp[k].omega_ + imudata_tmp[k + 1].omega_) * 0.5f;
                t_step = imudata_tmp[k + 1].stamp - imudata_tmp[k].stamp;
            } else if ((k > 0) && (k == (n - 1)))
            {
                double t = imudata_tmp[k + 1].stamp - imudata_tmp[k].stamp;//单帧imu时间间隔
                double t_picture = imudata_tmp[k + 1].stamp - picture_stamps[i + 1];
                //*******************补全下面代码*********************//
                acc = (imudata_tmp[k].acc_ + imudata_tmp[k + 1].acc_ -
                       (imudata_tmp[k + 1].acc_ - imudata_tmp[k].acc_) * (t_picture / t)) * 0.5f;
                w = (imudata_tmp[k].omega_ + imudata_tmp[k + 1].omega_ -
                     (imudata_tmp[k + 1].omega_ - imudata_tmp[k].omega_) * (t_picture / t)) * 0.5f;
                t_step = picture_stamps[i + 1] - imudata_tmp[k].stamp;
            }
            //*******************补全下面3行代码*********************//
            // 记录IMU数据计算出来的加速度，角速度，时间间隔
            preintegration_data_temp.acc_group.push_back(acc);
            preintegration_data_temp.omega_group.push_back(w);
            preintegration_data_temp.delta_t.push_back(t_step);
        }
        preintegration_data_temp.imu_data_size = n;//两帧图像之间imu数据数量
        preintegration_data_temp.picture_cur_stamp = picture_stamps[i];//第一张图像的时间戳
        preintegration_data_temp.picture_next_stamp = picture_stamps[i + 1]; //第二张图像的时间戳

        MeasureGroups_.push_back(preintegration_data_temp);
        std::cout << "preintegration" << i << " has " << n + 1 << " imu data between picture " << i << " with stamp " \
 << setprecision(13) << picture_stamps[i] << " and picture " << i + 1 << " with stamp " << setprecision(13)
                  << picture_stamps[i + 1] << std::endl << std::endl;
    }

    return MeasureGroups_.size();
}

int main(void)
{
    //读取图像时间戳
    ifstream infile;
    double temp;
    infile.open(picture_stamp_path);
    while (infile >> temp)
    {
        temp = temp / 1e9;
        picture_stamp_.push_back(temp);
        //cout << std::setprecision(13) << temp << endl;
    }
    infile.close();
    int pic_size = picture_stamp_.size();
    std::cout << "We get " << pic_size << " picture stamp." << std::endl << std::endl;

    //读取imu数据
    ImuData imudata_temp;
    string s;
    ifstream infile_imu;
    infile_imu.open(imu_data_path);
    while (!infile_imu.eof())
    {
        string s;
        getline(infile_imu, s);
        if (s[0] == '#')//第一行注释的说明不读取
        {
            continue;
        }

        string item;
        size_t pos = 0;
        double data[7];
        int count = 0;
        while ((pos = s.find(',')) != string::npos)
        {
            item = s.substr(0, pos);
            data[count++] = stod(item);
            s.erase(0, pos + 1);
        }
        item = s.substr(0, pos);
        data[6] = atof(item.c_str());


        imudata_temp.stamp = data[0] / 1e9;
        imudata_temp.acc_[0] = data[4];
        imudata_temp.acc_[1] = data[5];
        imudata_temp.acc_[2] = data[6];
        imudata_temp.omega_[0] = data[1];
        imudata_temp.omega_[1] = data[2];
        imudata_temp.omega_[2] = data[3];
        imudata_v_.push_back(imudata_temp);//所有的imu数据

    }
    infile_imu.close();

    int preintegration_num = 0;

    preintegration_num = ComputeMeasurements(picture_stamp_, imudata_v_);

    std::cout << "We should comupute " << preintegration_num << " times preintegration according selected pictiures !"
              << std::endl;


    return 0;
}

