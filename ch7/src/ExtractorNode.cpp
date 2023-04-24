#include "ORB/ExtractorNode.hpp"

namespace ORB {
    void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4) {
        //得到当前提取器节点所在图像区域的一半长宽，当然结果需要取整
        const int halfX = std::ceil(static_cast<float>(UR.x - UL.x) / 2);
        const int halfY = std::ceil(static_cast<float>(BR.y - UL.y) / 2);

        //Define boundaries of childs
        //下面的操作大同小异，将一个图像区域再细分成为四个小图像区块
        //n1 存储左上区域的边界
        n1.UL = UL;
        n1.UR = cv::Point2i(UL.x + halfX, UL.y);
        n1.BL = cv::Point2i(UL.x, UL.y + halfY);
        n1.BR = cv::Point2i(UL.x + halfX, UL.y + halfY);
        //用来存储在该节点对应的图像网格中提取出来的特征点的vector
        n1.vKeys.reserve(vKeys.size());

        //n2 存储右上区域的边界
        n2.UL = n1.UR;
        n2.UR = UR;
        n2.BL = n1.BR;
        n2.BR = cv::Point2i(UR.x, UL.y + halfY);
        n2.vKeys.reserve(vKeys.size());

        //n3 存储左下区域的边界
        n3.UL = n1.BL;
        n3.UR = n1.BR;
        n3.BL = BL;
        n3.BR = cv::Point2i(n1.BR.x, BL.y);
        n3.vKeys.reserve(vKeys.size());

        //n4 存储右下区域的边界
        n4.UL = n3.UR;
        n4.UR = n2.BR;
        n4.BL = n3.BR;
        n4.BR = BR;
        n4.vKeys.reserve(vKeys.size());

        //Associate points to childs
        //遍历当前提取器节点的vkeys中存储的特征点
        for (size_t i = 0; i < vKeys.size(); i++) {
            //获取这个特征点对象
            const cv::KeyPoint &kp = vKeys[i];
            //判断这个特征点在当前特征点提取器节点图像的哪个区域，更严格地说是属于那个子图像区块
            //然后就将这个特征点追加到那个特征点提取器节点的vkeys中
            //NOTICE BUG REVIEW 这里也是直接进行比较的，但是特征点的坐标是在“半径扩充图像”坐标系下的，而节点区域的坐标则是在“边缘扩充图像”坐标系下的
            if (kp.pt.x < n1.UR.x) {
                if (kp.pt.y < n1.BR.y)
                    n1.vKeys.push_back(kp);
                else
                    n3.vKeys.push_back(kp);
            } else if (kp.pt.y < n1.BR.y)
                n2.vKeys.push_back(kp);
            else
                n4.vKeys.push_back(kp);
        }//遍历当前提取器节点的vkeys中存储的特征点

        //判断每个子特征点提取器节点所在的图像中特征点的数目（就是分配给子节点的特征点数目），然后做标记
        //这里判断是否数目等于1的目的是确定这个节点还能不能再向下进行分裂
        if (n1.vKeys.size() == 1)
            n1.bNoMore = true;
        if (n2.vKeys.size() == 1)
            n2.bNoMore = true;
        if (n3.vKeys.size() == 1)
            n3.bNoMore = true;
        if (n4.vKeys.size() == 1)
            n4.bNoMore = true;
    }
}