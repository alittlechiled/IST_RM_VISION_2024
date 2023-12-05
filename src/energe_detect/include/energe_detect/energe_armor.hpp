#ifndef _ENERGE_ARMOR_HPP_
#define _ENERGE_ARMOR_HPP_
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

namespace EnergeDetect
{
    struct EnergeArmor
    {
        cv::RotatedRect minract;
        cv::Rect rect;
        std::vector<cv::Point2f> points;//the key point order is tl tr br bl;
        cv::Point Center;
        cv::Point3f Point_3d;
        bool Power = false;
        bool IsFan = false;
        float R;
        EnergeArmor(const cv::RotatedRect &minract_,cv::Rect rect_)
        {
            minract = minract_;
            rect = rect_;
            
        }

        void operator()(cv::RotatedRect Small,cv::RotatedRect Big)
        {
            cv::Point2f SmallPoints[4],BigPoints[4],SmallKeyPoints[2],BigKeyPoints[2];
            cv::Point2f SmallCenterPoint,BigCenterPoint;
            cv::Point3f CenterVector,TempVector;
            float distance1,distance2,distance3,distance4;
            std::vector<std::pair<float,int>> distances;
            std::pair<float,int> MinDistance;
            Small.points(SmallPoints);SmallCenterPoint = Small.center;
            Big.points(BigPoints);BigCenterPoint = Big.center;
            CenterVector = cv::Point3f((SmallCenterPoint-BigCenterPoint).x,(SmallCenterPoint-BigCenterPoint).y,0.0);


            distance1 = cv::norm((SmallPoints[0] + SmallPoints[1])/2-BigCenterPoint);
            distance2 = cv::norm((SmallPoints[1] + SmallPoints[2])/2-BigCenterPoint);
            distance3 = cv::norm((SmallPoints[2] + SmallPoints[3])/2-BigCenterPoint);
            distance4 = cv::norm((SmallPoints[3] + SmallPoints[0])/2-BigCenterPoint);

            distances.push_back(std::pair<float,int>(distance1,1));
            distances.push_back(std::pair<float,int>(distance2,2));
            distances.push_back(std::pair<float,int>(distance3,3));
            distances.push_back(std::pair<float,int>(distance4,4));

            std::sort(distances.begin(),distances.end(),[](const std::pair<float,int> &lhs,const std::pair<float,int> &rhs){
                return lhs.first < rhs.first;
            });
    
            MinDistance = distances[0];

            if(MinDistance.second == 1)
            {
                SmallKeyPoints[0] = SmallPoints[0];
                SmallKeyPoints[1] = SmallPoints[1];
            }
            else if(MinDistance.second == 2)
            {
                SmallKeyPoints[0] = SmallPoints[1];
                SmallKeyPoints[1] = SmallPoints[2];
            }
            else if(MinDistance.second == 3)
            {
                SmallKeyPoints[0] = SmallPoints[2];
                SmallKeyPoints[1] = SmallPoints[3];
            }
            else if(MinDistance.second == 4)
            {
                SmallKeyPoints[0] = SmallPoints[3];
                SmallKeyPoints[1] = SmallPoints[0]; 
            }

            TempVector = cv::Point3f((SmallKeyPoints[0]-SmallKeyPoints[1]).x,(SmallKeyPoints[0]-SmallKeyPoints[1]).y,0.0);

            auto Resualt = CenterVector.cross(TempVector);

            if(Resualt.z < 0)
                std::swap(SmallKeyPoints[0],SmallKeyPoints[1]);

            distances.clear();

            distance1 = cv::norm((BigPoints[0] + BigPoints[1])/2-SmallCenterPoint);
            distance2 = cv::norm((BigPoints[1] + BigPoints[2])/2-SmallCenterPoint);
            distance3 = cv::norm((BigPoints[2] + BigPoints[3])/2-SmallCenterPoint);
            distance4 = cv::norm((BigPoints[3] + BigPoints[0])/2-SmallCenterPoint);

            distances.push_back(std::pair<float,int>(distance1,1));
            distances.push_back(std::pair<float,int>(distance2,2));
            distances.push_back(std::pair<float,int>(distance3,3));
            distances.push_back(std::pair<float,int>(distance4,4));

            std::sort(distances.begin(),distances.end(),[](const std::pair<float,int> &lhs,const std::pair<float,int> &rhs){
                return lhs.first < rhs.first;
            });
    
            MinDistance = distances[0];

            if(MinDistance.second == 1)
            {
                BigKeyPoints[0] = BigPoints[0];
                BigKeyPoints[1] = BigPoints[1];
            }
            else if(MinDistance.second == 2)
            {
                BigKeyPoints[0] = BigPoints[1];
                BigKeyPoints[1] = BigPoints[2];
            }
            else if(MinDistance.second == 3)
            {
                BigKeyPoints[0] = BigPoints[2];
                BigKeyPoints[1] = BigPoints[3];
            }
            else if(MinDistance.second == 4)
            {
                BigKeyPoints[0] = BigPoints[3];
                BigKeyPoints[1] = BigPoints[0]; 
            }

            TempVector = cv::Point3f((BigKeyPoints[0]-BigKeyPoints[1]).x,(BigKeyPoints[0]-BigKeyPoints[1]).y,0.0);

            Resualt = CenterVector.cross(TempVector);//judge the points' position by using the cross

            if(Resualt.z > 0)
                std::swap(BigKeyPoints[0],BigKeyPoints[1]);

            points.push_back(SmallKeyPoints[0]);
            points.push_back(SmallKeyPoints[1]);
            points.push_back(BigKeyPoints[0]);
            points.push_back(BigKeyPoints[1]);

        }
    };
}

#endif