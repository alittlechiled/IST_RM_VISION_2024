#ifndef _ENERGE_DETECT_HPP_
#define _ENERGE_DETECT_HPP_
#include <opencv2/opencv.hpp>
#include <energe_detect/energe_armor.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

namespace EnergeDetect
{
    class EnergeDetecter
    {
        public:
            EnergeDetecter(const int &ThresholdGray_,const int &ThresholdColor_,const std::string &DetectRuneColor_);
            void SetParams(const int &ThresholdGray_,const int &ThresholdColor_,const int &ThresholdKeyPoint_);
            cv::Mat GetBinaryImg(void);
            cv::Mat GetBinaryColorImg(void);
            cv::Mat GetKeyPointImg(void);
            cv::Mat GetResultImg(void);
            cv::Mat GetDrawImg(void);
            cv::Mat GetFinalImg(void);
            void Detect(cv::Mat &rgb_src);
        private:
            void ImageDeal(cv::Mat &rgb_src);
            void GetCenterAndFan(cv::Mat &rgb_src);
            bool FindFan(std::vector<cv::Point> &contour);
            bool FindCenter(std::vector<cv::Point> &contour);
            bool GetFanKeyPoint(std::vector<cv::Point> &contour);
            bool IsSign(cv::RotatedRect &rect);
            bool IsMatch(cv::RotatedRect &rect1,cv::RotatedRect &rect2);

            bool IsKeyPointRect(cv::RotatedRect rect);
            bool IsKeyPointMatch(cv::RotatedRect &rect1,cv::RotatedRect &rect2);

            void DrawRect(cv::Rect &rect);

            void DrawPoint(EnergeArmor &armor);

            void DrawPotateRect(cv::RotatedRect &rect,cv::Scalar &color);

            cv::Mat BinaryImg;
            cv::Mat BinaryColorImg;
            cv::Mat KeyPointImg;
            cv::Mat ResultImg;


            cv::Mat DrawImg;
            cv::Mat FinalImg;

            std::vector<EnergeArmor> EnergeArmors;

            int ThresholdGray;
            int ThresholdColor;
            int ThresholdKeyPoint;
            std::string DetectRuneColor;

            bool debug = true;
    };
}

#endif