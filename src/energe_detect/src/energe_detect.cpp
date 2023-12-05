#include "energe_detect/energe_detect.hpp"

namespace EnergeDetect
{
    EnergeDetecter::EnergeDetecter(const int &ThresholdGray_,const int &ThresholdColor_,const std::string &DetectRuneColor_):
   ThresholdGray(ThresholdGray_),ThresholdColor(ThresholdColor_),DetectRuneColor(DetectRuneColor_)
    {

    }

    void EnergeDetecter::ImageDeal(cv::Mat &rgb_src)
    {
        cv::Mat img;
        rgb_src.copyTo(img);
        // ImgRect = cv::Rect(cv::Point(0, 0), rgb_src.size());

        // if (m_isDetected == true)
        // {
        //     getImageROI(m_final_target);
        //     ImgRectLast = ImgRect + ImgRectLast.tl();
        //     ImgRectLast = ImgRectLast & ImgRect;
        //     img = rgb_src(ImgRectLast);
        // }
        // else
        // {
        //     ImgRectLast = ImgRect;
        //     img = rgb_src;
        // }
        //获取亮度二值图
        cv::cvtColor(img, BinaryImg, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(BinaryImg, BinaryImg, cv::Size(3, 3), 0);
        cv::threshold(BinaryImg, BinaryImg, ThresholdGray, 255, cv::THRESH_BINARY);
        
        //获取颜色二值图
        std::vector<cv::Mat> channels;
        cv::split(img, channels);
        if (DetectRuneColor == "blue")
        {
            BinaryColorImg = channels.at(0) - channels.at(2);
            cv::threshold(channels.at(2),KeyPointImg , ThresholdKeyPoint, 255, cv::THRESH_BINARY);
        }
        else
        {
            BinaryColorImg = channels.at(2) - channels.at(0);
            cv::threshold(channels.at(0),KeyPointImg , ThresholdKeyPoint, 255, cv::THRESH_BINARY);
        }


        cv::threshold(BinaryColorImg, BinaryColorImg, ThresholdColor, 255, cv::THRESH_BINARY);

        cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
        cv::Mat element3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat element4 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
        cv::Mat element5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::Mat element6 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6));

        cv::dilate(BinaryColorImg, BinaryColorImg, element2);
        cv::dilate(KeyPointImg, KeyPointImg, element5);
        //得到最终处理后的图像
        cv::bitwise_and(BinaryColorImg, BinaryImg, ResultImg);
        cv::morphologyEx(ResultImg, ResultImg, cv::MORPH_CLOSE, element5);
    }

    void EnergeDetecter::Detect(cv::Mat &rgb_src)
    {
        ImageDeal(rgb_src);
        GetCenterAndFan(rgb_src);
    }

    void EnergeDetecter::SetParams(const int &ThresholdGray_,const int &ThresholdColor_,const int &ThresholdKeyPoint_)
    {
        
        this->ThresholdColor = ThresholdColor_;
        this->ThresholdGray = ThresholdGray_;
        this->ThresholdKeyPoint = ThresholdKeyPoint_;
    }

    void EnergeDetecter::GetCenterAndFan(cv::Mat &rgb_src)
    {
        rgb_src.copyTo(this->DrawImg);
        rgb_src.copyTo(this->FinalImg);
        EnergeArmors.clear();
        std::vector<int> possible_fan_indexs;
        bool isfindcenter = false;
        size_t center_index = 0;
        std::vector<cv::Vec4i> hierarchy;
        std::vector<cv::Vec4i> hierarchy2; 
        std::vector<std::vector<cv::Point>> contours;
        std::vector<std::vector<cv::Point>> contours2;
        cv::findContours(this->ResultImg,contours,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_NONE);
        cv::findContours(this->KeyPointImg,contours2,hierarchy2,cv::RETR_CCOMP,cv::CHAIN_APPROX_NONE);

        if(contours.size()<2)
            return;

        for(size_t i = 0;i<contours.size();i++)
        {
            if(FindFan(contours[i])){
                possible_fan_indexs.push_back(i);
            }
        }
        for(size_t i = 0;i<contours.size();i++)
        {
            if(std::find(possible_fan_indexs.begin(),possible_fan_indexs.end(),i) != possible_fan_indexs.end())
                continue;
            else{
                if(FindCenter(contours[i])){
                    center_index = i;
                    isfindcenter = true;
                    break;
                }
                    
            }
        }

        if(!isfindcenter)
            return;
        for(size_t i = 0;i<contours.size()-1;i++){
            if(std::find(possible_fan_indexs.begin(),possible_fan_indexs.end(),i) != possible_fan_indexs.end() || i == center_index)
                continue;
            if(contours[i].size()<190)
                continue;
            auto rect1 = cv::minAreaRect(contours[i]); 
            if(IsSign(rect1))
                continue;
            for(size_t j = i+1;j<contours.size();j++){
                if(std::find(possible_fan_indexs.begin(),possible_fan_indexs.end(),j) != possible_fan_indexs.end() || j == center_index)
                    continue;
                if(contours[j].size()<190)
                    continue;
                auto rect2 = cv::minAreaRect(contours[j]); 
                if(IsSign(rect2))
                    continue;
                for(auto &it:EnergeArmors){
                    if(it.rect.contains(rect1.center)&&it.rect.contains(rect2.center)){
                        if(IsMatch(rect1,rect2)){
                            it.Power = true;
                            it.IsFan = true;
                        }
                            
                    }
                }
            }
        }

        if(contours2.size() < 2)
            return;
        for(size_t i = 0;i<contours2.size()-1;i++){
            auto rect1 = cv::minAreaRect(contours2[i]); 
            if(IsKeyPointRect(rect1)){
                for(size_t j = i+1;j<contours2.size();j++){
                    auto rect2 = cv::minAreaRect(contours2[j]); 
                    if(IsKeyPointRect(rect2)){
                        for(auto &it:EnergeArmors){
                            if(it.rect.contains(rect1.center) && it.rect.contains(rect2.center)){
                                if(IsKeyPointMatch(rect1,rect2)){
                                    cv::Scalar color(225,105,65);
                                    DrawPotateRect(rect1,color);
                                    DrawPotateRect(rect2,color);
                                    it(rect1,rect2);
                                    it.IsFan = true;
                                }                                
                            }
                        }
                    }
                }
            }
        }

        for(auto it = EnergeArmors.begin();it < EnergeArmors.end();it++){
            if(it->IsFan){
                if(it->Power){
                    cv::Scalar color(255,255,0);
                    DrawPotateRect(it->minract,color);
                }else
                    DrawPoint(*it);                
            }
        }
    }

    bool EnergeDetecter::FindFan(std::vector<cv::Point> &contour)
    {
        if(contour.size()<600)
            return false;
        cv::RotatedRect rotatedrect = cv::minAreaRect(contour);
        cv::Rect rect = cv::boundingRect(contour);
        cv::Point2f rectPoints[4];
        rotatedrect.points(rectPoints);
        double proportion;
        double width = cv::norm(rectPoints[0]-rectPoints[1]);
        double lenght = cv::norm(rectPoints[0]-rectPoints[3]);
        if(width>lenght)
            proportion = lenght/width;
        else
            proportion = width/lenght;
        if((proportion <= 0.49 || proportion >= 0.56))
            return false;
        EnergeArmors.push_back(EnergeArmor(rotatedrect,rect));

        if(debug){
            cv::Scalar color(113,179,60);
            DrawRect(rect);
            DrawPotateRect(rotatedrect,color);
        }

        return true;
    }

    bool EnergeDetecter::FindCenter(std::vector<cv::Point> &contour)
    {
        if(contour.size()<90 || contour.size()>250)
            return false;
        cv::RotatedRect rotatedrect = cv::minAreaRect(contour);
        cv::Point2f rectPoints[4];
        rotatedrect.points(rectPoints);
        double proportion;
        double width = cv::norm(rectPoints[0]-rectPoints[1]);
        double lenght = cv::norm(rectPoints[0]-rectPoints[3]);
        if(width>lenght)
            proportion = lenght/width;
        else
            proportion = width/lenght;
        if(proportion <= 0.85)
            return false;

        for(auto it:EnergeArmors)
        {
            if(it.rect.contains(rotatedrect.center))
                return false;
        }

        for(auto &it:EnergeArmors)
        {
            it.Center = rotatedrect.center;
        }

        if(debug)
            cv::circle(this->DrawImg,rotatedrect.center,3,cv::Scalar(219,112,147),5);
        return true;
    }

    bool EnergeDetecter::IsSign(cv::RotatedRect &rect){
        cv::Point2f rectPoints[4];
        rect.points(rectPoints);
        double proportion;
        double width = cv::norm(rectPoints[0]-rectPoints[1]);
        double lenght = cv::norm(rectPoints[0]-rectPoints[3]);
        if(width>lenght)
            proportion = lenght/width;
        else
            proportion = width/lenght;
        if(proportion <= 0.37 || proportion >= 0.4)
            return true;
        return false;
    }

    bool EnergeDetecter::IsMatch(cv::RotatedRect &rect1,cv::RotatedRect &rect2)
    {
        cv::Point2f rectPoints1[4];
        rect1.points(rectPoints1);
        double width1 = cv::norm(rectPoints1[0]-rectPoints1[1]);
        double lenght1 = cv::norm(rectPoints1[0]-rectPoints1[3]);
        if(lenght1 < width1)
            std::swap(width1,lenght1);
        cv::Point2f rectPoints2[4];
        rect2.points(rectPoints2);
        double width2 = cv::norm(rectPoints2[0]-rectPoints2[1]);
        double lenght2 = cv::norm(rectPoints2[0]-rectPoints2[3]);
        if(lenght2 < width2)
            std::swap(width2,lenght2);
        double CenterDistance = cv::norm(rect1.center-rect2.center);
        double proportion = 2*CenterDistance/(lenght1+lenght2);
        if(proportion >=0.61 ||  proportion <=0.57)
            return false;
        if(debug){
            cv::Scalar color(0,69,255);
            DrawPotateRect(rect1,color);
            DrawPotateRect(rect2,color);
        }
        return true;
    }

    bool EnergeDetecter::IsKeyPointRect(cv::RotatedRect rect){
        cv::Point2f rectPoints[4];
        rect.points(rectPoints);
        double proportion;
        double width = cv::norm(rectPoints[0]-rectPoints[1]);
        double lenght = cv::norm(rectPoints[0]-rectPoints[3]);
        if(width>lenght)
            proportion = lenght/width;
        else
            proportion = width/lenght;
        if((proportion>0.38 && proportion<0.41)||(proportion>0.77 && proportion <0.8))
            return true;
        return false;
    }

    bool EnergeDetecter::IsKeyPointMatch(cv::RotatedRect &rect1,cv::RotatedRect &rect2){
        cv::Point2f rectPoints1[4];
        rect1.points(rectPoints1);
        double width1 = cv::norm(rectPoints1[0]-rectPoints1[1]);
        double lenght1 = cv::norm(rectPoints1[0]-rectPoints1[3]);
        if(lenght1 < width1)
            std::swap(width1,lenght1);
        cv::Point2f rectPoints2[4];
        rect2.points(rectPoints2);
        double width2 = cv::norm(rectPoints2[0]-rectPoints2[1]);
        double lenght2 = cv::norm(rectPoints2[0]-rectPoints2[3]);
        if(lenght2 < width2)
            std::swap(width2,lenght2);
        float proportion1 = width1/lenght1; 
        float proportion2 = width2/lenght2; 
        if(((proportion1 > 0.38 && proportion1 < 0.41) && (proportion2 > 0.77 && proportion2 <0.8))||
            ((proportion2 > 0.38 && proportion2 < 0.41) && (proportion1 > 0.77 && proportion1 <0.8)))
        {
            if((proportion2 > 0.38 && proportion2 < 0.41) && (proportion1 > 0.77 && proportion1 <0.8))
                std::swap(rect1,rect2);
            return true;
        }
        return false;
    }

    bool EnergeDetecter::GetFanKeyPoint(std::vector<cv::Point> &contour)
    {

        return true;
    }
    cv::Mat EnergeDetecter::GetBinaryImg(void)
    {
        return this->BinaryImg;
    }

    cv::Mat EnergeDetecter::GetBinaryColorImg(void)
    {
        return this->BinaryColorImg;
    }

    cv::Mat EnergeDetecter::GetKeyPointImg(void)
    {
        return this->KeyPointImg;
    }

    cv::Mat EnergeDetecter::GetResultImg(void)
    {
        return this->ResultImg;
    }

    cv::Mat EnergeDetecter::GetDrawImg(void)
    {
        return this->DrawImg;
    }

    cv::Mat EnergeDetecter::GetFinalImg(void)
    {
        return this->FinalImg;
    }

    void EnergeDetecter::DrawRect(cv::Rect &rect){
        cv::rectangle(this->DrawImg,rect.tl(),rect.br(),cv::Scalar(0,255,0),3.0);
    }

    void EnergeDetecter::DrawPotateRect(cv::RotatedRect &rect,cv::Scalar &color){
        cv::Point2f points[4];
        rect.points(points);
        for(size_t i = 0;i<4;i++)
            cv::line(this->DrawImg,points[i],points[(i+1)%4],color,3);
    }

    void EnergeDetecter::DrawPoint(EnergeArmor &armor){
        cv::circle(this->FinalImg,armor.points[0],3,cv::Scalar(50,205,50),5);
        cv::circle(this->FinalImg,armor.points[1],3,cv::Scalar(50,205,50),5);
        cv::circle(this->FinalImg,armor.points[2],3,cv::Scalar(50,205,50),5);
        cv::circle(this->FinalImg,armor.points[3],3,cv::Scalar(50,205,50),5);
        cv::circle(this->FinalImg,armor.Center,3,cv::Scalar(50,205,50),10);
    }

}