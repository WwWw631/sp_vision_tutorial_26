#pragma once
#include "hikrobot/include/MvCameraControl.h"
#include <opencv2/opencv.hpp>
#include <unordered_map>

class myCamera
{
public:
    myCamera();              
    ~myCamera();             
    cv::Mat read();          

private:
    void* handle_;          
    bool is_opened_;        

    cv::Mat convertFrame(MV_FRAME_OUT& raw);   
};