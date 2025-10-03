#include "my_camera.hpp"
#include <iostream>

myCamera::myCamera()
    : handle_(nullptr), is_opened_(false)
{
    MV_CC_DEVICE_INFO_LIST device_list;
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK) {
        std::cerr << "[myCamera] Enum devices failed: " << ret << std::endl;
        return;
    }

    if (device_list.nDeviceNum == 0) {
        std::cerr << "[myCamera] No camera found." << std::endl;
        return;
    }

    ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) {
        std::cerr << "[myCamera] Create handle failed: " << ret << std::endl;
        return;
    }

    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
        std::cerr << "[myCamera] Open device failed: " << ret << std::endl;
        return;
    }

    // 相机参数设置
    MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(handle_, "ExposureTime", 10000);
    MV_CC_SetFloatValue(handle_, "Gain", 20);
    MV_CC_SetFrameRate(handle_, 60);

    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
        std::cerr << "[myCamera] Start grabbing failed: " << ret << std::endl;
        return;
    }

    is_opened_ = true;
    std::cout << "[myCamera] Camera initialized successfully." << std::endl;
}

myCamera::~myCamera()
{
    if (is_opened_) {
        MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        std::cout << "[myCamera] Camera released." << std::endl;
    }
}

cv::Mat myCamera::read()
{
    if (!is_opened_) {
        std::cerr << "[myCamera] Camera not opened!" << std::endl;
        return cv::Mat();
    }

    MV_FRAME_OUT raw;
    int ret = MV_CC_GetImageBuffer(handle_, &raw, 100);
    if (ret != MV_OK) {
        std::cerr << "[myCamera] GetImageBuffer failed: " << ret << std::endl;
        return cv::Mat();
    }

    cv::Mat img = convertFrame(raw);

    MV_CC_FreeImageBuffer(handle_, &raw);
    return img;
}

cv::Mat myCamera::convertFrame(MV_FRAME_OUT& raw)
{
    cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

    auto pixel_type = raw.stFrameInfo.enPixelType;
    const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2BGR},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2BGR},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2BGR},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2BGR}
    };

    auto it = type_map.find(pixel_type);
    if (it != type_map.end()) {
        cv::Mat bgr;
        cv::cvtColor(img, bgr, it->second);
        return bgr;
    } 
    
    else {
        return img.clone();
    }
}
