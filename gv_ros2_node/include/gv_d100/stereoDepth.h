/** Copyright 2023 GrandVision Inc, Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GV_STEREO_DEPTH_H
#define GV_STEREO_DEPTH_H
// #include <pybind11/pybind11.h>
// #include <pybind11/stl.h>
// #include <pybind11/stl_bind.h>
#include <memory>
#include <algorithm>
#include <vector>
#include <thread>
#include <functional>
#include <string>
#include "libuvc/libuvc.h"
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <libusb-1.0/libusb.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <ctime>
#include <assert.h>
//#include <opencv2/opencv.hpp>

#include "sendDataByUSB.h"
#include "depth2color.h"

namespace gv
{
    enum RgbFrameType{
      VGA = 0,
      HD = 1
    };

    class GvStereoDepth
    {
    public:
        GvStereoDepth();
        int initDeivce();
        std::vector<unsigned char> getAlignedDepthColorData(int idx);          
        void getPointCloud(int idx, std::vector<double> &pointcloud);
        void getPointCloud(int idx, int flag, const std::vector<unsigned char> &depthData, std::vector<double> &pointcloud);
        void getPointCloud(int idx, int flag, const std::vector<double> &depthData, std::vector<double> &pointcloud);
        void getAlignedDepthData(int idx, std::vector<double> &alignDepthData, std::vector<unsigned char> &alignColorData);
        void getAlignedDepthData(int idx, std::vector<double> &alignDepthData, std::vector<unsigned char> &alignColorData, std::vector<double> &alignPointcloud);
        void getAlignedHDDepthData(int idx, std::vector<double> &alignDepthData, std::vector<unsigned char> &alignColorData);
        std::vector<unsigned char> getIrStreamData(int idx);
        std::vector<unsigned char> getDepthStreamData(int idx);
        std::vector<unsigned char> getColorStreamData(int idx); 
        bool setGvDeviceStreamStatus(int idx, int status);
        bool setGvDeviceIrStatus(int idx, int status);
        bool setGvDeviceAeStatus(int idx, int status);
        bool setGvStreamFps(int idx, int fps);
        //bool setGvStreamBandwidth(int idx, int bandwidth);
        bool setGvStreamBandwidthAndRgbFormat(int idx , int bandwidth, int format, int type);
        bool setGvIRLightPower(int idx, int irPower);

        bool setGvExposureGain(const size_t &devIndex, const size_t &cameraIndex, 
            int autoManualExposure, float manualExposureValue, float manualGainValue);
        bool setGvSPWB(int addressIdx, int cameraIdx, int autoManualWb, float wbR, float wbGr, float wbGb, float wbB);
        bool setGvGamma(int addressIdx, int cameraIdx, int onOff);
        bool setGvSharpness(int addressIdx, int cameraIdx, int onOff, int denoiseLevel, int sharpnessLevel);
        //cproc id : brightness(1), saturation(3),  hue(2), contrast(4)
        bool setGvCrop(int addressIdx, int cameraIdx, int cprocId, int onOff, float value);

        const bool stopDeivce(void);
        int getDeviceNumber(void);
        int getDeviceInfo(void);

        bool getGvExposure(int addressIdx, int cameraIdx);
        bool getGvSPWB(int addressIdx, int cameraIdx);
        bool getGvSharpness(int addressIdx, int cameraIdx);
        bool getGvGamma(int addressIdx, int cameraIdx);
        bool getGvCrop(int addressIdx, int cameraIdx);

        //stereo deivce 
        int  initGvCamera();
        void startGvCamera();
        void quit();
        void resetDevices();
        bool enableCamera(int idx, bool enable = false, int bandwidth = 512, int format = 3, int type = 0);
        bool hardwareResetCamera();
        bool setGvRgbRatio( int rgbRatioCmdType);


        //paramsInterfaces;
        int getDeviceNum();
        std::vector<std::string> getDeviceSn();
        std::vector<std::string> getFirmwareVersion804();
        std::vector<std::string> getFirmwareVersion805();
        std::vector<std::string> getUsbPortId();

      public:
        typedef struct gv_camera_params
        {
          std::string devName;
          std::string cameraSn;
          std::string firmwareVersion804;
          std::string firmwareVersion805;
          std::string usbPortId;
          int width;
          int height;
          float fx;
          float fy;
          float cx;
          float cy;
          float baseline;
          std::vector<float> leftIRCameraMatrix;
          std::vector<float> leftIRDistCoeffs;
          std::vector<float> rightIRCameraMatrix;
          std::vector<float> rightIRDistCoeffs;
          std::vector<float> IR2IRRotationMatrix;
          std::vector<float> IR2IRTranslationMatrix;
          std::vector<float> leftRGBCameraMatrix;
          std::vector<float> leftRGBDistCoeffs;
          std::vector<float> rightRGBCameraMatrix;
          std::vector<float> rightRGBDistCoeffs;
          std::vector<float> IR2RGBRotation;
          std::vector<float> IR2RGBTranslation;
          std::vector<float> rightHdRGBCameraMatrix;
          std::vector<float> rightHdRGBDistCoeffs;
          std::vector<float> IR2RGBHdRotation;
          std::vector<float> IR2RGBHdTranslation;
        }GV_CAMERA_PARAMS;
      private:
        std::vector<GV_CAMERA_PARAMS> gvCameraParams_;
        
        std::vector<std::string> cameraSn_;
        std::vector<std::string> usbPortId_;
        std::vector<std::string> firmwareVersion804_;
        std::vector<std::string> firmwareVersion805_;
      public:
        std::vector<GV_CAMERA_PARAMS> getGvCameraParams();
        GV_CAMERA_PARAMS getCameraParams(int idx);

      public:
        typedef struct gv_dev_uvcusb_info
        {
          char sn[64];
          uint8_t address;
          uvc_device_t *dev;
          uvc_device_handle_t *devh;
          uvc_stream_ctrl_t ctrl;
          uvc_context_t *ctx;
          std::shared_ptr<std::vector<unsigned char>> irDataPtr;
          std::shared_ptr<std::vector<unsigned char>> depthDataPtr;
          std::shared_ptr<std::vector<unsigned char>> cloudDataPtr;
          std::shared_ptr<std::vector<unsigned char>> colorDataPtr;
          std::string port;
          bool isp_update_tag;
        } DEV_USB_UVC_GV_INFO;

        std::vector<unsigned char> mdata_;
        
      private: 
        int resetDeviceConnection(uint16_t vid, uint16_t pid);
        std::string get_device_port(libusb_device* usb_device);
        std::vector<DEV_USB_UVC_GV_INFO> query_and_open_gv_devs();
        void imageCallBack(uvc_frame_t *frame, void *ptr);
        static void uvcCallback(uvc_frame_t *frame, void *ptr);


      private:
        //camera params configure
        bool setImageChannel(const size_t &devIndex, bool bLeft, bool bRight, bool bDepth, bool bRGB);
        bool setIRPower(const size_t &devIndex, int value);
        bool setIRSwitch(const size_t &devIndex, bool swi);
        bool setAESwitch(const size_t &devIndex, bool swi);
        bool setFPS(const size_t &devIndex, int value);
        bool setUSBPacketLen(const size_t &devIndex, const size_t &n_dev_cnt);
        bool setDeviceEnable(const size_t &devIndex, bool enable = false);
        //bool setBandwidth(const size_t &devIndex, int bandwidth=1020);
        bool setBandwidthAndRgbFormat(const size_t &devIndex, int bandwidth, int format, int type);

        //bool setISPExposure(const size_t &devIndex, const size_t &cameraIndex, float autoExposureRange, float autoExposureTarget, 
        //  int autoManualExposure, float manualExposureValue, float manualGainValue, float resultExposure, float resultGain);
        bool setISPExposure(const size_t &devIndex, const size_t &cameraIndex, int autoManualExposure, float manualExposureValue, float manualGainValue);
        bool setISPSharpness(const size_t &devIndex, const size_t &cameraIndex, int onOff, int denoiseLevel, int sharpnessLevel);
        bool setISPGamma(const size_t &devIndex, const size_t &cameraIndex, int onOff);
        bool setISPWB(const size_t &devIndex, const size_t &cameraIndex, int autoManualWb, float wbR, float wbGr, float wbGb, float wbB);
        bool setISPCrop(const size_t &devIndex, ISP_Cproc_Control_t &data);

        bool readCalibration(const size_t &devIndex, Calc_Factor_t &fac);
        bool readSN(const size_t &devIndex, std::string &sn);

        bool getFirmwareVersion(const size_t &devIndex, CK_Version_t &version);
        bool getIRParam(const size_t &devIndex, IR_IR_Calib_Para_t &ir2ir);
        bool getRGBParam(const size_t &devIndex, IR_RGB_Calib_Para_t &ir2rgb);

        bool getISPExposure(const size_t &devIndex,  ISP_Exposure_Control_t &res);
        bool getISPSharpness(const size_t &devIndex,  ISP_Sharpness_Control_t &res);
        bool getISPGamma(const size_t &devIndex,  ISP_Gamma_Control_t &res);
        bool getISPWB(const size_t &devIndex,  ISP_WB_Control_t &res);
        bool getISPCrop(const size_t &devIndex,  ISP_Cproc_Control_t &res);

        SendDataByUSB sendDataTemp_;
        int USB_SEND_CMD_CYCL = 3;
        int nCyclTime_ = 0;

        uvc_context_t *ctx_;
        //std::vector<DEV_USB_UVC_GV_INFO> gvDevs_;
        uvc_device_t **devsList_;
        uint8_t devPtr_ [256];
        const int outMaxPacketSizeTable_ [7] = {0, 512,	1020,	1536,	2040,	2556,	3060};

        //depth2Rgb class
        gv::DepthAlignToColor depth2Color_;
        //camera matrix 
        std::vector<gv::DepthAlignToColor::Parameters> d2cParams_;
  };
}

#endif // GvStereoDepth_H