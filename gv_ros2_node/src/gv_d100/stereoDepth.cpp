#include "gv_d100/stereoDepth.h"

//gv::GvStereoDepth gvCamTemp_;
std::vector<gv::GvStereoDepth::DEV_USB_UVC_GV_INFO> gvDevs_;

namespace gv {
GvStereoDepth::GvStereoDepth()
{
  std::cout << " GvStereoDepth !" << std::endl;
}

int GvStereoDepth::initDeivce()
{
  std::cout << "initDevice" << std::endl;
  int res = initGvCamera();
  std::cout << "res : " << res << std::endl;
  if(res)
  {
    startGvCamera();
    return 1;
  }
  else
  {
    return 0;
  }
}

std::string GvStereoDepth::get_device_port(libusb_device* usb_device)
{
    auto usb_bus = std::to_string(libusb_get_bus_number(usb_device));
    // As per the USB 3.0 specs, the current maximum limit for the depth is 7.
    const auto max_usb_depth = 8;
    uint8_t usb_ports[max_usb_depth] = {};
    std::stringstream port_path;
    auto port_count = libusb_get_port_numbers(usb_device, usb_ports, max_usb_depth);
    libusb_device_descriptor dev_desc;
    auto r= libusb_get_device_descriptor(usb_device,&dev_desc);

    for (size_t i = 0; i < port_count; ++i)
    {
        port_path << std::to_string(usb_ports[i]) << (((i+1) < port_count)?".":"");
    }
    return usb_bus + "-" + port_path.str();
}

std::vector<GvStereoDepth::DEV_USB_UVC_GV_INFO> GvStereoDepth::query_and_open_gv_devs() {
  std::vector<DEV_USB_UVC_GV_INFO> res_list;
  uvc_error_t res = uvc_init(&ctx_, NULL);
  //std::cout << "uvc_init : " << res << std::endl;
  if (res < 0) {
    uvc_perror(res, "uvc_init");
    return res_list;
  }

  res = uvc_get_device_list(ctx_, &devsList_);
  //std::cout << "uvc get device list : " << res << std::endl;
  //std::cout << "device list : " << *devsList_ << std::endl;
  if (res < 0) {
    uvc_perror(res, "uvc_get_device_list");
    return res_list;
  }
  int idx_ = 0;
  while (1) {
    uvc_device_t *dev = devsList_[idx_];
    if (dev == NULL) {
      break;
    }
    uvc_device_handle_t *devh;
    idx_++;
    uvc_device_descriptor_t *desc;
    res = uvc_get_device_descriptor(dev, &desc);
    if (res == UVC_SUCCESS && desc->idProduct == 0x0017 && desc->idVendor == 0x04cc) 
    {
      printf("Device found index %d; address: %d\n", idx_, uvc_get_device_address(dev));
      uvc_free_device_descriptor(desc);
      res = uvc_open(dev, &devh);
      //std::cout << "uvc open : " << res << std::endl;
      if (res < 0) {
        if ((int)res == -3) {
          static int tag_dev_deinded = 0;
          tag_dev_deinded ++;
          std::cout << tag_dev_deinded <<" uvc devices access deinded " <<" \n please check the permission of the camera.  " << std::endl;
          continue;
        } else {
          uvc_perror(res, "uvc_open"); /* unable to open device */
          std::cout << "ubable to open device " << std::endl;
          continue;
        }
      }
    // } else {
    //   // puts("Device opened");
    //   // uvc_print_diag(devh, stderr);
    // }
  
    } else {
      uvc_free_device_descriptor(desc);
      //uvc_close(devh);
      continue;
    }

    DEV_USB_UVC_GV_INFO a = {
      "", // [ex: gv + (n * ' ')(dev_custom_tag) -16-][ex: 202303060007(dev_special_id_tag) -48-]
      uvc_get_device_address(dev),
      dev,
      devh,
      {},
      ctx_,
      // nullptr,
      // nullptr,
      // nullptr,
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      "",
      false
    };
    res_list.push_back(a);
  }

  int resetStatus = 0;
  libusb_context * context;
  libusb_init(&context);

  libusb_device ** lists;
  size_t count_l =  libusb_get_device_list(context, &lists);

  size_t idx = 0;
  while (1) {
    libusb_device *dev = lists[idx];
    struct libusb_device_descriptor kk;
    if (dev == NULL) {
      break;
    }
    libusb_get_device_descriptor(dev,  &kk);
    if (kk.idVendor == 0x04cc && kk.idProduct == 0x0017 ) {
      auto address = libusb_get_device_address(dev);
      struct libusb_device_handle* dev_handle;
      libusb_open(dev, &dev_handle);
      if (dev_handle == NULL){
        printf("error!\n");
        continue;
      }
      for (size_t i = 0; i < res_list.size() ; i++) {
        if(res_list[i].address == address) {
          res_list[i].port = get_device_port(dev);
        }
      }
      libusb_close(dev_handle);
      dev_handle=NULL;
    }
    idx ++;
  }
  libusb_exit(context);

  return res_list;
}

int GvStereoDepth::resetDeviceConnection(uint16_t vid, uint16_t pid) {
    /*Open libusb*/
    int resetStatus = 0;
    libusb_context * context;
    libusb_init(&context);

    libusb_device ** lists;
    size_t res =  libusb_get_device_list(context, &lists);

    size_t idx = 0;
    while (1) {
      libusb_device *dev = lists[idx];
      struct libusb_device_descriptor kk;
      if (dev == NULL) {
        break;
      }
      libusb_get_device_descriptor(dev,  &kk);
      if (kk.idVendor == vid && kk.idProduct == pid ) {
        printf("Gv camera found ---> device address : %d\n", libusb_get_device_address(dev));
        // printf("Device found %d\n", idx);
        struct libusb_device_handle* dev_handle;
        int result = libusb_open(dev, &dev_handle);
        if (result != LIBUSB_SUCCESS)
        {
          std::cout << "open libusb device faile !" << std::endl;
          resetStatus = 1;
          std::string command = "ps -ef | grep 'uvcdynctrl -d' | grep -v grep | cut -c 9-16 | xargs sudo kill -9";
          system(command.c_str());
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          continue;
        }
        if (dev_handle == NULL){
          printf("usb resetting unsuccessful! No matching device found, or error encountered!\n");
          resetStatus = 1;
          continue;
        }
        libusb_reset_device(dev_handle);
      }
      
      idx ++;
    }
    libusb_exit(context);
    return resetStatus;
}

void GvStereoDepth::resetDevices() {
  resetDeviceConnection(0x04cc,0x0017);
}

/* This callback function runs once per frame. Use it to perform any
* quick processing you need, or have it put the frame into your application's
* input queue. If this function takes too long, you'll start losing frames. */
//to do : static 
void GvStereoDepth::uvcCallback(uvc_frame_t *frame, void *ptr) {
  gv::GvStereoDepth *gvPtr = (gv::GvStereoDepth*)ptr;
  //std::cout << "uvcCallback gv dev size : " << gvDevs_.size() << std::endl;
  gvPtr->imageCallBack(frame, ptr);
}

void GvStereoDepth::imageCallBack(uvc_frame_t *frame, void *ptr)
{
  if(!frame)  {
    std::cout << "bad frame!!!!" << std::endl;
    return;
  }
  uint8_t id = *((uint8_t *)ptr);
  //std::cout << "id : " << static_cast<int>(id) << std::endl;
  //std::cout << "gvDevs size : " << gvDevs_.size() << std::endl;
  // static int i = 0;
  uvc_error_t ret;
  uint32_t data_bytes;
  switch (frame->frame_format) {
  case UVC_COLOR_FORMAT_YUYV: 
  {
    //std::cout << "uvc format : UVC_COLOR_FORMAT_YUYV " << std::endl;

    unsigned char bt = ( (unsigned char *)frame->data )[0];
    unsigned char bt_rgb= ( (unsigned char *)frame->data )[12];

    if (bt == 1) {
      if (frame->data_bytes < 640 * 800) {
        std::cout << "bad frame!!!!" << std::endl;
        return;
      }
      if(frame->data_bytes == 513280)
      { 
        auto ptr_buff_ir = std::make_shared<std::vector<unsigned char>>(frame->data_bytes);
        memcpy(ptr_buff_ir->data(), frame->data, frame->data_bytes);
        //std::cout << "ir data " << std::endl;
        //if (gvDevs_[id].irfunc) gvDevs_[id].irfunc(ptr_buff_ir);
        //std::cout << "ptr buff ir size : " << ptr_buff_ir->size()<< std::endl;
        //   //mdata = ptr_buff_ir->data();
        // }
        gvDevs_[id].irDataPtr = ptr_buff_ir;
        //std::cout << "ir data ptr size : " << gvDevs_[id].irDataPtr->size() << std::endl;
      }
    }
    if (bt == 3) {
      if (frame->data_bytes < 640 * 400 * 2) {
        std::cout << "bad frame!!!!" << std::endl;
        return;
      }
      if(frame->data_bytes == 513280)
      {
        auto ptr_buff_depth = std::make_shared<std::vector<unsigned char>>(frame->data_bytes);
        memcpy(ptr_buff_depth->data(), frame->data, frame->data_bytes);
        gvDevs_[id].depthDataPtr = ptr_buff_depth;
        //std::cout << "depth data ptr size : " << gvDevs_[id].depthDataPtr->size() << std::endl;
        //if (gvDevs_[id].depthfunc) gvDevs_[id].depthfunc(ptr_buff_depth);
      }
    }
    if (bt_rgb == 5) {
      if(frame->data_bytes < 513280)
      {
        auto ptr_buff_rgb = std::make_shared<std::vector<unsigned char>>(frame->data_bytes + 1);
        memcpy(ptr_buff_rgb->data(), frame->data, frame->data_bytes);
        gvDevs_[id].colorDataPtr = ptr_buff_rgb;
      }
      //std::cout << "color data ptr size: " << gvDevs_[id].colorDataPtr->size() << std::endl;
      //if (gvDevs_[id].colorfunc) gvDevs_[id].colorfunc(ptr_buff_rgb);
    }
    break;
  }
  case UVC_FRAME_FORMAT_MJPEG:
  {
    //std::cout << "uvc format : UVC_FRAME_FORMAT_MJPEG " << std::endl;
    unsigned char bt = ((unsigned char *)frame->data)[0];
    unsigned char bt_rgb = ((unsigned char *)frame->data)[12];

    if (bt == 1)
    {
      if (frame->data_bytes < 640 * 400)
      {
        std::cout << "bad frame!!!!" << std::endl;
        return;
      }
      // if(frame->data_bytes == 513280)
      //{
      auto ptr_buff_ir = std::make_shared<std::vector<unsigned char>>(frame->data_bytes);
      memcpy(ptr_buff_ir->data(), frame->data, frame->data_bytes);
      gvDevs_[id].irDataPtr = ptr_buff_ir;
    }
    if (bt == 3)
    {
      if (frame->data_bytes < 640 * 400 * 2)
      {
        std::cout << "bad frame!!!!" << std::endl;
        return;
      }
      if (frame->data_bytes == 513280)
      {
        auto ptr_buff_depth = std::make_shared<std::vector<unsigned char>>(frame->data_bytes);
        memcpy(ptr_buff_depth->data(), frame->data, frame->data_bytes);
        gvDevs_[id].depthDataPtr = ptr_buff_depth;
      }
      else
      {
        return;
      }
    }
    if (bt_rgb == 5)
    {
      auto ptr_buff_rgb = std::make_shared<std::vector<unsigned char>>(frame->data_bytes + 1);
      memcpy(ptr_buff_rgb->data(), frame->data, frame->data_bytes);
      gvDevs_[id].colorDataPtr = ptr_buff_rgb;
    }
    break;
  }
  default:
    // todo
    break;
  }
}

int GvStereoDepth::initGvCamera() 
{
  //on_off_tag_ = true;
  for (int i = 0; i < 256; i++) 
  {
    devPtr_[i] = i;
  }
  resetDeviceConnection(0x04cc,0x0017);

  gvDevs_ = query_and_open_gv_devs();
  //std::cout << "query_and_open_gv_devs size:" << gvDevs_.size() << std::endl;
  if (gvDevs_.size() < 1) 
  {
    std::cout << "No device found." << std::endl;
    exit(-1);
    return -1;
  }
  gvCameraParams_.resize(gvDevs_.size());
  d2cParams_.resize(gvDevs_.size());

  for (size_t idx_local_current = 0; idx_local_current < gvDevs_.size(); idx_local_current ++) 
  {
    std::string gv_dev = "device";
    const size_t idx_curent = gvDevs_[idx_local_current].address;
    gvCameraParams_[idx_local_current].devName = gv_dev + std::to_string(idx_local_current);
    gvCameraParams_[idx_local_current].usbPortId = gvDevs_[idx_local_current].port;
    usbPortId_.push_back(gvDevs_[idx_local_current].port);

    Calc_Factor_t calFac;
    if(readCalibration(idx_curent, calFac))
    {
      gvCameraParams_[idx_local_current].fx = calFac.focus;
      gvCameraParams_[idx_local_current].fy = calFac.focus;
      gvCameraParams_[idx_local_current].cx = calFac.algo_cx;
      gvCameraParams_[idx_local_current].cy = calFac.algo_cy;
      gvCameraParams_[idx_local_current].baseline = calFac.algo_baseline;
      d2cParams_[idx_local_current].focus = calFac.focus;
      d2cParams_[idx_local_current].algo_cx = calFac.focus;
      d2cParams_[idx_local_current].algo_cy = calFac.algo_cx;
      d2cParams_[idx_local_current].baseline = calFac.algo_baseline;
      std::cout << idx_local_current << " readCalibartion success ! " << std::endl;
    }

    IR_IR_Calib_Para_t ir2irParams;
    if(getIRParam(idx_curent, ir2irParams))
    {
      //std::cout << "sizeof left camera matrix : " << sizeof(ir2irParams.left_camera_matrix)/sizeof(ir2irParams.left_camera_matrix[0]) << std::endl;
      //std::cout << "sizeof left dist coeffs   : " << sizeof(ir2irParams.left_dist_coeffs)/sizeof(ir2irParams.left_dist_coeffs[0]) << std::endl;
      gvCameraParams_[idx_local_current].leftIRCameraMatrix.clear();
      gvCameraParams_[idx_local_current].rightIRCameraMatrix.clear();
      gvCameraParams_[idx_local_current].rightHdRGBCameraMatrix.clear();
      for(int i = 0; i < sizeof(ir2irParams.left_camera_matrix)/sizeof(ir2irParams.left_camera_matrix[0]); i++)
      {
        gvCameraParams_[idx_local_current].leftIRCameraMatrix.push_back(ir2irParams.left_camera_matrix[i]);
        gvCameraParams_[idx_local_current].rightIRCameraMatrix.push_back(ir2irParams.right_camera_matrix[i]);
        gvCameraParams_[idx_local_current].rightHdRGBCameraMatrix.push_back(ir2irParams.left_camera_matrix_unRectify[i]);
      }

      gvCameraParams_[idx_local_current].leftIRDistCoeffs.clear();
      gvCameraParams_[idx_local_current].rightIRDistCoeffs.clear();
      gvCameraParams_[idx_local_current].rightHdRGBDistCoeffs.clear();
      for(int i = 0; i < sizeof(ir2irParams.left_dist_coeffs)/sizeof(ir2irParams.left_dist_coeffs[0]); i++)
      {
        gvCameraParams_[idx_local_current].leftIRDistCoeffs.push_back(ir2irParams.left_dist_coeffs[i]);
        gvCameraParams_[idx_local_current].rightIRDistCoeffs.push_back(ir2irParams.right_dist_coeffs[i]);
        gvCameraParams_[idx_local_current].rightHdRGBDistCoeffs.push_back(ir2irParams.left_dist_coeffs_unRectify[i]);
      }

      d2cParams_[idx_local_current].hdRight_fc[0] = ir2irParams.left_camera_matrix_unRectify[0];
      d2cParams_[idx_local_current].hdRight_fc[1] = ir2irParams.left_camera_matrix_unRectify[4];
      d2cParams_[idx_local_current].hdRight_cc[0] = ir2irParams.left_camera_matrix_unRectify[2];
      d2cParams_[idx_local_current].hdRight_cc[1] = ir2irParams.left_camera_matrix_unRectify[5];
      d2cParams_[idx_local_current].hdRight_kc[0] = ir2irParams.left_dist_coeffs_unRectify[0];
      d2cParams_[idx_local_current].hdRight_kc[1] = ir2irParams.left_dist_coeffs_unRectify[1];
      d2cParams_[idx_local_current].hdRight_kc[2] = ir2irParams.left_dist_coeffs_unRectify[2];
      d2cParams_[idx_local_current].hdRight_kc[3] = ir2irParams.left_dist_coeffs_unRectify[3];
      d2cParams_[idx_local_current].hdRight_kc[4] = 0.;
      for(int i = 0; i < 9; i++)
      {
        d2cParams_[idx_local_current].hdR[i] = ir2irParams.right_camera_matrix_unRectify[i];
        if(i < 3)
          d2cParams_[idx_local_current].hdT[i] = ir2irParams.left_dist_coeffs_unRectify[i]/1000.;

      }
      //memcpy(gvCameraParams_[idx_local_current].leftIRDistCoeffs, ir2irParams.left_dist_coeffs, sizeof(ir2irParams.left_dist_coeffs));
      //memcpy(gvCameraParams_[idx_local_current].rightIRCameraMatrix, ir2irParams.right_camera_matrix, sizeof(ir2irParams.right_camera_matrix));
      //memcpy(gvCameraParams_[idx_local_current].rightIRDistCoeffs, ir2irParams.right_dist_coeffs, sizeof(ir2irParams.right_dist_coeffs));
      //memcpy(gvCameraParams_[idx_local_current].IR2IRRotationMatrix, ir2irParams.R, sizeof(ir2irParams.R));
      //memcpy(gvCameraParams_[idx_local_current].IR2IRTranslationMatrix, ir2irParams.T, sizeof(ir2irParams.T));
      gvCameraParams_[idx_local_current].IR2IRRotationMatrix.clear();
      gvCameraParams_[idx_local_current].IR2RGBHdRotation.clear();
      for(int i = 0; i < sizeof(ir2irParams.R)/sizeof(ir2irParams.R[0]); i++)
      {
        gvCameraParams_[idx_local_current].IR2IRRotationMatrix.push_back(ir2irParams.R[i]);
        gvCameraParams_[idx_local_current].IR2RGBHdRotation.push_back(ir2irParams.right_camera_matrix_unRectify[i]);
      }

      gvCameraParams_[idx_local_current].IR2IRTranslationMatrix.clear();
      gvCameraParams_[idx_local_current].IR2RGBHdTranslation.clear();
      for(int i = 0; i < sizeof(ir2irParams.T)/sizeof(ir2irParams.T[0]); i++)
      {
        gvCameraParams_[idx_local_current].IR2IRTranslationMatrix.push_back(ir2irParams.T[i]/1000.);
        gvCameraParams_[idx_local_current].IR2RGBHdTranslation.push_back(ir2irParams.right_dist_coeffs_unRectify[i]/1000.);
      }
      //std::cout << idx_local_current << " getIR2IRParam success !" << std::endl;
    }

    IR_RGB_Calib_Para_t ir2rgbParams;
    if(getRGBParam(idx_curent, ir2rgbParams))
    {
      //memset( gvCameraParams_[idx_local_current].leftRGBCameraMatrix, 0., sizeof( gvCameraParams_[idx_local_current].leftRGBCameraMatrix));
      gvCameraParams_[idx_local_current].leftRGBCameraMatrix.clear();
      gvCameraParams_[idx_local_current].leftRGBCameraMatrix.resize(9, 0);
      gvCameraParams_[idx_local_current].leftRGBCameraMatrix[0] = ir2rgbParams.left_fc[0];
      gvCameraParams_[idx_local_current].leftRGBCameraMatrix[4] = ir2rgbParams.left_fc[1];
      gvCameraParams_[idx_local_current].leftRGBCameraMatrix[2] = ir2rgbParams.left_cc[0];
      gvCameraParams_[idx_local_current].leftRGBCameraMatrix[5] = ir2rgbParams.left_cc[1];

      d2cParams_[idx_local_current].left_fc[0] = ir2rgbParams.left_fc[0];
      d2cParams_[idx_local_current].left_fc[1] = ir2rgbParams.left_fc[1];
      d2cParams_[idx_local_current].left_cc[0] = ir2rgbParams.left_cc[0];
      d2cParams_[idx_local_current].left_cc[1] = ir2rgbParams.left_cc[1];
      //memcpy(gvCameraParams_[idx_local_current].leftRGBDistCoeffs, ir2rgbParams.right_kc, sizeof(ir2rgbParams.right_kc));
     
      //memset( gvCameraParams_[idx_local_current].rightRGBCameraMatrix, 0., sizeof(gvCameraParams_[idx_local_current].rightRGBCameraMatrix));
      gvCameraParams_[idx_local_current].rightRGBCameraMatrix.clear();
      gvCameraParams_[idx_local_current].rightRGBCameraMatrix.resize(9, 0);
      gvCameraParams_[idx_local_current].rightRGBCameraMatrix[0] = ir2rgbParams.right_fc[0];
      gvCameraParams_[idx_local_current].rightRGBCameraMatrix[4] = ir2rgbParams.right_fc[1];
      gvCameraParams_[idx_local_current].rightRGBCameraMatrix[2] = ir2rgbParams.right_cc[0];
      gvCameraParams_[idx_local_current].rightRGBCameraMatrix[5] = ir2rgbParams.right_cc[1];

      d2cParams_[idx_local_current].right_fc[0] = ir2rgbParams.right_fc[0];
      d2cParams_[idx_local_current].right_fc[1] = ir2rgbParams.right_fc[1];
      d2cParams_[idx_local_current].right_cc[0] = ir2rgbParams.right_cc[0];
      d2cParams_[idx_local_current].right_cc[1] = ir2rgbParams.right_cc[1];

      gvCameraParams_[idx_local_current].leftRGBDistCoeffs.clear();
      gvCameraParams_[idx_local_current].rightRGBDistCoeffs.clear();
      for(int i = 0; i < sizeof(ir2rgbParams.right_kc)/sizeof(ir2rgbParams.right_kc[0]); i++)
      {
        gvCameraParams_[idx_local_current].leftRGBDistCoeffs.push_back(ir2rgbParams.left_kc[i]);
        d2cParams_[idx_local_current].left_kc[i] = ir2rgbParams.left_kc[i];
        gvCameraParams_[idx_local_current].rightRGBDistCoeffs.push_back(ir2rgbParams.right_kc[i]);
        d2cParams_[idx_local_current].right_kc[i] = ir2rgbParams.right_kc[i];
      }

      gvCameraParams_[idx_local_current].leftRGBDistCoeffs[4] = 0.;
      d2cParams_[idx_local_current].left_kc[4] = 0.;
      gvCameraParams_[idx_local_current].rightRGBDistCoeffs[4] = 0.;
      d2cParams_[idx_local_current].right_kc[4] = 0.;

      //memcpy(gvCameraParams_[idx_local_current].rightRGBDistCoeffs, ir2rgbParams.right_kc, sizeof(ir2rgbParams.right_kc));
      //memcpy(gvCameraParams_[idx_local_current].IR2RGBRotation, ir2rgbParams.R, sizeof(ir2rgbParams.R));
      //memcpy(gvCameraParams_[idx_local_current].IR2RGBTranslation, ir2rgbParams.T, sizeof(ir2rgbParams.T));
      gvCameraParams_[idx_local_current].IR2RGBRotation.clear();
      for(int i = 0; i < sizeof(ir2rgbParams.R)/sizeof(ir2rgbParams.R[0]); i++)
      {
        gvCameraParams_[idx_local_current].IR2RGBRotation.push_back(ir2rgbParams.R[i]);
        d2cParams_[idx_local_current].R[i] = ir2rgbParams.R[i];
      }

      gvCameraParams_[idx_local_current].IR2RGBTranslation.clear();
      for(int i = 0; i < sizeof(ir2rgbParams.T)/sizeof(ir2rgbParams.T[0]); i++)
      {
        gvCameraParams_[idx_local_current].IR2RGBTranslation.push_back(ir2rgbParams.T[i]/1000.);
        d2cParams_[idx_local_current].T[i] = ir2rgbParams.T[i]/1000.;
      }
      std::cout << idx_local_current << " getIR2RGBParam success !" << std::endl;
    }

    CK_Version_t version; 
    if(getFirmwareVersion(idx_curent, version))
    {
      std::string ck_804 = (char *)version.ck_804;
      std::string ck_805 = (char *)version.ck_805;
      gvCameraParams_[idx_local_current].firmwareVersion804 = (char *)version.ck_804;
      gvCameraParams_[idx_local_current].firmwareVersion805 = (char *)version.ck_805;
      firmwareVersion804_.push_back(ck_804);
      firmwareVersion805_.push_back(ck_805);
      std::cout << idx_local_current << " getFirmwareVersion success !" << std::endl;
    }
    
    std::string sn;
    if(readSN(idx_curent, sn))
    {
      gvCameraParams_[idx_local_current].cameraSn = sn;
      cameraSn_.push_back(sn);
      std::cout << idx_local_current << " readSN success !" << std::endl;
    }
    //gv_dev += sn.c_str();
    memset(gvDevs_[idx_local_current].sn, '\0', 64);
    strcpy(gvDevs_[idx_local_current].sn, sn.c_str());
    //std::string local_idx_tmp = "device";
    //local_idx_tmp += std::to_string(idx_local_current);
    //std::cout << "local idx : " << idx_local_current << std::endl;
  }
  return 1;
}
  
void GvStereoDepth::startGvCamera()
{
  //std::cout << "function : startGvCamera " << std::endl;
  //std::cout << "gvDevs_ size :  " << gvDevs_.size() << std::endl;
  for (size_t idx_local_current = 0; idx_local_current < gvDevs_.size(); idx_local_current ++) 
  {
    //std::cout << "id_loca_current : " << idx_local_current << std::endl;
    //std::string gv_dev = "GV";
    const size_t idx_curent = gvDevs_[idx_local_current].address;
    //std::cout << "id_loca_current : " << idx_local_current << " " << idx_curent << std::endl;
    if(!setUSBPacketLen(idx_curent, gvDevs_.size()))
    {
      std::cout << idx_local_current << " setUSBPacketLen error !" << std::endl;
    }
    
    if(!setIRPower(idx_curent,500))
    {
      std::cout << idx_local_current << " idx_local_current error !" << std::endl;
    }

    if(!setFPS(idx_curent,30))
    {
      std::cout << idx_local_current << " setFPS error!" << std::endl;
    }

    if(!setImageChannel(idx_curent,false, false, false, false))
    {
      std::cout << idx_local_current << " setImageChannel error !" << std::endl;
    }
  
    if(!setIRSwitch(idx_curent, true))
    {
      std::cout << idx_local_current << " setIRSwitch error !" << std::endl;
    }

    if(!setAESwitch(idx_curent, true))
    {
      std::cout << idx_local_current << " setAESwitch error !" << std::endl;
    }
  }
}

bool GvStereoDepth::enableCamera(int idx_local_current, bool enable, int bandwidth, int format, int type)
{
    //std::cout << std::endl << "device " << idx_local_current << " -- bandwidth set to " << bandwidth << "effect when next time startstreaming." << std::endl;
    if(enable)
    {
      //std::cout << "setDevice enable " << std::endl;
      //setBandwidth(idx_local_current, 512);
      bool res =  setGvStreamBandwidthAndRgbFormat(idx_local_current, bandwidth, format, type);
      //bool res = uvc_start_streaming(gvDevs_[idx_local_current].devh, &gvDevs_[idx_local_current].ctrl, uvcCallback, (void *)(devPtr_ + idx_local_current), 0);
      //std::cout << "************* start steaming res : " << res << std::endl;
      if (res) 
      { 
        const size_t idx_curent = gvDevs_[idx_local_current].address;
        bool flag = false;  
        //std::cout << " uvc start streaming success " << std::endl; 
        if(!setIRPower(idx_curent, 800))
        {
          std::cout << idx_local_current << " setIRPower again error !" << std::endl;
          flag = true;
        }

        if(!setFPS(idx_curent,30))
        {
          std::cout << idx_local_current << " setFPS again error !" << std::endl;
          flag = true;
        }

        if(!setImageChannel(idx_curent, false, false, false, false))
        {
          std::cout << idx_local_current << " setImageChannel again error !" << std::endl;
          flag = true;
        }

        if(!setIRSwitch(idx_curent, true))
        {
          std::cout << idx_local_current << " setIRSwitch error !" << std::endl;
          flag = true;
        }

        //std::cout << "uvc flag : " << flag << std::endl;
        if(flag)
        {
          std::cout << " uvc config streaming fail !" << std::endl;
          flag = false;
        }
        else
        {
          //std::cout << " uvc config streaming success !" << std::endl;
          flag = true;
        }
        return flag;
        
      }
      else 
      {
          std::cout << "can't open device with error code:" << res <<  std::endl;
          // assert(0);
          return false;
      }
    } 
    // else 
    // {
    //     uvc_stop_streaming(gvDevs_[idx_local_current].devh);
    //     return true;
    // }
  //}
}

void GvStereoDepth::quit() {
  for (auto &it : gvDevs_) {
    uvc_stop_streaming(it.devh);
    uvc_close(it.devh);
    uvc_unref_device(it.dev);
  }
  resetDeviceConnection(0x04cc,0x0017);
  std::cout << "Exit." << std::endl;
}

bool GvStereoDepth::setGvDeviceStreamStatus(int idx, int status)
{
    // std::cout << "**************************" << std::endl;
    // std::cout << "set stereo device  stream status : " << status  << " " << gvDevs_.size() << std::endl;
    // std::cout << "status: 0 (close), 1(ir), 2(depth), 3(color), 4(ir+depth), 5(color+depth)" << std::endl;
    // std::cout << "**************************" << std::endl;
    //status: 0 (close), 1(ir), 2(depth), 3(color),  4(color+depth), 5(irSwitch off), 6(irSwitch on)
    bool flag = false;
    //for(size_t idx_local = 0; idx_local < gvDevs_.size(); idx_local++)
    //{
    const size_t idx_curent = gvDevs_[idx].address;
    //const size_t idx_curent = 34;
    if(status == 0)
    {
      setImageChannel(idx_curent,false, false, false, false);
      flag = true;
    }
    else if (status == 1)
    {
      setImageChannel(idx_curent, true, true, false, false);
      flag = true;

    }
    else if (status == 2)
    {
      setImageChannel(idx_curent, false, false, true, false);
      //setImageChannel(idx_curent, false, false, true, true);
      flag = true;
    }
    else if (status == 3)
    {
      setImageChannel(idx_curent, false, false, false, true);
      flag = true;

    }
    else if (status == 4)
    {
      setImageChannel(idx_curent, true, true, true, false);
      flag = true;

    }
    else if (status == 5)
    {
      setImageChannel(idx_curent, false, false, true, true);
      flag = true;

    }
    else
    {
      std::cerr << "stream status is not correct !" << std::endl;
    }
    return flag;
}


bool GvStereoDepth::setGvDeviceIrStatus(int idx, int status)
{
    std::cout << "**************************" << std::endl;
    std::cout << "set stereo device  ir light status : " << status  << " " << gvDevs_.size() << std::endl;
    std::cout << "status: 0 (close), 1(open)" << std::endl;
    std::cout << "**************************" << std::endl;
    bool flag = false;
    const size_t idx_curent = gvDevs_[idx].address;
    if(status == 0)
    {
      setIRSwitch(idx_curent, false);
      flag = true;

    }
    else if(status == 1)
    {
      setIRSwitch(idx_curent, true);
      flag = true;
    }
    else
    {
      std::cerr << "ir status is not correct !" << std::endl;
    }
    return flag;
}

bool GvStereoDepth::setGvDeviceAeStatus(int idx, int status)
{
    std::cout << "**************************" << std::endl;
    std::cout << "set stereo device  ae status : " << status  << " " << gvDevs_.size() << std::endl;
    std::cout << "status: 0 (close), 1(open)" << std::endl;
    std::cout << "**************************" << std::endl;
    bool flag = false;
    const size_t idx_curent = gvDevs_[idx].address;
    if(status == 0)
    {
      setAESwitch(idx_curent, false);
      flag = true;

    }
    else if(status == 1)
    {
      setAESwitch(idx_curent, true);
      flag = true;
    }
    else
    {
      std::cerr << "ae status is not correct !" << std::endl;
    }
    return flag;
}



bool GvStereoDepth::setGvStreamBandwidthAndRgbFormat(int idx , int bandwidth, int format, int type)
{
  //std::cout << "set Gv Stream Bandwidth and rgb format : " << idx << " " << bandwidth << " " << format << " " << type << std::endl;
  if(idx < 0)
  {
    std::cout << "idx >= 0" << std::endl;
    return false;
  }
  if((bandwidth < 0) || (bandwidth > 3060))
  {
    std::cout << "bandwidth value : 0, 512,	1020,	1536,	2040,	2556,	3060 !" << std::endl;
    return false;
  }
  // if((format != 3) && (format != 7))
  // {
  //   std::cout << "frame format value : 3(yuv), 7(mjpeg) !" << std::endl;
  //   return false;
  // }
  // if((type != 0) && (type != 1))
  // {
  //   std::cout << "rgb resolution only support vga(0) or hd(1), ir resolution only support vga(0) ! " << std::endl;
  // }

  if(setBandwidthAndRgbFormat(idx, bandwidth, format, type))
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool GvStereoDepth::setGvStreamFps(int idx, int fps)
{
  if(idx < 0)
  {
    std::cout << "idx >= 0" << std::endl;
    return false;
  }
  if((fps < 0) || (fps > 30))
  {
    std::cout << "fps value : 5, 10, 15, 20, 25, 30" << std::endl;
    return false;
  }

  if(setFPS(idx, fps))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool GvStereoDepth::setGvIRLightPower(int idx, int irPower)
{
    if(idx < 0)
    {
      std::cout << "idx >= 0" << std::endl;
      return false;
    }

    if((irPower < 0) || (irPower > 1300))
    {
      std::cout << "irPower ranger is [0, 1300]" << std::endl;
      return false;
    }
   
    if(setIRPower(idx, irPower))
    {
      return true;
    }
    else
    {
      return false;
    }
}


bool GvStereoDepth::setGvExposureGain(const size_t &addressIdx, const size_t &cameraIndex, 
            int autoManualExposure, float manualExposureValue, float manualGainValue)
{
    if(addressIdx < 0)
    {
      std::cout << "addressIdx >= 0" << std::endl;
      return false;
    }
    if(setISPExposure(addressIdx, cameraIndex,  autoManualExposure, manualExposureValue, manualGainValue))
    {
      return true;
    }
    else
    {
      return false;
    }
}

bool GvStereoDepth::setGvSPWB(int addressIdx, int cameraIdx, int autoManualWb, float wbR, float wbGr, float wbGb, float wbB)
{
    if(addressIdx < 0)
    {
      std::cout << "addressIdx >= 0" << std::endl;
      return false;
    } 

    if(setISPWB(addressIdx, cameraIdx, autoManualWb, wbB, wbGr, wbGb, wbB))
    {
      return true;
    }
    else
    {
      return false;
    }
}

bool GvStereoDepth::setGvGamma(int addressIdx, int cameraIdx, int onOff)
{
    if(addressIdx < 0)
    {
      std::cout << "addressIdx >= 0" << std::endl;
      return false;
    } 

    if(setISPGamma(addressIdx, cameraIdx, onOff))
    {
      return true;
    }
    else
    {
      return false;
    }
}

bool GvStereoDepth::setGvSharpness(int addressIdx, int cameraIdx, int onOff, int denoiseLevel, int sharpnessLevel)
{
  if(addressIdx < 0)
  {
    std::cout << "idx >= 0" << std::endl;
    return false;
  }

  if(setISPSharpness(addressIdx, cameraIdx, onOff, denoiseLevel, sharpnessLevel))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool GvStereoDepth::setGvCrop(int addressIdx, int cameraIdx, int cprocId, int onOff, float value)
{
  if(addressIdx < 0)
  {
    std::cout << "idx >= 0" << std::endl;
    return false;
  }

  ISP_Cproc_Control_t data;
  data.cproc_id = cprocId;
  data.camera_index =cameraIdx;
  data.on_off = onOff;
  data.value = value;
  if(setISPCrop(addressIdx, data))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool GvStereoDepth::getGvExposure(int addressIdx, int cameraIdx)
{
    if(addressIdx < 0)
    {
      std::cout << "addressIdx must be >= 0" << std::endl;
      return false;
    } 
    ISP_Exposure_Control_t res;
    res.camera_index = cameraIdx;
    if(getISPExposure(addressIdx, res))
    {
      std::cout << "Get Exposure and Gain result : " 
                << res.auto_exposure_range 
                << " " 
                << res.auto_exposure_target 
                << " " 
                << res.auto_manual_exposure 
                << " "
                << res.manual_exposure_value
                << " "
                << res.manual_gain_value
                << " "
                << res.result_exposure
                << " "
                << res.result_gain
                << std::endl;
      return true;
    }
    else
    {
      return false;
    }
}

bool GvStereoDepth::getGvSPWB(int addressIdx, int cameraIdx)
{
    if(addressIdx < 0)
    {
      std::cout << "addressIdx >= 0" << std::endl;
      return false;
    } 
    ISP_WB_Control_t res;
    res.camera_index = cameraIdx;
    if(getISPWB(addressIdx, res))
    {
      std::cout << "get WB result :  "  
                << res.camera_index  <<  " "
                << res.wb_b << " "
                << res.wb_r << " " 
                << res.wb_gr << " "
                << res.wb_gb << " "
                << res.auto_manual_wb << std::endl; 
      return true;
    }
    else
    {
      return false;
    }
}

bool GvStereoDepth::getGvSharpness(int addressIdx, int cameraIdx)
{
    if(addressIdx < 0)
    {
      std::cout << "addressIdx >= 0" << std::endl;
      return false;
    } 
    ISP_Sharpness_Control_t  res;
    res.camera_index = cameraIdx;
    if(getISPSharpness(addressIdx, res))
    {
      std::cout << "get Sharpness resutl : " 
                << res.camera_index << " "
                << res.on_off << " "
                << res.denoise_level << " "
                << res.sharpness_level << std::endl;
      return true;
    }
    else
    {
      return false;
    }
}

bool GvStereoDepth::getGvGamma(int addressIdx, int cameraIdx)
{
    if(addressIdx < 0)
    {
      std::cout << "addressIdx >= 0" << std::endl;
      return false;
    } 
    ISP_Gamma_Control_t  res;
    res.camera_index = cameraIdx;
    if(getISPGamma(addressIdx, res))
    {
      std::cout << "get Gamma resutl : " 
                << res.camera_index << " "
                << res.on_off << std::endl;
      return true;
    }
    else
    {
      return false;
    }
}

bool GvStereoDepth::getGvCrop(int addressIdx, int cameraIdx)
{
    if(addressIdx < 0)
    {
      std::cout << "addressIdx >= 0" << std::endl;
      return false;
    } 
    ISP_Cproc_Control_t  res;
    res.camera_index = cameraIdx;
    if(getISPCrop(addressIdx, res))
    {
      return true;
    }
    else
    {
      return false;
    }
}


std::vector<unsigned char> GvStereoDepth::getIrStreamData(int idx)
{
   std::vector<unsigned char> data;
   if(!(gvDevs_[idx].irDataPtr == nullptr))
   {
        //std::cout << "ir data size : "  << gvDevs_[idx].irDataPtr->size() << " " << gvDevs_[idx].irDataPtr.use_count() << std::endl;
        data = *gvDevs_[idx].irDataPtr.get();
        //std::cout << "Data size  : " << data.size() << std::endl;
   }
   return data;
}


std::vector<unsigned char> GvStereoDepth::getDepthStreamData(int idx)
{
  std::vector<unsigned char> data;
  if(!(gvDevs_[idx].depthDataPtr == nullptr))
  {
    //std::cout << "depth data size 1 : "  << gvDevs_[idx].depthDataPtr->size() << " " << gvDevs_[idx].depthDataPtr.use_count() << std::endl;
    data = *gvDevs_[idx].depthDataPtr.get();
    //std::cout << "Data size  : " << data.size() << std::endl;
  }
  return data;
}

std::vector<unsigned char> GvStereoDepth::getColorStreamData(int idx)
{
   std::vector<unsigned char> data;
   if(!(gvDevs_[idx].colorDataPtr == nullptr))
   {
        //std::cout << "color data size 1 : "  << gvDevs_[idx].colorDataPtr->size() << " " << gvDevs_[idx].colorDataPtr.use_count() << std::endl;
        data = *gvDevs_[idx].colorDataPtr.get();
        //std::cout << "Data size  : " << data.size() << std::endl;
   }
   return data;
}

void GvStereoDepth::getPointCloud(int idx, std::vector<double> &pointcloud)
{
  if(!(gvDevs_[idx].depthDataPtr == nullptr))
  {
      GV_CAMERA_PARAMS gvParams = gvCameraParams_[idx];
      //std::cout << "depth data size : "  << gvDevs_[0].depthDataPtr->size() << " " << gvDevs_[0].depthDataPtr.use_count() << std::endl;
      std::vector<unsigned char> depthData = *gvDevs_[idx].depthDataPtr.get();
      int height = 400;
      int width = 640;
      const int img_size = width * height;
      pointcloud.resize(img_size*3, 0.);	
		
      double fx = gvParams.fx;
      double fy = gvParams.fy;
      double cx = gvParams.cx;
      double cy = gvParams.cy;
      std::cout << "camera params : " << fx << " " << fy << " " << cx << " " << cy << std::endl;
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
          ushort value = static_cast<ushort>(depthData[i * width * 2 + j * 2 + 1 ] << 8 | depthData[i * width * 2 + j * 2]);
          double depthXy = (double)(value/1000.);
          //double Depth_xy = Depth[i * width + j];
          //double Depth_xy = alignDepth[i * width + j];
          if(depthXy > 0.)
          {
            //alignDepthFilter[i * width + j] = Depth_xy;
            double X = (j - cx) / fx  * depthXy;
            double Y = (i - cy) / fy  * depthXy;
            double Z = depthXy;
      
            pointcloud[i * width * 3 + j * 3] = X;
            pointcloud[i * width * 3 + j * 3 + 1] = Y;
            pointcloud[i * width * 3 + j * 3 + 2] = Z;
          }
        }
      }
  }
  //return data;
}


void GvStereoDepth::getPointCloud(int idx, int flag, const std::vector<unsigned char> &depthData, std::vector<double> &pointcloud)
{
  if(!(depthData.empty()))
  {
      GV_CAMERA_PARAMS gvParams = gvCameraParams_[idx];
      //std::cout << "depth data size : "  << gvDevs_[0].depthDataPtr->size() << " " << gvDevs_[0].depthDataPtr.use_count() << std::endl;
      int height = 400;
      int width = 640;
      const int img_size = width * height;
      pointcloud.resize(img_size*3, 0.);	
		
      double fx = gvParams.fx;
      double fy = gvParams.fy;
      double cx = gvParams.cx;
      double cy = gvParams.cy;
      if(flag)
      {
        std::cout <<  " calculate aligin depth pointcloud " << std::endl;
        fx = gvParams.rightRGBCameraMatrix[0];
        fy = gvParams.rightRGBCameraMatrix[4];
        cx = gvParams.rightRGBCameraMatrix[2];
        cy = gvParams.rightRGBCameraMatrix[5];
      }
      
      std::cout << "camera params : " << fx << " " << fy << " " << cx << " " << cy << std::endl;
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
          ushort value = static_cast<ushort>(depthData[i * width * 2 + j * 2 + 1 ] << 8 | depthData[i * width * 2 + j * 2]);
          double depthXy = (double)(value/1000.);
          //double Depth_xy = Depth[i * width + j];
          //double Depth_xy = alignDepth[i * width + j];
          if(depthXy > 0.)
          {
            //alignDepthFilter[i * width + j] = Depth_xy;
            double X = (j - cx) / fx  * depthXy;
            double Y = (i - cy) / fy  * depthXy;
            double Z = depthXy;
      
            pointcloud[i * width * 3 + j * 3] = X;
            pointcloud[i * width * 3 + j * 3 + 1] = Y;
            pointcloud[i * width * 3 + j * 3 + 2] = Z;
          }
        }
      }
  }
  //return data;
}


void GvStereoDepth::getAlignedDepthData(int idx, std::vector<double> &alignDepthData, std::vector<unsigned char> &alignColorData)
{
  std::vector<unsigned char> data;
  if((!(gvDevs_[idx].depthDataPtr == nullptr)) && (!(gvDevs_[idx].colorDataPtr == nullptr)))
  {
      gv::DepthAlignToColor::Parameters d2cParams = d2cParams_[idx];
      //std::cout << "depth data size : "  << gvDevs_[0].depthDataPtr->size() << " " << gvDevs_[0].depthDataPtr.use_count() << std::endl;
      std::vector<unsigned char> depthData = *gvDevs_[idx].depthDataPtr.get();
      std::vector<unsigned char> colorData = *gvDevs_[idx].colorDataPtr.get();
      int height = d2cParams.height;
      int width = d2cParams.width;
      const int img_size = width * height;
      double* Depth = new double[img_size];
      double* alignDepth= new double[img_size];
      double* alignPointcloud= new double[img_size*3];
      alignDepthData.resize(img_size);
      for (int i = 0; i < height; i++) {
          for (int j = 0; j < width; j++) {
              ushort value = static_cast<ushort>(depthData[i * width * 2 + j * 2 + 1 ] << 8 | depthData[i * width * 2 + j * 2]);
              Depth[i * width + j] = (double)(value/1000.);
              alignDepth[i * width + j] = 0.;
              alignPointcloud[i * width * 3 + j * 3]  = 0.;
              alignPointcloud[i * width * 3 + j * 3 + 1] = 0.;
              alignPointcloud[i * width * 3 + j * 3 + 2] = 0.;
          }
      } 
      depth2Color_.depthAlignToColor(Depth, d2cParams, alignDepth, alignPointcloud);
      for (int i = 0; i < height; i++) 
      {
        for (int j = 0; j < width; j++) 
        {
          if(alignDepth[i * width + j] > 0.)
            alignDepthData[i * width + j] = alignDepth[i * width + j];
        }
       
      }
      alignColorData = colorData;
      delete(Depth);
      delete(alignDepth);
      delete(alignPointcloud);
  }
  //return data;
}


void GvStereoDepth::getAlignedDepthData(int idx, std::vector<double> &alignDepthData, std::vector<unsigned char> &alignColorData, std::vector<double> &alignPointcloud)
{
  std::vector<unsigned char> data;
  if((!(gvDevs_[idx].depthDataPtr == nullptr)) && (!(gvDevs_[idx].colorDataPtr == nullptr)))
  {
      gv::DepthAlignToColor::Parameters d2cParams = d2cParams_[idx];
      //std::cout << "depth data size : "  << gvDevs_[0].depthDataPtr->size() << " " << gvDevs_[0].depthDataPtr.use_count() << std::endl;
      std::vector<unsigned char> depthData = *gvDevs_[idx].depthDataPtr.get();
      std::vector<unsigned char> colorData = *gvDevs_[idx].colorDataPtr.get();
      int height = d2cParams.height;
      int width = d2cParams.width;
      const int img_size = width * height;
      double* Depth = new double[img_size];
      double* alignDepth= new double[img_size];
      double* pointcloud= new double[img_size*3];
      alignPointcloud.resize(img_size * 3, 0.);
      alignDepthData.resize(img_size);
      for (int i = 0; i < height; i++) {
          for (int j = 0; j < width; j++) {
              ushort value = static_cast<ushort>(depthData[i * width * 2 + j * 2 + 1 ] << 8 | depthData[i * width * 2 + j * 2]);
              Depth[i * width + j] = (double)(value/1000.);
              alignDepth[i * width + j] = 0.;
              pointcloud[i * width * 3 + j * 3]  = 0.;
              pointcloud[i * width * 3 + j * 3 + 1] = 0.;
              pointcloud[i * width * 3 + j * 3 + 2] = 0.;
          }
      } 
      depth2Color_.depthAlignToColor(Depth, d2cParams, alignDepth, pointcloud);
      for (int i = 0; i < height; i++) 
      {
        for (int j = 0; j < width; j++) 
        {
          if(alignDepth[i * width + j] > 0.)
            alignDepthData[i * width + j] = alignDepth[i * width + j];
            alignPointcloud[i * width * 3 + j * 3] = pointcloud[i * width * 3 + j * 3];
            alignPointcloud[i * width * 3 + j * 3 + 1] = pointcloud[i * width * 3 + j * 3 + 1];
            alignPointcloud[i * width * 3 + j * 3 + 2] = pointcloud[i * width * 3 + j * 3 + 2];
        }
       
      }
      alignColorData = colorData;
      delete(Depth);
      delete(alignDepth);
      delete(pointcloud);
  }
  //return data;
}


void GvStereoDepth::getAlignedHDDepthData(int idx, std::vector<double> &alignDepthData, std::vector<unsigned char> &alignColorData)
{
  std::vector<unsigned char> data;
  if((!(gvDevs_[idx].depthDataPtr == nullptr)) && (!(gvDevs_[idx].colorDataPtr == nullptr)))
  {
      gv::DepthAlignToColor::Parameters d2cParams = d2cParams_[idx];
      //std::cout << "depth data size : "  << gvDevs_[0].depthDataPtr->size() << " " << gvDevs_[0].depthDataPtr.use_count() << std::endl;
      std::vector<unsigned char> depthData = *gvDevs_[idx].depthDataPtr.get();
      std::vector<unsigned char> colorData = *gvDevs_[idx].colorDataPtr.get();
      int height = 640;
      int width = 400;
      const int img_size = width * height;
      double* Depth = new double[img_size];
      for (int i = 0; i < height; i++) {
          for (int j = 0; j < width; j++) {
              ushort value = static_cast<ushort>(depthData[i * width * 2 + j * 2 + 1 ] << 8 | depthData[i * width * 2 + j * 2]);
              Depth[i * width + j] = (double)(value/1000.);
          }
      } 

      int hdHeight = 960;
      int hdWidth = 1280;
      double* alignDepth= new double[hdWidth * hdHeight];
      double* alignPointcloud= new double[hdWidth * hdHeight *3];
     
      alignDepthData.resize(hdWidth * hdHeight);
      for (int i = 0; i < hdHeight; i++) {
        for (int j = 0; j < hdWidth; j++) {
              alignDepth[i * hdWidth + j] = 0.;
              alignPointcloud[i * hdWidth * 3 + j * 3]  = 0.;
              alignPointcloud[i * hdWidth * 3 + j * 3 + 1] = 0.;
              alignPointcloud[i * hdWidth * 3 + j * 3 + 2] = 0.;
          }
      } 
      depth2Color_.depthAlignToHDColor(Depth, d2cParams, alignDepth, alignPointcloud);
      for (int i = 0; i < hdHeight; i++) 
      {
        for (int j = 0; j < hdWidth; j++) 
        {
          if(alignDepth[i * hdWidth + j] > 0.)
            alignDepthData[i * hdWidth + j] = alignDepth[i * hdWidth + j];
        }
       
      }
      alignColorData = colorData;
      delete(Depth);
      delete(alignDepth);
      delete(alignPointcloud);
  }
  //return data;
}


std::vector<unsigned char> GvStereoDepth::getAlignedDepthColorData(int idx)
{
  std::vector<unsigned char> alignData;
  if((!(gvDevs_[idx].depthDataPtr == nullptr)) && (!(gvDevs_[idx].colorDataPtr == nullptr)))
  {
      gv::DepthAlignToColor::Parameters d2cParams = d2cParams_[idx];
      std::vector<unsigned char> depthData = *gvDevs_[idx].depthDataPtr.get();
      std::vector<unsigned char> colorData = *gvDevs_[idx].colorDataPtr.get();
      int height = d2cParams.height;
      int width = d2cParams.width;
      const int img_size = width * height;
      double* Depth = new double[img_size];
      double* alignDepth= new double[img_size];
      double* alignPointcloud= new double[img_size*3];
      //alignDepthData.resize(img_size);
      for (int i = 0; i < height; i++) {
          for (int j = 0; j < width; j++) {
              //low|high
              ushort value = static_cast<ushort>(depthData[i * width * 2 + j * 2 + 1 ] << 8 | depthData[i * width * 2 + j * 2]);
              Depth[i * width + j] = (double)(value/1000.);
              alignDepth[i * width + j] = 0.;
              alignPointcloud[i * width * 3 + j * 3]  = 0.;
              alignPointcloud[i * width * 3 + j * 3 + 1] = 0.;
              alignPointcloud[i * width * 3 + j * 3 + 2] = 0.;
          }
      } 
      depth2Color_.depthAlignToColor(Depth, d2cParams, alignDepth, alignPointcloud);
      alignData.resize(height * width * 2, 0);
      for (int i = 0; i < height; i++) 
      {
        for (int j = 0; j < width; j++) 
        {
          if(alignDepth[i * width + j] > 0.)
          {
            ushort value = static_cast<ushort>(alignDepth[i * width + j] * 1000);
            unsigned char high_byte = (value >> 8) & 0xFF;
            unsigned char low_byte = value & 0xFF;
            alignData[i * width * 2 + j * 2] = low_byte; 
            alignData[i * width * 2 + j * 2 + 1] = high_byte; 
          }
        }
      }
      alignData.insert(alignData.end(), colorData.begin(), colorData.end());
      delete(Depth);
      delete(alignDepth);
      delete(alignPointcloud);
  }
  return alignData;
}


const bool GvStereoDepth::stopDeivce(void)
{
    quit();
    return true;
}

int GvStereoDepth::getDeviceNumber(void)
{
    int num = getDeviceNum();
    return static_cast<int>(num);
}

int GvStereoDepth::getDeviceInfo(void)
{
  std::vector<GvStereoDepth::GV_CAMERA_PARAMS> params = getGvCameraParams();
  return 1;
}

bool GvStereoDepth::setImageChannel(const size_t &devIndex, bool bLeft, bool bRight, bool bDepth, bool bRGB)
{
  //std::cout << "streame status : " << bLeft << " " << bRight << " " << bDepth << " " << bRGB << std::endl;
  nCyclTime_ = 0;
  while (true) {
    nCyclTime_++;
    if(sendDataTemp_.sendImgChannel(devIndex, bLeft, bRight, bDepth, bRGB, 500)) {
      //std::cout<<"set Image Stream OK"<<std::endl;
      return true;
    }

    if(nCyclTime_>=USB_SEND_CMD_CYCL) {
      std::cout<<"set Image Stream Error"<<std::endl;
      return false;
    }
  }
}

bool GvStereoDepth::setIRPower(const size_t &devIndex, int value)
{
  nCyclTime_ = 0;
  nCyclTime_++;
  auto tmp1 = ((double)value - 3.91) / 7.83;
  // value * 7.83 + 3.91
  if(sendDataTemp_.sendIRCurrentSelect(devIndex, tmp1, 500)){
      // std::cout<<"set TX current: 0x7F(1A)"<<std::endl;
      return true;
  }

  if(nCyclTime_>=USB_SEND_CMD_CYCL){
      std::cout<<"set TX current Error"<<std::endl;
      return false;
  }
  return false;
}

bool GvStereoDepth::setIRSwitch(const size_t &devIndex, bool swi)
{    
  nCyclTime_ = 0;
  nCyclTime_++;
  if(sendDataTemp_.sendCloseIRCmd(devIndex, !swi, 500)) {
    return true;
  }

  if(nCyclTime_ >= USB_SEND_CMD_CYCL) {
    std::cout<<"set setIRSwitch Error"<<std::endl;
    return false;
  }
  return false;
}

bool GvStereoDepth::setAESwitch(const size_t &devIndex, bool swi)
{
  nCyclTime_ = 0;
  nCyclTime_++;
  if(sendDataTemp_.sendCloseAECmd(devIndex, !swi, 500)) {
    // std::cout<<"set FPS : "<<value<<std::endl;
    return true;
  }

  if(nCyclTime_ >= USB_SEND_CMD_CYCL) {
    std::cout<<"set setAESwitch Error"<<std::endl;
    return false;
  }
  return false;
}

bool GvStereoDepth::setFPS(const size_t &devIndex, int value)
{
  nCyclTime_ = 0;
  nCyclTime_++;
  if(sendDataTemp_.sendFrameRate(devIndex, value, 500)) {
    // std::cout<<"set FPS : "<<value<<std::endl;
    return true;
  }

  if(nCyclTime_ >= USB_SEND_CMD_CYCL) {
    std::cout<<"set FPS Error"<<std::endl;
    return false;
  }
  return false;
}

bool GvStereoDepth::setISPExposure(const size_t &devIndex, const size_t &cameraIndex, 
    int autoManualExposure, float manualExposureValue, float manualGainValue)
{
  ISP_Exposure_Control_t res_;
  res_.camera_index = static_cast<int>(cameraIndex);
  res_.auto_manual_exposure = static_cast<int>(autoManualExposure);
  res_.manual_exposure_value = static_cast<float>(manualExposureValue);
  res_.manual_gain_value = static_cast<float>(manualGainValue);
  res_.auto_exposure_range = 0.; //static_cast<float>(autoExposureRange);
  res_.auto_exposure_target = 0.; //static_cast<float>(autoExposureTarget);
  res_.result_exposure = 0.; //static_cast<float>(resultExposure);
  res_.result_gain = 0.; //static_cast<float>(resultGain);
  if(sendDataTemp_.Set_ISP_Exposure(gvDevs_[devIndex].address, (char *)&res_, ISP_EXPOSURE_CONTROL_SIZE, 500))
  {
    std::cout << "set setExposureISP success ! " << std::endl;
    return true;
  }
  else
  {
    std::cout << "WriteTWeakParam Param Error" << std::endl;
    return false;
  }
}

bool GvStereoDepth::getISPExposure(const size_t &devIndex, ISP_Exposure_Control_t &res)
{
  nCyclTime_ = 0;
  nCyclTime_++;
  std::cout << "bingo" << std::endl;
  float fExposure;
  float fGain;
  int nState;
  while (true)
  {
    nCyclTime_++;
    if (sendDataTemp_.Get_ISP_Exposure(gvDevs_[devIndex].address, (char *)&res, ISP_EXPOSURE_CONTROL_SIZE, &fExposure, &fGain, &nState, 500))
    {
      std::cout << fGain << "|" << fExposure << "|" <<nState << std::endl ;
      res.camera_index = gvDevs_[devIndex].address;
      res.manual_gain_value = fGain;
      res.manual_exposure_value = fExposure;
      res.auto_manual_exposure = nState;
      std::cout << "res : " << res.camera_index << " " << res.manual_gain_value << " " << res.manual_exposure_value << " " << res.auto_manual_exposure << std::endl;
      return true;
    }
    else
    {
      std::cout << "tes " << std::endl;
      return false;
    }

    if (nCyclTime_ >= USB_SEND_CMD_CYCL)
    {
      std::cout<<"GetExposureISP Error"<<std::endl;
      return false;
    }
  }

}


bool GvStereoDepth::setISPWB(const size_t &devIndex, const size_t &cameraIndex, int autoManualWb, float wbR, float wbGr, float wbGb, float wbB)
{ 
    ISP_WB_Control_t data;
    data.camera_index = static_cast<int>(cameraIndex);
    data.auto_manual_wb = autoManualWb;
    data.wb_r = wbR;
    data.wb_gr = wbGr;
    data.wb_gb = wbGb;
    data.wb_b = wbB;
    nCyclTime_ = 0;
    nCyclTime_++;
    if (sendDataTemp_.Set_ISP_WB(gvDevs_[devIndex].address, (char *)&data, ISP_WB_CONTROL_SIZE, 500))
    {
      std::cout << "set Set_ISP_WB : " << std::endl;
      return true;
    }

    if (nCyclTime_ >= USB_SEND_CMD_CYCL)
    {
      std::cout << "set Set_ISP_Exposure Error" << std::endl;
      return false;
    }
    return false;

}

bool GvStereoDepth::getISPWB(const size_t &devIndex, ISP_WB_Control_t &res)
{
    nCyclTime_ = 0;
    nCyclTime_++;
    int tmpi;
    float tmpf;
    float wb_R_Gain;
    float wb_Gr_Gain;
    float wb_Gb_Gain;
    float wb_B_Gain;
    int nState;
    while (true)
    {
      nCyclTime_++;
      if (sendDataTemp_.Get_ISP_WB(gvDevs_[devIndex].address, (char *)&res, ISP_WB_CONTROL_SIZE, &wb_R_Gain, &wb_Gr_Gain, &wb_Gb_Gain, &wb_B_Gain, &nState, 500))
      {
        res.wb_b = wb_B_Gain;
        res.wb_r = wb_R_Gain;
        res.wb_gr = wb_Gr_Gain;
        res.wb_gb = wb_Gb_Gain;
        res.auto_manual_wb = nState;
        return true;
      }

      if (nCyclTime_ >= USB_SEND_CMD_CYCL)
      {
        std::cout << "GetWBISP Error" << std::endl;
        return false;
      }
    }
}

bool  GvStereoDepth::setISPGamma(const size_t &devIndex, const size_t &cameraIndex,  int onOff)
{
  ISP_Gamma_Control_t data;
  data.camera_index = static_cast<int>(cameraIndex);
  data.on_off = static_cast<int>(onOff);
  nCyclTime_ = 0;
  nCyclTime_++;
  if (sendDataTemp_.Set_ISP_Gamma(gvDevs_[devIndex].address, (char *)&data, ISP_GAMMA_CONTROL_SIZE, 500))
  {
    std::cout << "set isp gamma success: " << std::endl;
    return true;
  }

  if (nCyclTime_ >= USB_SEND_CMD_CYCL)
  {
    std::cout << "set Set_ISP_Gamma Error" << std::endl;
    return false;
  }
  return false;
}

bool GvStereoDepth::getISPGamma(const size_t &devIndex,  ISP_Gamma_Control_t &res)
{
    nCyclTime_ = 0;
    nCyclTime_++;
    int tmpi;
    while (true)
    {
      nCyclTime_++;
      // Get_ISP_Gamma(int nDevID, char* pData, int nLen, int *nState
      if (sendDataTemp_.Get_ISP_Gamma(gvDevs_[devIndex].address, (char *)&res, ISP_GAMMA_CONTROL_SIZE, &tmpi, 500))
      {
        //res.camera_index = cameraIndex,;
        res.on_off = tmpi;
        std::cout << "get ISP Gamma success !" << std::endl;
        return true;
      }

      if (nCyclTime_ >= USB_SEND_CMD_CYCL)
      {
        std::cout << "getGammaISP Error" << std::endl;
        return false;
      }
    }
}


bool GvStereoDepth::setISPSharpness(const size_t &devIndex, const size_t &cameraIndex, int onOff, int denoiseLevel, int sharpnessLevel)
{
    ISP_Sharpness_Control_t data;
    data.camera_index = static_cast<int>(cameraIndex);
    data.on_off = onOff;
    data.denoise_level = denoiseLevel;
    data.sharpness_level = sharpnessLevel;
    nCyclTime_ = 0;
    nCyclTime_++;
    if (sendDataTemp_.Set_ISP_Sharpness(gvDevs_[devIndex].address, (char *)&data, ISP_SHARPNESS_CONTROL_SIZE, 500))
    {
      std::cout << "set isp sharpness success !" << std::endl;
      return true;
    }

    if (nCyclTime_ >= USB_SEND_CMD_CYCL)
    {
      std::cout << "set Set_ISP_Sharpness Error" << std::endl;
      return false;
    }
    return false;
}

bool GvStereoDepth::getISPSharpness(const size_t &devIndex,  ISP_Sharpness_Control_t &res)
{
    nCyclTime_ = 0;
    nCyclTime_++;
    int tmpi;
    float tmpf;
    int nDenoise_Level;
    int nSharpen_Level;
    int nState;
    while (true)
    {
      nCyclTime_++;
      if (sendDataTemp_.Get_ISP_Sharpness(gvDevs_[devIndex].address, (char *)&res, ISP_SHARPNESS_CONTROL_SIZE, &nDenoise_Level, &nSharpen_Level, &nState, 500))
      {
        std::cout << "get isp sharpness success !" << std::endl;
        res.camera_index = gvDevs_[devIndex].address;
        res.denoise_level = nDenoise_Level;
        res.sharpness_level = nSharpen_Level;
        res.on_off = nState;
        return true;
      }

      if (nCyclTime_ >= USB_SEND_CMD_CYCL)
      {
        std::cout << "getSharpnessISP Error" << std::endl;
        return false;
      }
    }
}

bool GvStereoDepth::setISPCrop(const size_t &devIndex, ISP_Cproc_Control_t &data)
{   
    nCyclTime_ = 0;
    nCyclTime_++;
    if (sendDataTemp_.Set_ISP_Cproc(gvDevs_[devIndex].address, (char *)&data, ISP_CPROC_CONTROL_SIZE, 500))
    {
      std::cout << "set isp crop success !" << std::endl;
      return true;
    }

    if (nCyclTime_ >= USB_SEND_CMD_CYCL)
    {
      std::cout << "set Set_ISP_WB Error" << std::endl;
      return false;
    }
    return false;
}

bool GvStereoDepth::getISPCrop(const size_t &devIndex, ISP_Cproc_Control_t &res)
{
      // res_.camera_index = 0;
    nCyclTime_ = 0;
    nCyclTime_++;
    int nBrightness;
    float nSaturation;
    float nHue;
    float nContrast;
    int nState;
    while (true)
    {
      nCyclTime_++;

      if (sendDataTemp_.Get_ISP_Cproc(gvDevs_[devIndex].address, (char *)&res, ISP_CPROC_CONTROL_SIZE, &nBrightness, &nSaturation, &nHue, &nContrast, &nState, 500))
      {
        std::cout << "get ISP Cpros result : " << nBrightness << " " << nSaturation << " " << nHue << " " << nContrast << " " << nState << std::endl;
        res.camera_index = res.camera_index;
        return true;
      }

      if (nCyclTime_ >= USB_SEND_CMD_CYCL)
      {
        std::cout << "getISPCrop Error" << std::endl;
        return false;
      }
    }
}


bool GvStereoDepth::setUSBPacketLen(const size_t &devIndex, const size_t &n_dev_cnt)
{ 
  nCyclTime_ = 0;
  nCyclTime_++;
  if (sendDataTemp_.Set_USB_PacketLen(devIndex, n_dev_cnt, USB_TIMEOUT))
    return true;

  if(nCyclTime_ >= USB_SEND_CMD_CYCL) {
    std::cout<<"set Set_USB_PacketLen Error"<<std::endl;
    return false;
  }
  return false;
}

bool GvStereoDepth::readCalibration(const size_t &devIndex, Calc_Factor_t &fac)
{
  nCyclTime_ = 0;
  //Calc_Factor_t fac;
  float arrFactor[4];
  nCyclTime_++;
  if(sendDataTemp_.RequestCalcFactor(devIndex, arrFactor, 500)){
    fac.focus = arrFactor[0];
    fac.algo_cx = arrFactor[1];
    fac.algo_cy = arrFactor[2];
    fac.algo_baseline = arrFactor[3];
    return true;
  }

  if(nCyclTime_>=USB_SEND_CMD_CYCL){
    std::cout<<"Read Calibration Data Error"<<std::endl;
    return false;
  }
}

bool GvStereoDepth::readSN(const size_t &devIndex, std::string &sn)
{
  nCyclTime_ = 0;
  unsigned char sn_data[5] = { 0 };
  while(true){
    nCyclTime_++;
    //if(sendDataTemp_.RequestCameraSN(devIndex, true, sn_data, 500)) {
    if(sendDataTemp_.RequestCameraSN(devIndex, sn_data, 500)) {
      unsigned short nVal = 0;
      nVal = (sn_data[3] << 8) + sn_data[4];
      char cData[20] = { 0 };
      sprintf(cData, "%02d%02d%02d%04d", 2000 + sn_data[0], sn_data[1], sn_data[2], nVal);
      // std::cout<<"SN: "<<cData<<std::endl;
      sn = cData;
      return true;
    }

    if(nCyclTime_ >= USB_SEND_CMD_CYCL){
      std::cout<<"Read SN Error"<<std::endl;
      return false;
    }
  }
}


bool GvStereoDepth::getFirmwareVersion(const size_t &devIndex, CK_Version_t &version)
{
  nCyclTime_ = 0;
  while (true) {
    nCyclTime_++;
    // bool sendCKVersion(int nDevID, bool bRead, unsigned char *pData, int nTimeOut);	//for gv
    if (sendDataTemp_.sendCKVersion(devIndex,  (unsigned char *)(&version), 500)) {
      return true;
    }

    if(nCyclTime_ >= USB_SEND_CMD_CYCL){
      std::cout<<"get sendCKVersion Error"<<std::endl;
      return false;
    }
  }
}

bool GvStereoDepth::getIRParam(const size_t &devIndex, IR_IR_Calib_Para_t &ir2ir)
{
  nCyclTime_ = 0;
  while (true) {
    nCyclTime_++;
    if(sendDataTemp_.ReadIRIRCalibrationParam(devIndex, &ir2ir, IR_IR_CALIB_PARA_SIZE, 1000)) {
      // std::cout << "get RIRCalibration Success." << std::endl;
      return true;
    }

    if(nCyclTime_ >= USB_SEND_CMD_CYCL) {
      std::cout<<"Read RIRCalibration Error"<<std::endl;
      return false;
    }
  }
}

bool GvStereoDepth::getRGBParam(const size_t &devIndex, IR_RGB_Calib_Para_t &ir2rgb)
{
  nCyclTime_ = 0;
  while(true) {
    nCyclTime_++;
    if(sendDataTemp_.ReadIRRGBCalibrationParam(devIndex, &ir2rgb, IR_RGB_CALIB_PARA_SIZE, 1000)) {
      // std::cout << "get ReadIRRGBCalibrationParam Success." << std::endl;
      return true;
    }

    if(nCyclTime_ >= USB_SEND_CMD_CYCL) {
      std::cout<<"Read ReadIRRGBCalibrationParam Error"<<std::endl;
      return false;
    }
  }
}


bool GvStereoDepth::setDeviceEnable(const size_t &devIndex, bool enable)
{
  bool devEnable = false;
  if(enable)
    devEnable = enable;
  return devEnable;
}

bool GvStereoDepth::setBandwidthAndRgbFormat(const size_t &devIndex, int bandwidth, int format, int type)
{
  //std::cout << "setBandwidthAndRgbFormat : " << devIndex << " " << bandwidth  << " " << format <<  " " << type << std::endl;
  int image_width = 640;
  int image_height = 401;
  enum uvc_frame_format uvc_formate;
  
  if((format == UVC_FRAME_FORMAT_MJPEG) && (type == RgbFrameType::HD))
  {
      //std::cout << "uvc hd mjpeg !" << std::endl;
      image_width = 1280;
      image_height = 720;
      uvc_formate = UVC_FRAME_FORMAT_MJPEG;
  }
  else if((format == UVC_FRAME_FORMAT_MJPEG) && (type == RgbFrameType::VGA))
  {
      //std::cout << "uvc vga mjpeg !" << std::endl;
      image_width = 640;
      image_height = 400;
      uvc_formate = UVC_FRAME_FORMAT_MJPEG;
  }
  else if (format == UVC_FRAME_FORMAT_YUYV)
  {
    //std::cout << "uvc yuyv !" << std::endl;
    image_width = 640;
    image_height = 401;
    uvc_formate = UVC_FRAME_FORMAT_YUYV;
  }
  else
  {
    std::cout << "uvc formate " << std::to_string(format) <<  " is not support !" << std::endl;
    return false;
  }

  //std::cout << "image size : " << image_width << " " << image_height << std::endl;
  auto res = uvc_get_stream_ctrl_format_size (
  gvDevs_[devIndex].devh, &gvDevs_[devIndex].ctrl, /* result stored in ctrl */
    uvc_formate,
    image_width, image_height, 30 /* width, height, fps */
  );
  
  //std::cout << "uvc_get_stream_ctrl_format_size res : " << res << std::endl;

  if (bandwidth < 0) {
    bandwidth = gvDevs_[devIndex].ctrl.dwMaxPayloadTransferSize;
    //std::cout << "bandwidth : " << bandwidth << std::endl;
  } else {
    size_t i = 0;
    // for (; i < 6; i ++) {
    //   std::cout << i << " outMaxPacketSizeTable : " << outMaxPacketSizeTable_[i] << std::endl;
    // }
    for (i=0; i < 6; i ++) {
      if (outMaxPacketSizeTable_[i] < bandwidth && bandwidth <=  outMaxPacketSizeTable_[i + 1]) {
        bandwidth = outMaxPacketSizeTable_[i + 1];
        break;
      }
    }
    //std::cout << "i : " << i << std::endl;
    if (i == 6)
      bandwidth = outMaxPacketSizeTable_[6];
    gvDevs_[devIndex].ctrl.dwMaxPayloadTransferSize = bandwidth;
  }
  
  //std::cout << "set bandwidth success !" << std::endl;
  std::string currSn;
  readSN(gvDevs_[devIndex].address, currSn);
  //std::cout << std::endl << "device-" << static_cast<std::string>(currSn) << " start streaming with bandwidth: " << bandwidth << std::endl;
  //return true;
  res = uvc_start_streaming(gvDevs_[devIndex].devh, &gvDevs_[devIndex].ctrl, uvcCallback, (void *)(devPtr_ + devIndex), 0);
  if (res == 0) 
  {
    std::cout << " *********** uvc start streaming ************** " << std::endl;
    return true;
  }
  else 
  {
    std::cout << "can't open device with error code:" << res <<  std::endl;
      // assert(0);
    return false;
  }
}

bool GvStereoDepth::hardwareResetCamera()
{
    bool bReturn = false;
    std::cout << "camera harware restart !" << std::endl;
    for (auto &it : gvDevs_)
    {
      uvc_stop_streaming(it.devh);
      bReturn = sendDataTemp_.Hardware_Restart(it.address, 1000);
      std::cout << "hardware restart result : " << bReturn << std::endl;
      uvc_close(it.devh);
      uvc_unref_device(it.dev);
      std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    std::cout << "reset device connection !" << std::endl;

    if (bReturn)
      resetDeviceConnection(0x04cc, 0x0017);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    // auto cfg = ParameterServer::instance()->getCfgCtrlRoot();
    //  cfg = configtw::Config::object();
    std::cout << "return hardware restart result !" << std::endl;
    return bReturn;
}

bool GvStereoDepth::setGvRgbRatio(int rgbRatioCmdType)
{
  std::cout << "rgb ration cmd type : " << rgbRatioCmdType << std::endl;
  for (auto &it : gvDevs_)
  {
    if(sendDataTemp_.sendRgbRatioCmd(it.address, rgbRatioCmdType, 500))
    {
      std::cout << "send rgb ratio cmd success !" << std::endl;
      return true;
    }
  }
  return false;
}

int GvStereoDepth::getDeviceNum()
{
  return static_cast<int>(gvDevs_.size());
}

std::vector<std::string> GvStereoDepth::getDeviceSn()
{
  return cameraSn_;
}

std::vector<std::string> GvStereoDepth::getFirmwareVersion804()
{
  return firmwareVersion804_;
}

std::vector<std::string> GvStereoDepth::getFirmwareVersion805()
{
  return firmwareVersion805_;
}

std::vector<std::string> GvStereoDepth::getUsbPortId()
{
  return usbPortId_;
}

std::vector<GvStereoDepth::GV_CAMERA_PARAMS> GvStereoDepth::getGvCameraParams()
{
  return gvCameraParams_;
}

GvStereoDepth::GV_CAMERA_PARAMS GvStereoDepth::getCameraParams(int idx)
{
  return gvCameraParams_[idx];
}
}
