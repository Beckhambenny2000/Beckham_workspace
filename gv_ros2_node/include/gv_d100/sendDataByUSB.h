#ifndef SEND_DATA_BY_USB_H
#define SEND_DATA_BY_USB_H

#include "def.h"
namespace gv
{
	class SendDataByUSB
	{
	public:
		SendDataByUSB(){};
		~SendDataByUSB();
	protected:

		int m_nCurrent_count;

		int init();
		void uinit();
		void uinit2();

	public:
		void GetMaxPacketSize(bool bShowLeftIR, bool bShowRightIR, bool bShowDepth, bool bShowRGB, int nInFPS, int *nOutMaxPacketSize, int *nOutMaxFPS);

		//bool usbDownloadData(int nDevID, char* pData, int nDataLen, bool m_bAddHead = true, int m_runAddr = 0x1000, int m_baseAddr = 0x41000, int nTimeOut = 10000);

		bool sendCloseIRCmd(int nDevID, bool bClose, int nTimeOut);		
		bool sendCloseAECmd(int nDevID, bool bClose, int nTimeOut);		
		bool sendIRCurrentSelect(int nDevID, int nLevel, int nTimeOut);
		bool RequestCalcFactor(int nDevID, float *arrFactor, int nTimeOut);
		bool RequestCameraSN(int nDevID, unsigned char *pData, int nTimeOut);	

		bool sendImgChannel(int nDevID, bool bLeft, bool bRight, bool bDepth, bool bRGB, int nTimeOut);

		bool sendFrameRate(int nDevID, unsigned char nVal, int nTimeOut);
		bool sendCKVersion(int nDevID, unsigned char *pData, int nTimeOut);	
		bool sendRgbRatioCmd(int nDevID, unsigned char nType, int nTimeOut);

		bool ReadIRIRCalibrationParam(int nDevID, IR_IR_Calib_Para_t *arrIR_IR_Factor, int nLen, int nTimeOut);	
		bool ReadIRRGBCalibrationParam(int nDevID, IR_RGB_Calib_Para_t *arrIR_RGB_Factor, int nLen, int nTimeOut);
		bool Set_USB_PacketLen(int nDevID, int nTotalDevNum, int nTimeOut);	

		bool Get_ISP_Exposure(int nDevID, char* pData, int nLen, float *fExposure, float *fGain, int *nState, int nTimeOut);
		bool Set_ISP_Exposure(int nDevID, char* pData, int nLen, int nTimeOut);

		bool Get_ISP_WB(int nDevID, char* pData, int nLen, float *wb_R_Gain, float *wb_Gr_Gain, float *wb_Gb_Gain, float *wb_B_Gain, int *nState, int nTimeOut);
		bool Set_ISP_WB(int nDevID, char* pData, int nLen, int nTimeOut);

		bool Get_ISP_Cproc(int nDevID, char* pData, int nLen, int *nBrightness, float *nSaturation, float *nHue, float *nContrast, int *nState, int nTimeOut);
		bool Set_ISP_Cproc(int nDevID, char* pData, int nLen, int nTimeOut);
		
		bool Get_ISP_Sharpness(int nDevID, char* pData, int nLen, int *nDenoise_Level, int *nSharpen_Level, int *nState, int nTimeOut);
		bool Set_ISP_Sharpness(int nDevID, char* pData, int nLen, int nTimeOut);
		
		bool Get_ISP_Gamma(int nDevID, char* pData, int nLen, int *nState, int nTimeOut);
		bool Set_ISP_Gamma(int nDevID, char* pData, int nLen, int nTimeOut);

		bool Hardware_Restart(int nDevID, int nTimeOut);
	};
}

#endif

