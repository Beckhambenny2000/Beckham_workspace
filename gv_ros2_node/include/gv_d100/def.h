#pragma once

//#include <QtEndian>
#include<string.h>

typedef unsigned char       uint8_t;
typedef unsigned short      uint16_t;
typedef unsigned int        uint32_t;

typedef signed char         int8_t;
typedef signed short        int16_t;
typedef signed int          int32_t;

#define USB_TIMEOUT		500//3000

#define SUCCESS          0 
#define EPARAM          -1 
#define ESETCFG         -2 
#define ECLAIMINF       -3 
#define EWRITE          -4 
#define EREAD           -5 
#define EOPEN           -6 
#define ECONFIG         -7 
#define ECALC			-8


#define BUF_1M_LEN				(1 * 1024 * 1024)						/* 1M */ 
#define BUF_5M_LEN				(5 * 1024 * 1024)                       /*5M*/
#define BUF_10M_LEN				(10 * 1024 * 1024)						/* 10M */ 
#define BUF_20M_LEN				(20 * 1024 * 1024)						/* 20M */ 

#define FRAME_LEN				512		//每一个报文长度
#define DATA_PER_LEN			(FRAME_LEN-CRC_LEN-HEAD_LEN)   //bin部分实际长度
#define CRC_LEN					0x04    //CRC长度
#define HEAD_LEN				12    //头长

#define FIXED_BURNING_SIZE				(4*1024)   //4K

#define EXTENT_LEN		16
#define BIN_HEAD_LEN	16


//握手
#define USB_UPGRADE_CONNECT_SEND_CMD	0x10    
#define USB_UPGRADE_CONNECT_ACK_CMD		0x30
//数据总长度信息
#define USB_UPGRADE_SEND_BIN_LEN_CMD	0x11
#define USB_UPGRADE_ACK_BIN_LEN_CMD		0x31
//发送固件
#define USB_UPGRADE_SEND_BIN_CMD		0x12	//发送
#define USB_UPGRADE_ACK_BIN_OK_CMD		0x32    //成功接收
//开始烧录
#define USB_UPGRADE_PROGRAMMER_BIN_CMD	0x13    
#define USB_UPGRADE_PROGRAMMER_OK_CMD	0x33    //烧录成功
//烧录完成，不再烧录的时候进行复位
#define USB_UPGRADE_RESET_SEND_CMD		0x14    //发送复位
#define USB_UPGRADE_RESET_OK_CMD		0x34    //执行复位
//读取板子数据
#define USB_READ_PROGRAMMER_BIN_CMD		0x15	//读取板子数据指令
#define USB_READ_PROGRAMMER_OK_CMD		0x35    //板子返回的数据

//请求算法系数
#define USB_CALC_FACTOR					0xFC	
//关闭AE和IR-LED
#define USB_BD_CLOSE_AE_IR				0xF7
//IR current Value Gears
#define USB_IR_CURRENT_GEAR				0xF8
//关闭IR
#define USB_CLOSE_IR					0xF9
// 读取写入SN
#define USB_RD_WR_SN					0xFA

#define USB_IMAGE_CHANNEL				0xFE
#define USB_SLAVE_SENSOR_STANDBY		0xFF
#define USB_SLAVE_SENSOR_STEAMING		0xE0

#define USB_CK805_ENTER_TWEAKING_MODE	0xE1
#define USB_CK805_PARAM_TWEAK			0xE2
#define USB_FRAMERATE_ADJUST			0xE3		//设置帧率
#define USB_OBTAIN_SOFTVERSION			0xE4		// 获取编译器版本
#define USB_RGB_RESOLUTION				0xE5		// RGB分辨率设置
#define USB_RGB_CALIB_MODE				0xE6		// 设置RGB标定模式

#define USB_RW_CALIBRATION_PARAMS_IR_IR		0xE8
#define USB_RW_CALIBRATION_PARAMS_IR_RGB	0xE9

#define USB_ISP_EXPOSURE_CONTROL			0xA1
#define USB_ISP_WB_CONTROL					0xA2
#define USB_ISP_CPROC_CONTROL				0xA3
#define USB_ISP_SHARPNESS_CONTROL			0xA4
#define USB_ISP_GAMMA_CONTROL				0xA5

#define USB_PACKET_LEN						0xDF

#define HARDWARE_RESTART					0x99

#define G_BASE_ADDR 0xBA7000	//双目偏移地址

#define DATA_LEN 0x08


#pragma pack(1)


//发送的头 PC->510
struct send_pack_head_t {
	send_pack_head_t()
	{
		head[0] = 'A';
		head[1] = 'B';
		head[2] = 'H';
		cmd = 0;
		nDataSize = 0;
	}
	uint8_t head[3];
	uint8_t cmd;
	uint32_t nAddr;			//烧录时是地址，命令时就是指令
	uint32_t nDataSize;

	uint32_t FromBigEndian1(uint32_t data)
	{
		uint32_t result_val;

		result_val = (data & 0xff) << 24;
		result_val |= ((data >> 8) & 0xff) << 16;
		result_val |= ((data >> 16) & 0xff) << 8;
		result_val |= (data >> 24) & 0xff;

		return result_val;
	}

	void ready()
	{
		nAddr = FromBigEndian1(nAddr);
		nDataSize = FromBigEndian1(nDataSize);
	}
};

//接收数据的协议的帧头 510->PC
 struct recv_pack_head_t {
	recv_pack_head_t()
	{
		head[0] = 'A';
		head[1] = 'B';
		head[2] = 'H';
		cmd = 0;
		result = 0;
		nAddr = 0;
		nDataSize = 0;
	}
	uint8_t head[3];
	uint8_t cmd;  
	uint8_t result;
	uint32_t nAddr;
	uint32_t nDataSize;	
	//uint8_t checkSum;

	uint32_t FromBigEndian1(uint32_t data)
	{
		uint32_t result_val;

		result_val = (data & 0xff) << 24;
		result_val |= ((data >> 8) & 0xff) << 16;
		result_val |= ((data >> 16) & 0xff) << 8;
		result_val |= (data >> 24) & 0xff;

		return result_val;
	}

	void ready()
	{
		nAddr = FromBigEndian1(nAddr);
		nDataSize = FromBigEndian1(nDataSize);
	}
};

 //16字节对齐头
 struct bin_head_t {
	 uint8_t magic[4];
	 uint8_t load_addr[4];
	 uint8_t img_len[4];
	 uint8_t res[4];
 };

 struct Sub_Header_t
 {
	 uint8_t nSerial;		//序号
	 uint32_t nMsgSize;		//报文大小
 };

struct Calibration_Param_t
{
	double cx;
	double cy;
	double focus;
	double baseLine;
};

//重影误差
struct Calibration_Error_t
{
	float err_1;
	float err_2;
};

//数据总长度信息
struct Data_Len_t
{
	Data_Len_t()
	{
		nAddr = 0;
		nDataLen = 0;
	}
	uint32_t nAddr;			//数据地址
	uint32_t nDataLen;		//数据长度

	uint32_t FromBigEndian1(uint32_t data)
	{
		uint32_t result_val;

		result_val = (data & 0xff) << 24;
		result_val |= ((data >> 8) & 0xff) << 16;
		result_val |= ((data >> 16) & 0xff) << 8;
		result_val |= (data >> 24) & 0xff;

		return result_val;
	}

	void ready()
	{
		nAddr = FromBigEndian1(nAddr);
		nDataLen = FromBigEndian1(nDataLen);
	}
};

//算法系数
struct Calc_Factor_t
{
	Calc_Factor_t()
	{
		init();
	}

	void init()
	{
		focus = 381.867;
		algo_cx = 326.56;
		algo_cy = 174.009;
		algo_baseline = 51.4085;
	}

	float algo_cx;
	float algo_cy;
	float focus;
	float algo_baseline;
};

struct Calc_Factor_Roi_t
{
	Calc_Factor_Roi_t()
	{
		init();
	}

	void init()
	{
		algo_roi_x = 0;
		algo_roi_y = 0;
		algo_roi_width = 0;
		algo_roi_height = 0;
	}

	int algo_roi_x;
	int algo_roi_y;
	int algo_roi_width;
	int algo_roi_height;
};

struct Control_AE_Model_t
{
	Control_AE_Model_t()
	{
		ae_on_off = 0;
		ir_led_reg_val = 0;
		ir_on_off = 0;
		rd_wr_sn = 0;
		video_selected = 0;
		memset((char*)serial_number, 0, 5);
	}
	unsigned char ae_on_off;
	unsigned char ir_led_reg_val;
	unsigned char ir_on_off;	//1是开 0是关
	unsigned char rd_wr_sn;		// 1是写SN，0是读SN
	unsigned char video_selected;	//图像流模式
	unsigned char serial_number[5];
};

#define IR_IR_CALIB_PARA_SIZE		68*4
struct IR_IR_Calib_Para_t
{
	float left_camera_matrix_unRectify[9];
	float left_dist_coeffs_unRectify[5];
	float right_camera_matrix_unRectify[9];
	float right_dist_coeffs_unRectify[5];
	float left_camera_matrix[9];
	float left_dist_coeffs[5];
	float right_camera_matrix[9];
	float right_dist_coeffs[5];
	float R[9];
	float T[3];
};

#define IR_RGB_CALIB_PARA_SIZE		40*4
struct IR_RGB_Calib_Para_t
{
	float left_fc[2];
	float left_cc[2];
	float left_kc[5];
	float left_alpha_c;
	float right_fc[2];
	float right_cc[2];
	float right_kc[5];
	float right_alpha_c;
	float R[9];
	float T[3];
	float Reseve[8];
};

#define ISP_EXPOSURE_CONTROL_SIZE 8 * 4
struct ISP_Exposure_Control_t
{
	int camera_index;			// 0:IR相机； 2:RGB相机
	int auto_manual_exposure;	// 0 / 1
	float auto_exposure_target; //
	float auto_exposure_range;	//
	float manual_exposure_value;
	float manual_gain_value;
	float result_exposure;
	float result_gain;
};

#define ISP_WB_CONTROL_SIZE 6 * 4
struct ISP_WB_Control_t
{
	int camera_index;	// 0:IR相机； 2:RGB相机
	int auto_manual_wb; // bit[1]=0:manual mode; bit[1]=1:auto mode; bit[0]=1:get result;
	float wb_r;
	float wb_gr;
	float wb_gb;
	float wb_b;
};

#define ISP_CPROC_CONTROL_SIZE 4 * 4
struct ISP_Cproc_Control_t
{
	int cproc_id;	  //
	int camera_index; // 0:IR相机； 2:RGB相机
	int on_off;		  // 0:off; 1:on
	float value;
};

#define ISP_SHARPNESS_CONTROL_SIZE 4 * 4
struct ISP_Sharpness_Control_t
{
	int camera_index; // 0:IR相机； 2:RGB相机
	int on_off;		  // 0:off; 1:on
	int denoise_level;
	int sharpness_level;
};

#define ISP_GAMMA_CONTROL_SIZE 2 * 4
struct ISP_Gamma_Control_t
{
	int camera_index; // 0:IR相机； 2:RGB相机
	int on_off;		  // 0:off; 1:on
};



struct Frame_Rate_t
{
	unsigned char nFrameRate;		//帧率 5 10 15 20 25
};

struct CK_Version_t
{
	CK_Version_t()
	{
		nType = 0;
		memset(ck_804, 0, 25);
		memset(ck_805, 0, 25);
	}
	unsigned char nType;	// 1 write 0 read
	unsigned char ck_804[25];		// ck804版本
	unsigned char ck_805[25];		// ck805版本
};

#pragma pack()