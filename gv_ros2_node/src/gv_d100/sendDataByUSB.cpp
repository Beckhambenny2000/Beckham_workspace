// SendDataByUSB.cpp: 实现文件
//

#include "gv_d100/sendDataByUSB.h"
#include "gv_d100/usb.h"
//#include <WinSock2.h>
//#include <winsock.h>

#include<iostream>
#include<fstream>
using namespace std;

// SendDataByUSB
//
//SendDataByUSB::SendDataByUSB()
//{
//}
//
//SendDataByUSB::~SendDataByUSB()
//{
//}

int m_nCurrent_count;
const uint8_t MSG_TOP[3] = { 'T','S','M' };

// SendDataByUSB 成员函数
namespace gv
{
	// uint32_t FromBigEndian(uint32_t data);
	// void store_uint32(uint8_t *addr, uint32_t value_1);
	uint32_t FromBigEndian(uint32_t data)
	{
		uint32_t result_val;

		result_val = (data & 0xff) << 24;
		result_val |= ((data >> 8) & 0xff) << 16;
		result_val |= ((data >> 16) & 0xff) << 8;
		result_val |= (data >> 24) & 0xff;

		return result_val;
	}

	void store_uint32(uint8_t *addr, uint32_t value_1)   /*small endian*/
	{
		//uint32_t value = htonl(value_1);
		uint32_t value;
		value = (value_1 & 0xff) << 24;
		value |= ((value_1 >> 8) & 0xff) << 16;
		value |= ((value_1 >> 16) & 0xff) << 8;
		value |= (value_1 >> 24) & 0xff;

		addr[0] = value >> 24;
		addr[1] = (value >> 16) & 0xff;
		addr[2] = (value >> 8) & 0xff;
		addr[3] = value & 0xff;
	}

	bool sendHandShakeCmd(int nDevID, int nTimeOut)
	{
		m_nCurrent_count = 0;
		send_pack_head_t sendCmd;
		sendCmd.head[0] = 'T'; sendCmd.head[1] = 'S'; sendCmd.head[2] = 'M';
		sendCmd.cmd = USB_UPGRADE_CONNECT_SEND_CMD;
		sendCmd.nAddr = 0;
		sendCmd.nDataSize = 0;
		sendCmd.ready();
		int nHeadLen = sizeof(send_pack_head_t);
		uint32_t nCheckSum = crc32_bit((uint8_t*)&sendCmd + nHeadLen, 0);
		nCheckSum = FromBigEndian(nCheckSum);
		char cData[FRAME_LEN] = { 0 };
		memcpy(cData, &sendCmd, nHeadLen);
		memcpy(cData + nHeadLen, &nCheckSum, sizeof(uint32_t));
		int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(uint32_t), nTimeOut);
		if (nRet >= 0)
		{
			memset(cData, 0, FRAME_LEN);
			nRet = usb_read(nDevID, cData, FRAME_LEN, nTimeOut);
			if (nRet >= 0)
			{
				recv_pack_head_t recvCmd;
				recvCmd.head[0] = 'T'; recvCmd.head[1] = 'S'; recvCmd.head[2] = 'M';
				memcpy(&recvCmd, cData, sizeof(recvCmd));

				if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2] && recvCmd.cmd == USB_UPGRADE_CONNECT_ACK_CMD)
				{
					return true;
				}
				else
				{
					return false;
				}
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}


	bool sendFinishCmd(int nDevID, int nTimeOut)
	{
		send_pack_head_t sendCmd;
		sendCmd.head[0] = 'T'; sendCmd.head[1] = 'S'; sendCmd.head[2] = 'M';
		sendCmd.cmd = USB_UPGRADE_PROGRAMMER_BIN_CMD;
		sendCmd.nAddr = 0;
		sendCmd.nDataSize = 0;
		sendCmd.ready();
		int nHeadLen = sizeof(send_pack_head_t);
		uint32_t nCheckSum = crc32_bit((uint8_t*)&sendCmd + nHeadLen, 0);
		nCheckSum = FromBigEndian(nCheckSum);
		char cData[FRAME_LEN] = { 0 };
		memcpy(cData, &sendCmd, nHeadLen);
		memcpy(cData + nHeadLen, &nCheckSum, sizeof(uint32_t));
		int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(uint32_t), nTimeOut);
		if (nRet >= 0)
		{
			memset(cData, 0, FRAME_LEN);
			nRet = usb_read(nDevID, cData, FRAME_LEN, nTimeOut);
			if (nRet >= 0)
			{
				recv_pack_head_t recvCmd;
				recvCmd.head[0] = 'T'; recvCmd.head[1] = 'S'; recvCmd.head[2] = 'M';
				memcpy(&recvCmd, cData, sizeof(recvCmd));

				if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2] && recvCmd.cmd == USB_UPGRADE_PROGRAMMER_OK_CMD)
				{
					///qDebug() << "Step_4 success to finish 4K";
					return true;
				}
				else
				{
					return false;
				}
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}

	bool sendFixedBurnData(int nDevID, char* pData, int nLen, int nTimeOut)
	{
		bool bReturn = true;
		int nTimes = nLen / DATA_PER_LEN;
		int nRemain = nLen % DATA_PER_LEN;

		send_pack_head_t tempHeader;
		tempHeader.head[0] = 'T'; tempHeader.head[1] = 'S'; tempHeader.head[2] = 'M';
		tempHeader.cmd = USB_UPGRADE_SEND_BIN_CMD;		//数据总长度信息

		int nHeadLen = sizeof(send_pack_head_t);

		//++m_nCurrent_count;
		// qDebug() << "nTimes: " << nTimes;
		for (int i = 0; i < nTimes; i++)
		{
			char cData[FRAME_LEN + 1] = { 0 };
			tempHeader.nDataSize = DATA_PER_LEN;
			tempHeader.nAddr = m_nCurrent_count;
			tempHeader.ready();
			memcpy(cData, &tempHeader, nHeadLen);
			memcpy(cData + nHeadLen, pData + i * DATA_PER_LEN, DATA_PER_LEN);

			//qDebug() << QByteArray(pData + i * DATA_PER_LEN, DATA_PER_LEN).toHex(' ');

			uint32_t nChecksum = crc32_bit((uint8_t*)cData + nHeadLen, DATA_PER_LEN);
			nChecksum = FromBigEndian(nChecksum);
			memcpy(cData + nHeadLen + DATA_PER_LEN, &nChecksum, CRC_LEN);

			int nSendLen = nHeadLen + DATA_PER_LEN + CRC_LEN;
			static int nsendedIndex = 0;
			nsendedIndex++;
			int nRet = usb_write(nDevID, cData, nSendLen, nTimeOut);
			if (nRet >= 0)
			{
				//qDebug() << "send";

				memset(cData, 0, FRAME_LEN);
				nRet = usb_read(nDevID, cData, FRAME_LEN, nTimeOut);
				if (nRet >= 0)
				{
					recv_pack_head_t recvCmd;
					recvCmd.head[0] = 'T'; recvCmd.head[1] = 'S'; recvCmd.head[2] = 'M';
					memcpy(&recvCmd, cData, sizeof(recvCmd));
					if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2] && recvCmd.cmd == USB_UPGRADE_ACK_BIN_OK_CMD)
					{
						bReturn = true;
					}
					else
					{
						bReturn = false;
						break;
					}
				}
				else
				{
					bReturn = false;
					break;
				}

			}
			else
			{
				bReturn = false;
				break;
			}

			++m_nCurrent_count;
		}


		if (nRemain > 0)
		{
			tempHeader.nDataSize = nRemain;
			char cData[FRAME_LEN] = { 0 };
			tempHeader.nAddr = m_nCurrent_count;
			tempHeader.ready();
			memcpy(cData, &tempHeader, nHeadLen);
			memcpy(cData + nHeadLen, pData + nTimes * DATA_PER_LEN, nRemain);
			uint32_t nChecksum = crc32_bit((uint8_t*)cData + nHeadLen, nRemain);
			nChecksum = FromBigEndian(nChecksum);
			memcpy(cData + nHeadLen + nRemain, &nChecksum, CRC_LEN);

			int nSendLen = nHeadLen + nRemain + CRC_LEN;
			int nRet = usb_write(nDevID, cData, nSendLen, nTimeOut);
			if (nRet >= 0)
			{
				memset(cData, 0, FRAME_LEN);
				nRet = usb_read(nDevID, cData, FRAME_LEN, nTimeOut);
				if (nRet >= 0)
				{
					recv_pack_head_t recvCmd;
					recvCmd.head[0] = 'T'; recvCmd.head[1] = 'S'; recvCmd.head[2] = 'M';
					memcpy(&recvCmd, cData, sizeof(recvCmd));
					if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2] && recvCmd.cmd == USB_UPGRADE_ACK_BIN_OK_CMD)
					{
						//qDebug() << "Step_3 finished";
						//emit sig_progress(i * 100 / nTimes);
						////int mmm = 0;
						bReturn = true;
					}
					else
					{
						bReturn = false;
					}
				}
				else
				{
					bReturn = false;
				}
			}
			else
			{
				bReturn = false;
			}
			++m_nCurrent_count;
		}

		return bReturn;
	}

	bool sendBurnData(int nDevID, char* pBurnData, int nBurnDataLen, int m_baseAddr, int nTimeOut)
	{
		bool bReturn = true;
		int nAllDataLen = nBurnDataLen;

		if (nAllDataLen <= 0)
			return false;

		int nTimes = nAllDataLen / FIXED_BURNING_SIZE;		//分成多少个4K的包
		int nRemain = nAllDataLen % FIXED_BURNING_SIZE;		//剩余多少
		char* pData = pBurnData;

		for (int index = 0; index < nTimes; index++)
		{
			char cData[FRAME_LEN] = { 0 };

			send_pack_head_t tempHeader;
			tempHeader.head[0] = 'T'; tempHeader.head[1] = 'S'; tempHeader.head[2] = 'M';
			tempHeader.cmd = USB_UPGRADE_SEND_BIN_LEN_CMD;		//数据总长度信息
			tempHeader.nAddr = m_nCurrent_count;
			tempHeader.nDataSize = DATA_LEN;
			tempHeader.ready();
			int nHeadLen = sizeof(send_pack_head_t);
			memcpy(cData, &tempHeader, nHeadLen);
			Data_Len_t tempDataLen_t;
			tempDataLen_t.nAddr = m_baseAddr + index * FIXED_BURNING_SIZE;

			tempDataLen_t.nDataLen = FIXED_BURNING_SIZE;
			tempDataLen_t.ready();
			memcpy(cData + nHeadLen, &tempDataLen_t, DATA_LEN);
			uint32_t nChecksum = crc32_bit((uint8_t*)cData + nHeadLen, DATA_LEN);
			nChecksum = FromBigEndian(nChecksum);
			memcpy(cData + nHeadLen + DATA_LEN, &nChecksum, CRC_LEN);

			int nSendLen = nHeadLen + DATA_LEN + CRC_LEN;

			int nRet = usb_write(nDevID, cData, nSendLen, USB_TIMEOUT);
			if (nRet >= 0)
			{
				memset(cData, 0, FRAME_LEN);
				nRet = usb_read(nDevID, cData, FRAME_LEN, USB_TIMEOUT);
				if (nRet >= 0)
				{
					recv_pack_head_t recvCmd;
					recvCmd.head[0] = 'T'; recvCmd.head[1] = 'S'; recvCmd.head[2] = 'M';
					memcpy(&recvCmd, cData, sizeof(recvCmd));
					if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2] && recvCmd.cmd == USB_UPGRADE_ACK_BIN_LEN_CMD)
					{
						bReturn = sendFixedBurnData(nDevID, pData + index * FIXED_BURNING_SIZE, FIXED_BURNING_SIZE, nTimeOut);
						if (!bReturn)
						{
							bReturn = false;
							break;
						}
						else
						{
							bReturn = true;
						}
					}
					else
					{
						bReturn = false;
						break;
					}
				}
				else
				{
					bReturn = false;
					break;
				}
			}
			else
			{
				bReturn = false;
				break;
			}

			
			bReturn = sendFinishCmd(nDevID, nTimeOut);
			//emit sig_progress(index * 100 / nTimes, QString("download %1 / %2, burn_addr=0x%3").arg(index + 1).arg(nTimes).arg(QString::number(tempDataLen_t.nAddr, 16)));
			++m_nCurrent_count;
		}

		if (nRemain > 0)
		{
			char cData[FRAME_LEN] = { 0 };
			send_pack_head_t tempHeader;
			tempHeader.head[0] = 'T'; tempHeader.head[1] = 'S'; tempHeader.head[2] = 'M';
			tempHeader.cmd = USB_UPGRADE_SEND_BIN_LEN_CMD;		//数据总长度信息
			tempHeader.nAddr = m_nCurrent_count;
			tempHeader.nDataSize = DATA_LEN;
			tempHeader.ready();
			int nHeadLen = sizeof(send_pack_head_t);
			memcpy(cData, &tempHeader, nHeadLen);
			Data_Len_t tempDataLen_t;
			tempDataLen_t.nAddr = m_baseAddr + nTimes * FIXED_BURNING_SIZE;
			tempDataLen_t.nDataLen = nRemain;
			tempDataLen_t.ready();
			memcpy(cData + nHeadLen, &tempDataLen_t, DATA_LEN);
			uint32_t nChecksum = crc32_bit((uint8_t*)cData + nHeadLen, DATA_LEN);
			nChecksum = FromBigEndian(nChecksum);
			memcpy(cData + nHeadLen + DATA_LEN, &nChecksum, CRC_LEN);
			int nSendLen = nHeadLen + DATA_LEN + CRC_LEN;
			int nRet = usb_write(nDevID, cData, nSendLen, USB_TIMEOUT);
			if (nRet >= 0)
			{
				memset(cData, 0, FRAME_LEN);
				nRet = usb_read(nDevID, cData, FRAME_LEN, USB_TIMEOUT);
				if (nRet >= 0)
				{
					recv_pack_head_t recvCmd;
					recvCmd.head[0] = 'T'; recvCmd.head[1] = 'S'; recvCmd.head[2] = 'M';
					memcpy(&recvCmd, cData, sizeof(recvCmd));
					if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2] /*&& recvCmd.cmd == USB_UPGRADE_ACK_BIN_LEN_CMD*/)
					{
						bReturn = sendFixedBurnData(nDevID, pData + nTimes * FIXED_BURNING_SIZE, nRemain, nTimeOut);
						//emit sig_progress(100, QString("finished to load file!!!!"), true);
						if (!bReturn)
						{
							bReturn = false;
						}
						else
						{
							bReturn = true;
						}
					}
					else
					{
						bReturn = false;
					}
				}
				else
				{
					bReturn = false;
				}
			}
			else
			{
				bReturn = false;
			}

			bReturn = sendFinishCmd(nDevID, nTimeOut);
			++m_nCurrent_count;
		}

		return bReturn;
	}

	bool sendBurnDataWithHead(int nDevID, char* pData, int nDataLen, int m_runAddr, int m_baseAddr, int nTimeOut)
	{
		bool bReturn = false;
		int nAllDataLen = nDataLen;
		int nExtent_len = 0;
		if (nAllDataLen % 16 != 0)
		{
			nExtent_len = 16 - (nAllDataLen % 16);
		}
		bin_head_t bin_head;
		bin_head.magic[0] = 'Q';
		bin_head.magic[1] = 'W';
		bin_head.magic[2] = 'Z';
		bin_head.magic[3] = 'N';

		/****************小端**********************/
		m_runAddr = FromBigEndian(m_runAddr);
		store_uint32(bin_head.load_addr, m_runAddr);	//运行地址
		store_uint32(bin_head.img_len, nAllDataLen + nExtent_len);
		store_uint32(bin_head.res, 0);
		/*****************************************/
		int nBurnDataLen = nAllDataLen + nExtent_len + BIN_HEAD_LEN;
		char *pBurnData = new char[nBurnDataLen];
		memset(pBurnData, 0, nBurnDataLen);

		memcpy(pBurnData, &bin_head, BIN_HEAD_LEN);
		memcpy(pBurnData + BIN_HEAD_LEN, pData, nAllDataLen);
		bReturn = sendBurnData(nDevID, pBurnData, nBurnDataLen, m_baseAddr, nTimeOut);
		delete[] pBurnData;
		pBurnData = nullptr;
		return bReturn;
	}

	bool sendCalcedData(int nDevID, bool m_bAddHead, char* pData, int nLen, int m_runAddr,int m_baseAddr, int nTimeOut)
	{
		bool bReturn = true;

		if (m_bAddHead)
			bReturn = sendBurnDataWithHead(nDevID, pData, nLen, m_runAddr, m_baseAddr, nTimeOut);
		else
		{
			bReturn = sendBurnData(nDevID, pData, nLen, m_baseAddr, nTimeOut);
		}

			return bReturn;

#pragma region old code 
		//

#pragma endregion

	}

	int OutMaxPacketSizeTable[3][6] =
	{
		{512,	1020,	1536,	2040,	2556,	3060},
		{1020,	1536,	2040,	2556,	3060,	3060},
		{1020,	1536,	2556,	3060,	3060,	3060}
	};
	int OutMaxFPSTable[3][6] =
	{
		{5,	10,	15,	20,	25,	30},
		{5,	10,	15,	20,	25,	30},
		{5,	10,	15,	20,	20,	20}
	};

	void SendDataByUSB::GetMaxPacketSize(bool bShowLeftIR, bool bShowRightIR, bool bShowDepth, bool bShowRGB, int nInFPS, int *nOutMaxPacketSize, int *nOutMaxFPS)
	{
		int nOpenImageChannelIndex = -1;
		int nInputFPSIndex = -1;

		if (bShowLeftIR || bShowRightIR || bShowDepth || bShowRGB)
			nOpenImageChannelIndex = 0;

		if (bShowLeftIR && bShowRightIR)
			nOpenImageChannelIndex = 1;

		if ((bShowDepth && bShowLeftIR && bShowRightIR) || 
			(bShowDepth && bShowLeftIR && bShowRGB) || 
			(bShowLeftIR && bShowRGB) || 
			(bShowDepth && bShowRGB))
			nOpenImageChannelIndex = 2;

		if ((bShowDepth && bShowLeftIR && bShowRightIR && bShowRGB) ||
			(bShowRGB && bShowLeftIR && bShowRightIR)||
			(bShowRGB && bShowRightIR))
			nOpenImageChannelIndex = -1;

		if (nInFPS == 5)
			nInputFPSIndex = 0;
		else if(nInFPS == 10)
			nInputFPSIndex = 1;
		else if (nInFPS == 15)
			nInputFPSIndex = 2;
		else if (nInFPS == 20)
			nInputFPSIndex = 3;
		else if (nInFPS == 25)
			nInputFPSIndex = 4;
		else if (nInFPS == 30)
			nInputFPSIndex = 5;
		
		if (nOpenImageChannelIndex == -1 || nInputFPSIndex == -1)
		{
			*nOutMaxPacketSize = 0;
			*nOutMaxFPS = 0;
		}
		else
		{
			*nOutMaxPacketSize = OutMaxPacketSizeTable[nOpenImageChannelIndex][nInputFPSIndex];
			*nOutMaxFPS = OutMaxFPSTable[nOpenImageChannelIndex][nInputFPSIndex];
		}
	}


	int SendDataByUSB::init()
	{
		//return usb_initial();
		return usb_initial();
	}

	void SendDataByUSB::uinit()
	{
		usb_release();
	}

	void SendDataByUSB::uinit2()
	{
		usb_release2();
	}

	bool SendDataByUSB::Hardware_Restart(int nDevID, int nTimeOut)
	{
		bool bReturn = false;
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T'; sendCmd.head[1] = 'S'; sendCmd.head[2] = 'M';
			sendCmd.cmd = HARDWARE_RESTART;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = 0;
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			char cData[FRAME_LEN] = { 0 };

			memcpy(cData, &sendCmd, nHeadLen);
			//memcpy(cData + nHeadLen, pData, nLen);

			//uint32_t nCheckSum = crc32_bit((uint8_t*)&sendCmd + nHeadLen, nLen);
			//nCheckSum = FromBigEndian(nCheckSum);
			//memcpy(cData + nHeadLen + nLen, &nCheckSum, sizeof(uint32_t));
			//int nRet = usb_write(cData, nHeadLen + nLen + sizeof(uint32_t), nTimeOut);

			int nRet = usb_write(nDevID, cData, nHeadLen, nTimeOut);

			if (nRet >= 0)
			{
				bReturn = true;
			}
			else
			{
				bReturn = false;
			}
		}
		else
		{
			bReturn = false;
		}

		SendDataByUSB::uinit();
		return bReturn;
	}

	bool SendDataByUSB::sendCloseIRCmd(int nDevID, bool bClose, int nTimeOut)
	{
		//int nRet = this->init();
		int nRet = SendDataByUSB::init();
		//if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T'; sendCmd.head[1] = 'S'; sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_CLOSE_IR;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = sizeof(Control_AE_Model_t);
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = { 0 };
			unsigned char nAEState = 0;
			if (bClose)
			{
				tempStu.ir_on_off = 0;
			}
			else
			{
				tempStu.ir_on_off = 1;
			}
			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));

			uint32_t nCheckSum = crc32_bit((uint8_t*)&sendCmd + nHeadLen, sizeof(Control_AE_Model_t));
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), &nCheckSum, sizeof(uint32_t));

			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(uint32_t) + sizeof(Control_AE_Model_t), nTimeOut);
			if (nRet < 0)
			{
				//this->uinit();
				SendDataByUSB::uinit();
				return false;
			}
		}

		//this->uinit();
		SendDataByUSB::uinit();
		return true;
	}

	bool SendDataByUSB::sendCloseAECmd(int nDevID, bool bClose, int nTimeOut)
	{
		int nRet = this->init();
		//if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T'; sendCmd.head[1] = 'S'; sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_BD_CLOSE_AE_IR;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = sizeof(Control_AE_Model_t);
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = { 0 };
			unsigned char nAEState = 0;
			if (bClose)
			{
				tempStu.ae_on_off = 0;
			}
			else
			{
				tempStu.ae_on_off = 1;
			}
			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));

			uint32_t nCheckSum = crc32_bit((uint8_t*)&sendCmd + nHeadLen, sizeof(Control_AE_Model_t));
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), &nCheckSum, sizeof(uint32_t));
			//qDebug() << QByteArray(cData, nHeadLen + sizeof(uint32_t) + sizeof(Control_AE_Model_t)).toHex(' ');
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(uint32_t) + sizeof(Control_AE_Model_t), nTimeOut);
			if (nRet < 0)
			{
				SendDataByUSB::uinit();
				return false;
			}
		}
		SendDataByUSB::uinit();
		return true;
	}

	bool SendDataByUSB::sendIRCurrentSelect(int nDevID, int nLevel, int nTimeOut)
	{
		//int nRet = this->init();
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T'; sendCmd.head[1] = 'S'; sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_IR_CURRENT_GEAR;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = sizeof(Control_AE_Model_t);
			sendCmd.ready();
			Control_AE_Model_t tempStu;
			tempStu.ir_led_reg_val = nLevel;

			char cData[FRAME_LEN] = { 0 };
			int nHeadLen = sizeof(send_pack_head_t);
			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));

			uint32_t nCheckSum = crc32_bit((uint8_t*)&sendCmd + nHeadLen, sizeof(Control_AE_Model_t));
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), &nCheckSum, sizeof(uint32_t));
			//qDebug() << QByteArray(cData, nHeadLen + sizeof(uint32_t) + sizeof(Control_AE_Model_t)).toHex(' ');
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(uint32_t) + sizeof(Control_AE_Model_t), nTimeOut);

			if (nRet >= 0)
			{
				//memset(cData, 0, FRAME_LEN);
				//nRet = usb_read(cData, FRAME_LEN, USB_TIMEOUT);

				//if (nRet >= 0)
				//{
				//	recv_pack_head_t recvCmd;
				//	memcpy(&recvCmd, cData, sizeof(recvCmd));

				//	if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2] /* && recvCmd.cmd == USB_UPGRADE_PROGRAMMER_OK_CMD*/)
				//	{
				//		//qDebug() << "send IR current gear";
				//		AfxMessageBox("send IR current gear", MB_OK | MB_ICONERROR);
				//	}
				//	else
				//	{
				//		this->uinit();
				//		return false;
				//	}
				//}
				//else
				//{
				//	this->uinit();
				//	return false;
				//}
			}
			else
			{
				//this->uinit();
				SendDataByUSB::uinit();
				return false;
			}
		}
		else
		{
			//this->uinit();
			SendDataByUSB::uinit();
			return false;
		}


		//this->uinit();
		SendDataByUSB::uinit();
		return true;
	}

	bool SendDataByUSB::RequestCameraSN(int nDevID, unsigned char *pData, int nTimeOut)
	{
		//int nRet = this->init();
		int nRet = SendDataByUSB::init();
		//if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T'; sendCmd.head[1] = 'S'; sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_RD_WR_SN;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = sizeof(Control_AE_Model_t);
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = { 0 };
			unsigned char nAEState = 0;
			if (1)
			{
				tempStu.rd_wr_sn = 0;
			}
			else
			{
				tempStu.rd_wr_sn = 1;
				memcpy(tempStu.serial_number, pData, 5);
			}
			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));

			uint32_t nCheckSum = crc32_bit((uint8_t*)&sendCmd + nHeadLen, sizeof(Control_AE_Model_t));
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), &nCheckSum, sizeof(uint32_t));
			//qDebug() << QByteArray(cData, nHeadLen + sizeof(uint32_t) + sizeof(Control_AE_Model_t)).toHex(' ');
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(uint32_t) + sizeof(Control_AE_Model_t), nTimeOut);
			if (nRet >= 0)
			{
				//qDebug() << "send SN CMD";

				memset(cData, 0, FRAME_LEN);
				nRet = usb_read(nDevID, cData, FRAME_LEN, nTimeOut);
				if (nRet >= 0)
				{
					//recv_pack_head_t recvCmd;
					send_pack_head_t recvCmd;
					recvCmd.head[0] = 'T'; recvCmd.head[1] = 'S'; recvCmd.head[2] = 'M';
					memcpy(&recvCmd, cData, sizeof(recvCmd));

					if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2])
					{
						//qDebug() << "recv sn reply";
						recvCmd.nDataSize = FromBigEndian(recvCmd.nDataSize);
						if (recvCmd.nDataSize == 6)
						{
							memcpy(pData, cData + sizeof(send_pack_head_t) + 1, recvCmd.nDataSize - 1);
						}
						else
						{
							//this->uinit();
							SendDataByUSB::uinit();
							return false;
						}

					}
					else
					{
						//this->uinit();
						SendDataByUSB::uinit();
						return false;
					}
				}
				else
				{
					//this->uinit();
					SendDataByUSB::uinit();
					return false;
				}
			}
			else
			{
				//this->uinit();
				SendDataByUSB::uinit();
				return false;
			}
		}

		//this->uinit();
		SendDataByUSB::uinit();
		return true;

	}

	bool SendDataByUSB::RequestCalcFactor(int nDevID, float *arrFactor, int nTimeOut)
	{
		Calc_Factor_t m_calc_factor;

		//int nRet = this->init();
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T'; sendCmd.head[1] = 'S'; sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_CALC_FACTOR;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = 0;
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);
			uint32_t nCheckSum = crc32_bit((uint8_t*)&sendCmd + nHeadLen, 0);
			nCheckSum = FromBigEndian(nCheckSum);
			char cData[FRAME_LEN] = { 0 };
			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &nCheckSum, sizeof(uint32_t));
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(uint32_t), nTimeOut);
			if (nRet >= 0)
			{
				memset(cData, 0, FRAME_LEN);
				nRet = usb_read(nDevID, cData, FRAME_LEN, nTimeOut);
				if (nRet >= 0)
				{
					recv_pack_head_t recvCmd;
					recvCmd.head[0] = 'T'; recvCmd.head[1] = 'S'; recvCmd.head[2] = 'M';
					memcpy(&recvCmd, cData, sizeof(recvCmd));
					if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2] /* && recvCmd.cmd == USB_UPGRADE_PROGRAMMER_OK_CMD*/)
					{
						recvCmd.ready();
						int nStrtSize = sizeof(Calc_Factor_t);
						if (nStrtSize == recvCmd.nDataSize)
						{
							memcpy(&m_calc_factor, cData + sizeof(recv_pack_head_t), nStrtSize);
							arrFactor[0] = m_calc_factor.focus;
							arrFactor[1] = m_calc_factor.algo_cx;
							arrFactor[2] = m_calc_factor.algo_cy;
							arrFactor[3] = m_calc_factor.algo_baseline;
						}
						else
						{
							//this->uinit();
							SendDataByUSB::uinit();
							return false;
						}

					}
					else
					{
						//this->uinit();
						SendDataByUSB::uinit();
						return false;
					}
				}
				else
				{
					//this->uinit();
					SendDataByUSB::uinit();
					return false;
				}
			}
			else
			{
				//this->uinit();
				SendDataByUSB::uinit();
				return false;
			}
		}
		else
		{
			//this->uinit();
			SendDataByUSB::uinit();
			return false;
		}

		//this->uinit();
		SendDataByUSB::uinit();
		return true;
	}

	bool SendDataByUSB::sendImgChannel(int nDevID, bool bLeft, bool bRight, bool bDepth, bool bRGB, int nTimeOut)
	{
		//int nRet = this->init();
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T'; sendCmd.head[1] = 'S'; sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_IMAGE_CHANNEL;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = sizeof(Control_AE_Model_t);
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = { 0 };
			unsigned char nAEState = 0;

			unsigned char nLeftVal = 0;
			unsigned char nRightVal = 0;
			unsigned char nDepthVal = 0;
			unsigned char nRgbVal = 0;
			if (bLeft)
			{
				nLeftVal = 0b00000001;
			}
			if (bRight)
			{
				nRightVal = 0b00000010;
			}
			if (bDepth)
			{
				nDepthVal = 0b00000100;
			}
			if (bRGB)		//RGB
			{
				nRgbVal = 0b00001000;
			}
			tempStu.video_selected = (nLeftVal | nRightVal | nDepthVal | nRgbVal);

			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));

			uint32_t nCheckSum = crc32_bit((uint8_t*)&sendCmd + nHeadLen, sizeof(Control_AE_Model_t));
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), &nCheckSum, sizeof(uint32_t));
			//qDebug() << QByteArray(cData, nHeadLen + sizeof(uint32_t) + sizeof(Control_AE_Model_t)).toHex(' ');
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(uint32_t) + sizeof(Control_AE_Model_t), nTimeOut);
			if (nRet >= 0)
			{
				//qDebug() << "send Image Channel--" << QByteArray(cData, nHeadLen + sizeof(uint32_t) + sizeof(Control_AE_Model_t)).toHex(' ');
				//this->uinit();
				SendDataByUSB::uinit();
				return true;
			}
			else
			{
				//qDebug() << "send Image Channel failed";
			}
		}
		else
		{
			//qDebug() << "usb init failed";
		}
		//this->uinit();
		SendDataByUSB::uinit();
		return false;
	}
	

	bool SendDataByUSB::sendFrameRate(int nDevID, unsigned char nVal, int nTimeOut)
	{
		bool bReturn = false;
		int nRet = this->init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T'; sendCmd.head[1] = 'S'; sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_FRAMERATE_ADJUST;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = 1;
			sendCmd.ready();

			char cData[FRAME_LEN] = { 0 };
			int nHeadLen = sizeof(send_pack_head_t);
			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &nVal, sizeof(unsigned char));

			uint32_t nCheckSum = crc32_bit((uint8_t*)&sendCmd + nHeadLen, sizeof(unsigned char));
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(unsigned char), &nCheckSum, sizeof(uint32_t));
			int nLen = nHeadLen + sizeof(uint32_t) + sizeof(unsigned char);
			//qDebug() << QByteArray(cData, nLen).toHex(' ');
			int nRet = usb_write(nDevID, cData, nLen, nTimeOut);

			if (nRet >= 0)
			{
				//qDebug() << QTime::currentTime().toString("hhmmss--") << "send frame rate ok";
				this->uinit();
				return true;
			}
		}
		this->uinit();
		return bReturn;
	}


	bool SendDataByUSB::sendCKVersion(int nDevID, unsigned char *pData, int nTimeOut)
	{
		bool bRead = true;
		bool bReturn = false;
		int nRet = this->init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T'; sendCmd.head[1] = 'S'; sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_OBTAIN_SOFTVERSION;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = sizeof(CK_Version_t);
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			char cData[FRAME_LEN] = { 0 };
			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, pData, sizeof(CK_Version_t));

			uint32_t nCheckSum = crc32_bit((uint8_t*)&sendCmd + nHeadLen, sizeof(CK_Version_t));
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(CK_Version_t), &nCheckSum, sizeof(uint32_t));
			//qDebug() << QByteArray(cData, nHeadLen + sizeof(uint32_t) + sizeof(CK_Version_t)).toHex(' ');
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(uint32_t) + sizeof(CK_Version_t), nTimeOut);
			if (nRet >= 0)
			{
				//qDebug() << "send CK VERSION CMD";
				bReturn = true;
				if (bRead)
				{
					bReturn = false;
					memset(cData, 0, FRAME_LEN);
					nRet = usb_read(nDevID, cData, FRAME_LEN, nTimeOut);
					if (nRet >= 0)
					{
						send_pack_head_t recvCmd;
						memcpy(&recvCmd, cData, sizeof(recvCmd));

						if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2])
						{
							/*qDebug() << "recv CK VERSION reply";
							qDebug() << QByteArray(cData, 30).toHex(' ');*/
							recvCmd.nDataSize = FromBigEndian(recvCmd.nDataSize);
							if (recvCmd.nDataSize == sizeof(CK_Version_t))
							{
								memcpy(pData, cData + sizeof(send_pack_head_t), recvCmd.nDataSize);
								//qDebug() << "recv CK VERSION reply ok";
								bReturn = true;
							}
						}
					}
				}
			}
		}
		this->uinit();
		return bReturn;
	}

	bool SendDataByUSB::sendRgbRatioCmd(int nDevID, unsigned char nType, int nTimeOut)
	{
		bool bReturn = false;
		int nRet = this->init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T'; sendCmd.head[1] = 'S'; sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_RGB_RESOLUTION;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = 1;
			sendCmd.ready();

			char cData[FRAME_LEN] = { 0 };
			int nHeadLen = sizeof(send_pack_head_t);
			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &nType, sizeof(unsigned char));

			uint32_t nCheckSum = crc32_bit((uint8_t*)&sendCmd + nHeadLen, sizeof(unsigned char));
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(unsigned char), &nCheckSum, sizeof(uint32_t));
			int nLen = nHeadLen + sizeof(uint32_t) + sizeof(unsigned char);
			//qDebug() << QByteArray(cData, nLen).toHex(' ');
			int nRet = usb_write(nDevID, cData, nLen, nTimeOut);

			if (nRet >= 0)
			{
				//qDebug() << QTime::currentTime().toString("hhmmss--") << "send RGB solution ok";
				bReturn = true;
			}
		}
		this->uinit();
		return bReturn;
	}


	bool SendDataByUSB::ReadIRIRCalibrationParam(int nDevID, IR_IR_Calib_Para_t *arrIR_IR_Factor, int nLen, int nTimeOut)
	{
		bool bReturn = false;
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T'; sendCmd.head[1] = 'S'; sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_RW_CALIBRATION_PARAMS_IR_IR;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = sizeof(IR_IR_Calib_Para_t);
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = { 0 };

			//读参数
			tempStu.rd_wr_sn = 0;

			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), arrIR_IR_Factor, nLen);

			uint32_t nCheckSum = crc32_bit((uint8_t*)&sendCmd + nHeadLen, nLen);
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t) + nLen, &nCheckSum, sizeof(uint32_t));
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(Control_AE_Model_t) + nLen + sizeof(uint32_t), nTimeOut);

			if (nRet >= 0)
			{
				memset(cData, 0, FRAME_LEN);
				nRet = usb_read(nDevID, cData, FRAME_LEN, nTimeOut);
				if (nRet >= 0)
				{
					recv_pack_head_t recvCmd;
					recvCmd.head[0] = 'T'; recvCmd.head[1] = 'S'; recvCmd.head[2] = 'M';
					memcpy(&recvCmd, cData, sizeof(recvCmd));

					if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2])
					{
						recvCmd.nDataSize = FromBigEndian(recvCmd.nDataSize);

						if (IR_IR_CALIB_PARA_SIZE == recvCmd.nDataSize)
						{
							memcpy(arrIR_IR_Factor, cData + sizeof(recv_pack_head_t), IR_IR_CALIB_PARA_SIZE);
							bReturn = true;
						}
						else
						{
							bReturn = false;
						}
					}
					else
					{
						bReturn = false;
					}
				}
				else
				{
					bReturn = false;
				}
			}
			else
			{
				bReturn = false;
			}
		}
		else
		{
			bReturn = false;
		}

		SendDataByUSB::uinit();
		return bReturn;
	}


	bool SendDataByUSB::ReadIRRGBCalibrationParam(int nDevID, IR_RGB_Calib_Para_t *arrIR_RGB_Factor, int nLen, int nTimeOut)
	{
		bool bReturn = false;
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T'; sendCmd.head[1] = 'S'; sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_RW_CALIBRATION_PARAMS_IR_RGB;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = sizeof(IR_RGB_Calib_Para_t);
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = { 0 };

			//读参数
			tempStu.rd_wr_sn = 0;

			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), arrIR_RGB_Factor, nLen);

			uint32_t nCheckSum = crc32_bit((uint8_t*)&sendCmd + nHeadLen, nLen);
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t) + nLen, &nCheckSum, sizeof(uint32_t));
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(Control_AE_Model_t) + nLen + sizeof(uint32_t), nTimeOut);

			if (nRet >= 0)
			{
				memset(cData, 0, FRAME_LEN);
				nRet = usb_read(nDevID, cData, FRAME_LEN, nTimeOut);
				if (nRet >= 0)
				{
					recv_pack_head_t recvCmd;
					recvCmd.head[0] = 'T'; recvCmd.head[1] = 'S'; recvCmd.head[2] = 'M';
					memcpy(&recvCmd, cData, sizeof(recvCmd));

					if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2])
					{
						recvCmd.nDataSize = FromBigEndian(recvCmd.nDataSize);

						if (IR_RGB_CALIB_PARA_SIZE == recvCmd.nDataSize)
						{
							memcpy(arrIR_RGB_Factor, cData + sizeof(recv_pack_head_t), IR_RGB_CALIB_PARA_SIZE);
							bReturn = true;
						}
						else
						{
							bReturn = false;
						}
					}
					else
					{
						bReturn = false;
					}
				}
				else
				{
					bReturn = false;
				}
			}
			else
			{
				bReturn = false;
			}
		}
		else
		{
			bReturn = false;
		}


		SendDataByUSB::uinit();
		return bReturn;
	}

	bool SendDataByUSB::Get_ISP_Exposure(int nDevID, char *pData, int nLen, float *fExposure, float *fGain, int *nState, int nTimeOut)
	{
		float fArr_Temp[4] = {0}; //[0]item num; [1]state; [2]exposure value; [3]gain value;
		bool bReturn = false;
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T';
			sendCmd.head[1] = 'S';
			sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_ISP_EXPOSURE_CONTROL;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = 0;
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = {0};

			// 读参数
			tempStu.rd_wr_sn = 0;

			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), pData, nLen);

			uint32_t nCheckSum = crc32_bit((uint8_t *)&sendCmd + nHeadLen, nLen);
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t) + nLen, &nCheckSum, sizeof(uint32_t));
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(Control_AE_Model_t) + nLen + sizeof(uint32_t), nTimeOut);

			if (nRet >= 0)
			{
				memset(cData, 0, FRAME_LEN);
				nRet = usb_read(nDevID, cData, FRAME_LEN, nTimeOut);
				if (nRet >= 0)
				{
					recv_pack_head_t recvCmd;
					recvCmd.head[0] = 'T';
					recvCmd.head[1] = 'S';
					recvCmd.head[2] = 'M';
					memcpy(&recvCmd, cData, sizeof(recvCmd));

					// FILE* fp_1 = fopen("Get_ISP_Exposure_1.txt", "w+");
					// for (int i = 0; i < FRAME_LEN; i++)
					// {
					// 	fprintf(fp_1, "%d\n", (uint8_t)cData[i]);
					// }
					// fprintf(fp_1, "recvCmd.nDataSize:%d\n", recvCmd.nDataSize);
					// fclose(fp_1);

					if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2])
					{
						recvCmd.nDataSize = FromBigEndian(recvCmd.nDataSize);

						memcpy(fArr_Temp, cData + sizeof(recv_pack_head_t), sizeof(fArr_Temp));
						*nState = (int)fArr_Temp[1];
						*fGain = fArr_Temp[2];
						*fExposure = fArr_Temp[3];

						// FILE* fp_2 = fopen("Get_ISP_Exposure_2.txt", "w+");
						// for (int i = 0; i < FRAME_LEN; i++)
						// {
						// 	fprintf(fp_2, "%d\n", (uint8_t)cData[i]);
						// }
						// fprintf(fp_2, "recvCmd.nDataSize:%d\n", recvCmd.nDataSize);
						// fprintf(fp_2, "nState:%d; fGain:%f; fExposure:%f\n", recvCmd.nDataSize, *fGain, *fExposure);
						// fclose(fp_2);

						bReturn = true;
					}
					else
					{
						bReturn = false;
					}
				}
				else
				{
					bReturn = false;
				}
			}
			else
			{
				bReturn = false;
			}
		}
		else
		{
			bReturn = false;
		}

		SendDataByUSB::uinit();
		return bReturn;
	}

	bool SendDataByUSB::Set_ISP_Exposure(int nDevID, char *pData, int nLen, int nTimeOut)
	{
		bool bReturn = false;
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T';
			sendCmd.head[1] = 'S';
			sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_ISP_EXPOSURE_CONTROL;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = 0;
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = {0};

			// 写参数
			tempStu.rd_wr_sn = 1;

			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), pData, nLen);

			uint32_t nCheckSum = crc32_bit((uint8_t *)&sendCmd + nHeadLen, nLen);
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t) + nLen, &nCheckSum, sizeof(uint32_t));
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(Control_AE_Model_t) + nLen + sizeof(uint32_t), nTimeOut);

			if (nRet >= 0)
			{
				bReturn = true;
			}
			else
			{
				bReturn = false;
			}
		}
		else
		{
			bReturn = false;
		}
		SendDataByUSB::uinit();
		return bReturn;
	}

	bool SendDataByUSB::Get_ISP_WB(int nDevID, char *pData, int nLen, float *wb_R_Gain, float *wb_Gr_Gain, float *wb_Gb_Gain, float *wb_B_Gain, int *nState, int nTimeOut)
	{
		float fArr_Temp[6] = {0}; //[0]item num; [1]state; [2]wb_R_Gain; [3]wb_Gr_Gain; [4]wb_Gb_Gain; [5]wb_B_Gain;
		bool bReturn = false;
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T';
			sendCmd.head[1] = 'S';
			sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_ISP_WB_CONTROL;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = 0;
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = {0};

			// 读参数
			tempStu.rd_wr_sn = 0;

			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), pData, nLen);

			uint32_t nCheckSum = crc32_bit((uint8_t *)&sendCmd + nHeadLen, nLen);
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t) + nLen, &nCheckSum, sizeof(uint32_t));
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(Control_AE_Model_t) + nLen + sizeof(uint32_t), nTimeOut);

			if (nRet >= 0)
			{
				memset(cData, 0, FRAME_LEN);
				nRet = usb_read(nDevID, cData, FRAME_LEN, nTimeOut);
				if (nRet >= 0)
				{
					recv_pack_head_t recvCmd;
					recvCmd.head[0] = 'T';
					recvCmd.head[1] = 'S';
					recvCmd.head[2] = 'M';
					memcpy(&recvCmd, cData, sizeof(recvCmd));

					// FILE* fp_1 = fopen("Get_ISP_WB_1.txt", "w+");
					// for (int i = 0; i < FRAME_LEN; i++)
					// {
					// 	fprintf(fp_1, "%d\n", (uint8_t)cData[i]);
					// }
					// fprintf(fp_1, "recvCmd.nDataSize:%d\n", recvCmd.nDataSize);
					// fclose(fp_1);

					if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2])
					{
						recvCmd.nDataSize = FromBigEndian(recvCmd.nDataSize);

						memcpy(fArr_Temp, cData + sizeof(recv_pack_head_t), sizeof(fArr_Temp));
						*nState = (int)fArr_Temp[1];
						*wb_R_Gain = fArr_Temp[2];
						*wb_Gr_Gain = fArr_Temp[3];
						*wb_Gb_Gain = fArr_Temp[4];
						*wb_B_Gain = fArr_Temp[5];

						// FILE* fp_2 = fopen("Get_ISP_WB_2.txt", "w+");
						// for (int i = 0; i < FRAME_LEN; i++)
						// {
						// 	fprintf(fp_2, "%d\n", (uint8_t)cData[i]);
						// }
						// fprintf(fp_2, "recvCmd.nDataSize:%d\n", recvCmd.nDataSize);
						// fprintf(fp_2, " nArr_Temp[0]: %f\n", fArr_Temp[0]);
						// fprintf(fp_2, "nState: %f\n", fArr_Temp[1]);
						// fprintf(fp_2, "wb_R_Gain: %f, %f\n", fArr_Temp[2], *wb_R_Gain);
						// fprintf(fp_2, "wb_Gr_Gain: %f, %f\n", fArr_Temp[3], *wb_Gr_Gain);
						// fprintf(fp_2, "wb_Gb_Gain: %f, %f\n", fArr_Temp[4], *wb_Gb_Gain);
						// fprintf(fp_2, "wb_B_Gain: %f, %f\n", fArr_Temp[5], *wb_B_Gain);
						// fclose(fp_2);

						bReturn = true;
					}
					else
					{
						bReturn = false;
					}
				}
				else
				{
					bReturn = false;
				}
			}
			else
			{
				bReturn = false;
			}
		}
		else
		{
			bReturn = false;
		}

		SendDataByUSB::uinit();
		return bReturn;
	}

	bool SendDataByUSB::Set_ISP_WB(int nDevID, char *pData, int nLen, int nTimeOut)
	{
		bool bReturn = false;
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T';
			sendCmd.head[1] = 'S';
			sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_ISP_WB_CONTROL;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = 0;
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = {0};

			// 写参数
			tempStu.rd_wr_sn = 1;

			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), pData, nLen);

			uint32_t nCheckSum = crc32_bit((uint8_t *)&sendCmd + nHeadLen, nLen);
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t) + nLen, &nCheckSum, sizeof(uint32_t));
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(Control_AE_Model_t) + nLen + sizeof(uint32_t), nTimeOut);

			if (nRet >= 0)
			{
				bReturn = true;
			}
			else
			{
				bReturn = false;
			}
		}
		else
		{
			bReturn = false;
		}
		SendDataByUSB::uinit();
		return bReturn;
	}

	bool SendDataByUSB::Get_ISP_Cproc(int nDevID, char *pData, int nLen, int *nBrightness, float *nSaturation, float *nHue, float *nContrast, int *nState, int nTimeOut)
	{
		float fArr_Temp[6] = {0}; //[0]item num; [1]nState; [2]fBrightness; [3]fSaturation; [4]fHue; [5]fContrast
		bool bReturn = false;
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T';
			sendCmd.head[1] = 'S';
			sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_ISP_CPROC_CONTROL;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = 0;
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = {0};

			// 读参数
			tempStu.rd_wr_sn = 0;

			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), pData, nLen);

			uint32_t nCheckSum = crc32_bit((uint8_t *)&sendCmd + nHeadLen, nLen);
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t) + nLen, &nCheckSum, sizeof(uint32_t));
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(Control_AE_Model_t) + nLen + sizeof(uint32_t), nTimeOut);

			if (nRet >= 0)
			{
				memset(cData, 0, FRAME_LEN);
				nRet = usb_read(nDevID, cData, FRAME_LEN, nTimeOut);
				if (nRet >= 0)
				{
					recv_pack_head_t recvCmd;
					recvCmd.head[0] = 'T';
					recvCmd.head[1] = 'S';
					recvCmd.head[2] = 'M';
					memcpy(&recvCmd, cData, sizeof(recvCmd));

					// FILE* fp_1 = fopen("Get_ISP_Cproc_1.txt", "w+");
					// for (int i = 0; i < FRAME_LEN; i++)
					// {
					// 	fprintf(fp_1, "%d\n", (uint8_t)cData[i]);
					// }
					// fprintf(fp_1, "recvCmd.nDataSize:%d\n", recvCmd.nDataSize);
					// fclose(fp_1);

					if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2])
					{
						recvCmd.nDataSize = FromBigEndian(recvCmd.nDataSize);

						memcpy(fArr_Temp, cData + sizeof(recv_pack_head_t), sizeof(fArr_Temp));
						*nState = (int)fArr_Temp[1];
						*nBrightness = (int)fArr_Temp[2];
						*nSaturation = fArr_Temp[3];
						*nHue = fArr_Temp[4];
						*nContrast = fArr_Temp[5];

						// FILE* fp_2 = fopen("Get_ISP_Cproc_2.txt", "w+");
						// for (int i = 0; i < FRAME_LEN; i++)
						// {
						// 	fprintf(fp_2, "%d\n", (uint8_t)cData[i]);
						// }
						// fprintf(fp_2, "recvCmd.nDataSize:%d\n", recvCmd.nDataSize);
						// fprintf(fp_2, " nArr_Temp[0]: %f\n", fArr_Temp[0]);
						// fprintf(fp_2, "nState: %f\n", fArr_Temp[1]);
						// fprintf(fp_2, "nBrightness: %f, %d\n", fArr_Temp[2], *nBrightness);
						// fprintf(fp_2, "nSaturation: %f, %f\n", fArr_Temp[3], *nSaturation);
						// fprintf(fp_2, "nHue: %f, %f\n", fArr_Temp[4], *nHue);
						// fprintf(fp_2, "nContrast: %f, %f\n", fArr_Temp[5], *nContrast);
						// fclose(fp_2);

						bReturn = true;
					}
					else
					{
						bReturn = false;
					}
				}
				else
				{
					bReturn = false;
				}
			}
			else
			{
				bReturn = false;
			}
		}
		else
		{
			bReturn = false;
		}

		SendDataByUSB::uinit();
		return bReturn;
	}

	bool SendDataByUSB::Set_ISP_Cproc(int nDevID, char *pData, int nLen, int nTimeOut)
	{
		bool bReturn = false;
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T';
			sendCmd.head[1] = 'S';
			sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_ISP_CPROC_CONTROL;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = 0;
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = {0};

			// 写参数
			tempStu.rd_wr_sn = 1;

			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), pData, nLen);

			uint32_t nCheckSum = crc32_bit((uint8_t *)&sendCmd + nHeadLen, nLen);
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t) + nLen, &nCheckSum, sizeof(uint32_t));
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(Control_AE_Model_t) + nLen + sizeof(uint32_t), nTimeOut);

			if (nRet >= 0)
			{
				bReturn = true;
			}
			else
			{
				bReturn = false;
			}
		}
		else
		{
			bReturn = false;
		}
		SendDataByUSB::uinit();
		return bReturn;
	}

	bool SendDataByUSB::Get_ISP_Sharpness(int nDevID, char *pData, int nLen, int *nDenoise_Level, int *nSharpen_Level, int *nState, int nTimeOut)
	{
		float fArr_Temp[4] = {0}; //[0]item num; [1]nState; [2]nDenoise_Level; [3]nSharpen_Level
		bool bReturn = false;
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T';
			sendCmd.head[1] = 'S';
			sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_ISP_SHARPNESS_CONTROL;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = 0;
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = {0};

			// 读参数
			tempStu.rd_wr_sn = 0;

			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), pData, nLen);

			uint32_t nCheckSum = crc32_bit((uint8_t *)&sendCmd + nHeadLen, nLen);
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t) + nLen, &nCheckSum, sizeof(uint32_t));
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(Control_AE_Model_t) + nLen + sizeof(uint32_t), nTimeOut);

			if (nRet >= 0)
			{
				memset(cData, 0, FRAME_LEN);
				nRet = usb_read(nDevID, cData, FRAME_LEN, nTimeOut);
				if (nRet >= 0)
				{
					recv_pack_head_t recvCmd;
					recvCmd.head[0] = 'T';
					recvCmd.head[1] = 'S';
					recvCmd.head[2] = 'M';
					memcpy(&recvCmd, cData, sizeof(recvCmd));

					// FILE* fp_1 = fopen("Get_ISP_Sharpness_1.txt", "w+");
					// for (int i = 0; i < FRAME_LEN; i++)
					// {
					// 	fprintf(fp_1, "%d\n", (uint8_t)cData[i]);
					// }
					// fprintf(fp_1, "recvCmd.nDataSize:%d\n", recvCmd.nDataSize);
					// fclose(fp_1);

					if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2])
					{
						recvCmd.nDataSize = FromBigEndian(recvCmd.nDataSize);

						memcpy(fArr_Temp, cData + sizeof(recv_pack_head_t), sizeof(fArr_Temp));
						*nState = (int)fArr_Temp[1];
						*nDenoise_Level = (int)fArr_Temp[2];
						*nSharpen_Level = (int)fArr_Temp[3];

						// FILE* fp_2 = fopen("Get_ISP_Sharpness_2.txt", "w+");
						// for (int i = 0; i < FRAME_LEN; i++)
						// {
						// 	fprintf(fp_2, "%d\n", (uint8_t)cData[i]);
						// }
						// fprintf(fp_2, "recvCmd.nDataSize:%d\n", recvCmd.nDataSize);
						// fprintf(fp_2, "nState:%d\n", *nState);
						// fprintf(fp_2, "nDenoise_Level:%d(%f)\n", *nDenoise_Level, fArr_Temp[2]);
						// fprintf(fp_2, "nSharpen_Level:%d(%f)\n", *nSharpen_Level, fArr_Temp[3]);
						// fclose(fp_2);

						bReturn = true;
					}
					else
					{
						bReturn = false;
					}
				}
				else
				{
					bReturn = false;
				}
			}
			else
			{
				bReturn = false;
			}
		}
		else
		{
			bReturn = false;
		}

		SendDataByUSB::uinit();
		return bReturn;
	}

	bool SendDataByUSB::Set_ISP_Sharpness(int nDevID, char *pData, int nLen, int nTimeOut)
	{
		bool bReturn = false;
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T';
			sendCmd.head[1] = 'S';
			sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_ISP_SHARPNESS_CONTROL;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = 0;
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = {0};

			// 写参数
			tempStu.rd_wr_sn = 1;

			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), pData, nLen);

			uint32_t nCheckSum = crc32_bit((uint8_t *)&sendCmd + nHeadLen, nLen);
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t) + nLen, &nCheckSum, sizeof(uint32_t));
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(Control_AE_Model_t) + nLen + sizeof(uint32_t), nTimeOut);

			if (nRet >= 0)
			{
				bReturn = true;
			}
			else
			{
				bReturn = false;
			}
		}
		else
		{
			bReturn = false;
		}
		SendDataByUSB::uinit();
		return bReturn;
	}

	bool SendDataByUSB::Get_ISP_Gamma(int nDevID, char *pData, int nLen, int *nState, int nTimeOut)
	{
		float fArr_Temp[2] = {0}; //[0]item num; [1]nState
		bool bReturn = false;
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T';
			sendCmd.head[1] = 'S';
			sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_ISP_GAMMA_CONTROL;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = 0;
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = {0};

			// 读参数
			tempStu.rd_wr_sn = 0;

			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), pData, nLen);

			uint32_t nCheckSum = crc32_bit((uint8_t *)&sendCmd + nHeadLen, nLen);
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t) + nLen, &nCheckSum, sizeof(uint32_t));
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(Control_AE_Model_t) + nLen + sizeof(uint32_t), nTimeOut);

			if (nRet >= 0)
			{
				memset(cData, 0, FRAME_LEN);
				nRet = usb_read(nDevID, cData, FRAME_LEN, nTimeOut);
				if (nRet >= 0)
				{
					recv_pack_head_t recvCmd;
					recvCmd.head[0] = 'T';
					recvCmd.head[1] = 'S';
					recvCmd.head[2] = 'M';
					memcpy(&recvCmd, cData, sizeof(recvCmd));

					// FILE* fp_1 = fopen("Get_ISP_Gamma_1.txt", "w+");
					// for (int i = 0; i < FRAME_LEN; i++)
					// {
					// 	fprintf(fp_1, "%d\n", (uint8_t)cData[i]);
					// }
					// fprintf(fp_1, "recvCmd.nDataSize:%d\n", recvCmd.nDataSize);
					// fclose(fp_1);

					if (recvCmd.head[0] == MSG_TOP[0] && recvCmd.head[1] == MSG_TOP[1] && recvCmd.head[2] == MSG_TOP[2])
					{
						recvCmd.nDataSize = FromBigEndian(recvCmd.nDataSize);

						memcpy(fArr_Temp, cData + sizeof(recv_pack_head_t), sizeof(fArr_Temp));
						*nState = (int)fArr_Temp[1];

						// FILE* fp_2 = fopen("Get_ISP_Gamma_2.txt", "w+");
						// for (int i = 0; i < FRAME_LEN; i++)
						// {
						// 	fprintf(fp_2, "%d\n", (uint8_t)cData[i]);
						// }
						// fprintf(fp_2, "recvCmd.nDataSize:%d\n", recvCmd.nDataSize);
						// fprintf(fp_2, "nState:%d\n", *nState);
						// fclose(fp_2);

						bReturn = true;
					}
					else
					{
						bReturn = false;
					}
				}
				else
				{
					bReturn = false;
				}
			}
			else
			{
				bReturn = false;
			}
		}
		else
		{
			bReturn = false;
		}

		SendDataByUSB::uinit();
		return bReturn;
	}

	bool SendDataByUSB::Set_ISP_Gamma(int nDevID, char *pData, int nLen, int nTimeOut)
	{
		bool bReturn = false;
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T';
			sendCmd.head[1] = 'S';
			sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_ISP_GAMMA_CONTROL;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = 0;
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = {0};

			// 写参数
			tempStu.rd_wr_sn = 1;

			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), pData, nLen);

			uint32_t nCheckSum = crc32_bit((uint8_t *)&sendCmd + nHeadLen, nLen);
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t) + nLen, &nCheckSum, sizeof(uint32_t));
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(Control_AE_Model_t) + nLen + sizeof(uint32_t), nTimeOut);

			if (nRet >= 0)
			{
				bReturn = true;
			}
			else
			{
				bReturn = false;
			}
		}
		else
		{
			bReturn = false;
		}
		SendDataByUSB::uinit();
		return bReturn;
	}

	
	bool SendDataByUSB::Set_USB_PacketLen(int nDevID, int nTotalDevNum, int nTimeOut)
	{
		//int nRet = this->init();
		int nRet = SendDataByUSB::init();
		if (nRet == SUCCESS)
		{
			send_pack_head_t sendCmd;
			sendCmd.head[0] = 'T'; sendCmd.head[1] = 'S'; sendCmd.head[2] = 'M';
			sendCmd.cmd = USB_PACKET_LEN;
			sendCmd.nAddr = 0;
			sendCmd.nDataSize = sizeof(Control_AE_Model_t);
			sendCmd.ready();
			int nHeadLen = sizeof(send_pack_head_t);

			Control_AE_Model_t tempStu;
			char cData[FRAME_LEN] = { 0 };
			unsigned char nAEState = 0;

			tempStu.video_selected = nTotalDevNum;

			memcpy(cData, &sendCmd, nHeadLen);
			memcpy(cData + nHeadLen, &tempStu, sizeof(Control_AE_Model_t));

			uint32_t nCheckSum = crc32_bit((uint8_t*)&sendCmd + nHeadLen, sizeof(Control_AE_Model_t));
			nCheckSum = FromBigEndian(nCheckSum);
			memcpy(cData + nHeadLen + sizeof(Control_AE_Model_t), &nCheckSum, sizeof(uint32_t));
			int nRet = usb_write(nDevID, cData, nHeadLen + sizeof(uint32_t) + sizeof(Control_AE_Model_t), nTimeOut);
			if (nRet >= 0)
			{
				SendDataByUSB::uinit();
				return true;
			}
			else
			{
				//qDebug() << "send Image Channel failed";
			}
		}
		else
		{
			//qDebug() << "usb init failed";
		}
		SendDataByUSB::uinit();
		return false;
	}
	SendDataByUSB::~SendDataByUSB() {
		usb_exit();
	}

}


