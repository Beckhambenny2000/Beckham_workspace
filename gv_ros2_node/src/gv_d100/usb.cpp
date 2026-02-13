//#include "stdafx.h"
#include "gv_d100/usb.h"

#include "stdio.h"
#include <iostream>
#include <vector>
#include <map>

std::map<uint8_t, struct libusb_device_handle*> usbs;
static struct libusb_context * context =0 ;

static uint32_t Table2[256];

uint32_t reverse_table_crc(uint8_t *data, int32_t len, uint32_t *table)
{
	uint32_t crc = 0xffffffff;
	uint8_t *p = data;
	int i;
	for (i = 0; i < len; i++) {
		crc = table[(crc ^ (*(p + i))) & 0xff] ^ (crc >> 8);
	}
	return  ~crc;
}

uint64_t reflect(uint64_t ref, uint8_t ch)
{
	int i;
	uint64_t value = 0;
	for (i = 1; i < (ch + 1); i++) {
		if (ref & 1) {
			value |= 1 << (ch - i);
		}
		ref >>= 1;
	}
	return value;
}

void gen_normal_table(uint32_t *table)
{
	uint32_t gx = 0x04c11db7;
	uint32_t temp;
	int i, j;
	for (i = 0; i <= 0xFF; i++) {
		temp = reflect(i, 8);
		table[i] = temp << 24;
		for (j = 0; j < 8; j++) {
			unsigned long int t1, t2;
			unsigned long int flag = table[i] & 0x80000000;
			t1 = (table[i] << 1);
			if (flag == 0) {
				t2 = 0;
			}
			else {
				t2 = gx;
			}
			table[i] = t1 ^ t2;
		}
		table[i] = reflect(table[i], 32);
	}
}

uint32_t crc32_bit(uint8_t *ptr, uint32_t len)
{
	uint32_t crc;
	static uint8_t init_table = 0;
	if (init_table == 0) {
		init_table = 1;
		gen_normal_table(Table2);
	}
	crc = reverse_table_crc(ptr, len, Table2);
	return crc;
}

uint32_t crc32_bit_ext(uint32_t &v)
{
    unsigned table=0x03480;
    unsigned roll=v;
    for(int i=0;i<16;++i){

        roll<<=1;
    }

}

static void usb_close(libusb_device_handle * dev_handle )
{
    libusb_close(dev_handle);
    dev_handle=NULL;
    libusb_exit(context);
}

static int usb_config(libusb_device_handle *devh)
{
	// printf("usb_config\n");
    int err = 0;// libusb_set_auto_detach_kernel_driver (usb, 1);
    if(err!=0){
        std::cout<<"faild auto detach\n";
        return -1;
    }
    int curconfig=0;

    err = libusb_get_configuration(devh,&curconfig);
    if(err!=0){
        std::cout<<"failed get config \n";
        return -1;
    }

    if(curconfig!=1)
     err = libusb_set_configuration(devh,1);

     if(err!=0){
         std::cout<<"failed to set config\n";
         return -1;
     }

     err = libusb_claim_interface (devh, MY_INT);
     if(err!=0){
         std::cout<<"failed claim\n"<<std::endl;
         return -1;
     }

	return SUCCESS;
}

int usb_initial(void)
{
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

      if (kk.idVendor == IDVendor && kk.idProduct == IDProduct ) {
        struct libusb_device_handle* dev_handle;
        libusb_open(dev, &dev_handle);
        if (dev_handle == NULL){
          printf("error!\n");
          continue;
        }
		usbs.insert({libusb_get_device_address(dev), dev_handle});
        // libusb_reset_device(dev_handle);
      }
      idx ++;
    }

	if (usbs.size() == 0)
		return SUCCESS;

	for (auto &dev : usbs) {
		if (usb_config(dev.second) < 0) {
			usb_close(dev.second);
			return ECONFIG;
		}
	}
	return SUCCESS;
}

int usb_write(char *dat, int size, int timeout)
{
    int ret = 0;
    int tsfed=0;

	if (usbs.size() > 0)
	{
		if (!dat)
			return EPARAM;
		for (auto &dev : usbs) {
			ret = libusb_bulk_transfer(dev.second, EP_OUT,(unsigned char *) dat, size, &tsfed,timeout);  //·¢ËÍ¶Ë¿ÚÊÇEP_OUT  0x02
			if (ret < 0 || tsfed != size)
			return EWRITE;
		}
	}
	return ret;
}

int usb_read(char *dat, int size, int timeout)
{
	int ret = 0;
    int tsfed = 0;

	if(usbs.size() > 0) {
		if (!dat)
			return EPARAM;
		for (auto &dev : usbs) {
			ret = libusb_bulk_transfer(dev.second, EP_IN, (unsigned char *)dat, size, &tsfed, timeout*2);  //·¢ËÍ¶Ë¿ÚÊÇEP_OUT  0x02
			if (ret < 0)
			{
				if(ret==LIBUSB_ERROR_TIMEOUT) std::cout<<"read timeout \n";
				return EREAD;
			}
			else
				return ret;
		}
	}
}

int usb_release()
{
	if (usbs.size() > 0 ){
		for (auto &dev : usbs) {
			if (dev.second != NULL) {
				libusb_release_interface(dev.second, MY_INT);
				if (dev.second != NULL) {
					libusb_close(dev.second);
				}
			}
		}
	}
	usbs.clear();
	return SUCCESS;
}

int usb_release2(void) {
	if (usbs.size() > 0 ){
		for (auto &dev : usbs) {
			if (dev.second != NULL) {
				libusb_release_interface(dev.second, MY_INT);
				if (dev.second != NULL) {
					usb_close(dev.second);
					dev.second = NULL;
				}
			}
		}
	}
	usbs.clear();
	return SUCCESS;
}

int usb_write(uint8_t usb_address_id, char *dat, int size, int timeout) {
	int ret = 0;
    int tsfed=0;

	if (usbs.size() > 0) {
		if (!dat)
			return EPARAM;
		
		ret = libusb_bulk_transfer(usbs[usb_address_id], EP_OUT,(unsigned char *) dat, size, &tsfed,timeout);  //·¢ËÍ¶Ë¿ÚÊÇEP_OUT  0x02
		if (ret < 0 || tsfed != size)
			return EWRITE;
	}
	return ret;
}

int usb_read(uint8_t usb_address_id, char *dat, int size, int timeout) {
	int ret = 0;
    int tsfed = 0;

	if(usbs.size() > 0) {
		if (!dat)
			return EPARAM;
		ret = libusb_bulk_transfer(usbs[usb_address_id], EP_IN, (unsigned char *)dat, size, &tsfed, timeout*2);  //·¢ËÍ¶Ë¿ÚÊÇEP_OUT  0x02
		if (ret < 0)
		{
			if(ret==LIBUSB_ERROR_TIMEOUT) std::cout<<"read timeout \n";
			return EREAD;
		}
		else
			return ret;
	}
}

void usb_exit() {
	libusb_exit(context);
}
