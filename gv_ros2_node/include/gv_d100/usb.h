#pragma once

#include "def.h"
#include <cstdint>

#include <libusb-1.0/libusb.h>
#define IDVendor        0x04CC
#define IDProduct       0x0017
#define EP_OUT          0x02
#define EP_IN           0x82
#define MY_CONFIG		0x01
#define MY_INT          0x02

uint32_t reverse_table_crc(uint8_t *data, int32_t len, uint32_t *table);
uint64_t reflect(uint64_t ref, uint8_t ch);
void gen_normal_table(uint32_t *table);
uint32_t crc32_bit(uint8_t *ptr, uint32_t len);

int usb_initial(void);
int usb_write(char *dat, int size, int timeout);
int usb_read(char *dat, int size, int timeout);
int usb_release(void);
int usb_release2(void);

int usb_write(uint8_t usb_address_id, char *dat, int size, int timeout);
int usb_read(uint8_t usb_address_id, char *dat, int size, int timeout);
void usb_exit();
