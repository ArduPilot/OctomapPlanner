#ifndef OPTORUSB_H
#define OPTORUSB_H

typedef struct libusb_device	cyusb_device;
typedef struct libusb_device_handle	cyusb_handle;

int cyusb_open(void);
cyusb_handle * cyusb_gethandle(int index);
int cyusb_control_transfer(cyusb_handle *h, unsigned char bmRequestType, unsigned char bRequest,
                           unsigned short wValue, unsigned short wIndex, unsigned char *data, unsigned short wLength,
                           unsigned int timeout);
int cyusb_bulk_transfer(cyusb_handle *h, unsigned char endpoint, unsigned char *data, int length,
                        int *transferred, int timeout);
void cyusb_close(void);

#endif
