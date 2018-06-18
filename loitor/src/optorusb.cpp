#include <stdio.h>

#include <libusb-1.0/libusb.h>

#include "loitor/optorusb.h"

/* This is the maximum number of 'devices of interest' we are willing to store as default. */
/* These are the maximum number of devices we will communicate with simultaneously */
#define MAXDEVICES        10

/* This is the maximum number of VID/PID pairs that this library will consider. This limits
   the number of valid VID/PID entries in the configuration file.
 */
#define MAX_ID_PAIRS    100

/* This is the maximum length for the description string for a device in the configuration
   file. If the actual string in the file is longer, only the first MAX_STR_LEN characters
   will be considered.
 */
#define MAX_STR_LEN     30

/* Maximum length of a string read from the Configuration file (/etc/cyusb.conf) for the library. */
#define MAX_CFG_LINE_LENGTH                     (120)


struct cydev
{
    cyusb_device *dev;
    cyusb_handle *handle;
    unsigned short vid;
    unsigned short pid;         /* Product ID */
    unsigned char is_open;      /* When device is opened, val = 1 */
    unsigned char busnum;       /* The bus number of this device */
    unsigned char devaddr;      /* The device address*/
    unsigned char filler;       /* Padding to make struct = 16 bytes */
};

static struct cydev cydev[MAXDEVICES];
static int number_of_cameras;		/* Number of Interesting Devices	*/

struct VPD
{
    unsigned short vid;
    unsigned short pid;
    char desc[MAX_STR_LEN];
};

static struct VPD vpd[MAX_ID_PAIRS];

static libusb_device **libusb_device_list;
static unsigned int checksum = 0;

char pidfile[256];
char logfile[256];
int logfd;
int pidfd;

static int device_is_of_interest(cyusb_device *d)
{
    int found = 0;
    struct libusb_device_descriptor desc;
    libusb_get_device_descriptor(d, &desc);
    if ( (0x04b4 == desc.idVendor) && (0x1003 == desc.idProduct) )
        found = 1;
    return found;
}

int cyusb_open(void)
{
    int r;

    r = libusb_init(NULL);
    if (r)
    {
        printf("Error in initializing libusb library...\n");
        return -2;
    }

    int numdev = libusb_get_device_list(NULL, &libusb_device_list);
    if ( numdev < 0 )
    {
        printf("Library: Error in enumerating devices...\n");
        return -4;
    }

    number_of_cameras = 0;

    for (int i = 0; i < numdev; ++i )
    {
        cyusb_device *tdev = libusb_device_list[i];
        if ( device_is_of_interest(tdev) )
        {
            cydev[number_of_cameras].dev = tdev;
            r = libusb_open(tdev, &cydev[number_of_cameras].handle);
            if ( r )
            {
                printf("Error in opening device, r=%d\n",r);
                return -5;
            }
            ++number_of_cameras;
        }
        if(number_of_cameras>=MAXDEVICES)
            break;
    }
    return number_of_cameras;
}

cyusb_handle * cyusb_gethandle(int index)
{
    return cydev[index].handle;
}

int cyusb_control_transfer(cyusb_handle *h, unsigned char bmRequestType, unsigned char bRequest,
                           unsigned short wValue, unsigned short wIndex, unsigned char *data, unsigned short wLength,
                           unsigned int timeout)
{
    return ( libusb_control_transfer(h, bmRequestType, bRequest, wValue, wIndex, data, wLength, timeout) );
}

int cyusb_bulk_transfer(cyusb_handle *h, unsigned char endpoint, unsigned char *data, int length,
                        int *transferred, int timeout)
{
    return ( libusb_bulk_transfer(h, endpoint, data, length, transferred, timeout) );
}

void cyusb_close(void)
{
    for (int i = 0; i < number_of_cameras; ++i )
    {
        libusb_close(cydev[i].handle);
    }
    libusb_free_device_list(libusb_device_list, 1);
    libusb_exit(NULL);
}
