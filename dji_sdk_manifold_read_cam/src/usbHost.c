/* Copyright (C) 2015, DJI Innovations, Inc. All rights reserved.
 *
 * The software is licensed under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <strings.h>
#include <stdbool.h>
#include "usbHost.h"

static libusb_device_handle* g_hdev;
static libusb_device* g_dev;
int g_hostGet_endpoint_out = 0, g_hostGet_endpoint_in = 0;

int usb_sync_read(unsigned char *buffer, int len, int endpoint)
{
    int actural_length = 0;
    int ret;
	if(!g_hdev)
	{
		printf("hdev NULL!  %s %d\n",__FILE__,__LINE__);
		return -1;
	}
		
	ret = libusb_bulk_transfer(g_hdev, endpoint, (unsigned char*)buffer, len, &actural_length, 0);
	if ( 0 == ret )
		return actural_length;
	else
		return -1;
}

int usb_sync_write(const char *buffer, int len , int endpoint)
{
    int actural_length = 0;
    int ret;
	if(!g_hdev)
	{
		printf("hdev NULL!  %s %d\n",__FILE__,__LINE__);
		return -1;
	}

    ret = libusb_bulk_transfer(g_hdev, endpoint, (unsigned char*)buffer, len, &actural_length, 0);
	if ( 0 == ret )
		return actural_length;
	else
		return -1;
}

int GetEndPoint(libusb_device *dev)
{
	struct libusb_config_descriptor *pConfig;
	int interface_idx;
    int ep_idx;
    int altsetting_idx;
	int r = libusb_get_active_config_descriptor(dev, &pConfig);
	if (r < 0)
		return -1;

	for (interface_idx = 0; interface_idx < pConfig->bNumInterfaces; interface_idx++) 
	{
		const struct libusb_interface *pInterface = &pConfig->interface[interface_idx];

		for (altsetting_idx = 0; altsetting_idx < pInterface->num_altsetting; altsetting_idx++) 
		{
	        const struct libusb_interface_descriptor *pAltsetting = &pInterface->altsetting[altsetting_idx];
	        for (ep_idx = 0; ep_idx < pAltsetting->bNumEndpoints; ep_idx++)
	        {
		        const struct libusb_endpoint_descriptor *pEndpoint = &pAltsetting->endpoint[ep_idx];

		        if(IS_TO_DEVICE == ( pEndpoint->bEndpointAddress & 0x80 ) && IS_BULK == (pEndpoint->bmAttributes & 0x03))
		        {
			        g_hostGet_endpoint_out = pEndpoint->bEndpointAddress;
			        printf("Usb Host get EndPoint_out:[%x]\n",g_hostGet_endpoint_out);
		        }
		        if(IS_TO_HOST == ( pEndpoint->bEndpointAddress & 0x80 ) && IS_BULK == (pEndpoint->bmAttributes & 0x03))
		        {
			        g_hostGet_endpoint_in = pEndpoint->bEndpointAddress;
			        printf("Usb Host get EndPoint_in:[%x]\n",g_hostGet_endpoint_in);
		        }
	        }
        }

	}
	libusb_free_config_descriptor(pConfig);
	return 0;
}

void print_devs(libusb_device **devs)
{
	libusb_device *dev;
	int i = 0;

	while ((dev = devs[i++]) != NULL)
	{
		struct libusb_device_descriptor desc;
		int r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0)
		{
			fprintf(stderr, "failed to get device descriptor");
			return;
		}

		printf("%04x:%04x (bus %d, device %d)\n", desc.idVendor, 
		                                          desc.idProduct,
			                                      libusb_get_bus_number(dev),
				                                  libusb_get_device_address(dev));
	}
}

int init_usbHost()
{
    bool success = 1;
	int i;
	
	int result = libusb_init(NULL);
	if(result != LIBUSB_SUCCESS)
	{
		printf("Failed to initialise libusb. libusb error: %d\n", result);
		return 1;
	}

	struct libusb_device **devs;
	int ret = libusb_get_device_list(NULL, &devs);
	if (ret < 0)
	{
		printf("libusb_get_device_list FAILED !!! \n");
		return 1;
	}
	else
	{
		printf("Found %d usb device: \n",ret);
		print_devs(devs);
	}
	
	for (i = 0;  devs[i] != NULL; ++i )
	{
		libusb_device *dev = devs[i];
		struct libusb_device_descriptor desc;
		int r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0)
		{
			fprintf(stderr, "failed to get device descriptor");
			continue;
		}
		if ( LB_VENDORID == desc.idVendor )
		{
			printf("Found USB device VendorID = 0x%x \n",LB_VENDORID);
			r = libusb_open(dev, &g_hdev);
			if (r < 0)
			{
				printf( "open failed,%s %d\n", __FILE__, __LINE__ );
				g_hdev = NULL;
				return 1;
			}
			if(g_hdev == NULL)
			{
				printf("Could not open device VENDOR_ID = 0x%x \n",LB_VENDORID);
				return 1;
			}
			g_dev = libusb_get_device(g_hdev);
			result = libusb_claim_interface(g_hdev, 0);
			if (result != LIBUSB_SUCCESS)
			{
				printf("Claiming libusb_claim_interface failed!\n");
				libusb_detach_kernel_driver(g_hdev, 0);
				result = libusb_claim_interface(g_hdev, 0);
			}
			if (result != LIBUSB_SUCCESS)
			{
				printf("Claiming libusb_claim_interface failed!\n");
				libusb_detach_kernel_driver(g_hdev, 0);
			}
			result = GetEndPoint(dev);
			if(result < 0)
			    return 1;
			success = 0;
			break;
		}
	}
	libusb_free_device_list(devs, 1);
	return success;
}

void ReleaseUsb()
{
	libusb_exit(NULL);
    printf( "release usb,%s %d\n", __FILE__, __LINE__ );
}

