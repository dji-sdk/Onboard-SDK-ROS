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

#ifndef __GADGET_H
#define __GADGET_H

#if HAVE_LINUX_USB_SUBDIR
#include <linux/usb/ch9.h>
#else
#include <linux/usb_ch9.h>
#endif
#include "./libusb-gadget/src/usb-gadget.h"

#define OTG_VENDORID                    0x0547
#define OTG_PRODUCTID                   0x1002

#define STRING_MANUFACTURER             25
#define STRING_PRODUCT                  45
#define STRING_SERIAL                   101
#define STRING_LOOPBACK                 250
#define CONFIG_LOOPBACK         	    2


static struct usb_gadget_string strings[] = {
	{STRING_MANUFACTURER, "The manufacturer",},
	{STRING_PRODUCT, "The product",},
	{STRING_SERIAL, "0123456789.0123456789.0123456789",},
	{STRING_LOOPBACK, "The loopback",},
};

static struct usb_gadget_strings loopback_strings = {
	.language = 0x0409,		/* en-us */
    .strings = strings,
};

static struct usb_device_descriptor loopback_device_descriptor = {
	.bLength = sizeof(loopback_device_descriptor),
	.bDescriptorType = USB_DT_DEVICE,

	.bcdUSB = usb_gadget_cpu_to_le16(0x0200),
	.bDeviceClass = USB_CLASS_VENDOR_SPEC,

	.iManufacturer = usb_gadget_cpu_to_le16(STRING_MANUFACTURER),
	.iProduct = usb_gadget_cpu_to_le16(STRING_PRODUCT),
	.iSerialNumber = usb_gadget_cpu_to_le16(STRING_SERIAL),
    .bNumConfigurations = 1,
};

static struct usb_config_descriptor loopback_config_descriptor = {
	.bLength = sizeof(loopback_config_descriptor),
	.bDescriptorType = USB_DT_CONFIG,

	.bNumInterfaces = 1,
	.bConfigurationValue = CONFIG_LOOPBACK,
	.iConfiguration = STRING_LOOPBACK,
	.bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower = 1,		/* self-powered */
};

static const struct usb_interface_descriptor loopback_interface_descriptor = {
	.bLength = sizeof(loopback_interface_descriptor),
	.bDescriptorType = USB_DT_INTERFACE,

	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_VENDOR_SPEC,
	.iInterface = STRING_LOOPBACK,
};

static struct usb_endpoint_descriptor loopback_ep_in_descriptor = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_IN | 6, /* number is mandatory for gadgetfs */
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = usb_gadget_cpu_to_le16(64), /* mandatory for gadgetfs */
};

static struct usb_endpoint_descriptor loopback_ep_out_descriptor = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_OUT | 2, /* number is mandatory for gadgetfs */
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = usb_gadget_cpu_to_le16(64), /* mandatory for gadgetfs */
};

static struct usb_endpoint_descriptor loopback_hs_ep_in_descriptor = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_IN | 6, /* number is mandatory for gadgetfs */
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = usb_gadget_cpu_to_le16(512), /* mandatory for gadgetfs */
};

static struct usb_endpoint_descriptor loopback_hs_ep_out_descriptor = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_OUT | 2, /* number is mandatory for gadgetfs */
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = usb_gadget_cpu_to_le16(512), /* mandatory for gadgetfs */
};

static struct usb_descriptor_header *loopback_config[] = {
	(struct usb_descriptor_header *)&loopback_config_descriptor,
	(struct usb_descriptor_header *)&loopback_interface_descriptor,
	(struct usb_descriptor_header *)&loopback_ep_in_descriptor,
	(struct usb_descriptor_header *)&loopback_ep_out_descriptor,
	NULL,
};

static struct usb_descriptor_header *loopback_hs_config[] = {
	(struct usb_descriptor_header *)&loopback_config_descriptor,
	(struct usb_descriptor_header *)&loopback_interface_descriptor,
	(struct usb_descriptor_header *)&loopback_hs_ep_in_descriptor,
	(struct usb_descriptor_header *)&loopback_hs_ep_out_descriptor,
	NULL,
};

#endif

