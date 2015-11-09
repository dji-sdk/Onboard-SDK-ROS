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

#ifndef __HOST_H
#define __HOST_H

#include <libusb-1.0/libusb.h>

#define LB_VENDORID         	0x0547		/*LIGHTBRIDGE ID--THE SAME ID of 68013*/
#define IS_TO_DEVICE	    	0		    /* to device */
#define IS_TO_HOST		        0x80		/* to host */
#define IS_BULK			        2

int usb_sync_read( unsigned char *buffer, int len , int endpoint);
int usb_sync_write(const char *buffer, int len , int endpoint);
int GetEndPoint(libusb_device *dev);
void print_devs(libusb_device **devs);
int init_usbHost();
void ReleaseUsb();
#endif
