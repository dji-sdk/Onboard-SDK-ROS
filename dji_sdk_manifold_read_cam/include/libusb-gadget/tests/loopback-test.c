/*
 * Copyright (C) 2009 Daiki Ueno <ueno@unixuser.org>
 * This file is part of libusb-gadget.
 *
 * libusb-gadget is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libusb-gadget is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <usb.h>

int main (int argc, char **argv)
{
  struct usb_device *dev = NULL;
  struct usb_bus *bus;
  usb_dev_handle *handle;
  int vendor_id, product_id;

  if (argc != 2 || sscanf (argv[1], "%X:%X", &vendor_id, &product_id) < 2)
    {
      fprintf (stderr, "Usage: %s VEND:PROD\n", argv[0]);
      return 1;
    }

  usb_init ();
  usb_set_debug (2);
  usb_find_busses ();
  usb_find_devices ();

  for (bus = usb_busses; bus && !dev; bus = bus->next)
    for (dev = bus->devices; dev; dev = dev->next)
      if (dev->descriptor.idVendor == vendor_id
	  && dev->descriptor.idProduct == product_id)
	break;

  if (!dev)
    {
      fprintf (stderr, "Can't find device %04X:%04X\n", vendor_id, product_id);
      return 1;
    }

  handle = usb_open (dev);
  usb_claim_interface (handle, 0);

  while (1)
    {
      char buf[BUFSIZ], *p;
      int ret;

      memset (buf, 0, sizeof(buf));
      printf ("Write some text: ");
      fflush (stdout);
      if (fgets (buf, 512, stdin) == NULL)
	break;
      p = strchr (buf, '\n');
      if (p)
	*p = '\0';
      ret = usb_bulk_write (handle, 3, buf, strlen (buf), 100);
      if (ret < 0)
	{
	  perror ("usb_bulk_write");
	  continue;
	}
      ret = usb_bulk_read (handle, 7, buf, sizeof(buf), 100);
      if (ret < 0)
	perror ("usb_bulk_read");
      printf ("Text from the USB device: ");
      fflush (stdout);
      write (fileno(stdout), buf, ret);
      putchar ('\n');
    }
  usb_release_interface (handle, 0);
  usb_close (handle);

  return 0;
}
