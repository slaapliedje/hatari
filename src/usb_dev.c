/*
  Hatari - usb_dev.c

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  The emulated USB device framework: the control pipe with the standard
  requests (descriptors, address, configuration, status, features), the
  class requests handed to the device, the other endpoints passed through.
  A device is a usb_device_t with its descriptors and handlers; the host
  controller (usb_isp1160.c) calls usb_dev_setup/in/out per packet.
*/
const char UsbDev_fileid[] = "Hatari usb_dev.c";

#include <string.h>
#include "main.h"
#include "log.h"
#include "usb_dev.h"

/* bmRequestType */
#define RT_DIR_IN	0x80
#define RT_TYPE_MASK	0x60
#define RT_STANDARD	0x00
#define RT_CLASS	0x20
#define RT_VENDOR	0x40
#define RT_RECIP_MASK	0x1f
#define RT_DEVICE	0
#define RT_INTERFACE	1
#define RT_ENDPOINT	2

/* standard requests */
#define REQ_GET_STATUS		0
#define REQ_CLEAR_FEATURE	1
#define REQ_SET_FEATURE		3
#define REQ_SET_ADDRESS		5
#define REQ_GET_DESCRIPTOR	6
#define REQ_SET_DESCRIPTOR	7
#define REQ_GET_CONFIGURATION	8
#define REQ_SET_CONFIGURATION	9
#define REQ_GET_INTERFACE	10
#define REQ_SET_INTERFACE	11

#define DT_DEVICE	1
#define DT_CONFIG	2
#define DT_STRING	3

#define FEAT_ENDPOINT_HALT	0


void usb_dev_reset(usb_device_t *dev)
{
	dev->address = 0;
	dev->pending_address = -1;
	dev->config = 0;
	dev->halted_in = 0;
	dev->halted_out = 0;
	dev->ctrl_state = USB_CTRL_IDLE;
	dev->ctrl_len = 0;
	dev->ctrl_pos = 0;
	if (dev->reset)
		dev->reset(dev);
	LOG_TRACE(TRACE_USB, "usb %s: reset\n", dev->name);
}


/* a string descriptor from an ASCII string, UTF-16LE */
static int string_descriptor(usb_device_t *dev, int index, uint8_t *buf)
{
	const char *s;
	int i, n;

	if (index == 0)
	{
		buf[0] = 4; buf[1] = DT_STRING; buf[2] = 0x09; buf[3] = 0x04;	/* en-US */
		return 4;
	}
	if (index >= USB_DEV_MAX_STRINGS || !(s = dev->strings[index]))
		return -1;
	n = strlen(s);
	if (n > 126)
		n = 126;
	buf[0] = 2 + 2 * n;
	buf[1] = DT_STRING;
	for (i = 0; i < n; i++)
	{
		buf[2 + 2 * i] = (uint8_t)s[i];
		buf[3 + 2 * i] = 0;
	}
	return 2 + 2 * n;
}


/* GET_DESCRIPTOR into ctrl_buf; -1 = stall */
static int get_descriptor(usb_device_t *dev, int type, int index)
{
	switch (type)
	{
	 case DT_DEVICE:
		memcpy(dev->ctrl_buf, dev->dev_desc, dev->dev_desc_len);
		return dev->dev_desc_len;
	 case DT_CONFIG:
		if (index != 0)
			return -1;
		memcpy(dev->ctrl_buf, dev->cfg_desc, dev->cfg_desc_len);
		return dev->cfg_desc_len;
	 case DT_STRING:
		return string_descriptor(dev, index, dev->ctrl_buf);
	}
	return -1;
}


/* The SETUP packet of a control transfer. Returns USB_DEV_ACK (the
 * device took it; the data and status stages follow on endpoint 0) or
 * USB_DEV_STALL. */
int usb_dev_setup(usb_device_t *dev, const uint8_t *setup8)
{
	int rtype = setup8[0], req = setup8[1];
	int wValue = usb_le16(setup8 + 2), wIndex = usb_le16(setup8 + 4), wLength = usb_le16(setup8 + 6);
	int len = 0, r;

	memcpy(dev->setup, setup8, 8);
	dev->ctrl_pos = 0;
	dev->ctrl_len = 0;
	dev->ctrl_state = USB_CTRL_IDLE;
	LOG_TRACE(TRACE_USB, "usb %s: setup type=0x%02x req=%d value=0x%04x index=%d length=%d\n",
	          dev->name, rtype, req, wValue, wIndex, wLength);

	if ((rtype & RT_TYPE_MASK) != RT_STANDARD)
	{
		/* a class or vendor request: with an OUT data stage it is
		 * answered once the data is here; otherwise now */
		if (!dev->class_setup)
			goto stall;
		if (!(rtype & RT_DIR_IN) && wLength > 0)
		{
			if (wLength > USB_DEV_CTRL_MAX)
				goto stall;
			dev->ctrl_len = wLength;
			dev->ctrl_state = USB_CTRL_DATA_OUT;
			return USB_DEV_ACK;
		}
		len = 0;
		r = dev->class_setup(dev, dev->setup, dev->ctrl_buf, &len);
		if (r != USB_DEV_ACK)
			goto stall;
		goto reply;
	}

	switch (req)
	{
	 case REQ_GET_DESCRIPTOR:
		if ((rtype & RT_RECIP_MASK) == RT_INTERFACE)
		{
			/* a class descriptor (HID, report): the device's */
			if (!dev->class_setup)
				goto stall;
			len = 0;
			if (dev->class_setup(dev, dev->setup, dev->ctrl_buf, &len) != USB_DEV_ACK)
				goto stall;
			break;
		}
		len = get_descriptor(dev, wValue >> 8, wValue & 0xff);
		if (len < 0)
			goto stall;
		break;
	 case REQ_SET_ADDRESS:
		dev->pending_address = wValue & 0x7f;
		break;
	 case REQ_SET_CONFIGURATION:
		dev->config = wValue & 0xff;
		dev->halted_in = 0;
		dev->halted_out = 0;
		break;
	 case REQ_GET_CONFIGURATION:
		dev->ctrl_buf[0] = dev->config;
		len = 1;
		break;
	 case REQ_GET_STATUS:
		dev->ctrl_buf[0] = 0;
		dev->ctrl_buf[1] = 0;
		if ((rtype & RT_RECIP_MASK) == RT_ENDPOINT)
		{
			int ep = wIndex & 0xf;
			if ((wIndex & 0x80) ? (dev->halted_in & (1 << ep)) : (dev->halted_out & (1 << ep)))
				dev->ctrl_buf[0] = 1;
		}
		len = 2;
		break;
	 case REQ_CLEAR_FEATURE:
		if ((rtype & RT_RECIP_MASK) == RT_ENDPOINT && wValue == FEAT_ENDPOINT_HALT)
		{
			int ep = wIndex & 0xf;
			if (wIndex & 0x80)
				dev->halted_in &= ~(1 << ep);
			else
				dev->halted_out &= ~(1 << ep);
		}
		break;
	 case REQ_SET_FEATURE:
		if ((rtype & RT_RECIP_MASK) == RT_ENDPOINT && wValue == FEAT_ENDPOINT_HALT)
		{
			int ep = wIndex & 0xf;
			if (wIndex & 0x80)
				dev->halted_in |= 1 << ep;
			else
				dev->halted_out |= 1 << ep;
		}
		break;
	 case REQ_SET_INTERFACE:
		break;
	 case REQ_GET_INTERFACE:
		dev->ctrl_buf[0] = 0;
		len = 1;
		break;
	 default:
		goto stall;
	}

reply:
	if (rtype & RT_DIR_IN)
	{
		if (len > wLength)
			len = wLength;
		dev->ctrl_len = len;
		dev->ctrl_state = USB_CTRL_DATA_IN;
	}
	else
		dev->ctrl_state = USB_CTRL_STATUS_IN;
	return USB_DEV_ACK;

stall:
	LOG_TRACE(TRACE_USB, "usb %s: request stalled\n", dev->name);
	dev->ctrl_state = USB_CTRL_STALL;
	return USB_DEV_STALL;
}


/* an IN packet: endpoint 0 is the control pipe's data or status stage */
int usb_dev_in(usb_device_t *dev, int ep, uint8_t *buf, int len)
{
	int n;

	if (ep != 0)
	{
		if (dev->halted_in & (1 << ep))
			return USB_DEV_STALL;
		if (!dev->ep_in)
			return USB_DEV_STALL;
		return dev->ep_in(dev, ep, buf, len);
	}
	switch (dev->ctrl_state)
	{
	 case USB_CTRL_DATA_IN:
		n = dev->ctrl_len - dev->ctrl_pos;
		if (n > len)
			n = len;
		memcpy(buf, dev->ctrl_buf + dev->ctrl_pos, n);
		dev->ctrl_pos += n;
		return n;
	 case USB_CTRL_STATUS_IN:
	 case USB_CTRL_DATA_OUT:		/* the status stage after an OUT data stage */
		dev->ctrl_state = USB_CTRL_IDLE;
		if (dev->pending_address >= 0)
		{
			dev->address = dev->pending_address;
			dev->pending_address = -1;
			LOG_TRACE(TRACE_USB, "usb %s: address %d\n", dev->name, dev->address);
		}
		return 0;
	 case USB_CTRL_STALL:
		return USB_DEV_STALL;
	}
	return USB_DEV_STALL;
}


/* an OUT packet: endpoint 0 is the control pipe's data or status stage */
int usb_dev_out(usb_device_t *dev, int ep, const uint8_t *buf, int len)
{
	int n, r, l;

	if (ep != 0)
	{
		if (dev->halted_out & (1 << ep))
			return USB_DEV_STALL;
		if (!dev->ep_out)
			return USB_DEV_STALL;
		return dev->ep_out(dev, ep, buf, len);
	}
	switch (dev->ctrl_state)
	{
	 case USB_CTRL_DATA_OUT:
		n = dev->ctrl_len - dev->ctrl_pos;
		if (n > len)
			n = len;
		memcpy(dev->ctrl_buf + dev->ctrl_pos, buf, n);
		dev->ctrl_pos += n;
		if (dev->ctrl_pos >= dev->ctrl_len)
		{
			l = dev->ctrl_len;
			r = dev->class_setup ? dev->class_setup(dev, dev->setup, dev->ctrl_buf, &l) : USB_DEV_STALL;
			if (r != USB_DEV_ACK)
			{
				dev->ctrl_state = USB_CTRL_STALL;
				return USB_DEV_STALL;
			}
			/* the IN status stage follows: handled as DATA_OUT in usb_dev_in */
		}
		return USB_DEV_ACK;
	 case USB_CTRL_DATA_IN:			/* the status stage after an IN data stage */
		dev->ctrl_state = USB_CTRL_IDLE;
		return USB_DEV_ACK;
	 case USB_CTRL_STALL:
		return USB_DEV_STALL;
	}
	return USB_DEV_ACK;
}
