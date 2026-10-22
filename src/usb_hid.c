/*
  Hatari - usb_hid.c

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  An emulated USB keyboard and mouse: one low speed device with two
  boot-protocol HID interfaces (0 = keyboard, 1 = mouse), fed from the
  host's SDL events when --usb-input is on (usb_hid_key / usb_hid_mouse,
  called from the input code instead of the IKBD). The keyboard sends
  8-byte boot reports (modifiers, reserved, six keys), the mouse 4-byte
  ones (buttons, dx, dy, wheel), on interrupt endpoints 1 and 2, only
  when something changed (the idle rate is 0).
*/
const char UsbHid_fileid[] = "Hatari usb_hid.c";

#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "log.h"
#include "usb_dev.h"

#define EP_KBD	1
#define EP_MOUSE	2
#define QUEUE	64

/* HID class requests */
#define REQ_GET_REPORT		0x01
#define REQ_GET_IDLE		0x02
#define REQ_GET_PROTOCOL	0x03
#define REQ_SET_REPORT		0x09
#define REQ_SET_IDLE		0x0a
#define REQ_SET_PROTOCOL	0x0b
#define DT_HID			0x21
#define DT_REPORT		0x22

typedef struct
{
	uint8_t report[8];
} kbd_report_t;

typedef struct
{
	uint8_t report[4];
} mouse_report_t;

typedef struct
{
	/* the keyboard's state and queued reports */
	uint8_t modifiers;
	uint8_t keys[6];
	kbd_report_t kq[QUEUE];
	int kq_head, kq_tail;
	uint8_t leds;
	int protocol[2];
	int idle[2];
	/* the mouse */
	int buttons;
	int dx, dy, wheel;
	bool mouse_pending;
	uint64_t reports;
} hid_t;

static const uint8_t hid_dev_desc[18] =
{
	18, 1,
	0x10, 0x01,		/* USB 1.10 */
	0, 0, 0,
	8,			/* bMaxPacketSize0 */
	0x09, 0x12,		/* idVendor 0x1209 */
	0x02, 0x00,		/* idProduct 0x0002 */
	0x00, 0x01,
	1, 2, 0,
	1
};

static const uint8_t kbd_report_desc[] =
{
	0x05, 0x01, 0x09, 0x06, 0xa1, 0x01,		/* usage page desktop, keyboard, application collection */
	0x05, 0x07, 0x19, 0xe0, 0x29, 0xe7,		/* keycodes: modifiers e0-e7 */
	0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
	0x95, 0x01, 0x75, 0x08, 0x81, 0x01,		/* reserved byte */
	0x95, 0x05, 0x75, 0x01, 0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02,	/* 5 LEDs */
	0x95, 0x01, 0x75, 0x03, 0x91, 0x01,		/* LED padding */
	0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00,	/* 6 keys */
	0xc0
};

static const uint8_t mouse_report_desc[] =
{
	0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x09, 0x01, 0xa1, 0x00,	/* desktop, mouse, pointer */
	0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00, 0x25, 0x01,	/* 3 buttons */
	0x95, 0x03, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05, 0x81, 0x01,
	0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,			/* X, Y, wheel */
	0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06,
	0xc0, 0xc0
};

static uint8_t hid_cfg_desc[9 + 2 * (9 + 9 + 7)];


static void build_config(void)
{
	uint8_t *p = hid_cfg_desc;
	int total = sizeof(hid_cfg_desc);
	static const uint8_t iface[2][9] = {
		{ 9, 4, 0, 0, 1, 3, 1, 1, 0 },		/* HID, boot, keyboard */
		{ 9, 4, 1, 0, 1, 3, 1, 2, 0 },		/* HID, boot, mouse */
	};
	int i;

	p[0] = 9; p[1] = 2; p[2] = total; p[3] = total >> 8; p[4] = 2; p[5] = 1; p[6] = 0; p[7] = 0xa0; p[8] = 50;
	p += 9;
	for (i = 0; i < 2; i++)
	{
		int rlen = i == 0 ? sizeof(kbd_report_desc) : sizeof(mouse_report_desc);
		memcpy(p, iface[i], 9);
		p += 9;
		p[0] = 9; p[1] = DT_HID; p[2] = 0x11; p[3] = 0x01; p[4] = 0; p[5] = 1; p[6] = DT_REPORT; p[7] = rlen; p[8] = rlen >> 8;
		p += 9;
		p[0] = 7; p[1] = 5; p[2] = 0x80 | (i == 0 ? EP_KBD : EP_MOUSE); p[3] = 3; p[4] = 8; p[5] = 0; p[6] = 10;
		p += 7;
	}
}


static hid_t *pHid;		/* the one HID device, for the input hooks */


static void kbd_queue(hid_t *h)
{
	int next = (h->kq_tail + 1) % QUEUE;

	if (next == h->kq_head)
		return;				/* full: the report is lost */
	h->kq[h->kq_tail].report[0] = h->modifiers;
	h->kq[h->kq_tail].report[1] = 0;
	memcpy(h->kq[h->kq_tail].report + 2, h->keys, 6);
	h->kq_tail = next;
}


/* a key by its HID usage (SDL scancodes are these) */
void usb_hid_key(int usage, bool down)
{
	hid_t *h = pHid;
	int i;

	if (!h || usage <= 0 || usage > 0xe7)
		return;
	if (usage >= 0xe0)
	{
		if (down)
			h->modifiers |= 1 << (usage - 0xe0);
		else
			h->modifiers &= ~(1 << (usage - 0xe0));
	}
	else if (down)
	{
		for (i = 0; i < 6; i++)
			if (h->keys[i] == usage)
				break;
		if (i == 6)
		{
			for (i = 0; i < 6; i++)
				if (h->keys[i] == 0)
				{
					h->keys[i] = usage;
					break;
				}
			if (i == 6)
				return;		/* rollover: the key is dropped */
		}
	}
	else
	{
		for (i = 0; i < 6; i++)
			if (h->keys[i] == usage)
				h->keys[i] = 0;
	}
	kbd_queue(h);
}


/* mouse motion and buttons (bit 0 left, 1 right, 2 middle), wheel steps */
void usb_hid_mouse(int dx, int dy, int buttons, int wheel)
{
	hid_t *h = pHid;

	if (!h)
		return;
	h->dx += dx;
	h->dy += dy;
	h->wheel += wheel;
	h->buttons = buttons;
	h->mouse_pending = true;
}


static int clamp8(int v)
{
	return v < -127 ? -127 : v > 127 ? 127 : v;
}


static int hid_ep_in(usb_device_t *dev, int ep, uint8_t *buf, int len)
{
	hid_t *h = dev->priv;

	if (ep == EP_KBD)
	{
		if (h->kq_head == h->kq_tail)
			return USB_DEV_NAK;
		if (len > 8)
			len = 8;
		memcpy(buf, h->kq[h->kq_head].report, len);
		h->kq_head = (h->kq_head + 1) % QUEUE;
		h->reports++;
		return len;
	}
	if (ep == EP_MOUSE)
	{
		int x, y, w;

		if (!h->mouse_pending)
			return USB_DEV_NAK;
		x = clamp8(h->dx);
		y = clamp8(h->dy);
		w = clamp8(h->wheel);
		h->dx -= x;
		h->dy -= y;
		h->wheel -= w;
		if (h->dx == 0 && h->dy == 0 && h->wheel == 0)
			h->mouse_pending = false;
		buf[0] = h->buttons & 7;
		buf[1] = (uint8_t)x;
		buf[2] = (uint8_t)y;
		buf[3] = (uint8_t)w;
		h->reports++;
		return len < 4 ? len : 4;
	}
	return USB_DEV_STALL;
}


static int hid_class_setup(usb_device_t *dev, const uint8_t *setup, uint8_t *buf, int *len)
{
	hid_t *h = dev->priv;
	int rtype = setup[0], req = setup[1];
	int wValue = usb_le16(setup + 2), wIndex = usb_le16(setup + 4), wLength = usb_le16(setup + 6);
	int ifn = wIndex & 1;

	/* the HID and report descriptors are fetched with a standard
	 * GET_DESCRIPTOR to the interface, which usb_dev.c hands over here
	 * only when its type is not standard... so they come via the class
	 * hook with the standard request code 6 */
	if ((rtype & 0x60) == 0 && req == 6)
	{
		const uint8_t *d;
		int n;

		if ((wValue >> 8) == DT_REPORT)
		{
			d = ifn == 0 ? kbd_report_desc : mouse_report_desc;
			n = ifn == 0 ? sizeof(kbd_report_desc) : sizeof(mouse_report_desc);
		}
		else if ((wValue >> 8) == DT_HID)
		{
			d = hid_cfg_desc + 9 + 9 + ifn * (9 + 9 + 7);
			n = 9;
		}
		else
			return USB_DEV_STALL;
		if (n > wLength)
			n = wLength;
		memcpy(buf, d, n);
		*len = n;
		return USB_DEV_ACK;
	}
	switch (req)
	{
	 case REQ_SET_PROTOCOL:
		h->protocol[ifn] = wValue;
		*len = 0;
		return USB_DEV_ACK;
	 case REQ_GET_PROTOCOL:
		buf[0] = h->protocol[ifn];
		*len = 1;
		return USB_DEV_ACK;
	 case REQ_SET_IDLE:
		h->idle[ifn] = wValue >> 8;
		*len = 0;
		return USB_DEV_ACK;
	 case REQ_GET_IDLE:
		buf[0] = h->idle[ifn];
		*len = 1;
		return USB_DEV_ACK;
	 case REQ_SET_REPORT:
		if (ifn == 0 && *len >= 1)
		{
			h->leds = buf[0];
			LOG_TRACE(TRACE_USB, "usb hid: leds 0x%02x\n", h->leds);
		}
		*len = 0;
		return USB_DEV_ACK;
	 case REQ_GET_REPORT:
		if (ifn == 0)
		{
			buf[0] = h->modifiers;
			buf[1] = 0;
			memcpy(buf + 2, h->keys, 6);
			*len = wLength < 8 ? wLength : 8;
		}
		else
		{
			buf[0] = h->buttons & 7;
			buf[1] = buf[2] = buf[3] = 0;
			*len = wLength < 4 ? wLength : 4;
		}
		return USB_DEV_ACK;
	}
	return USB_DEV_STALL;
}


static void hid_reset(usb_device_t *dev)
{
	hid_t *h = dev->priv;

	h->protocol[0] = h->protocol[1] = 1;	/* report protocol until SET_PROTOCOL */
	h->idle[0] = h->idle[1] = 0;
	h->kq_head = h->kq_tail = 0;
	h->mouse_pending = false;
	h->dx = h->dy = h->wheel = 0;
}


static void hid_info(usb_device_t *dev, FILE *fp)
{
	hid_t *h = dev->priv;

	fprintf(fp, "  keyboard and mouse: protocol %s/%s, leds 0x%02x, %d reports queued, %llu sent\n",
	        h->protocol[0] ? "report" : "boot", h->protocol[1] ? "report" : "boot", h->leds,
	        (h->kq_tail - h->kq_head + QUEUE) % QUEUE, (unsigned long long)h->reports);
}


usb_device_t *usb_hid_create(void)
{
	usb_device_t *dev;
	hid_t *h;

	h = calloc(1, sizeof(*h));
	dev = calloc(1, sizeof(*dev));
	if (!h || !dev)
	{
		free(h);
		free(dev);
		return NULL;
	}
	build_config();
	dev->name = "keyboard+mouse";
	dev->low_speed = true;
	dev->dev_desc = hid_dev_desc;
	dev->dev_desc_len = sizeof(hid_dev_desc);
	dev->cfg_desc = hid_cfg_desc;
	dev->cfg_desc_len = sizeof(hid_cfg_desc);
	dev->strings[1] = "Hatari";
	dev->strings[2] = "USB keyboard and mouse";
	dev->class_setup = hid_class_setup;
	dev->ep_in = hid_ep_in;
	dev->reset = hid_reset;
	dev->info = hid_info;
	dev->priv = h;
	pHid = h;
	usb_dev_reset(dev);
	return dev;
}


void usb_hid_destroy(usb_device_t *dev)
{
	if (!dev)
		return;
	if (dev->priv == pHid)
		pHid = NULL;
	free(dev->priv);
	free(dev);
}
