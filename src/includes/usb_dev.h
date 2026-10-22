/*
  Hatari - usb_dev.h

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  Emulated USB devices behind the ISP1160 host controller (usb_isp1160.c):
  the device framework (descriptors, the standard requests, the control
  pipe), with the mass storage and HID devices in usb_storage.c and
  usb_hid.c.
*/
#ifndef HATARI_USB_DEV_H
#define HATARI_USB_DEV_H

#include "sysdeps.h"

/* what an endpoint handler answers for a packet */
#define USB_DEV_ACK	0		/* OUT: data taken */
#define USB_DEV_NAK	-1		/* nothing to say / not ready: the host retries */
#define USB_DEV_STALL	-2		/* endpoint halted */

#define USB_DEV_CTRL_MAX	1024	/* a control transfer's data stage at most */
#define USB_DEV_MAX_STRINGS	8

typedef struct usb_device_s usb_device_t;

struct usb_device_s
{
	const char *name;		/* for traces and 'info usb' */
	bool low_speed;

	/* descriptors: the device one (18 bytes) and the whole configuration
	 * (configuration, interfaces, class, endpoints); strings as ASCII,
	 * index 1 upwards */
	const uint8_t *dev_desc;
	int dev_desc_len;
	const uint8_t *cfg_desc;
	int cfg_desc_len;
	const char *strings[USB_DEV_MAX_STRINGS];

	/* class hooks (optional): a class or vendor request on the control
	 * pipe, with its data stage in buf (IN: the device fills *len, at
	 * most wLength; OUT: the host's data, called once it is there);
	 * return USB_DEV_ACK or USB_DEV_STALL */
	int (*class_setup)(usb_device_t *dev, const uint8_t *setup, uint8_t *buf, int *len);
	/* a packet on endpoint ep (1..15): IN gives bytes (0..len) or
	 * NAK/STALL, OUT takes them or NAK/STALL */
	int (*ep_in)(usb_device_t *dev, int ep, uint8_t *buf, int len);
	int (*ep_out)(usb_device_t *dev, int ep, const uint8_t *buf, int len);
	/* a USB reset (the port reset), and 'info usb' */
	void (*reset)(usb_device_t *dev);
	void (*info)(usb_device_t *dev, FILE *fp);
	void *priv;

	/* state kept by usb_dev.c */
	int address;			/* 0 until SET_ADDRESS has completed */
	int pending_address;		/* takes effect after the status stage */
	int config;			/* 0 = unconfigured */
	uint16_t halted_in;		/* endpoint halt bits, IN and OUT, bit n = ep n */
	uint16_t halted_out;
	/* the control transfer in progress */
	uint8_t setup[8];
	int ctrl_state;			/* USB_CTRL_* */
	uint8_t ctrl_buf[USB_DEV_CTRL_MAX];
	int ctrl_len;			/* bytes in ctrl_buf (IN: to send; OUT: expected) */
	int ctrl_pos;
};

/* control pipe states */
#define USB_CTRL_IDLE		0
#define USB_CTRL_DATA_IN	1	/* data stage to the host, then an OUT status */
#define USB_CTRL_DATA_OUT	2	/* data stage from the host, then an IN status */
#define USB_CTRL_STATUS_IN	3	/* no data: an IN status (zero length) */
#define USB_CTRL_STALL		4

/* the framework */
extern void usb_dev_reset(usb_device_t *dev);
extern int usb_dev_setup(usb_device_t *dev, const uint8_t *setup8);
extern int usb_dev_in(usb_device_t *dev, int ep, uint8_t *buf, int len);
extern int usb_dev_out(usb_device_t *dev, int ep, const uint8_t *buf, int len);

/* the devices */
extern usb_device_t *usb_storage_create(const char *image);
extern void usb_storage_destroy(usb_device_t *dev);
extern usb_device_t *usb_hid_create(void);
extern void usb_hid_destroy(usb_device_t *dev);
extern void usb_hid_key(int usage, bool down);
extern void usb_hid_mouse(int dx, int dy, int buttons, int wheel);

/* little endian helpers */
static inline uint16_t usb_le16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static inline uint32_t usb_le32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline uint32_t usb_be32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]; }
static inline uint16_t usb_be16(const uint8_t *p) { return (p[0] << 8) | p[1]; }

#endif /* HATARI_USB_DEV_H */
