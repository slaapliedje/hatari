/*
  Hatari - usb_storage.c

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  An emulated USB mass storage device (bulk-only transport, SCSI
  transparent command set) backed by a disk image on the host: what a
  USB stick looks like to the ISP1160 host controller. Full speed, two
  bulk endpoints of 64 bytes, one LUN, 512-byte blocks.
*/
const char UsbStorage_fileid[] = "Hatari usb_storage.c";

#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "main.h"
#include "log.h"
#include "file.h"
#include "usb_dev.h"

#define EP_IN	1	/* 0x81: data to the host, the CSW */
#define EP_OUT	2	/* 0x02: the CBW, data from the host */
#define MPS	64
#define BLOCK	512

#define CBW_SIZE	31
#define CSW_SIZE	13
#define CBW_SIGNATURE	0x43425355UL	/* "USBC" */
#define CSW_SIGNATURE	0x53425355UL	/* "USBS" */

/* bulk-only class requests */
#define REQ_BBB_RESET		0xff
#define REQ_GET_MAX_LUN		0xfe

/* SCSI */
#define SC_TEST_UNIT_READY	0x00
#define SC_REQUEST_SENSE	0x03
#define SC_INQUIRY		0x12
#define SC_MODE_SENSE6		0x1a
#define SC_START_STOP		0x1b
#define SC_PREVENT_ALLOW	0x1e
#define SC_READ_FORMAT_CAP	0x23
#define SC_READ_CAPACITY	0x25
#define SC_READ10		0x28
#define SC_WRITE10		0x2a
#define SC_VERIFY10		0x2f
#define SC_SYNC_CACHE		0x35
#define SC_MODE_SENSE10		0x5a

typedef struct
{
	FILE *fp;
	char path[FILENAME_MAX];
	uint32_t blocks;
	bool read_only;
	/* the transport */
	enum { ST_CBW, ST_DATA_IN, ST_DATA_OUT, ST_CSW } state;
	uint32_t tag;
	uint32_t expected;		/* dCBWDataTransferLength */
	uint32_t done;			/* bytes moved in the data stage */
	uint8_t status;			/* bCSWStatus */
	/* the data stage */
	uint8_t *data;			/* a small reply, or the read buffer */
	uint32_t data_len;
	uint32_t data_pos;
	uint8_t small[256];
	/* a write in progress */
	uint32_t lba;
	uint32_t nblocks;
	uint8_t block[BLOCK];
	int block_fill;
	/* the sense of the last failed command */
	uint8_t sense_key, asc, ascq;
	uint64_t reads, writes;
} storage_t;

static const uint8_t storage_dev_desc[18] =
{
	18, 1,			/* bLength, DEVICE */
	0x00, 0x02,		/* USB 2.00 (full speed device) */
	0, 0, 0,		/* class per interface */
	MPS,			/* bMaxPacketSize0 */
	0x09, 0x12,		/* idVendor 0x1209 (pid.codes) */
	0x01, 0x00,		/* idProduct 0x0001: a test id */
	0x00, 0x01,		/* bcdDevice 1.00 */
	1, 2, 3,		/* iManufacturer, iProduct, iSerialNumber */
	1			/* bNumConfigurations */
};

static const uint8_t storage_cfg_desc[32] =
{
	9, 2, 32, 0, 1, 1, 0, 0x80, 50,			/* configuration: 32 bytes, 1 interface, bus powered 100 mA */
	9, 4, 0, 0, 2, 0x08, 0x06, 0x50, 0,		/* interface 0: mass storage, SCSI, bulk-only */
	7, 5, 0x80 | EP_IN, 2, MPS, 0, 0,		/* endpoint 1 IN bulk 64 */
	7, 5, EP_OUT, 2, MPS, 0, 0,			/* endpoint 2 OUT bulk 64 */
};


static void sense(storage_t *s, int key, int asc, int ascq)
{
	s->sense_key = key;
	s->asc = asc;
	s->ascq = ascq;
}


/* a command failed: no data, the CSW says so */
static void fail(storage_t *s, int key, int asc, int ascq)
{
	sense(s, key, asc, ascq);
	s->status = 1;
	s->data_len = 0;
	s->state = ST_CSW;
}


/* a reply of n bytes from s->small (truncated to what the host asked) */
static void reply(storage_t *s, int n)
{
	s->data = s->small;
	s->data_len = n;
	if (s->data_len > s->expected)
		s->data_len = s->expected;
	s->data_pos = 0;
	s->status = 0;
	s->state = s->data_len ? ST_DATA_IN : ST_CSW;
}


static void scsi_command(usb_device_t *dev, storage_t *s, const uint8_t *cb, int cblen, bool dir_in)
{
	uint8_t *b = s->small;
	uint32_t lba, n;

	memset(b, 0, sizeof(s->small));
	LOG_TRACE(TRACE_USB, "usb %s: scsi 0x%02x len %u %s\n", dev->name, cb[0], s->expected, dir_in ? "in" : "out");
	switch (cb[0])
	{
	 case SC_TEST_UNIT_READY:
	 case SC_START_STOP:
	 case SC_PREVENT_ALLOW:
	 case SC_VERIFY10:
	 case SC_SYNC_CACHE:
		s->status = 0;
		s->data_len = 0;
		s->state = ST_CSW;
		sense(s, 0, 0, 0);
		break;

	 case SC_REQUEST_SENSE:
		b[0] = 0x70;			/* current error, fixed format */
		b[2] = s->sense_key;
		b[7] = 10;			/* additional length */
		b[12] = s->asc;
		b[13] = s->ascq;
		reply(s, 18);
		sense(s, 0, 0, 0);
		break;

	 case SC_INQUIRY:
		b[0] = 0x00;			/* direct access block device */
		b[1] = 0x80;			/* removable */
		b[2] = 0x04;			/* SPC-2 */
		b[3] = 0x02;			/* response data format 2 */
		b[4] = 31;			/* additional length (36 bytes in all) */
		memcpy(b + 8, "Hatari  ", 8);
		memcpy(b + 16, "USB stick       ", 16);
		memcpy(b + 32, "1.00", 4);
		reply(s, 36);
		break;

	 case SC_MODE_SENSE6:
		b[0] = 3;			/* mode data length (after this byte) */
		b[1] = 0;			/* medium type */
		b[2] = s->read_only ? 0x80 : 0;	/* device specific: write protect */
		b[3] = 0;			/* no block descriptors */
		reply(s, 4);
		break;

	 case SC_MODE_SENSE10:
		b[1] = 6;
		b[3] = s->read_only ? 0x80 : 0;
		reply(s, 8);
		break;

	 case SC_READ_FORMAT_CAP:
		b[3] = 8;			/* capacity list length */
		b[4] = s->blocks >> 24; b[5] = s->blocks >> 16; b[6] = s->blocks >> 8; b[7] = s->blocks;
		b[8] = 2;			/* formatted media */
		b[10] = BLOCK >> 8; b[11] = BLOCK & 0xff;
		reply(s, 12);
		break;

	 case SC_READ_CAPACITY:
		n = s->blocks - 1;
		b[0] = n >> 24; b[1] = n >> 16; b[2] = n >> 8; b[3] = n;
		b[6] = BLOCK >> 8; b[7] = BLOCK & 0xff;
		reply(s, 8);
		break;

	 case SC_READ10:
	 case SC_WRITE10:
		lba = usb_be32(cb + 2);
		n = usb_be16(cb + 7);
		if (lba + n > s->blocks || lba + n < lba)
		{
			fail(s, 0x05, 0x21, 0x00);	/* LBA out of range */
			break;
		}
		if (cb[0] == SC_WRITE10 && s->read_only)
		{
			fail(s, 0x07, 0x27, 0x00);	/* write protected */
			break;
		}
		s->lba = lba;
		s->nblocks = n;
		s->status = 0;
		s->data_pos = 0;
		s->data_len = n * BLOCK;
		if (s->data_len > s->expected)
			s->data_len = s->expected;
		if (cb[0] == SC_READ10)
		{
			s->data = NULL;			/* read block by block */
			s->block_fill = 0;
			s->state = s->data_len ? ST_DATA_IN : ST_CSW;
			s->reads += n;
		}
		else
		{
			s->block_fill = 0;
			s->state = s->data_len ? ST_DATA_OUT : ST_CSW;
			s->writes += n;
		}
		break;

	 default:
		LOG_TRACE(TRACE_USB, "usb %s: scsi command 0x%02x not supported\n", dev->name, cb[0]);
		fail(s, 0x05, 0x20, 0x00);	/* invalid command operation code */
		break;
	}
}


/* the next bytes of a READ: s->block holds the block at data_pos */
static int read_bytes(storage_t *s, uint8_t *buf, int len)
{
	int n = 0, k, off;

	while (n < len && s->data_pos < s->data_len)
	{
		off = s->data_pos % BLOCK;
		if (off == 0)
		{
			uint32_t lba = s->lba + s->data_pos / BLOCK;
			if (fseeko(s->fp, (off_t)lba * BLOCK, SEEK_SET) != 0 ||
			    fread(s->block, 1, BLOCK, s->fp) != BLOCK)
			{
				Log_Printf(LOG_WARN, "usb storage: read error at block %u: %s\n", lba, strerror(errno));
				memset(s->block, 0, BLOCK);
			}
		}
		k = BLOCK - off;
		if (k > len - n)
			k = len - n;
		if ((uint32_t)k > s->data_len - s->data_pos)
			k = s->data_len - s->data_pos;
		memcpy(buf + n, s->block + off, k);
		n += k;
		s->data_pos += k;
	}
	return n;
}


/* bytes of a WRITE: whole blocks go to the file */
static void write_bytes(storage_t *s, const uint8_t *buf, int len)
{
	int k;

	while (len > 0 && s->data_pos < s->data_len)
	{
		k = BLOCK - s->block_fill;
		if (k > len)
			k = len;
		memcpy(s->block + s->block_fill, buf, k);
		s->block_fill += k;
		buf += k;
		len -= k;
		s->data_pos += k;
		if (s->block_fill == BLOCK)
		{
			uint32_t lba = s->lba + (s->data_pos - 1) / BLOCK;
			if (fseeko(s->fp, (off_t)lba * BLOCK, SEEK_SET) != 0 ||
			    fwrite(s->block, 1, BLOCK, s->fp) != BLOCK)
			{
				Log_Printf(LOG_WARN, "usb storage: write error at block %u: %s\n", lba, strerror(errno));
				s->status = 1;
				sense(s, 0x03, 0x0c, 0x00);	/* write error */
			}
			s->block_fill = 0;
		}
	}
}


static int storage_ep_in(usb_device_t *dev, int ep, uint8_t *buf, int len)
{
	storage_t *s = dev->priv;
	int n;

	if (ep != EP_IN)
		return USB_DEV_STALL;
	switch (s->state)
	{
	 case ST_DATA_IN:
		if (s->data)
		{
			n = s->data_len - s->data_pos;
			if (n > len)
				n = len;
			memcpy(buf, s->data + s->data_pos, n);
			s->data_pos += n;
		}
		else
			n = read_bytes(s, buf, len);
		s->done += n;
		if (s->data_pos >= s->data_len)
			s->state = ST_CSW;
		return n;

	 case ST_CSW:
		if (len < CSW_SIZE)
			return USB_DEV_STALL;
		buf[0] = 'U'; buf[1] = 'S'; buf[2] = 'B'; buf[3] = 'S';
		buf[4] = s->tag; buf[5] = s->tag >> 8; buf[6] = s->tag >> 16; buf[7] = s->tag >> 24;
		n = s->expected - s->done;
		buf[8] = n; buf[9] = n >> 8; buf[10] = n >> 16; buf[11] = n >> 24;
		buf[12] = s->status;
		LOG_TRACE(TRACE_USB, "usb %s: csw status %d residue %d\n", dev->name, s->status, n);
		s->state = ST_CBW;
		fflush(s->fp);
		return CSW_SIZE;

	 case ST_DATA_OUT:
		/* the host wanted to send and asks instead: a phase error */
		return USB_DEV_STALL;
	 case ST_CBW:
		return USB_DEV_NAK;
	}
	return USB_DEV_STALL;
}


static int storage_ep_out(usb_device_t *dev, int ep, const uint8_t *buf, int len)
{
	storage_t *s = dev->priv;

	if (ep != EP_OUT)
		return USB_DEV_STALL;
	switch (s->state)
	{
	 case ST_CBW:
		if (len != CBW_SIZE || usb_le32(buf) != CBW_SIGNATURE)
		{
			LOG_TRACE(TRACE_USB, "usb %s: bad CBW (%d bytes)\n", dev->name, len);
			dev->halted_out |= 1 << EP_OUT;
			return USB_DEV_STALL;
		}
		s->tag = usb_le32(buf + 4);
		s->expected = usb_le32(buf + 8);
		s->done = 0;
		scsi_command(dev, s, buf + 15, buf[14] & 0x1f, (buf[12] & 0x80) != 0);
		return USB_DEV_ACK;

	 case ST_DATA_OUT:
		write_bytes(s, buf, len);
		s->done += len;
		if (s->data_pos >= s->data_len)
			s->state = ST_CSW;
		return USB_DEV_ACK;

	 default:
		return USB_DEV_STALL;
	}
}


static int storage_class_setup(usb_device_t *dev, const uint8_t *setup, uint8_t *buf, int *len)
{
	storage_t *s = dev->priv;

	switch (setup[1])
	{
	 case REQ_GET_MAX_LUN:
		buf[0] = 0;
		*len = 1;
		return USB_DEV_ACK;
	 case REQ_BBB_RESET:
		s->state = ST_CBW;
		*len = 0;
		return USB_DEV_ACK;
	}
	return USB_DEV_STALL;
}


static void storage_reset(usb_device_t *dev)
{
	storage_t *s = dev->priv;

	s->state = ST_CBW;
	s->block_fill = 0;
	sense(s, 0, 0, 0);
}


static void storage_info(usb_device_t *dev, FILE *fp)
{
	storage_t *s = dev->priv;

	fprintf(fp, "  mass storage: %s, %u blocks of %d (%s), %llu blocks read, %llu written\n",
	        s->path, s->blocks, BLOCK, s->read_only ? "read-only" : "read-write",
	        (unsigned long long)s->reads, (unsigned long long)s->writes);
}


/* a storage device on the image file; NULL when the file is not usable */
usb_device_t *usb_storage_create(const char *image)
{
	usb_device_t *dev;
	storage_t *s;
	off_t size;

	s = calloc(1, sizeof(*s));
	dev = calloc(1, sizeof(*dev));
	if (!s || !dev)
	{
		free(s);
		free(dev);
		return NULL;
	}
	s->fp = fopen(image, "r+b");
	if (!s->fp)
	{
		s->fp = fopen(image, "rb");
		s->read_only = true;
	}
	if (!s->fp)
	{
		Log_Printf(LOG_ERROR, "USB disk image '%s': %s\n", image, strerror(errno));
		free(s);
		free(dev);
		return NULL;
	}
	fseeko(s->fp, 0, SEEK_END);
	size = ftello(s->fp);
	s->blocks = (uint32_t)(size / BLOCK);
	if (s->blocks == 0)
	{
		Log_Printf(LOG_ERROR, "USB disk image '%s' is empty\n", image);
		fclose(s->fp);
		free(s);
		free(dev);
		return NULL;
	}
	strncpy(s->path, image, sizeof(s->path) - 1);
	s->state = ST_CBW;

	dev->name = "stick";
	dev->low_speed = false;
	dev->dev_desc = storage_dev_desc;
	dev->dev_desc_len = sizeof(storage_dev_desc);
	dev->cfg_desc = storage_cfg_desc;
	dev->cfg_desc_len = sizeof(storage_cfg_desc);
	dev->strings[1] = "Hatari";
	dev->strings[2] = "USB stick";
	dev->strings[3] = "0001";
	dev->class_setup = storage_class_setup;
	dev->ep_in = storage_ep_in;
	dev->ep_out = storage_ep_out;
	dev->reset = storage_reset;
	dev->info = storage_info;
	dev->priv = s;
	usb_dev_reset(dev);
	Log_Printf(LOG_INFO, "USB stick: %s, %u blocks%s\n", image, s->blocks, s->read_only ? " (read-only)" : "");
	return dev;
}


void usb_storage_destroy(usb_device_t *dev)
{
	storage_t *s;

	if (!dev)
		return;
	s = dev->priv;
	if (s)
	{
		if (s->fp)
			fclose(s->fp);
		free(s);
	}
	free(dev);
}
