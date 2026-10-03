/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
 * Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <config.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <glib.h>
#include <glib/gstdio.h>
#include "protocol.h"
#include "protocol_v2.h"

#define DS_CMD_GET_FW_VERSION		0xb0
#define DS_CMD_GET_REVID_VERSION	0xb1
#define DS_CMD_START			0xb2
#define DS_CMD_CONFIG			0xb3
#define DS_CMD_SETTING			0xb4
#define DS_CMD_CONTROL			0xb5
#define DS_CMD_STATUS			0xb6
#define DS_CMD_STATUS_INFO		0xb7
#define DS_CMD_WR_REG			0xb8
#define DS_CMD_WR_NVM			0xb9
#define DS_CMD_RD_NVM			0xba
#define DS_CMD_RD_NVM_PRE		0xbb
#define DS_CMD_GET_HW_INFO		0xbc

#define DS_START_FLAGS_STOP		(1 << 7)
#define DS_START_FLAGS_CLK_48MHZ	(1 << 6)
#define DS_START_FLAGS_SAMPLE_WIDE	(1 << 5)
#define DS_START_FLAGS_MODE_LA		(1 << 4)

#define DS_ADDR_COMB			0x68
#define DS_ADDR_EEWP			0x70
#define DS_ADDR_VTH			0x78

#define DS_MAX_LOGIC_DEPTH		SR_MHZ(16)
#define DS_MAX_LOGIC_SAMPLERATE		SR_MHZ(100)
#define DS_MAX_TRIG_PERCENT		90

#define DS_MODE_TRIG_EN			(1 << 0)
#define DS_MODE_CLK_TYPE		(1 << 1)
#define DS_MODE_CLK_EDGE		(1 << 2)
#define DS_MODE_RLE_MODE		(1 << 3)
#define DS_MODE_DSO_MODE		(1 << 4)
#define DS_MODE_HALF_MODE		(1 << 5)
#define DS_MODE_QUAR_MODE		(1 << 6)
#define DS_MODE_ANALOG_MODE		(1 << 7)
#define DS_MODE_FILTER			(1 << 8)
#define DS_MODE_INSTANT			(1 << 9)
#define DS_MODE_STRIG_MODE		(1 << 11)
#define DS_MODE_STREAM_MODE		(1 << 12)
#define DS_MODE_LPB_TEST		(1 << 13)
#define DS_MODE_EXT_TEST		(1 << 14)
#define DS_MODE_INT_TEST		(1 << 15)

#define DSLOGIC_ATOMIC_SAMPLES		(sizeof(uint64_t) * 8)
#define DSLOGIC_ATOMIC_BYTES		sizeof(uint64_t)

/*
 * The FPGA is configured with TLV tuples. Length is specified as the
 * number of 16-bit words.
 */
#define _DS_CFG(variable, wordcnt) ((variable << 8) | wordcnt)
#define DS_CFG_START			0xf5a5f5a5
#define DS_CFG_MODE			_DS_CFG(0, 1)
#define DS_CFG_DIVIDER			_DS_CFG(1, 2)
#define DS_CFG_COUNT			_DS_CFG(3, 2)
#define DS_CFG_TRIG_POS			_DS_CFG(5, 2)
#define DS_CFG_TRIG_GLB			_DS_CFG(7, 1)
#define DS_CFG_CH_EN			_DS_CFG(8, 1)
#define DS_CFG_TRIG			_DS_CFG(64, 160)
#define DS_CFG_END			0xfa5afa5a

#pragma pack(push, 1)

struct version_info {
	uint8_t major;
	uint8_t minor;
};

struct cmd_start_acquisition {
	uint8_t flags;
	uint8_t sample_delay_h;
	uint8_t sample_delay_l;
};

struct fpga_config {
	uint32_t sync;

	uint16_t mode_header;
	uint16_t mode;
	uint16_t divider_header;
	uint32_t divider;
	uint16_t count_header;
	uint32_t count;
	uint16_t trig_pos_header;
	uint32_t trig_pos;
	uint16_t trig_glb_header;
	uint16_t trig_glb;
	uint16_t ch_en_header;
	uint16_t ch_en;

	uint16_t trig_header;
	uint16_t trig_mask0[NUM_TRIGGER_STAGES];
	uint16_t trig_mask1[NUM_TRIGGER_STAGES];
	uint16_t trig_value0[NUM_TRIGGER_STAGES];
	uint16_t trig_value1[NUM_TRIGGER_STAGES];
	uint16_t trig_edge0[NUM_TRIGGER_STAGES];
	uint16_t trig_edge1[NUM_TRIGGER_STAGES];
	uint16_t trig_logic0[NUM_TRIGGER_STAGES];
	uint16_t trig_logic1[NUM_TRIGGER_STAGES];
	uint32_t trig_count[NUM_TRIGGER_STAGES];

	uint32_t end_sync;
};

#pragma pack(pop)

/*
 * This should be larger than the FPGA bitstream image so that it'll get
 * uploaded in one big operation. There seem to be issues when uploading
 * it in chunks.
 */
#define FW_BUFSIZE (1024 * 1024)

#define FPGA_UPLOAD_DELAY (10 * 1000)

#define USB_TIMEOUT (3 * 1000)

static int command_get_fw_version(libusb_device_handle *devhdl,
				  struct version_info *vi)
{
	int ret;

	ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_IN, DS_CMD_GET_FW_VERSION, 0x0000, 0x0000,
		(unsigned char *)vi, sizeof(struct version_info), USB_TIMEOUT);

	if (ret < 0) {
		sr_err("Unable to get version info: %s.",
		       libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int command_get_revid_version(struct sr_dev_inst *sdi, uint8_t *revid)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	libusb_device_handle *devhdl = usb->devhdl;
	int ret;

	ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_IN, DS_CMD_GET_REVID_VERSION, 0x0000, 0x0000,
		revid, 1, USB_TIMEOUT);

	if (ret < 0) {
		sr_err("Unable to get REVID: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int command_start_acquisition(const struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb;
	struct dslogic_mode mode;
	int ret;

	mode.flags = DS_START_FLAGS_MODE_LA | DS_START_FLAGS_SAMPLE_WIDE;
	mode.sample_delay_h = mode.sample_delay_l = 0;

	usb = sdi->conn;
	ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
			LIBUSB_ENDPOINT_OUT, DS_CMD_START, 0x0000, 0x0000,
			(unsigned char *)&mode, sizeof(mode), USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Failed to send start command: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int command_stop_acquisition(const struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb;
	struct dslogic_mode mode;
	int ret;

	mode.flags = DS_START_FLAGS_STOP;
	mode.sample_delay_h = mode.sample_delay_l = 0;

	usb = sdi->conn;
	ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
			LIBUSB_ENDPOINT_OUT, DS_CMD_START, 0x0000, 0x0000,
			(unsigned char *)&mode, sizeof(struct dslogic_mode), USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Failed to send stop command: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dslogic_fpga_firmware_upload(const struct sr_dev_inst *sdi)
{
	const char *name = NULL;
	uint64_t sum;
	struct sr_resource bitstream;
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	unsigned char *buf;
	ssize_t chunksize;
	int transferred;
	int result, ret;
	const uint8_t cmd[3] = {0, 0, 0};

	drvc = sdi->driver->context;
	devc = sdi->priv;
	usb = sdi->conn;

	if (!strcmp(devc->profile->model, "DSLogic")) {
		if (devc->cur_threshold < 1.40)
			name = DSLOGIC_FPGA_FIRMWARE_3V3;
		else
			name = DSLOGIC_FPGA_FIRMWARE_5V;
	} else if (!strcmp(devc->profile->model, "DSLogic Pro")){
		name = DSLOGIC_PRO_FPGA_FIRMWARE;
	} else if (!strcmp(devc->profile->model, "DSLogic Plus")){
		name = DSLOGIC_PLUS_FPGA_FIRMWARE;
	} else if (!strcmp(devc->profile->model, "DSLogic Basic")){
		name = DSLOGIC_BASIC_FPGA_FIRMWARE;
	} else if (!strcmp(devc->profile->model, "DSCope")) {
		name = DSCOPE_FPGA_FIRMWARE;
	} else {
		sr_err("Failed to select FPGA firmware.");
		return SR_ERR;
	}

	sr_dbg("Uploading FPGA firmware '%s'.", name);

	result = sr_resource_open(drvc->sr_ctx, &bitstream,
			SR_RESOURCE_FIRMWARE, name);
	if (result != SR_OK)
		return result;

	/* Tell the device firmware is coming. */
	if ((ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
			LIBUSB_ENDPOINT_OUT, DS_CMD_CONFIG, 0x0000, 0x0000,
			(unsigned char *)&cmd, sizeof(cmd), USB_TIMEOUT)) < 0) {
		sr_err("Failed to upload FPGA firmware: %s.", libusb_error_name(ret));
		sr_resource_close(drvc->sr_ctx, &bitstream);
		return SR_ERR;
	}

	/* Give the FX2 time to get ready for FPGA firmware upload. */
	g_usleep(FPGA_UPLOAD_DELAY);

	buf = g_malloc(FW_BUFSIZE);
	sum = 0;
	result = SR_OK;
	while (1) {
		chunksize = sr_resource_read(drvc->sr_ctx, &bitstream,
				buf, FW_BUFSIZE);
		if (chunksize < 0)
			result = SR_ERR;
		if (chunksize <= 0)
			break;

		if ((ret = libusb_bulk_transfer(usb->devhdl, 2 | LIBUSB_ENDPOINT_OUT,
				buf, chunksize, &transferred, USB_TIMEOUT)) < 0) {
			sr_err("Unable to configure FPGA firmware: %s.",
					libusb_error_name(ret));
			result = SR_ERR;
			break;
		}
		sum += transferred;
		sr_spew("Uploaded %" PRIu64 "/%" PRIu64 " bytes.",
			sum, bitstream.size);

		if (transferred != chunksize) {
			sr_err("Short transfer while uploading FPGA firmware.");
			result = SR_ERR;
			break;
		}
	}
	g_free(buf);
	sr_resource_close(drvc->sr_ctx, &bitstream);

	if (result == SR_OK)
		sr_dbg("FPGA firmware upload done.");

	return result;
}

SR_PRIV unsigned int enabled_channel_count(const struct sr_dev_inst *sdi)
{
	unsigned int count = 0;
	for (const GSList *l = sdi->channels; l; l = l->next) {
		const struct sr_channel *const probe = (struct sr_channel *)l->data;
		if (probe->enabled)
			count++;
	}
	return count;
}

SR_PRIV uint16_t enabled_channel_mask(const struct sr_dev_inst *sdi)
{
	unsigned int mask = 0;
	for (const GSList *l = sdi->channels; l; l = l->next) {
		const struct sr_channel *const probe = (struct sr_channel *)l->data;
		if (probe->enabled)
			mask |= 1 << probe->index;
	}
	return mask;
}

/*
 * 32-bit sibling of enabled_channel_mask(), for DSLOGIC_CAPS_CH32 devices
 * (currently only DSLogic U3Pro32) whose channel index can exceed 15.
 * enabled_channel_mask() itself stays 16-bit and untouched: V1's legacy
 * struct fpga_config.ch_en is a real uint16_t wire field, and every other
 * V2 profile tops out at 16 channels too, so there's no reason to touch
 * their code path.
 */
SR_PRIV uint32_t enabled_channel_mask32(const struct sr_dev_inst *sdi)
{
	uint32_t mask = 0;
	for (const GSList *l = sdi->channels; l; l = l->next) {
		const struct sr_channel *const probe = (struct sr_channel *)l->data;
		if (probe->enabled)
			mask |= 1U << probe->index;
	}
	return mask;
}

/*
 * Size of the trigger-position header packet the device sends on EP6 IN
 * right after a capture completes. Mirrors DSView's dsl_header_size()
 * (dsl.c): CAPS_FEATURE_USB30 devices send a 1KB header, everything else
 * 512 bytes. struct dslogic_trigger_pos's real fields all live in the
 * first 512 bytes regardless; the extra 512 bytes on USB30 devices are
 * padding we still have to size the read buffer/request for, or the
 * device's actual send (which DSView itself always sizes via this same
 * function, not a fixed struct size) silently never completes - the
 * trigger acquisition "hangs" with the header packet never arriving,
 * since the device is trying to send more than we asked to receive.
 * Confirmed root cause via USB capture against real U3Pro32 hardware.
 */
SR_PRIV int dslogic_header_size(const struct dev_context *devc)
{
	if (devc->profile->dev_caps & DSLOGIC_CAPS_USB30)
		return 1024;
	return 512;
}

/*
 * Get the session trigger and configure the FPGA structure
 * accordingly.
 * @return @c true if any triggers are enabled, @c false otherwise.
 */
static bool set_trigger(const struct sr_dev_inst *sdi, struct fpga_config *cfg)
{
	struct sr_trigger *trigger;
	struct sr_trigger_stage *stage;
	struct sr_trigger_match *match;
	struct dev_context *devc;
	const GSList *l, *m;
	const unsigned int num_enabled_channels = enabled_channel_count(sdi);
	int num_trigger_stages = 0;

	int channelbit, i = 0;
	uint32_t trigger_point;

	devc = sdi->priv;

	cfg->ch_en = enabled_channel_mask(sdi);

	for (i = 0; i < NUM_TRIGGER_STAGES; i++) {
		cfg->trig_mask0[i] = 0xffff;
		cfg->trig_mask1[i] = 0xffff;
		cfg->trig_value0[i] = 0;
		cfg->trig_value1[i] = 0;
		cfg->trig_edge0[i] = 0;
		cfg->trig_edge1[i] = 0;
		cfg->trig_logic0[i] = 2;
		cfg->trig_logic1[i] = 2;
		cfg->trig_count[i] = 0;
	}

	trigger_point = (devc->capture_ratio * devc->limit_samples) / 100;
	if (trigger_point < DSLOGIC_ATOMIC_SAMPLES)
		trigger_point = DSLOGIC_ATOMIC_SAMPLES;
	const uint32_t mem_depth = devc->profile->mem_depth;
	const uint32_t max_trigger_point = devc->continuous_mode ? ((mem_depth * 10) / 100) :
		((mem_depth * DS_MAX_TRIG_PERCENT) / 100);
	if (trigger_point > max_trigger_point)
		trigger_point = max_trigger_point;
	cfg->trig_pos = trigger_point & ~(DSLOGIC_ATOMIC_SAMPLES - 1);

	if (!(trigger = sr_session_trigger_get(sdi->session))) {
		sr_dbg("No session trigger found");
		return false;
	}

	for (l = trigger->stages; l; l = l->next) {
		stage = l->data;
		num_trigger_stages++;
		for (m = stage->matches; m; m = m->next) {
			match = m->data;
			if (!match->channel->enabled)
				/* Ignore disabled channels with a trigger. */
				continue;
			channelbit = 1 << (match->channel->index);
			/* Simple trigger support (event). */
			if (match->match == SR_TRIGGER_ONE) {
				cfg->trig_mask0[0] &= ~channelbit;
				cfg->trig_mask1[0] &= ~channelbit;
				cfg->trig_value0[0] |= channelbit;
				cfg->trig_value1[0] |= channelbit;
			} else if (match->match == SR_TRIGGER_ZERO) {
				cfg->trig_mask0[0] &= ~channelbit;
				cfg->trig_mask1[0] &= ~channelbit;
			} else if (match->match == SR_TRIGGER_FALLING) {
				cfg->trig_mask0[0] &= ~channelbit;
				cfg->trig_mask1[0] &= ~channelbit;
				cfg->trig_edge0[0] |= channelbit;
				cfg->trig_edge1[0] |= channelbit;
			} else if (match->match == SR_TRIGGER_RISING) {
				cfg->trig_mask0[0] &= ~channelbit;
				cfg->trig_mask1[0] &= ~channelbit;
				cfg->trig_value0[0] |= channelbit;
				cfg->trig_value1[0] |= channelbit;
				cfg->trig_edge0[0] |= channelbit;
				cfg->trig_edge1[0] |= channelbit;
			} else if (match->match == SR_TRIGGER_EDGE) {
				cfg->trig_edge0[0] |= channelbit;
				cfg->trig_edge1[0] |= channelbit;
			}
		}
	}

	cfg->trig_glb = (num_enabled_channels << 4) | (num_trigger_stages - 1);

	return num_trigger_stages != 0;
}

SR_PRIV int fpga_configure(const struct sr_dev_inst *sdi)
{
	const struct dev_context *const devc = sdi->priv;
	const struct sr_usb_dev_inst *const usb = sdi->conn;
	uint8_t c[3];
	struct fpga_config cfg;
	uint16_t mode = 0;
	uint32_t divider;
	int transferred, len, ret;

	sr_dbg("Configuring FPGA.");

	WL32(&cfg.sync, DS_CFG_START);
	WL16(&cfg.mode_header, DS_CFG_MODE);
	WL16(&cfg.divider_header, DS_CFG_DIVIDER);
	WL16(&cfg.count_header, DS_CFG_COUNT);
	WL16(&cfg.trig_pos_header, DS_CFG_TRIG_POS);
	WL16(&cfg.trig_glb_header, DS_CFG_TRIG_GLB);
	WL16(&cfg.ch_en_header, DS_CFG_CH_EN);
	WL16(&cfg.trig_header, DS_CFG_TRIG);
	WL32(&cfg.end_sync, DS_CFG_END);

	/* Pass in the length of a fixed-size struct. Really. */
	len = sizeof(struct fpga_config) / 2;
	c[0] = len & 0xff;
	c[1] = (len >> 8) & 0xff;
	c[2] = (len >> 16) & 0xff;

	ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
			LIBUSB_ENDPOINT_OUT, DS_CMD_SETTING, 0x0000, 0x0000,
			c, sizeof(c), USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Failed to send FPGA configure command: %s.",
			libusb_error_name(ret));
		return SR_ERR;
	}

	if (set_trigger(sdi, &cfg))
		mode |= DS_MODE_TRIG_EN;

	if (devc->mode == DS_OP_INTERNAL_TEST)
		mode |= DS_MODE_INT_TEST;
	else if (devc->mode == DS_OP_EXTERNAL_TEST)
		mode |= DS_MODE_EXT_TEST;
	else if (devc->mode == DS_OP_LOOPBACK_TEST)
		mode |= DS_MODE_LPB_TEST;

	if (devc->cur_samplerate == DS_MAX_LOGIC_SAMPLERATE * 2)
		mode |= DS_MODE_HALF_MODE;
	else if (devc->cur_samplerate == DS_MAX_LOGIC_SAMPLERATE * 4)
		mode |= DS_MODE_QUAR_MODE;

	if (devc->continuous_mode)
		mode |= DS_MODE_STREAM_MODE;
	if (devc->external_clock) {
		mode |= DS_MODE_CLK_TYPE;
		if (devc->clock_edge == DS_EDGE_FALLING)
			mode |= DS_MODE_CLK_EDGE;
	}
	if (devc->limit_samples > DS_MAX_LOGIC_DEPTH *
		ceil(devc->cur_samplerate * 1.0 / DS_MAX_LOGIC_SAMPLERATE)
		&& !devc->continuous_mode) {
		/* Enable RLE for long captures.
		 * Without this, captured data present errors.
		 */
		mode |= DS_MODE_RLE_MODE;
	}

	WL16(&cfg.mode, mode);
	divider = ceil(DS_MAX_LOGIC_SAMPLERATE * 1.0 / devc->cur_samplerate);
	WL32(&cfg.divider, divider);

	/* Number of 16-sample units. */
	WL32(&cfg.count, devc->limit_samples / 16);

	len = sizeof(struct fpga_config);
	ret = libusb_bulk_transfer(usb->devhdl, 2 | LIBUSB_ENDPOINT_OUT,
			(unsigned char *)&cfg, len, &transferred, USB_TIMEOUT);
	if (ret < 0 || transferred != len) {
		sr_err("Failed to send FPGA configuration: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dslogic_set_voltage_threshold(const struct sr_dev_inst *sdi, double threshold)
{
	int ret;
	struct dev_context *const devc = sdi->priv;
	const struct sr_usb_dev_inst *const usb = sdi->conn;
	const uint8_t value = (threshold / 5.0) * 255;
	const uint16_t cmd = value | (DS_ADDR_VTH << 8);

	/* Send the control command. */
	ret = libusb_control_transfer(usb->devhdl,
			LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
			DS_CMD_WR_REG, 0x0000, 0x0000,
			(unsigned char *)&cmd, sizeof(cmd), 3000);
	if (ret < 0) {
		sr_err("Unable to set voltage-threshold register: %s.",
		libusb_error_name(ret));
		return SR_ERR;
	}

	devc->cur_threshold = threshold;

	return SR_OK;
}

SR_PRIV int dslogic_dev_open(struct sr_dev_inst *sdi, struct sr_dev_driver *di)
{
	libusb_device **devlist;
	struct sr_usb_dev_inst *usb;
	struct libusb_device_descriptor des;
	struct dev_context *devc;
	struct drv_context *drvc;
	struct version_info vi;
	int ret = SR_ERR, i, device_count;
	uint8_t revid;
	char connection_id[64];

	drvc = di->context;
	devc = sdi->priv;
	usb = sdi->conn;

	device_count = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (device_count < 0) {
		sr_err("Failed to get device list: %s.",
		       libusb_error_name(device_count));
		return SR_ERR;
	}

	for (i = 0; i < device_count; i++) {
		libusb_get_device_descriptor(devlist[i], &des);

		if (des.idVendor != devc->profile->vid
		    || des.idProduct != devc->profile->pid)
			continue;

		if ((sdi->status == SR_ST_INITIALIZING) ||
				(sdi->status == SR_ST_INACTIVE)) {
			/* Check device by its physical USB bus/port address. */
			if (usb_get_port_path(devlist[i], connection_id, sizeof(connection_id)) < 0)
				continue;

			if (strcmp(sdi->connection_id, connection_id))
				/* This is not the one. */
				continue;
		}

		if (!(ret = libusb_open(devlist[i], &usb->devhdl))) {
			if (usb->address == 0xff)
				/*
				 * First time we touch this device after FW
				 * upload, so we don't know the address yet.
				 */
				usb->address = libusb_get_device_address(devlist[i]);
			devc->usb_speed = libusb_get_device_speed(devlist[i]);
		} else {
			sr_err("Failed to open device: %s.",
			       libusb_error_name(ret));
			ret = SR_ERR;
			break;
		}

		if (libusb_has_capability(LIBUSB_CAP_SUPPORTS_DETACH_KERNEL_DRIVER)) {
			if (libusb_kernel_driver_active(usb->devhdl, USB_INTERFACE) == 1) {
				if ((ret = libusb_detach_kernel_driver(usb->devhdl, USB_INTERFACE)) < 0) {
					sr_err("Failed to detach kernel driver: %s.",
						libusb_error_name(ret));
					ret = SR_ERR;
					break;
				}
			}
		}

		/*
		 * The V1 firmware-version probe uses bRequest 0xb0, which
		 * collides with the V2 envelope opcode CMD_CTL_WR. Only run
		 * the V1 probe + version check for V1 devices.
		 */
		if (devc->profile->protocol_version == DSL_PROTO_V1) {
			ret = command_get_fw_version(usb->devhdl, &vi);
			if (ret != SR_OK) {
				sr_err("Failed to get firmware version.");
				break;
			}

			ret = command_get_revid_version(sdi, &revid);
			if (ret != SR_OK) {
				sr_err("Failed to get REVID.");
				break;
			}

			/*
			 * Changes in major version mean incompatible/API changes,
			 * so bail out if we encounter an incompatible version.
			 * Different minor versions are OK, they should be compatible.
			 */
			if (vi.major != DSLOGIC_REQUIRED_VERSION_MAJOR) {
				sr_err("Expected firmware version %d.x, "
				       "got %d.%d.", DSLOGIC_REQUIRED_VERSION_MAJOR,
				       vi.major, vi.minor);
				ret = SR_ERR;
				break;
			}
		} else {
			/*
			 * V2 hello reads - DSView's hw_dev_open starts with
			 * DSL_CTL_FW_VERSION read (dsl.c) and dsl_dev_open
			 * follows with a DSL_CTL_HW_STATUS read (dsl.c).
			 * These appear to prime the firmware's state machine; without
			 * them, later HW_STATUS reads in the arm path stall. We
			 * don't validate the values - just perform the reads.
			 */
			{
				uint8_t v2_fw_ver[2] = {0, 0};
				uint8_t v2_hw_status = 0;
				struct ctl_rd_cmd v2_rd;

				v2_rd.header.dest = DSL_CTL_FW_VERSION;
				v2_rd.header.offset = 0;
				v2_rd.header.size = 2;
				v2_rd.data = v2_fw_ver;
				if (command_ctl_rd_v2(usb->devhdl, v2_rd) == SR_OK)
					sr_info("V2 firmware version: %u.%u",
						v2_fw_ver[0], v2_fw_ver[1]);

				v2_rd.header.dest = DSL_CTL_HW_STATUS;
				v2_rd.header.offset = 0;
				v2_rd.header.size = 1;
				v2_rd.data = &v2_hw_status;
				if (command_ctl_rd_v2(usb->devhdl, v2_rd) == SR_OK)
					sr_dbg("V2 HW_STATUS: 0x%02x", v2_hw_status);
			}
		}

		sr_info("Opened device on %d.%d (logical) / %s (physical), "
			"interface %d, firmware %d.%d.",
			usb->bus, usb->address, connection_id,
			USB_INTERFACE, vi.major, vi.minor);

		sr_info("Detected REVID=%d, it's a Cypress CY7C68013%s.",
			revid, (revid != 1) ? " (FX2)" : "A (FX2LP)");

		ret = SR_OK;

		break;
	}

	libusb_free_device_list(devlist, 1);

	return ret;
}

SR_PRIV struct dev_context *dslogic_dev_new(void)
{
	struct dev_context *devc;

	devc = g_malloc0(sizeof(struct dev_context));
	devc->profile = NULL;
	devc->fw_updated = 0;
	devc->cur_samplerate = 0;
	devc->limit_samples = 0;
	devc->capture_ratio = 0;
	devc->continuous_mode = FALSE;
	devc->clock_edge = DS_EDGE_RISING;
	/*
	 * Default channel-mode id. devc->profile isn't assigned yet at this
	 * point (scan() sets it right after this call returns), so we can't
	 * look up a real default here. 0 is each V2 family's own default
	 * entry id (see DSLOGIC_PLUS_DEFAULT_CH_MODE_ID /
	 * DSLOGIC_U3PRO32_DEFAULT_CH_MODE_ID in protocol_v2.c); V1 devices
	 * ignore ch_mode_id entirely.
	 */
	devc->ch_mode_id = 0;

	return devc;
}

static void abort_acquisition(struct dev_context *devc)
{
	int i;

	devc->acq_aborted = TRUE;

	for (i = devc->num_transfers - 1; i >= 0; i--) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}
}

static void finish_acquisition(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;

	devc = sdi->priv;

	std_session_send_df_end(sdi);

	usb_source_remove(sdi->session, devc->ctx);

	devc->num_transfers = 0;
	g_free(devc->transfers);
	g_free(devc->deinterleave_buffer);
}

/* Forward decl: trigger_receive is defined alongside the V1 acquisition
 * paths near the end of this file; rearm_chunk_in_place needs it as a
 * libusb callback for the trigger-position transfer. */
static void LIBUSB_CALL trigger_receive(struct libusb_transfer *transfer);

static void rearm_chunk_in_place(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct dslogic_trigger_pos *tpos;
	struct libusb_transfer *transfer;
	gint64 t_start_us;
	int ret;

	t_start_us = g_get_monotonic_time();

	/* Per-chunk state. total_deadline_us preserved across chunks. */
	devc->acq_aborted = FALSE;
	devc->rearm_pending = FALSE;
	devc->sent_samples = 0;
	devc->actual_samples = 0;
	devc->empty_transfer_count = 0;
	devc->wallclock_deadline_us = 0;

	g_free(devc->transfers);
	devc->transfers = NULL;
	devc->num_transfers = 0;
	g_free(devc->deinterleave_buffer);
	devc->deinterleave_buffer = NULL;

	/* Chunk boundary marker for downstream decoders. */
	std_session_send_df_trigger(sdi);

	if ((ret = devc->ops->acquisition_stop(sdi)) != SR_OK)
		goto fail;
	if ((ret = devc->ops->fpga_config(sdi)) != SR_OK)
		goto fail;

	/*
	 * Post the header-read URB before DSL_CTL_START, not after - see
	 * the comment in dslogic_acquisition_start() for why (missed
	 * one-shot header packet on fast captures).
	 */
	tpos = g_malloc0(dslogic_header_size(devc));
	transfer = libusb_alloc_transfer(0);
	libusb_fill_bulk_transfer(transfer, usb->devhdl,
			6 | LIBUSB_ENDPOINT_IN,
			(unsigned char *)tpos,
			dslogic_header_size(devc),
			trigger_receive, (void *)sdi, 0);
	if ((ret = libusb_submit_transfer(transfer)) < 0) {
		sr_err("Re-arm trigger transfer submit failed: %s.",
		       libusb_error_name(ret));
		libusb_free_transfer(transfer);
		g_free(tpos);
		goto fail;
	}

	devc->transfers = g_try_malloc0(sizeof(*devc->transfers));
	if (!devc->transfers) {
		sr_err("Re-arm transfer table malloc failed.");
		goto fail;
	}
	devc->num_transfers = 1;
	devc->submitted_transfers++;
	devc->transfers[0] = transfer;

	if ((ret = devc->ops->acquisition_start(sdi)) != SR_OK)
		goto fail;

	sr_dbg("Re-armed chunk in %.2f ms.",
	       (g_get_monotonic_time() - t_start_us) / 1000.0);
	return;

fail:
	sr_err("Chunk re-arm failed (%d); ending session.", ret);
	finish_acquisition(sdi);
}

/*
 * Re-arm the FPGA for the next chunk without ending the libsigrok session.
 *
 * Called from free_transfer when the previous chunk's transfers have all
 * been drained and rearm_pending was set. Sends an SR_DF_TRIGGER marker so
 * downstream decoders can see the chunk boundary, re-runs the arm sequence
 * (stop / fpga_config / acquisition_start), and re-submits a fresh trigger-
 * position transfer so the next chunk's data will flow on the same session.
 *
 * usb_source_add is NOT re-issued: the libusb event source is owned by the
 * outer session and remains valid across chunks. Per-chunk counters
 * (sent_samples, actual_samples, empty_transfer_count, wallclock_deadline_us)
 * are reset; total_deadline_us is preserved.
 */
static void free_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	unsigned int i;

	sdi = transfer->user_data;
	devc = sdi->priv;

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);

	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}

	devc->submitted_transfers--;
	/*
	 * Don't kick the re-arm here. We're inside a libusb callback;
	 * issuing synchronous USB control transfers from this context
	 * collides with libusb's internal transfer machinery and returns
	 * LIBUSB_ERROR_BUSY. Instead let the periodic receive_data tick
	 * notice submitted_transfers == 0 + rearm_pending and run the
	 * re-arm from a clean stack.
	 *
	 * For the no-rearm case (session end), finish_acquisition is
	 * still safe to call here because it only touches libsigrok
	 * bookkeeping (no USB I/O).
	 */
	if (devc->submitted_transfers == 0) {
		if (!(devc->rearm_pending
				&& devc->total_deadline_us
				&& g_get_monotonic_time() < devc->total_deadline_us)) {
			finish_acquisition(sdi);
		}
	}
}

static void resubmit_transfer(struct libusb_transfer *transfer)
{
	int ret;

	if ((ret = libusb_submit_transfer(transfer)) == LIBUSB_SUCCESS)
		return;

	sr_err("%s: %s", __func__, libusb_error_name(ret));
	free_transfer(transfer);

}

/*
 * channel_mask/channel_count can span up to 32 enabled channels
 * (DSLOGIC_CAPS_CH32 profiles); every other device tops out at 16.
 * unitsize (2 or 4 bytes, set by the caller from channel_count) picks
 * how many bytes of dst_ptr each decoded sample gets: for <=16 enabled
 * channels this writes the exact same uint16_t-per-sample layout the
 * original 16-channel-only version did, just through a byte pointer.
 */
static void deinterleave_buffer(const uint8_t *src, size_t length,
	uint8_t *dst_ptr, size_t channel_count, uint32_t channel_mask,
	unsigned int unitsize)
{
	uint32_t sample;
	const uint64_t *const src_end = (const uint64_t *)(src + length);

	/*
	 * Bound on "a full channel_count-word block remains", not just
	 * "any bytes remain": if actual_length isn't an exact multiple of
	 * channel_count * 8, the naive src_ptr < src_end check lets the
	 * loop start one block too many, reading past the transfer buffer
	 * and writing 64 more samples than devc->deinterleave_buffer was
	 * sized for - a real heap overflow (confirmed via a double-free
	 * crash on real hardware with a 1MB buffer and 3 enabled channels,
	 * where 1048576 isn't a multiple of 3*8=24). Any trailing partial
	 * block is simply dropped, matching the "Invalid transfer length!"
	 * warning already logged by the caller for this case.
	 */
	for (const uint64_t *src_ptr = (uint64_t*)src;
		src_ptr + channel_count <= src_end;
		src_ptr += channel_count) {
		for (int bit = 0; bit != 64; bit++) {
			const uint64_t *word_ptr = src_ptr;
			sample = 0;
			for (unsigned int channel = 0; channel != 32;
				channel++) {
				const uint32_t m = channel_mask >> channel;
				if (!m)
					break;
				if ((m & 1) && ((*word_ptr++ >> bit) & UINT64_C(1)))
					sample |= 1U << channel;
			}
			memcpy(dst_ptr, &sample, unitsize);
			dst_ptr += unitsize;
		}
	}
}

static void send_data(struct sr_dev_inst *sdi,
	uint8_t *data, size_t sample_count, unsigned int unitsize)
{
	const struct sr_datafeed_logic logic = {
		.length = sample_count * unitsize,
		.unitsize = unitsize,
		.data = data
	};

	const struct sr_datafeed_packet packet = {
		.type = SR_DF_LOGIC,
		.payload = &logic
	};

	sr_session_send(sdi, &packet);
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *const sdi = transfer->user_data;
	struct dev_context *const devc = sdi->priv;
	const size_t channel_count = enabled_channel_count(sdi);
	const uint32_t channel_mask = enabled_channel_mask32(sdi);
	const unsigned int unitsize = devc->sample_unitsize;
	/*
	 * Must match deinterleave_buffer()'s "complete blocks only" count
	 * exactly: (ATOMIC_SAMPLES * actual_length) / (ATOMIC_BYTES *
	 * channel_count), evaluated as a single division, can round UP
	 * relative to floor(actual_length / (ATOMIC_BYTES * channel_count))
	 * * ATOMIC_SAMPLES when actual_length isn't an exact multiple of
	 * ATOMIC_BYTES * channel_count - over-reporting how many samples
	 * were actually written and reading past the end of
	 * devc->deinterleave_buffer in send_data(). Divide first.
	 */
	const unsigned int cur_sample_count = DSLOGIC_ATOMIC_SAMPLES *
		(transfer->actual_length / (DSLOGIC_ATOMIC_BYTES * channel_count));

	gboolean packet_has_error = FALSE;
	unsigned int num_samples;
	int trigger_offset;

	/*
	 * If acquisition has already ended, just free any queued up
	 * transfer that come in.
	 */
	if (devc->acq_aborted) {
		free_transfer(transfer);
		return;
	}

	sr_dbg("receive_transfer(): status %s received %d bytes.",
		libusb_error_name(transfer->status), transfer->actual_length);

	/* Save incoming transfer before reusing the transfer struct. */

	switch (transfer->status) {
	case LIBUSB_TRANSFER_NO_DEVICE:
		abort_acquisition(devc);
		free_transfer(transfer);
		return;
	case LIBUSB_TRANSFER_COMPLETED:
	case LIBUSB_TRANSFER_TIMED_OUT: /* We may have received some data though. */
		break;
	default:
		packet_has_error = TRUE;
		break;
	}

	/*
	 * Stream+RLE wall-clock cutoff. Arm the deadline on the first
	 * non-empty transfer (so capture-start = first sample seen) and
	 * abort the moment we cross it. budget_for_deadline is the
	 * per-chunk capture duration (chunk_samples in chunk_loop mode,
	 * limit_samples otherwise); the 10% grace lets the FX2 drain its
	 * in-flight bytes before we cancel.
	 */
	if (devc->continuous_mode && devc->rle_mode
			&& transfer->actual_length > 0
			&& devc->wallclock_deadline_us == 0
			&& devc->cur_samplerate) {
		uint64_t budget_for_deadline = devc->chunk_loop
			? devc->chunk_samples : devc->limit_samples;
		if (budget_for_deadline) {
			gint64 dur_us = (gint64)((double)budget_for_deadline /
					(double)devc->cur_samplerate * 1e6 * 1.1);
			devc->wallclock_deadline_us =
				g_get_monotonic_time() + dur_us;
			sr_dbg("Stream+RLE wall-clock deadline armed at "
			       "+%" PRId64 " us (%.2f s)", dur_us, dur_us / 1e6);
		}
	}
	if (devc->wallclock_deadline_us
			&& g_get_monotonic_time() >= devc->wallclock_deadline_us) {
		sr_dbg("Stream+RLE: wall-clock deadline reached "
		       "(sent_samples=%u of budget=%" PRIu64 ").",
		       devc->sent_samples,
		       devc->actual_samples ? devc->actual_samples
					    : devc->limit_samples);
		/* If chunk_loop is on and we're still inside the session
		 * deadline, request a re-arm instead of ending the session. */
		if (devc->chunk_loop && devc->total_deadline_us
				&& g_get_monotonic_time() < devc->total_deadline_us) {
			devc->rearm_pending = TRUE;
		}
		abort_acquisition(devc);
		free_transfer(transfer);
		return;
	}

	if (transfer->actual_length == 0 || packet_has_error) {
		devc->empty_transfer_count++;
		if (devc->empty_transfer_count > MAX_EMPTY_TRANSFERS) {
			/*
			 * The FX2 gave up. End the acquisition, the frontend
			 * will work out that the samplecount is short.
			 */
			sr_info("Aborting acquisition after %u empty transfers; "
				"sent_samples=%u, budget=%" PRIu64 ", "
				"continuous=%d, rle=%d. The FPGA stopped "
				"emitting data before reaching the requested "
				"sample budget (likely RLE/USB-bandwidth "
				"overflow in stream+RLE, or end-of-capture "
				"in buffered mode).",
				devc->empty_transfer_count,
				devc->sent_samples,
				devc->actual_samples ? devc->actual_samples
						     : devc->limit_samples,
				(int)devc->continuous_mode,
				(int)devc->rle_mode);
			abort_acquisition(devc);
			free_transfer(transfer);
		} else {
			resubmit_transfer(transfer);
		}
		return;
	} else {
		devc->empty_transfer_count = 0;
	}

	/*
	 * The acquisition-stop budget is actual_samples (= limit_samples for
	 * normal captures, possibly less under RLE). Falls back to limit_samples
	 * if the trigger-position header has not arrived yet (actual_samples
	 * still zero from dslogic_acquisition_start).
	 */
	const uint64_t budget = devc->actual_samples
		? devc->actual_samples : devc->limit_samples;

	if (!budget || devc->sent_samples < budget) {
		if (budget && devc->sent_samples + cur_sample_count > budget)
			num_samples = budget - devc->sent_samples;
		else
			num_samples = cur_sample_count;

		/**
		 * The DSLogic emits sample data as sequences of 64-bit sample words
		 * in a round-robin i.e. 64-bits from channel 0, 64-bits from channel 1
		 * etc. for each of the enabled channels, then looping back to the
		 * channel.
		 *
		 * Because sigrok's internal representation is bit-interleaved channels
		 * we must recast the data.
		 *
		 * Hopefully in future it will be possible to pass the data on as-is.
		 */
		if (transfer->actual_length % (DSLOGIC_ATOMIC_BYTES * channel_count) != 0)
			sr_dbg("Transfer length %d isn't a multiple of the "
				"atomic block size (%zu bytes for %zu "
				"channels); trailing partial block dropped. "
				"Expected whenever the fixed buffered-mode "
				"buffer size doesn't evenly divide by the "
				"enabled channel count - not an error.",
				transfer->actual_length,
				DSLOGIC_ATOMIC_BYTES * channel_count, channel_count);
		deinterleave_buffer(transfer->buffer, transfer->actual_length,
			devc->deinterleave_buffer, channel_count, channel_mask,
			unitsize);

		/* Send the incoming transfer to the session bus. */
		if (devc->trigger_pos > devc->sent_samples
			&& devc->trigger_pos <= devc->sent_samples + num_samples) {
			/* DSLogic trigger in this block. Send trigger position. */
			trigger_offset = devc->trigger_pos - devc->sent_samples;
			/* Pre-trigger samples. */
			send_data(sdi, devc->deinterleave_buffer, trigger_offset, unitsize);
			devc->sent_samples += trigger_offset;
			/* Trigger position. */
			devc->trigger_pos = 0;
			std_session_send_df_trigger(sdi);
			/* Post trigger samples. */
			num_samples -= trigger_offset;
			send_data(sdi, (uint8_t *)devc->deinterleave_buffer
				+ (size_t)trigger_offset * unitsize, num_samples, unitsize);
			devc->sent_samples += num_samples;
		} else {
			send_data(sdi, devc->deinterleave_buffer, num_samples, unitsize);
			devc->sent_samples += num_samples;
		}
	}

	if (budget && devc->sent_samples >= budget) {
		/* Per-chunk budget consumed. In chunk_loop, queue a re-arm
		 * unless the overall session deadline has elapsed. */
		if (devc->chunk_loop && devc->total_deadline_us
				&& g_get_monotonic_time() < devc->total_deadline_us) {
			devc->rearm_pending = TRUE;
		}
		abort_acquisition(devc);
		free_transfer(transfer);
	} else
		resubmit_transfer(transfer);
}

static int receive_data(int fd, int revents, void *cb_data)
{
	struct timeval tv;
	struct drv_context *drvc;
	GSList *l;

	(void)fd;
	(void)revents;

	drvc = (struct drv_context *)cb_data;

	tv.tv_sec = tv.tv_usec = 0;
	libusb_handle_events_timeout(drvc->sr_ctx->libusb_ctx, &tv);

	/*
	 * Drain any deferred per-chunk re-arms. free_transfer can't issue
	 * synchronous control transfers from inside its libusb callback
	 * (returns LIBUSB_ERROR_BUSY), so it just sets rearm_pending +
	 * lets the prior chunk's transfers drain. Here, on a clean stack,
	 * we re-arm the FPGA and submit a fresh trigger-position transfer
	 * for the next chunk.
	 */
	for (l = drvc->instances; l; l = l->next) {
		struct sr_dev_inst *sdi = l->data;
		struct dev_context *devc = sdi->priv;
		if (devc && devc->rearm_pending
				&& devc->submitted_transfers == 0
				&& devc->total_deadline_us
				&& g_get_monotonic_time() < devc->total_deadline_us) {
			rearm_chunk_in_place(sdi);
		} else if (devc && devc->rearm_pending
				&& devc->submitted_transfers == 0) {
			/* Total deadline elapsed during drain. End the session. */
			devc->rearm_pending = FALSE;
			finish_acquisition(sdi);
		}
	}

	return TRUE;
}

static size_t to_bytes_per_ms(const struct sr_dev_inst *sdi)
{
	const struct dev_context *const devc = sdi->priv;
	const size_t ch_count = enabled_channel_count(sdi);

	if (devc->continuous_mode)
		return (devc->cur_samplerate * ch_count) / (1000 * 8);

	/*
	 * Buffered mode doesn't use this - see get_buffer_size()'s comment.
	 * Kept only so callers that still (wrongly) multiply by it in a
	 * continuous_mode-agnostic way don't divide by zero; actual
	 * buffered-mode sizing never reaches here.
	 */
	return 35000000 / (1000 * 10);
}

static size_t get_buffer_size(const struct sr_dev_inst *sdi)
{
	const struct dev_context *const devc = sdi->priv;

	/*
	 * Buffered (non-continuous) mode mirrors DSView's get_buffer_size()
	 * exactly: a flat 1MB, independent of channel count or samplerate.
	 * The previous channel-count-scaled heuristic here produced a
	 * buffer far smaller than what the device actually emits per burst
	 * for narrow channel counts (e.g. 3 of 32 enabled), causing a real
	 * USB-level overflow (confirmed via USB capture against real
	 * U3Pro32 hardware: one bulk completion landed with
	 * LIBUSB_TRANSFER_ERROR / EOVERFLOW at the exact byte count where
	 * the undersized buffer ran out), which then cascaded into
	 * persistent EPROTO errors on every following transfer. Wide
	 * channel counts (e.g. 32) happened to produce a large-enough
	 * buffer by coincidence under the old formula, which is why this
	 * only ever showed up for narrow channel selections.
	 */
	if (!devc->continuous_mode) {
		const size_t mb = 1024 * 1024;
		return (devc->usb_speed == LIBUSB_SPEED_SUPER)
			? (mb + 1023) & ~(size_t)1023
			: (mb + 511) & ~(size_t)511;
	}

	/*
	 * Streaming (continuous) mode: buffer should be large enough to
	 * hold 10ms of data and a multiple of the size of a data atom.
	 */
	{
		const size_t block_size = enabled_channel_count(sdi) * 512;
		const size_t s = 10 * to_bytes_per_ms(sdi);
		if (!block_size)
			return s;
		return ((s + block_size - 1) / block_size) * block_size;
	}
}

static unsigned int get_number_of_transfers(const struct sr_dev_inst *sdi)
{
	const struct dev_context *const devc = sdi->priv;
	unsigned int n;

	/* Buffered mode: DSView submits exactly one transfer (dsl.c). */
	if (!devc->continuous_mode)
		return 1;

	/* Streaming mode: total buffer size should hold about 100ms of data. */
	n = (100 * to_bytes_per_ms(sdi) + get_buffer_size(sdi) - 1) / get_buffer_size(sdi);
	return (n > NUM_SIMUL_TRANSFERS) ? NUM_SIMUL_TRANSFERS : n;
}

static unsigned int get_timeout(const struct sr_dev_inst *sdi)
{
	const struct dev_context *const devc = sdi->priv;
	size_t total_size;
	unsigned int timeout;

	/* Buffered mode: DSView uses a flat 20ms timeout (dsl.c), not a
	 * bandwidth-derived one. */
	if (!devc->continuous_mode)
		return 20;

	total_size = get_buffer_size(sdi) * get_number_of_transfers(sdi);
	timeout = total_size / to_bytes_per_ms(sdi);
	return timeout + timeout / 4; /* Leave a headroom of 25% percent. */
}

static int start_transfers(const struct sr_dev_inst *sdi)
{
	const size_t channel_count = enabled_channel_count(sdi);
	const size_t size = get_buffer_size(sdi);
	const unsigned int num_transfers = get_number_of_transfers(sdi);
	const unsigned int timeout = get_timeout(sdi);

	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct libusb_transfer *transfer;
	unsigned int i;
	int ret;
	unsigned char *buf;

	devc = sdi->priv;
	usb = sdi->conn;

	devc->sent_samples = 0;
	devc->acq_aborted = FALSE;
	devc->empty_transfer_count = 0;
	devc->submitted_transfers = 0;
	devc->wallclock_deadline_us = 0;

	g_free(devc->transfers);
	devc->transfers = g_try_malloc0(sizeof(*devc->transfers) * num_transfers);
	if (!devc->transfers) {
		sr_err("USB transfers malloc failed.");
		return SR_ERR_MALLOC;
	}

	/* 2 bytes/sample for <=16 enabled channels (every existing V1/V2
	 * device), 4 bytes for >16 (DSLOGIC_CAPS_CH32 profiles only). */
	devc->sample_unitsize = (channel_count > 16) ? 4 : 2;
	devc->deinterleave_buffer = g_try_malloc(DSLOGIC_ATOMIC_SAMPLES *
		(size / (channel_count * DSLOGIC_ATOMIC_BYTES)) * devc->sample_unitsize);
	if (!devc->deinterleave_buffer) {
		sr_err("Deinterleave buffer malloc failed.");
		g_free(devc->deinterleave_buffer);
		return SR_ERR_MALLOC;
	}

	devc->num_transfers = num_transfers;
	for (i = 0; i < num_transfers; i++) {
		if (!(buf = g_try_malloc(size))) {
			sr_err("USB transfer buffer malloc failed.");
			return SR_ERR_MALLOC;
		}
		transfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(transfer, usb->devhdl,
				6 | LIBUSB_ENDPOINT_IN, buf, size,
				receive_transfer, (void *)sdi, timeout);
		sr_info("submitting transfer: %d", i);
		if ((ret = libusb_submit_transfer(transfer)) != 0) {
			sr_err("Failed to submit transfer: %s.",
			       libusb_error_name(ret));
			libusb_free_transfer(transfer);
			g_free(buf);
			abort_acquisition(devc);
			return SR_ERR;
		}
		devc->transfers[i] = transfer;
		devc->submitted_transfers++;
	}

	/*
	 * NOTE: DF_HEADER is emitted exactly once per session by
	 * dslogic_acquisition_start, not here. start_transfers is re-entered
	 * on every chunk_loop re-arm via trigger_receive; emitting DF_HEADER
	 * here would tell sigrok-cli's session loop to call
	 * setup_output_format again, which creates a fresh srzip output
	 * context with zip_created=FALSE, and the next DF_LOGIC then
	 * unlinks the .sr file and starts a new archive - silently losing
	 * all data from prior chunks.
	 */

	return SR_OK;
}

static void LIBUSB_CALL trigger_receive(struct libusb_transfer *transfer)
{
	const struct sr_dev_inst *sdi;
	struct dslogic_trigger_pos *tpos;
	struct dev_context *devc;

	sdi = transfer->user_data;
	devc = sdi->priv;
	if (transfer->status == LIBUSB_TRANSFER_CANCELLED) {
		sr_dbg("Trigger transfer canceled.");
		/* Terminate session. */
		std_session_send_df_end(sdi);
		usb_source_remove(sdi->session, devc->ctx);
		devc->num_transfers = 0;
		g_free(devc->transfers);
	} else if (transfer->status == LIBUSB_TRANSFER_COMPLETED
			&& transfer->actual_length == dslogic_header_size(devc)) {
		tpos = (struct dslogic_trigger_pos *)transfer->buffer;
		sr_info("tpos real_pos %d ram_saddr %d cnt_h %d cnt_l %d", tpos->real_pos,
			tpos->ram_saddr, tpos->remain_cnt_h, tpos->remain_cnt_l);
		devc->trigger_pos = tpos->real_pos;
		{
			/*
			 * In buffered (one-shot) mode with RLE the FPGA may have
			 * captured fewer samples than requested (compressed buffer
			 * exhausted). remain_cnt tells us by how much. Without this
			 * adjustment the acquisition never reaches sent_samples >=
			 * limit_samples and hangs. Matches DSView dsl.c
			 * receive_header.
			 *
			 * In streaming (continuous) mode remain_cnt is an in-flight
			 * "samples-remaining-to-send" counter that updates as the
			 * FPGA streams; subtracting it from limit_samples gives a
			 * meaningless tiny number that would stop the acquisition
			 * almost immediately. Skip the shortening entirely for
			 * streaming and let limit_samples be the stop budget.
			 */
			/* Per-chunk budget when chunk_loop is active. */
			uint64_t per_chunk = (devc->chunk_loop && devc->chunk_samples)
				? devc->chunk_samples : devc->limit_samples;

			if (!devc->continuous_mode) {
				uint64_t remain = ((uint64_t)tpos->remain_cnt_h << 32)
					| (uint64_t)tpos->remain_cnt_l;
				if (per_chunk && remain < per_chunk)
					devc->actual_samples = per_chunk - remain;
				else
					devc->actual_samples = per_chunk;
				if (devc->actual_samples != per_chunk)
					sr_info("RLE shortened capture: %" PRIu64 " of %" PRIu64 " samples",
						devc->actual_samples, per_chunk);
			} else {
				devc->actual_samples = per_chunk;
			}
		}
		g_free(tpos);
		start_transfers(sdi);
	}
	libusb_free_transfer(transfer);
}

SR_PRIV int dslogic_acquisition_start(const struct sr_dev_inst *sdi)
{
	const unsigned int timeout = get_timeout(sdi);

	struct sr_dev_driver *di;
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct dslogic_trigger_pos *tpos;
	struct libusb_transfer *transfer;
	int ret;

	di = sdi->driver;
	drvc = di->context;
	devc = sdi->priv;
	usb = sdi->conn;

	devc->ctx = drvc->sr_ctx;
	devc->sent_samples = 0;
	devc->actual_samples = 0;
	devc->empty_transfer_count = 0;
	devc->acq_aborted = FALSE;
	devc->rearm_pending = FALSE;
	devc->wallclock_deadline_us = 0;

	/*
	 * chunk_loop: each chunk captures chunk_samples (default ~500ms of
	 * samples) and then re-arms. The overall session ends after
	 * total_deadline_us, derived from --time / limit_samples + a 10%
	 * grace for drain. When chunk_loop is off, chunk_samples = 0 and
	 * the existing single-shot semantics apply.
	 */
	if (devc->chunk_loop && devc->cur_samplerate && devc->limit_samples) {
		uint64_t default_chunk = devc->cur_samplerate / 2;  /* 500ms */
		if (default_chunk == 0)
			default_chunk = devc->cur_samplerate;
		devc->chunk_samples = devc->limit_samples < default_chunk
			? devc->limit_samples : default_chunk;
		devc->total_deadline_us = g_get_monotonic_time() +
			(gint64)((double)devc->limit_samples /
				 (double)devc->cur_samplerate * 1e6 * 1.1);
		sr_info("chunk_loop: per-chunk %" PRIu64 " samples "
			"(%.2f s), total deadline +%.2f s",
			devc->chunk_samples,
			(double)devc->chunk_samples / devc->cur_samplerate,
			((double)devc->total_deadline_us -
			 g_get_monotonic_time()) / 1e6);
	} else {
		devc->chunk_samples = 0;
		devc->total_deadline_us = 0;
	}

	usb_source_add(sdi->session, devc->ctx, timeout, receive_data, drvc);

	/*
	 * Emit DF_HEADER once per session, BEFORE the first arm. Used to
	 * live at the bottom of start_transfers, but in chunk_loop mode
	 * start_transfers runs per re-arm and re-emitted DF_HEADER caused
	 * sigrok-cli to re-initialize its output module, which made srzip
	 * unlink and recreate the .sr file each chunk - silently dropping
	 * all prior chunks' data.
	 */
	std_session_send_df_header(sdi);

	/* Stop any prior acquisition, then arm and start. Matches DSView's order
	 * at dslogic.c (STOP -> arm -> submit header-read transfer -> START):
	 * DSView's dsl_start_transfers() submits the EP6 IN read(s) BEFORE
	 * the DSL_CTL_START write, not after. The device fires its one-shot
	 * trigger-position header packet as soon as it finishes capturing,
	 * with no retry if nobody was listening; for a short/fast capture
	 * (e.g. 1000 samples at 1 MHz = ~1ms), the device can finish and
	 * send that packet before we'd otherwise get around to posting the
	 * read URB, permanently hanging the acquisition. Posting the read
	 * before START closes that race. Confirmed against real U3Pro32
	 * hardware: this exact ordering bug reproduced a hang after "Arm
	 * FPGA done." with no further progress. */
	if ((ret = devc->ops->acquisition_stop(sdi)) != SR_OK)
		return ret;

	if ((ret = devc->ops->fpga_config(sdi)) != SR_OK)
		return ret;

	sr_dbg("Getting trigger.");
	tpos = g_malloc0(dslogic_header_size(devc));
	transfer = libusb_alloc_transfer(0);
	libusb_fill_bulk_transfer(transfer, usb->devhdl, 6 | LIBUSB_ENDPOINT_IN,
			(unsigned char *)tpos, dslogic_header_size(devc),
			trigger_receive, (void *)sdi, 0);
	if ((ret = libusb_submit_transfer(transfer)) < 0) {
		sr_err("Failed to request trigger: %s.", libusb_error_name(ret));
		libusb_free_transfer(transfer);
		g_free(tpos);
		return SR_ERR;
	}

	devc->transfers = g_try_malloc0(sizeof(*devc->transfers));
	if (!devc->transfers) {
		sr_err("USB trigger_pos transfer malloc failed.");
		return SR_ERR_MALLOC;
	}
	devc->num_transfers = 1;
	devc->submitted_transfers++;
	devc->transfers[0] = transfer;

	/*
	 * Only now, with the header-read URB already posted, tell the FPGA
	 * to go. See the comment above acquisition_stop()/fpga_config() for
	 * why this must come after, not before, the transfer submission.
	 */
	if ((ret = devc->ops->acquisition_start(sdi)) != SR_OK)
		return ret;

	return ret;
}

SR_PRIV int dslogic_acquisition_stop(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	devc->ops->acquisition_stop(sdi);
	abort_acquisition(sdi->priv);
	return SR_OK;
}
