/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
 * Copyright (C) 2013-2017 DreamSourceLab <support@dreamsourcelab.com>
 * Copyright (C) 2026 Larry Hernandez <l.gr@dartmouth.edu>
 *
 * V2 (envelope) protocol implementation. Wire format mirrors DSView
 * 1.3.2's libsigrok4DSL/hardware/DSL/command.c and dsl.c.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License v3.
 */

#include <config.h>
#include <assert.h>
#include <string.h>
#include <glib.h>
#include <libusb.h>
#include "protocol.h"
#include "protocol_v2.h"

#define V2_USB_TIMEOUT_MS 3000

SR_PRIV int command_ctl_wr_v2(libusb_device_handle *devhdl, struct ctl_wr_cmd cmd)
{
	int ret;

	assert(devhdl);

	ret = libusb_control_transfer(devhdl,
		LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
		CMD_CTL_WR, 0x0000, 0x0000,
		(unsigned char *)&cmd,
		cmd.header.size + sizeof(struct ctl_header),
		V2_USB_TIMEOUT_MS);
	if (ret < 0) {
		sr_err("CMD_CTL_WR failed (dest=%u offset=%u size=%u): %s",
			cmd.header.dest, cmd.header.offset, cmd.header.size,
			libusb_error_name(ret));
		return SR_ERR;
	}
	return SR_OK;
}

SR_PRIV int command_ctl_rd_v2(libusb_device_handle *devhdl, struct ctl_rd_cmd cmd)
{
	int ret;

	assert(devhdl);

	/* Phase 1: write the header to set up the read. */
	ret = libusb_control_transfer(devhdl,
		LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
		CMD_CTL_RD_PRE, 0x0000, 0x0000,
		(unsigned char *)&cmd, sizeof(struct ctl_header),
		V2_USB_TIMEOUT_MS);
	if (ret < 0) {
		sr_err("CMD_CTL_RD_PRE failed (dest=%u offset=%u size=%u): %s",
			cmd.header.dest, cmd.header.offset, cmd.header.size,
			libusb_error_name(ret));
		return SR_ERR;
	}

	g_usleep(10 * 1000);

	/* Phase 2: read the requested bytes. */
	ret = libusb_control_transfer(devhdl,
		LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
		CMD_CTL_RD, 0x0000, 0x0000,
		(unsigned char *)cmd.data, cmd.header.size,
		V2_USB_TIMEOUT_MS);
	if (ret < 0) {
		sr_err("CMD_CTL_RD failed: %s", libusb_error_name(ret));
		return SR_ERR;
	}
	return SR_OK;
}

SR_PRIV int dsl_wr_reg_v2(const struct sr_dev_inst *sdi, uint16_t addr, uint8_t value)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct ctl_wr_cmd wr_cmd;

	wr_cmd.header.dest = DSL_CTL_I2C_REG;
	wr_cmd.header.offset = addr;
	wr_cmd.header.size = 1;
	wr_cmd.data[0] = value;
	return command_ctl_wr_v2(usb->devhdl, wr_cmd);
}

SR_PRIV int dsl_rd_reg_v2(const struct sr_dev_inst *sdi, uint16_t addr, uint8_t *value)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct ctl_rd_cmd rd_cmd;

	rd_cmd.header.dest = DSL_CTL_I2C_STATUS;
	rd_cmd.header.offset = addr;
	rd_cmd.header.size = 1;
	rd_cmd.data = value;
	return command_ctl_rd_v2(usb->devhdl, rd_cmd);
}

SR_PRIV int dsl_rd_nvm_v2(const struct sr_dev_inst *sdi, uint8_t *buf, uint16_t addr, uint8_t len)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct ctl_rd_cmd rd_cmd;

	rd_cmd.header.dest = DSL_CTL_NVM;
	rd_cmd.header.offset = addr;
	rd_cmd.header.size = len;
	rd_cmd.data = buf;
	return command_ctl_rd_v2(usb->devhdl, rd_cmd);
}

/*
 * ADC clock-config table, mirrors DSView's adc_clk_init_500m (dsl.h).
 * Each row: {dest, cnt, delay_ms, byte[4]}. dsl_config_adc()'s loop
 * (dsl.c) sleeps delay_ms *before* writing this row's first `cnt`
 * bytes to `dest`, one byte per DSL_CTL_I2C_REG write, then moves to
 * the next row; a dest==0 row terminates the table.
 */
struct dslogic_adc_config {
	uint8_t dest;
	uint8_t cnt;
	uint8_t delay_ms;
	uint8_t byte[4];
};

static const struct dslogic_adc_config adc_clk_init_500m[] = {
	{ ADCC_ADDR + 2, 1, 0,  { 0x01, 0x00, 0x00, 0x00 } }, /* ADC clock power up */
	{ ADCC_ADDR,     4, 0,  { 0x01, 0x61, 0x00, 0x30 } },
	{ ADCC_ADDR,     4, 0,  { 0x01, 0x40, 0xf1, 0x46 } },
	{ ADCC_ADDR,     4, 10, { 0x01, 0x62, 0x3d, 0x40 } },
	{ 0, 0, 0, { 0, 0, 0, 0 } },
};

SR_PRIV int dslogic_config_adc_v2(const struct sr_dev_inst *sdi)
{
	const struct dslogic_adc_config *cfg;
	int i;

	for (cfg = adc_clk_init_500m; cfg->dest; cfg++) {
		if (cfg->delay_ms > 0)
			g_usleep(cfg->delay_ms * 1000);
		for (i = 0; i < cfg->cnt; i++)
			dsl_wr_reg_v2(sdi, cfg->dest, cfg->byte[i]);
	}
	return SR_OK;
}

SR_PRIV int dslogic_hdl_version_v2(const struct sr_dev_inst *sdi, uint8_t *value)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct ctl_rd_cmd rd_cmd;
	uint8_t rdata[HDL_VERSION_ADDR + 1];
	int ret;

	rd_cmd.header.dest   = DSL_CTL_I2C_STATUS;
	rd_cmd.header.offset = 0;
	rd_cmd.header.size   = HDL_VERSION_ADDR + 1;
	rd_cmd.data          = rdata;
	if ((ret = command_ctl_rd_v2(usb->devhdl, rd_cmd)) != SR_OK) {
		sr_err("Sent DSL_CTL_I2C_STATUS command failed.");
		return ret;
	}
	*value = rdata[HDL_VERSION_ADDR];
	return SR_OK;
}

SR_PRIV int dsl_wait_hw_status_bit_v2(libusb_device_handle *hdl, uint8_t bit_mask, gboolean want_set, unsigned timeout_ms)
{
	uint8_t status;
	struct ctl_rd_cmd rd_cmd;
	gint64 deadline_us = g_get_monotonic_time() + (gint64)timeout_ms * 1000;

	rd_cmd.header.dest = DSL_CTL_HW_STATUS;
	rd_cmd.header.offset = 0;
	rd_cmd.header.size = 1;
	rd_cmd.data = &status;

	for (;;) {
		if (command_ctl_rd_v2(hdl, rd_cmd) != SR_OK)
			return SR_ERR;
		if (want_set && (status & bit_mask))
			return SR_OK;
		if (!want_set && !(status & bit_mask))
			return SR_OK;
		if (g_get_monotonic_time() >= deadline_us) {
			sr_err("Timeout waiting for HW_STATUS bit 0x%02x (%s)",
				bit_mask, want_set ? "set" : "clear");
			return SR_ERR;
		}
		g_usleep(1000);
	}
}

/* Security challenge-response helpers (mirror DSView dsl.c). */

static int v2_secu_reset(const struct sr_dev_inst *sdi)
{
	if (dsl_wr_reg_v2(sdi, SEC_CTRL_ADDR, 0) != SR_OK) return SR_ERR;
	if (dsl_wr_reg_v2(sdi, SEC_CTRL_ADDR + 1, 0) != SR_OK) return SR_ERR;
	g_usleep(10 * 1000);
	if (dsl_wr_reg_v2(sdi, SEC_CTRL_ADDR, 1) != SR_OK) return SR_ERR;
	if (dsl_wr_reg_v2(sdi, SEC_CTRL_ADDR + 1, 0) != SR_OK) return SR_ERR;
	return SR_OK;
}

static int v2_secu_write(const struct sr_dev_inst *sdi, uint16_t cmd, uint16_t din)
{
	if (dsl_wr_reg_v2(sdi, SEC_DATA_ADDR,     din & 0xff) != SR_OK) return SR_ERR;
	if (dsl_wr_reg_v2(sdi, SEC_DATA_ADDR + 1, (din >> 8) & 0xff) != SR_OK) return SR_ERR;
	if (dsl_wr_reg_v2(sdi, SEC_CTRL_ADDR,     cmd & 0xff) != SR_OK) return SR_ERR;
	if (dsl_wr_reg_v2(sdi, SEC_CTRL_ADDR + 1, (cmd >> 8) & 0xff) != SR_OK) return SR_ERR;
	return SR_OK;
}

static gboolean v2_secu_is_ready(const struct sr_dev_inst *sdi)
{
	uint8_t t = 0;
	if (dsl_rd_reg_v2(sdi, SEC_CTRL_ADDR, &t) != SR_OK)
		return FALSE;
	return (t & bmSECU_READY) ? TRUE : FALSE;
}

static gboolean v2_secu_is_pass(const struct sr_dev_inst *sdi)
{
	uint8_t t = 0;
	if (dsl_rd_reg_v2(sdi, SEC_CTRL_ADDR, &t) != SR_OK)
		return FALSE;
	return (t & bmSECU_PASS) ? TRUE : FALSE;
}

static uint16_t v2_secu_read(const struct sr_dev_inst *sdi)
{
	uint8_t hi = 0, lo = 0;
	if (dsl_rd_reg_v2(sdi, SEC_DATA_ADDR + 1, &hi) != SR_OK) return 0;
	if (dsl_rd_reg_v2(sdi, SEC_DATA_ADDR,     &lo) != SR_OK) return 0;
	return ((uint16_t)hi << 8) | lo;
}

static int v2_security_check(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	uint16_t encryption[SECU_STEPS];
	int i;
	int try_cnt;

	/*
	 * Only the DSLogic Plus Pango revision (PID 0x0034) implements this
	 * anti-clone EEPROM challenge-response; U3Pro32 and any future
	 * DSLOGIC_CAPS_SECURITY-less V2 profile don't have the circuitry
	 * for it at all, and calling this against them times out waiting
	 * for a "ready" bit that will never be set (confirmed on real
	 * U3Pro32 hardware: "Security ready timeout at step 7").
	 */
	if (!(devc->profile->dev_caps & DSLOGIC_CAPS_SECURITY))
		return SR_OK;

	/* "Dessert clear" - DSView writes CTR0_ADDR=0x70 to 0 before the
	 * encryption read (dsl.c). Without this the FPGA can be left in
	 * a state where subsequent HW_STATUS reads stall. */
	if (dsl_wr_reg_v2(sdi, 0x70, 0x00) != SR_OK) {
		sr_err("Failed CTR0_ADDR dessert-clear.");
		return SR_ERR;
	}

	if (dsl_rd_nvm_v2(sdi, (uint8_t *)encryption, SECU_EEP_ADDR, SECU_STEPS * 2) != SR_OK) {
		sr_err("Failed to read encryption blob from device NVM at 0x%04x.", SECU_EEP_ADDR);
		return SR_ERR;
	}

	if (v2_secu_reset(sdi) != SR_OK) {
		sr_err("Security reset failed.");
		return SR_ERR;
	}

	if (v2_secu_is_pass(sdi)) {
		sr_err("Security state is already 'pass' before challenge; rejected.");
		return SR_ERR;
	}

	if (v2_secu_write(sdi, SECU_START, 0) != SR_OK) {
		sr_err("Security start command failed.");
		return SR_ERR;
	}

	/* Step counts down from SECU_STEPS-1 to 0, mirrors DSView dsl.c. */
	for (i = SECU_STEPS - 1; i >= 0; i--) {
		if (v2_secu_is_pass(sdi)) {
			sr_err("Security passed prematurely at step %d.", i);
			return SR_ERR;
		}
		try_cnt = SECU_TRY_CNT;
		while (!v2_secu_is_ready(sdi)) {
			if (try_cnt-- == 0) {
				sr_err("Security ready timeout at step %d.", i);
				return SR_ERR;
			}
		}
		if (v2_secu_read(sdi) != 0) {
			sr_err("Security read non-zero at step %d.", i);
			return SR_ERR;
		}
		if (v2_secu_write(sdi, SECU_CHECK, encryption[i]) != SR_OK) {
			sr_err("Security check write failed at step %d.", i);
			return SR_ERR;
		}
	}

	sr_info("Security check pass!");
	return SR_OK;
}

/* FPGA arm sequence (mirrors DSView dsl_fpga_arm at dsl.c). */

/*
 * V2 FPGA bitstream upload (mirrors DSView dsl_fpga_config at dsl.c).
 *
 * V1 uses a single DS_CMD_CONFIG (0xb3) control transfer + bulk write of the
 * bitstream. V2 firmware does not recognise 0xb3 and stalls. The V2 sequence:
 *   1) DSL_CTL_PROG_B := ~bmWR_PROG_B (PROG_B low)
 *   2) DSL_CTL_LED    := off
 *   3) DSL_CTL_PROG_B := bmWR_PROG_B  (PROG_B high)
 *   4) poll HW_STATUS until bmFPGA_INIT_B is set
 *   5) DSL_CTL_INTRDY := ~bmWR_INTRDY (INTRDY low)
 *   6) DSL_CTL_BULK_WR with 3-byte bitstream-size announce
 *   7) bulk transfer the bitstream on ep2 OUT
 *   8) DSL_CTL_INTRDY := bmWR_INTRDY  (INTRDY high → data end)
 *   9) poll HW_STATUS until bmGPIF_DONE is set
 */
static int v2_fpga_firmware_upload(const struct sr_dev_inst *sdi)
{
	struct drv_context *drvc = sdi->driver->context;
	struct sr_usb_dev_inst *usb = sdi->conn;
	libusb_device_handle *hdl = usb->devhdl;
	struct dev_context *devc = sdi->priv;
	struct sr_resource bitstream;
	struct ctl_wr_cmd wr;
	struct ctl_rd_cmd rd;
	unsigned char *buf;
	uint8_t hw_status = 0;
	int transferred;
	int result, ret;
	const char *name = NULL;

	if (!strcmp(devc->profile->model, "DSLogic Plus")) {
		name = "dreamsourcelab-dslogic-plus-fpga.fw";
	} else if (!strcmp(devc->profile->model, "DSLogic U3Pro32")) {
		name = "dreamsourcelab-dslogic-u3pro32-fpga.fw";
	} else {
		sr_err("v2: no FPGA firmware for model '%s'.", devc->profile->model);
		return SR_ERR;
	}

	/*
	 * If the FPGA is already configured (PulseView Stop+Run or quick
	 * sigrok-cli reopen on the same device), skip the bitstream upload
	 * and do the same dessert-clear write DSView does in its
	 * already-configured branch (dsl_dev_open at dsl.c). Re-running
	 * the full PROG_B cycle on a live FPGA wedges the post-INTRDY
	 * FPGA_DONE poll because the previous capture engine has not been
	 * torn down on the host side.
	 */
	rd.header.dest   = DSL_CTL_HW_STATUS;
	rd.header.offset = 0;
	rd.header.size   = 1;
	rd.data          = &hw_status;
	if (command_ctl_rd_v2(hdl, rd) == SR_OK && (hw_status & bmFPGA_DONE)) {
		uint8_t hdl_ver = 0;

		if (dsl_wr_reg_v2(sdi, CTR0_ADDR, 0) != SR_OK)
			sr_warn("CTR0_ADDR dessert-clear failed on warm path.");

		/*
		 * The FPGA reports itself configured, but that bitstream may
		 * be stale - left over from a different DSView/driver release
		 * than this one (bmFPGA_DONE only reflects "something valid
		 * is loaded", not "the right version is loaded"). DSView
		 * detects exactly this with the same read and refuses to
		 * proceed ("incorrect firmware, please replug" - confirmed a
		 * version mismatch, not a timeout: the read succeeds and
		 * returns promptly). We can do better: PROG_B is something we
		 * already drive ourselves below, so on a mismatch we just
		 * fall through to a real re-flash instead of trusting the
		 * stale bitstream or requiring the user to physically
		 * power-cycle the device.
		 */
		if (dslogic_hdl_version_v2(sdi, &hdl_ver) == SR_OK
				&& hdl_ver == DSL_HDL_VERSION) {
			sr_info("FPGA already configured (HW_STATUS=0x%02x, "
				"HDL version 0x%02x matches); skipping "
				"bitstream upload.", hw_status, hdl_ver);
			return SR_OK;
		}
		sr_warn("FPGA reports configured but HDL version is 0x%02x "
			"(expected 0x%02x) - stale bitstream from a "
			"different release. Forcing a re-flash instead of "
			"the usual warm-path skip.", hdl_ver, DSL_HDL_VERSION);
	}

	sr_dbg("Uploading FPGA bitstream '%s' via V2 envelope protocol.", name);
	if ((result = sr_resource_open(drvc->sr_ctx, &bitstream,
			SR_RESOURCE_FIRMWARE, name)) != SR_OK)
		return result;

	/* 1) PROG_B low */
	wr.header.dest = DSL_CTL_PROG_B;
	wr.header.offset = 0;
	wr.header.size = 1;
	wr.data[0] = (uint8_t)~bmWR_PROG_B;
	if ((ret = command_ctl_wr_v2(hdl, wr)) != SR_OK) goto fail;

	/* 2) LEDs off */
	wr.header.dest = DSL_CTL_LED;
	wr.header.size = 1;
	wr.data[0] = (uint8_t)(~bmLED_GREEN & ~bmLED_RED);
	if ((ret = command_ctl_wr_v2(hdl, wr)) != SR_OK) goto fail;

	/* 3) PROG_B high */
	wr.header.dest = DSL_CTL_PROG_B;
	wr.header.size = 1;
	wr.data[0] = bmWR_PROG_B;
	if ((ret = command_ctl_wr_v2(hdl, wr)) != SR_OK) goto fail;

	/* 4) Wait bmFPGA_INIT_B set */
	if ((ret = dsl_wait_hw_status_bit_v2(hdl, bmFPGA_INIT_B, TRUE, 3000)) != SR_OK)
		goto fail;

	/* 5) INTRDY low */
	wr.header.dest = DSL_CTL_INTRDY;
	wr.header.size = 1;
	wr.data[0] = (uint8_t)~bmWR_INTRDY;
	if ((ret = command_ctl_wr_v2(hdl, wr)) != SR_OK) goto fail;

	/* 6) BULK_WR announce: 3-byte file size */
	wr.header.dest = DSL_CTL_BULK_WR;
	wr.header.size = 3;
	wr.data[0] = (uint8_t)bitstream.size;
	wr.data[1] = (uint8_t)(bitstream.size >> 8);
	wr.data[2] = (uint8_t)(bitstream.size >> 16);
	if ((ret = command_ctl_wr_v2(hdl, wr)) != SR_OK) goto fail;

	/* 7) bulk-transfer the bitstream */
	buf = g_malloc(bitstream.size);
	if (!buf) { ret = SR_ERR; goto fail; }
	{
		uint64_t sum = 0;
		ssize_t chunk;
		while ((chunk = sr_resource_read(drvc->sr_ctx, &bitstream,
				buf + sum, bitstream.size - sum)) > 0)
			sum += chunk;
		if ((int64_t)sum != (int64_t)bitstream.size) {
			sr_err("v2 fpga: short read of bitstream (%" PRIu64 "/%" PRIu64 ").",
				sum, bitstream.size);
			g_free(buf);
			ret = SR_ERR;
			goto fail;
		}
	}
	ret = libusb_bulk_transfer(hdl, 2 | LIBUSB_ENDPOINT_OUT,
		buf, (int)bitstream.size, &transferred, V2_USB_TIMEOUT_MS);
	g_free(buf);
	if (ret < 0) {
		sr_err("v2 fpga: bitstream bulk write failed: %s", libusb_error_name(ret));
		ret = SR_ERR;
		goto fail;
	}
	if (transferred != (int)bitstream.size) {
		sr_err("v2 fpga: bitstream short transfer (%d/%" PRIu64 ").",
			transferred, bitstream.size);
		ret = SR_ERR;
		goto fail;
	}

	/* 8) INTRDY high (signal data end) */
	wr.header.dest = DSL_CTL_INTRDY;
	wr.header.size = 1;
	wr.data[0] = bmWR_INTRDY;
	if ((ret = command_ctl_wr_v2(hdl, wr)) != SR_OK) goto fail;

	/* 9) Wait GPIF_DONE */
	if ((ret = dsl_wait_hw_status_bit_v2(hdl, bmGPIF_DONE, TRUE, 3000)) != SR_OK)
		goto fail;

	/* 10) INTRDY low (dsl.c) */
	wr.header.dest = DSL_CTL_INTRDY;
	wr.header.size = 1;
	wr.data[0] = (uint8_t)~bmWR_INTRDY;
	if ((ret = command_ctl_wr_v2(hdl, wr)) != SR_OK) goto fail;

	/* 11) Wait FPGA_DONE - confirms the FPGA bitstream is live (dsl.c) */
	if ((ret = dsl_wait_hw_status_bit_v2(hdl, bmFPGA_DONE, TRUE, 3000)) != SR_OK)
		goto fail;

	/* 12) Turn on the green LED (dsl.c) */
	wr.header.dest = DSL_CTL_LED;
	wr.header.size = 1;
	wr.data[0] = bmLED_GREEN;
	if ((ret = command_ctl_wr_v2(hdl, wr)) != SR_OK) goto fail;

	/*
	 * 13) Re-assert WORDWIDE high (dsl.c) - FX2/HighSpeed-only GPIF
	 * 16-bit-wide mode setting. Skipped when actually running at USB3
	 * SuperSpeed: DSL fork's dsl_fpga_arm() never issues this control
	 * write when usb_speed == LIBUSB_SPEED_SUPER (dsl.c:1211-1219) - the
	 * GPIF word-width concept doesn't apply to the FX3 SuperSpeed
	 * datapath. Gate on the actually-negotiated link speed, not just
	 * DSLOGIC_CAPS_USB30: a USB3-capable device plugged into a USB2
	 * port falls back to HighSpeed and still needs this write.
	 */
	if (devc->usb_speed != LIBUSB_SPEED_SUPER) {
		wr.header.dest = DSL_CTL_WORDWIDE;
		wr.header.size = 1;
		wr.data[0] = bmWR_WORDWIDE;
		if ((ret = command_ctl_wr_v2(hdl, wr)) != SR_OK) goto fail;
	}

	sr_info("FPGA configure done: %" PRIu64 " bytes.", bitstream.size);
	sr_resource_close(drvc->sr_ctx, &bitstream);
	return SR_OK;

fail:
	sr_resource_close(drvc->sr_ctx, &bitstream);
	return SR_ERR;
}

/*
 * Integer ceiling division helper - avoids pulling in <math.h>.
 * Returns ceil(a / b) as uint32_t.
 */
static inline uint32_t div_round_up(uint64_t a, uint64_t b)
{
	return (uint32_t)((a + b - 1) / b);
}

/*
 * Channel-mode table for the DSLogic Plus family (PIDs 0x0020, 0x0034).
 * Values mirror DSView's channel_modes[] entries for DSL_BUFFER100x16,
 * DSL_BUFFER200x8, DSL_BUFFER400x4, DSL_STREAM20x16, DSL_STREAM25x12,
 * DSL_STREAM50x6, DSL_STREAM100x3.
 */
static const struct dslogic_channel_mode dslogic_plus_modes[] = {
	/* id    stream  ch  min_sr      max_sr      hw_max     pre  descr */
	{   0,   FALSE,  16, SR_KHZ(50), SR_MHZ(100), SR_MHZ(100), 1,
		"16 channels, buffered (max 100 MHz)" },
	{   1,   FALSE,   8, SR_KHZ(50), SR_MHZ(200), SR_MHZ(100), 1,
		"8 channels, buffered (max 200 MHz)" },
	{   2,   FALSE,   4, SR_KHZ(50), SR_MHZ(400), SR_MHZ(100), 1,
		"4 channels, buffered (max 400 MHz)" },
	{   3,   TRUE,   16, SR_KHZ(50), SR_MHZ(20),  SR_MHZ(100), 1,
		"16 channels, streaming (max 20 MHz)" },
	{   4,   TRUE,   12, SR_KHZ(50), SR_MHZ(25),  SR_MHZ(100), 1,
		"12 channels, streaming (max 25 MHz)" },
	{   5,   TRUE,    6, SR_KHZ(50), SR_MHZ(50),  SR_MHZ(100), 1,
		"6 channels, streaming (max 50 MHz)" },
	{   6,   TRUE,    3, SR_KHZ(50), SR_MHZ(100), SR_MHZ(100), 1,
		"3 channels, streaming (max 100 MHz)" },
};

#define DSLOGIC_PLUS_DEFAULT_CH_MODE_ID 0   /* matches DSView's DSL_BUFFER100x16 */

/*
 * Channel-mode tables for DSLogic U3Pro32 (PID 0x002c). Buffered-mode
 * entries (readout happens after capture completes, from onboard RAM)
 * are link-speed independent and identical in both tables. Streaming
 * entries (readout happens live, bounded by USB throughput) differ
 * sharply: the USB2 HighSpeed fallback link only sustains the slow
 * "_3DN2" family, while the USB3 SuperSpeed link reaches far higher
 * rates. Values and hw_max_samplerate/pre_div mirror DSView's
 * channel_modes[] rows DSL_STREAM*_32_3DN2 / DSL_STREAM50x32 /
 * DSL_STREAM100x30 / DSL_STREAM250x12 / DSL_STREAM500x6 /
 * DSL_STREAM1000x3 / DSL_BUFFER250x32 / DSL_BUFFER500x16 /
 * DSL_BUFFER1000x8 (dsl.h).
 */
static const struct dslogic_channel_mode dslogic_u3pro32_hs_modes[] = {
	/* id  stream  ch  min_sr       max_sr       hw_max      pre  descr */
	{   0, FALSE,  32, SR_MHZ(1),   SR_MHZ(250), SR_MHZ(500), 5,
		"32 channels, buffered (max 250 MHz)" },
	{   1, FALSE,  16, SR_MHZ(1),   SR_MHZ(500), SR_MHZ(500), 5,
		"16 channels, buffered (max 500 MHz)" },
	{   2, FALSE,   8, SR_MHZ(1),   SR_GHZ(1),   SR_MHZ(500), 5,
		"8 channels, buffered (max 1 GHz)" },
	{   3, TRUE,   32, SR_KHZ(100), SR_MHZ(10),  SR_MHZ(500), 5,
		"32 channels, streaming (max 10 MHz)" },
	{   4, TRUE,   16, SR_KHZ(100), SR_MHZ(20),  SR_MHZ(500), 5,
		"16 channels, streaming (max 20 MHz)" },
	{   5, TRUE,   12, SR_KHZ(100), SR_MHZ(25),  SR_MHZ(500), 5,
		"12 channels, streaming (max 25 MHz)" },
	{   6, TRUE,    6, SR_KHZ(100), SR_MHZ(50),  SR_MHZ(500), 5,
		"6 channels, streaming (max 50 MHz)" },
	{   7, TRUE,    3, SR_KHZ(100), SR_MHZ(100), SR_MHZ(500), 5,
		"3 channels, streaming (max 100 MHz)" },
};

static const struct dslogic_channel_mode dslogic_u3pro32_ss_modes[] = {
	/* id  stream  ch  min_sr      max_sr       hw_max      pre  descr */
	{   0, FALSE,  32, SR_MHZ(1),  SR_MHZ(250), SR_MHZ(500), 5,
		"32 channels, buffered (max 250 MHz)" },
	{   1, FALSE,  16, SR_MHZ(1),  SR_MHZ(500), SR_MHZ(500), 5,
		"16 channels, buffered (max 500 MHz)" },
	{   2, FALSE,   8, SR_MHZ(1),  SR_GHZ(1),   SR_MHZ(500), 5,
		"8 channels, buffered (max 1 GHz)" },
	{   3, TRUE,   32, SR_MHZ(1),  SR_MHZ(50),  SR_MHZ(500), 5,
		"32 channels, streaming (max 50 MHz)" },
	{   4, TRUE,   30, SR_MHZ(1),  SR_MHZ(100), SR_MHZ(500), 5,
		"30 channels, streaming (max 100 MHz)" },
	{   5, TRUE,   12, SR_MHZ(1),  SR_MHZ(250), SR_MHZ(500), 5,
		"12 channels, streaming (max 250 MHz)" },
	{   6, TRUE,    6, SR_MHZ(1),  SR_MHZ(500), SR_MHZ(500), 5,
		"6 channels, streaming (max 500 MHz)" },
	{   7, TRUE,    3, SR_MHZ(1),  SR_GHZ(1),   SR_MHZ(500), 5,
		"3 channels, streaming (max 1 GHz)" },
};

#define DSLOGIC_U3PRO32_DEFAULT_CH_MODE_ID 0   /* matches DSView's DSL_BUFFER250x32 */

static const struct dslogic_channel_mode *dslogic_plus_channel_modes(size_t *count)
{
	if (count)
		*count = ARRAY_SIZE(dslogic_plus_modes);
	return dslogic_plus_modes;
}

/*
 * U3Pro32's streaming table depends on the negotiated USB link speed
 * (devc->usb_speed, queried once at dev_open). Falls back to the HS
 * (slower) table if speed isn't known yet, so early callers (before
 * dev_open has run) get a conservative answer rather than overclaiming.
 */
static const struct dslogic_channel_mode *dslogic_u3pro32_modes_table(
		const struct dev_context *devc, size_t *count)
{
	if (devc->usb_speed == LIBUSB_SPEED_SUPER) {
		if (count)
			*count = ARRAY_SIZE(dslogic_u3pro32_ss_modes);
		return dslogic_u3pro32_ss_modes;
	}
	if (count)
		*count = ARRAY_SIZE(dslogic_u3pro32_hs_modes);
	return dslogic_u3pro32_hs_modes;
}

SR_PRIV const struct dslogic_channel_mode *dslogic_channel_modes(
		const struct dev_context *devc, size_t *count)
{
	if (devc->profile->dev_caps & DSLOGIC_CAPS_CH32)
		return dslogic_u3pro32_modes_table(devc, count);
	return dslogic_plus_channel_modes(count);
}

SR_PRIV const struct dslogic_channel_mode *dslogic_channel_mode_default(
		const struct dev_context *devc)
{
	size_t count;
	const struct dslogic_channel_mode *modes = dslogic_channel_modes(devc, &count);
	uint8_t default_id = (devc->profile->dev_caps & DSLOGIC_CAPS_CH32) ?
		DSLOGIC_U3PRO32_DEFAULT_CH_MODE_ID : DSLOGIC_PLUS_DEFAULT_CH_MODE_ID;
	size_t i;

	for (i = 0; i < count; i++)
		if (modes[i].id == default_id)
			return &modes[i];
	return &modes[0];
}

SR_PRIV const struct dslogic_channel_mode *dslogic_channel_mode_by_id(
		const struct dev_context *devc, uint8_t id)
{
	size_t i, count;
	const struct dslogic_channel_mode *modes = dslogic_channel_modes(devc, &count);

	for (i = 0; i < count; i++)
		if (modes[i].id == id)
			return &modes[i];
	return NULL;
}

static const struct dslogic_channel_mode *v2_current_channel_mode(const struct dev_context *devc)
{
	const struct dslogic_channel_mode *m;

	m = dslogic_channel_mode_by_id(devc, devc->ch_mode_id);
	return m ? m : dslogic_channel_mode_default(devc);
}

/*
 * Pick the channel mode that fits cur_samplerate under continuous_mode.
 * Buffered mode trades samplerate for channel count: 16/8/4 ch at
 * 100/200/400 MHz. Streaming mode caps at 100 MHz and trades the same
 * way: 16/12/6/3 ch at 20/25/50/100 MHz. The smallest mode whose
 * max_samplerate covers the requested rate wins.
 */
/*
 * Pick the channel mode that fits the requested samplerate and the
 * smallest channel-count >= `need_channels` (max-enabled-channel-index+1)
 * under continuous_mode. Smaller channel counts use less USB bandwidth,
 * letting higher sample rates fit USB 2.0 HS's ~50 MB/s ceiling.
 */
SR_PRIV uint8_t dslogic_auto_pick_mode_id(const struct dev_context *devc,
		uint64_t samplerate, gboolean continuous, gboolean rle,
		unsigned int need_channels)
{
	size_t i, n;
	const struct dslogic_channel_mode *modes = dslogic_channel_modes(devc, &n);
	const struct dslogic_channel_mode *best = NULL;
	/*
	 * stream + RLE relaxes the per-mode max_samplerate cap. Each mode's
	 * cap reflects the FPGA's RAW (uncompressed) bandwidth; with RLE
	 * the FPGA emits compressed pairs over USB, so sparse traffic (e.g.
	 * intermittent SPI bursts) easily fits 50 MB/s even when the raw
	 * sample-rate × channel-count would not. Pick the smallest-channel
	 * mode that holds need_channels and let the requested samplerate
	 * be driven via the divider. Dense traffic can still abort with
	 * "Device only sent N samples"; user is expected to know their bus.
	 */
	gboolean ignore_max_sr = continuous && rle;

	if (need_channels == 0)
		need_channels = 1;

	for (i = 0; i < n; i++) {
		if (modes[i].stream != continuous)
			continue;
		if (modes[i].num_channels < need_channels)
			continue;
		if (!ignore_max_sr && samplerate > modes[i].max_samplerate)
			continue;
		/*
		 * Prefer the mode with the SMALLEST num_channels that still
		 * fits — minimises USB bandwidth so high samplerates can
		 * stream cleanly.
		 */
		if (!best || modes[i].num_channels < best->num_channels)
			best = &modes[i];
	}
	if (best)
		return best->id;
	/* No exact fit: fall back to the mode with the most channels at
	 * the highest max_samplerate that satisfies need_channels. */
	for (i = 0; i < n; i++) {
		if (modes[i].stream != continuous)
			continue;
		if (modes[i].num_channels < need_channels)
			continue;
		if (!best || modes[i].max_samplerate > best->max_samplerate)
			best = &modes[i];
	}
	if (best)
		return best->id;
	return dslogic_channel_mode_default(devc)->id;
}

/*
 * Build the struct DSL_setting that is bulk-written to the FPGA.
 *
 * Header field encoding: (register_index << 8) | word_count
 *   mirrors dsl.c.
 *
 * Samplerate divider uses hw_max_samplerate = 500 MHz and pre_div = 5,
 *   which are the correct values for the DSLogic Plus pgl12 16-channel
 *   mode (DSL_STREAM20x16_3DN2) as confirmed in dsl.h.
 *
 * Sample count is shifted right by 4 because the FPGA's minimum unit
 *   is 16 samples (dsl.c: "hardware minimum unit 64" [sic; actually
 *   16 because >>4 == /16]).
 */
/* Forward decl: defined later, used in v2_build_default_setting. */
static unsigned int v2_max_enabled_plus_one(const struct sr_dev_inst *sdi);

/*
 * Encode the libsigrok session trigger (if any) into the DSL_setting
 * trig_* fields and return the number of trigger stages (0 = no trigger).
 *
 * Field semantics (mirrors DSView trigger.c bit packing):
 *   trig_mask  bit i = 1 -> "don't care" or edge-trigger on channel i
 *   trig_value bit i   = expected level when mask bit is 0
 *   trig_edge  bit i = 1 -> require edge transition on channel i
 *   trig_logic   = (stage_logic << 1) + invert    [stage active]
 *                = 2 ("always true")             [unused stages]
 *
 * SR match types map per channel as:
 *   ZERO    -> mask=0,  value=0, edge=0
 *   ONE     -> mask=0,  value=1, edge=0
 *   FALLING -> mask=0,  value=0, edge=1
 *   RISING  -> mask=0,  value=1, edge=1
 *   EDGE    -> mask=1,  value=0, edge=1   (either edge)
 *
 * Mirroring trig_mask0/value0/edge0 to trig_mask1/value1/edge1 means
 * the same condition must hold for both halves of the FPGA's
 * comparator network; that's what DSView SIMPLE_TRIGGER does.
 */
static int v2_encode_trigger(const struct sr_dev_inst *sdi,
			     struct DSL_setting *s)
{
	struct sr_trigger *trigger;
	struct sr_trigger_stage *stage;
	struct sr_trigger_match *match;
	const GSList *l, *m;
	int num_stages = 0;
	int i;

	for (i = 0; i < NUM_TRIGGER_STAGES; i++) {
		s->trig_mask0[i]  = 0xffff;
		s->trig_mask1[i]  = 0xffff;
		s->trig_value0[i] = 0;
		s->trig_value1[i] = 0;
		s->trig_edge0[i]  = 0;
		s->trig_edge1[i]  = 0;
		s->trig_logic0[i] = 2;
		s->trig_logic1[i] = 2;
		s->trig_count[i]  = 0;
	}

	if (!(trigger = sr_session_trigger_get(sdi->session)))
		return 0;

	for (l = trigger->stages; l; l = l->next) {
		stage = l->data;
		num_stages++;
		for (m = stage->matches; m; m = m->next) {
			uint16_t bit;
			match = m->data;
			if (!match->channel->enabled)
				continue;
			if (match->channel->index >= 16)
				continue;
			bit = (uint16_t)(1U << match->channel->index);
			switch (match->match) {
			case SR_TRIGGER_ONE:
				s->trig_mask0[0]  = (uint16_t)(s->trig_mask0[0]  & ~bit);
				s->trig_mask1[0]  = (uint16_t)(s->trig_mask1[0]  & ~bit);
				s->trig_value0[0] = (uint16_t)(s->trig_value0[0] |  bit);
				s->trig_value1[0] = (uint16_t)(s->trig_value1[0] |  bit);
				break;
			case SR_TRIGGER_ZERO:
				s->trig_mask0[0]  = (uint16_t)(s->trig_mask0[0]  & ~bit);
				s->trig_mask1[0]  = (uint16_t)(s->trig_mask1[0]  & ~bit);
				break;
			case SR_TRIGGER_FALLING:
				s->trig_mask0[0]  = (uint16_t)(s->trig_mask0[0]  & ~bit);
				s->trig_mask1[0]  = (uint16_t)(s->trig_mask1[0]  & ~bit);
				s->trig_edge0[0]  = (uint16_t)(s->trig_edge0[0]  |  bit);
				s->trig_edge1[0]  = (uint16_t)(s->trig_edge1[0]  |  bit);
				break;
			case SR_TRIGGER_RISING:
				s->trig_mask0[0]  = (uint16_t)(s->trig_mask0[0]  & ~bit);
				s->trig_mask1[0]  = (uint16_t)(s->trig_mask1[0]  & ~bit);
				s->trig_value0[0] = (uint16_t)(s->trig_value0[0] |  bit);
				s->trig_value1[0] = (uint16_t)(s->trig_value1[0] |  bit);
				s->trig_edge0[0]  = (uint16_t)(s->trig_edge0[0]  |  bit);
				s->trig_edge1[0]  = (uint16_t)(s->trig_edge1[0]  |  bit);
				break;
			case SR_TRIGGER_EDGE:
				s->trig_edge0[0]  = (uint16_t)(s->trig_edge0[0]  |  bit);
				s->trig_edge1[0]  = (uint16_t)(s->trig_edge1[0]  |  bit);
				break;
			default:
				break;
			}
		}
	}

	if (num_stages > 0) {
		/* Active stage uses AND-of-conditions, non-inverted. */
		s->trig_logic0[0] = 0;
		s->trig_logic1[0] = 0;
		/*
		 * Enable the FPGA's trigger comparator. Without this bit the
		 * trig_* fields are ignored and the capture starts immediately
		 * on arm. (DSView dsl.c:1060 packs trigger_en into this bit.)
		 */
		s->mode |= (uint16_t)(1U << DS_MODE_TRIG_EN_BIT);
	}

	return num_stages;
}

static void v2_build_default_setting(const struct sr_dev_inst *sdi,
				     struct DSL_setting *s)
{
	struct dev_context *devc = sdi->priv;
	const struct dslogic_channel_mode *cm;
	uint32_t tmp_u32;
	uint64_t cur_sr;
	uint64_t count_units;
	uint32_t ch_en_mask;

	/*
	 * Re-pick the channel mode here so it sees the FINAL enabled-channel
	 * mask. sigrok-cli's -C and PulseView's channel checkboxes may
	 * disable channels AFTER config_set has run for samplerate /
	 * continuous, so the auto-pick at set-time may have used a stale
	 * channel count.
	 */
	devc->ch_mode_id = dslogic_auto_pick_mode_id(devc,
		devc->cur_samplerate, devc->continuous_mode, devc->rle_mode,
		v2_max_enabled_plus_one(sdi));
	cm = v2_current_channel_mode(devc);
	sr_dbg("Arm: picked mode id=%u (%s); cur_samplerate=%" PRIu64
	       " Hz, continuous=%s, rle=%s, need_ch=%u",
	       (unsigned)devc->ch_mode_id, cm->descr,
	       devc->cur_samplerate,
	       devc->continuous_mode ? "on" : "off",
	       devc->rle_mode ? "on" : "off",
	       v2_max_enabled_plus_one(sdi));

	memset(s, 0, sizeof(*s));

	/* Sync markers (dsl.c, 1046). */
	s->sync     = DSL_SETTING_SYNC;
	s->end_sync = DSL_SETTING_END_SYNC;

	/* Header values encode (register_index << 8) | word_count. */
	s->mode_header      = 0x0001;   /* reg 0,    1 word  */
	s->divider_header   = 0x0102;   /* reg 1,    2 words */
	s->count_header     = 0x0302;   /* reg 3,    2 words */
	s->trig_pos_header  = 0x0502;   /* reg 5,    2 words */
	s->trig_glb_header  = 0x0701;   /* reg 7,    1 word  */
	s->dso_count_header = 0x0802;   /* reg 8,    2 words */
	s->ch_en_header     = 0x0a02;   /* reg 0xa,  2 words */
	s->fgain_header     = 0x0c01;   /* reg 0xc,  1 word  */
	s->trig_header      = 0x40a0;   /* reg 0x40, 0xa0 words */

	/*
	 * mode bitfield. Mirrors DSView dsl.c (LOGIC mode branch):
	 *   bit 1  CLK_TYPE   - external clock if set
	 *   bit 2  CLK_EDGE   - falling edge if set
	 *   bit 3  RLE_MODE   - run-length encoding
	 *   bit 8  FILTER     - 1T glitch filter
	 *   bit 12 STREAM_MODE - streaming vs buffered
	 * The DSLogic Plus has no DSO/ANALOG/HALF/QUAR modes so those bits
	 * stay zero. Trigger bits stay zero until trigger support lands.
	 */
	s->mode = 0;
	if (cm->stream)
		s->mode |= (1 << DS_MODE_STREAM_MODE_BIT);
	if (devc->external_clock)
		s->mode |= (1 << DS_MODE_CLK_TYPE_BIT);
	if (devc->clock_edge == DS_EDGE_FALLING)
		s->mode |= (1 << DS_MODE_CLK_EDGE_BIT);
	if (devc->rle_mode)
		s->mode |= (1 << DS_MODE_RLE_MODE_BIT);
	if (devc->filter)
		s->mode |= (1 << DS_MODE_FILTER_BIT);

	/*
	 * Samplerate divider (dsl.c, LOGIC mode branch).
	 *   tmp_u32 = ceil(hw_max / cur_sr)
	 *   div_h   = ((tmp_u32 >= pre_div) ? (pre_div-1) : (tmp_u32-1)) << 8
	 *   tmp_u32 = ceil(tmp_u32 / pre_div)
	 *   div_l   = tmp_u32 & 0xffff
	 *   div_h  += tmp_u32 >> 16
	 */
	cur_sr = devc->cur_samplerate ? devc->cur_samplerate : SR_MHZ(1);
	/*
	 * Clamp to the active mode's advertised max_samplerate. The FPGA
	 * will otherwise accept a higher rate via the divider but the
	 * USB IN endpoint can't sustain it, leading to empty-transfer
	 * abort ("Device only sent N samples") a few seconds into
	 * acquisition. Warn loudly so the user knows to either drop a
	 * channel (to unlock a higher-rate stream mode) or switch off
	 * continuous mode.
	 */
	if (cur_sr > cm->max_samplerate) {
		if (devc->rle_mode && devc->continuous_mode) {
			/*
			 * Stream+RLE: the FPGA emits RLE-compressed pairs over
			 * USB, so the raw-bandwidth ceiling that defines
			 * max_samplerate doesn't apply directly. Run at the
			 * requested rate; dense traffic may still abort the
			 * capture early ("Device only sent N samples") if
			 * compressed throughput exceeds USB 2.0 HS's ceiling.
			 */
			sr_info("Stream+RLE: running at %" PRIu64 " Hz on %s "
				"(raw max %" PRIu64 " Hz). USB throughput is "
				"bounded by signal density via RLE; dense "
				"traffic can abort early.",
				cur_sr, cm->descr, cm->max_samplerate);
		} else {
			sr_warn("Requested samplerate %" PRIu64 " Hz exceeds "
				"%s's max of %" PRIu64 " Hz; clamping. "
				"Drop a channel for a higher-rate mode, "
				"enable RLE for compressed streaming, or "
				"disable continuous mode.",
				cur_sr, cm->descr, cm->max_samplerate);
			cur_sr = cm->max_samplerate;
			devc->cur_samplerate = cur_sr;
		}
	}
	tmp_u32 = div_round_up(cm->hw_max_samplerate, cur_sr);
	s->div_h = ((tmp_u32 >= (uint32_t)cm->pre_div) ?
		(uint32_t)(cm->pre_div - 1U) : (tmp_u32 - 1U)) << 8;
	tmp_u32 = div_round_up(tmp_u32, cm->pre_div);
	s->div_l = tmp_u32 & 0x0000ffff;
	s->div_h = (uint16_t)(s->div_h + (tmp_u32 >> 16));

	/*
	 * Capture counter (dsl.c, LOGIC mode branch).
	 * The FPGA's minimum unit is 16 samples (>>4). When chunk_loop is
	 * active, count per-chunk samples instead of the whole session, so
	 * the FPGA stops at each chunk boundary and the driver re-arms.
	 */
	{
		uint64_t per_chunk = (devc->chunk_loop && devc->chunk_samples)
			? devc->chunk_samples : devc->limit_samples;
		count_units = per_chunk >> 4;
	}
	s->cnt_l = count_units & 0xffff;
	s->cnt_h = (count_units >> 16) & 0xffff;

	/*
	 * Trigger position. capture_ratio is a percentage of limit_samples
	 * that should sit BEFORE the trigger fires (pre-trigger memory).
	 * The FPGA buffer is divided in 64-sample atomic blocks; align to
	 * that. Clamp to 10% in streaming mode (DSView dsl.c:1101-1104)
	 * and 90% in buffered mode (DSL_MAX_TRIG_PERCENT, dsl.c:1104).
	 */
	{
		/*
		 * mem_depth is 2 GiB (2^31) for U3Pro32: mem_depth * 90
		 * alone is ~193e9, which overflows a 32-bit intermediate
		 * (wraps to exactly 0 for this specific mem_depth, silently
		 * zeroing tpos_l/tpos_h for every buffered capture). Keep
		 * the whole computation in 64 bits and only narrow to
		 * uint32_t at the very end, where the value is already
		 * known to fit (it's bounded by mem_depth, itself way under
		 * UINT32_MAX in sample units).
		 */
		uint64_t mem_depth = devc->profile->mem_depth;
		uint64_t tpos = (devc->capture_ratio * devc->limit_samples) / 100U;
		uint64_t cap = devc->continuous_mode ? (mem_depth * 10U / 100U)
						     : (mem_depth * 90U / 100U);
		if (tpos < 64U)
			tpos = 64U;
		if (tpos > cap)
			tpos = cap;
		tpos &= ~(uint64_t)63U;   /* align down to 64-sample boundary */
		s->tpos_l = (uint16_t)(tpos & 0xffff);
		s->tpos_h = (uint16_t)((tpos >> 16) & 0xffff);
	}

	/* dso_cnt = 0 (unused in logic mode). */
	s->dso_cnt_l = 0;
	s->dso_cnt_h = 0;

	/*
	 * Channel enable mask = sigrok's enabled channels AND the active
	 * mode's capability cap. This drops bandwidth on the FPGA->host
	 * link to only what the user asked to see, letting high samplerates
	 * fit USB 2.0 HS's ~50 MB/s ceiling. If no channels are enabled
	 * (degenerate), fall back to ch0.
	 */
	{
		uint32_t cap_mask;
		/*
		 * enabled_channel_mask32() is safe to use unconditionally here
		 * (not just for DSLOGIC_CAPS_CH32 profiles): a ≤16ch device
		 * only ever creates ≤16 sr_channels, so the high bits are
		 * always 0 and this is equivalent to the old 16-bit mask.
		 */
		uint32_t user_mask = enabled_channel_mask32(sdi);
		if (cm->num_channels >= 32)
			cap_mask = 0xffffffffU;
		else
			cap_mask = (1U << cm->num_channels) - 1U;
		ch_en_mask = user_mask & cap_mask;
		if (ch_en_mask == 0)
			ch_en_mask = 1;
	}
	s->ch_en_l = (uint16_t)(ch_en_mask & 0xffffU);
	s->ch_en_h = (uint16_t)(ch_en_mask >> 16);

	/* fgain = 0 (no digital fine gain in logic mode). */
	s->fgain = 0;

	/*
	 * Trigger encoding. Reads sr_session_trigger_get() and packs
	 * channel-level conditions into stage 0; remaining stages are
	 * filled with "always true" defaults. trig_glb mirrors DSView
	 * dsl.c:1109 - upper 5 bits are enabled-channel count, low byte
	 * is the encoded stage count (0 = SIMPLE_TRIGGER single stage).
	 */
	{
		int num_stages = v2_encode_trigger(sdi, s);
		unsigned int ch_num = enabled_channel_count(sdi);
		unsigned int stage_field =
			(num_stages > 0) ? (unsigned int)(num_stages - 1) : 0U;
		s->trig_glb = (uint16_t)(((ch_num & 0x1fU) << 8) |
					 (stage_field & 0xffU));
	}
}

/*
 * Build the struct DSL_setting_ext32 second arm block, sent only for
 * DSLOGIC_CAPS_CH32 profiles (U3Pro32) immediately after struct
 * DSL_setting, carrying channels 16-31's trigger registers (DSL_setting
 * itself only covers channels 0-15). Mirrors DSView dsl_fpga_arm's
 * setting_ext32 construction (dsl.c:1021-1055, 1167-1172).
 *
 * Real multi-channel-group trigger support for channels 16-31 is out of
 * scope for this driver's current trigger model (v2_encode_trigger()
 * only ever encodes stage 0 against channels 0-15, same limitation the
 * DSLogic Plus V2 path already has). Every stage here is therefore
 * populated with the hardware's "always true" / don't-care default
 * (mask=0xffff, value=0, edge=0 - see v2_encode_trigger's doc comment
 * for the bit semantics), so an enabled channel in 16-31 never blocks
 * the trigger condition. A future patch adding real multi-channel
 * trigger support should extend both this and v2_encode_trigger() together.
 */
static void v2_build_default_setting_ext32(struct DSL_setting_ext32 *s)
{
	int i;

	memset(s, 0, sizeof(*s));
	s->sync        = DSL_SETTING_EXT32_SYNC;
	s->end_sync    = DSL_SETTING_EXT32_END_SYNC;
	s->trig_header = DSL_SETTING_EXT32_TRIG_HEADER;
	s->align_bytes = DSL_SETTING_EXT32_ALIGN_BYTES;

	for (i = 0; i < NUM_TRIGGER_STAGES; i++) {
		s->trig_mask0[i] = 0xffff;
		s->trig_mask1[i] = 0xffff;
		s->trig_value0[i] = 0;
		s->trig_value1[i] = 0;
		s->trig_edge0[i] = 0;
		s->trig_edge1[i] = 0;
	}
}

/*
 * Arm the FPGA by sending struct DSL_setting over bulk endpoint 2.
 *
 * Sequence (dsl.c):
 *   DSL_CTL_WORDWIDE → DSL_CTL_BULK_WR (3-byte word count) →
 *   poll bmSYS_CLR → bulk write setting blob on ep2 OUT →
 *   DSL_CTL_INTRDY → read HW_STATUS once, check bmGPIF_DONE.
 *
 * No DSL_CTL_STOP is issued here; DSView does not include it in the
 * arm sequence.
 */
static int v2_fpga_config(const struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	libusb_device_handle *hdl = usb->devhdl;
	struct dev_context *devc = sdi->priv;
	struct ctl_wr_cmd wr;
	struct ctl_rd_cmd rd;
	struct DSL_setting setting;
	struct DSL_setting_ext32 setting_ext32;
	uint32_t arm_size;
	uint8_t rd_data;
	int ret, transferred;

	/*
	 * 1) Set GPIF to word-wide (16-bit) mode (dsl.c). FX2/HighSpeed-only;
	 * skipped at USB3 SuperSpeed (see v2_fpga_firmware_upload's step 13
	 * for why - same DSL fork precedent, dsl.c:1211-1219).
	 */
	if (devc->usb_speed != LIBUSB_SPEED_SUPER) {
		wr.header.dest   = DSL_CTL_WORDWIDE;
		wr.header.offset = 0;
		wr.header.size   = 1;
		wr.data[0]       = bmWR_WORDWIDE;
		if ((ret = command_ctl_wr_v2(hdl, wr)) != SR_OK) {
			sr_err("DSL_CTL_WORDWIDE failed.");
			return SR_ERR;
		}
	}

	/*
	 * 2) Send bulk-write control command with 3-byte word count
	 *    (dsl.c).  arm_size is in uint16_t words.
	 */
	arm_size = sizeof(struct DSL_setting) / sizeof(uint16_t);
	wr.header.dest   = DSL_CTL_BULK_WR;
	wr.header.offset = 0;
	wr.header.size   = 3;
	wr.data[0] = (uint8_t)arm_size;
	wr.data[1] = (uint8_t)(arm_size >> 8);
	wr.data[2] = (uint8_t)(arm_size >> 16);
	if ((ret = command_ctl_wr_v2(hdl, wr)) != SR_OK) {
		sr_err("DSL_CTL_BULK_WR (arm announce) failed.");
		return SR_ERR;
	}

	/*
	 * 3) Poll bmSYS_CLR until the firmware asserts it
	 *    (dsl.c) - DSView polls immediately after BULK_WR with
	 *    no delay; the firmware appears to expect this fast cadence.
	 */
	if (dsl_wait_hw_status_bit_v2(hdl, bmSYS_CLR, TRUE, 3000) != SR_OK)
		return SR_ERR;

	/* 4) Build the settings blob and bulk-write it (dsl.c). */
	v2_build_default_setting(sdi, &setting);
	transferred = 0;
	ret = libusb_bulk_transfer(hdl, 2 | LIBUSB_ENDPOINT_OUT,
				   (unsigned char *)&setting,
				   sizeof(struct DSL_setting),
				   &transferred, V2_USB_TIMEOUT_MS);
	if (ret < 0) {
		sr_err("Arm FPGA bulk write failed: %s.", libusb_error_name(ret));
		return SR_ERR;
	}
	if (transferred != (int)sizeof(struct DSL_setting)) {
		sr_err("Arm FPGA bulk write short: %d/%zu.",
		       transferred, sizeof(struct DSL_setting));
		return SR_ERR;
	}

	/*
	 * 4b) DSLOGIC_CAPS_CH32 only: bulk-write the second arm block
	 * covering channels 16-31's trigger registers, immediately after
	 * struct DSL_setting and still under the same BULK_WR size announce
	 * / INTRDY handshake (dsl.c sends both blocks before asserting
	 * INTRDY; the BULK_WR announce above only counts DSL_setting's own
	 * size, mirroring dsl.c's arm_size calculation).
	 */
	if (devc->profile->dev_caps & DSLOGIC_CAPS_CH32) {
		v2_build_default_setting_ext32(&setting_ext32);
		transferred = 0;
		ret = libusb_bulk_transfer(hdl, 2 | LIBUSB_ENDPOINT_OUT,
					   (unsigned char *)&setting_ext32,
					   sizeof(struct DSL_setting_ext32),
					   &transferred, V2_USB_TIMEOUT_MS);
		if (ret < 0) {
			sr_err("Arm FPGA bulk write (ext32) failed: %s.",
			       libusb_error_name(ret));
			return SR_ERR;
		}
		if (transferred != (int)sizeof(struct DSL_setting_ext32)) {
			sr_err("Arm FPGA bulk write (ext32) short: %d/%zu.",
			       transferred, sizeof(struct DSL_setting_ext32));
			return SR_ERR;
		}
	}

	/* 5) Assert INTRDY high to signal end of data (dsl.c). */
	wr.header.dest   = DSL_CTL_INTRDY;
	wr.header.offset = 0;
	wr.header.size   = 1;
	wr.data[0]       = bmWR_INTRDY;
	if ((ret = command_ctl_wr_v2(hdl, wr)) != SR_OK) {
		sr_err("DSL_CTL_INTRDY failed.");
		return SR_ERR;
	}

	/*
	 * 6) Read HW_STATUS once and check bmGPIF_DONE (dsl.c).
	 *    DSView does NOT poll - a single read is the spec.
	 */
	rd.header.dest   = DSL_CTL_HW_STATUS;
	rd.header.offset = 0;
	rd.header.size   = 1;
	rd_data          = 0;
	rd.data          = &rd_data;
	if (command_ctl_rd_v2(hdl, rd) != SR_OK)
		return SR_ERR;
	if (rd_data & bmGPIF_DONE) {
		sr_info("Arm FPGA done.");
		return SR_OK;
	}
	sr_err("Arm FPGA: bmGPIF_DONE not set after INTRDY (HW_STATUS=0x%02x).", rd_data);
	return SR_ERR;
}

static int v2_acquisition_start(const struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct ctl_wr_cmd wr;

	wr.header.dest = DSL_CTL_START;
	wr.header.offset = 0;
	wr.header.size = 0;
	return command_ctl_wr_v2(usb->devhdl, wr);
}

static int v2_acquisition_stop(const struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct ctl_wr_cmd wr;

	/*
	 * DSView's dsl_dev_acquisition_stop is two-stage (dsl.c):
	 * write CTR0_ADDR := bmFORCE_RDY first (soft FPGA abort that releases
	 * the GPIF capture engine and resets the green LED), then send
	 * DSL_CTL_STOP. Without bmFORCE_RDY, the FPGA stays in capture state
	 * and the LED flashes after a "completed" acquisition.
	 */
	(void)dsl_wr_reg_v2(sdi, CTR0_ADDR, bmFORCE_RDY);

	wr.header.dest = DSL_CTL_STOP;
	wr.header.offset = 0;
	wr.header.size = 0;
	return command_ctl_wr_v2(usb->devhdl, wr);
}

/* Compute "need_channels" as max_enabled_index + 1, so contiguous-low
 * channels (0..N-1) trigger a small mode while sparse selections fall
 * back to a wider mode that covers the highest index in use. */
static unsigned int v2_max_enabled_plus_one(const struct sr_dev_inst *sdi)
{
	/*
	 * 32-bit mask + 32-iteration loop covers both the ≤16ch V2 Plus
	 * family and U3Pro32: the high 16 bits are simply always 0 for a
	 * device that only ever creates 16 sr_channels, so this is safe for
	 * both without needing a profile check here.
	 */
	uint32_t mask = enabled_channel_mask32(sdi);
	unsigned int hi = 0, i;
	for (i = 0; i < 32; i++) {
		if (mask & (1U << i))
			hi = i + 1;
	}
	return hi ? hi : 1;
}

static int v2_set_samplerate(const struct sr_dev_inst *sdi, uint64_t rate)
{
	struct dev_context *devc = sdi->priv;
	devc->cur_samplerate = rate;
	/* Auto-pick the channel mode that fits this rate + enabled-channel
	 * count under the current stream/buffer choice. */
	devc->ch_mode_id = dslogic_auto_pick_mode_id(devc, rate,
		devc->continuous_mode, devc->rle_mode,
		v2_max_enabled_plus_one(sdi));
	return SR_OK;
}

static int v2_set_voltage_threshold(const struct sr_dev_inst *sdi, double low, double high)
{
	/* DSLogic exposes a single threshold via VTH_ADDR; use the midpoint. */
	double mid = (low + high) / 2.0;
	uint8_t dac;

	if (mid < 0.0) mid = 0.0;
	if (mid > 3.3) mid = 3.3;
	dac = (uint8_t)(mid / 3.3 * (1.5 / 2.5) * 255.0);

	return dsl_wr_reg_v2(sdi, VTH_ADDR, dac);
}

static int v2_set_trigger(const struct sr_dev_inst *sdi)
{
	/* Triggers are encoded into struct DSL_setting at arm time
	 * (see v2_build_default_setting). Nothing to do at config time. */
	(void)sdi;
	return SR_OK;
}

static int v2_set_external_clock(const struct sr_dev_inst *sdi, gboolean ext)
{
	struct dev_context *devc = sdi->priv;
	devc->external_clock = ext;
	/* Applied at next arm via the mode bitfield in DSL_setting. */
	return SR_OK;
}

static int v2_set_clock_edge(const struct sr_dev_inst *sdi, int edge)
{
	struct dev_context *devc = sdi->priv;
	devc->clock_edge = edge;
	return SR_OK;
}

SR_PRIV const struct dslogic_protocol_ops dslogic_v2_ops = {
	.fpga_firmware_upload  = v2_fpga_firmware_upload,
	.fpga_config           = v2_fpga_config,
	.acquisition_start     = v2_acquisition_start,
	.acquisition_stop      = v2_acquisition_stop,
	.set_samplerate        = v2_set_samplerate,
	.set_voltage_threshold = v2_set_voltage_threshold,
	.set_trigger           = v2_set_trigger,
	.set_external_clock    = v2_set_external_clock,
	.set_clock_edge        = v2_set_clock_edge,
	.security_check        = v2_security_check,
};
