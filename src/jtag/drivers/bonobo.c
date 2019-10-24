/***************************************************************************
 *   Copyright (C) 2019 by LambdaConcept                                   *
 *   Pierre-Olivier Vauboin <po@lambdaconcept.com>                         *
 *   Ramtin Amin <ramtin@lambdaconcept.com>                                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <jtag/interface.h>
#include <jtag/swd.h>
#include <transport/transport.h>
#include <helper/time_support.h>

#if IS_CYGWIN == 1
#include <windows.h>
#endif

#include <assert.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "libusb_common.h"

/* Multiplexer */

#define MUX_MODE_NC                 0
#define MUX_MODE_SDQ                1
#define MUX_MODE_SWD                2
#define MUX_MODE_ACCRESET           3

/* SDQ Core */

#define SDQ_MODE_IDLE               0
#define SDQ_MODE_SNR                2
#define SDQ_MODE_SERIAL             3
#define SDQ_MODE_SWD                4
#define SDQ_MODE_RESET              5

#define SDQ_STATUS_DONE             (1 << 0)
#define SDQ_STATUS_TIMEOUT          (1 << 1)
#define SDQ_STATUS_PRESENT          (1 << 2)

/* SWD Core */

#define SWDCORE_PARITYOK    0x10
#define SWDCORE_DONE        0x08
#define SWDCORE_OK          0x04
#define SWDCORE_ACK_MASK    0x07

/* USB Protocol Header */

#define CMD_SWD_RESET       0
#define CMD_SWD_JTAG2SWD    1
#define CMD_SWD_SWD2JTAG    2
#define CMD_SWD_EXEC        3

#define CMD_SWD_FREQ        4
#define CMD_MUX_SEL         5
#define CMD_SDQ_DEFAULT     6
#define CMD_SDQ_SEL         7
#define CMD_SDQ_RESULT      8

/* USB */

#define VID  0xffff
#define PID  0x1234

#define BULK_TRANSFER_SIZE  512
#define BULK_EP_IN      (1 | LIBUSB_ENDPOINT_IN)
#define BULK_EP_OUT     (2 | LIBUSB_ENDPOINT_OUT)
#define BULK_EP_TIMEOUT 1000

struct queue_s {
	uint8_t cmd;
	uint32_t data_w;
	uint32_t *data_r;
	uint32_t data_rlog;
	uint8_t ack;
};

static inline const char* debug_reg_name(int cmd)
{
	char *reg_name = "UNK";
	int addr = (cmd & SWD_CMD_A32) >> 3;

	if (cmd & SWD_CMD_APnDP) {
		switch(addr) {
		case 0:
			reg_name = "AP_O";
			break;
		case 1:
			reg_name = "AP_4";
			break;
		case 2:
			reg_name = "AP_8";
			break;
		case 3:
			reg_name = "AP_C";
			break;
		}
	} else {
		switch(addr) {
		case 0:
			if (cmd & SWD_CMD_RnW)
				reg_name = "DPIDR";
			else
				reg_name = "ABORT";
			break;
		case 1:
			reg_name = "CTRL/STAT";
			break;
		case 2:
			reg_name = "SELECT";
			break;
		case 3:
			reg_name = "RDBUFF";
			break;
		}
	}

	return reg_name;
}

static inline void debug_queue(struct queue_s *q)
{
	LOG_DEBUG_IO("%5s %2s %10s 0x%08x %5s",
		(q->cmd & SWD_CMD_RnW) ? "READ" : "WRITE",
		(q->cmd & SWD_CMD_APnDP) ? "AP" : "DP",
		debug_reg_name(q->cmd),
		(q->cmd & SWD_CMD_RnW) ? q->data_rlog : q->data_w,
		(q->ack == SWD_ACK_OK) ? "OK" : (q->ack == SWD_ACK_WAIT) ? "WAIT" : (q->ack == SWD_ACK_FAULT) ? "FAULT" : "JUNK");
}

struct bonobo_s {
	struct jtag_libusb_device_handle *usb_handle;
	struct queue_s *queue;
	int queue_len;
	int queue_alloc;
};

static struct bonobo_s *g_sess;

/* libusb functions */

static int bonobo_usb_open(void)
{
	const uint16_t vids[] = { VID, 0 };
	const uint16_t pids[] = { PID, 0 };

	if (jtag_libusb_open(vids, pids, NULL, &g_sess->usb_handle) != ERROR_OK) {
		LOG_ERROR("Failed to open or find the device");
		return ERROR_FAIL;
	}

	/* Claim the Bonobo (bulk transfer) interface */

	if (jtag_libusb_claim_interface(g_sess->usb_handle, 0) != ERROR_OK) {
		LOG_ERROR("Failed to claim Bonobo (bulk transfer) interface");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static void bonobo_usb_close(void)
{
	jtag_libusb_close(g_sess->usb_handle);
}

/* interface functions */

static int bonobo_acc_reset(void)
{
	int ret;
	uint8_t buf[4];

	/* Reset the ACC line */

	buf[0] = CMD_MUX_SEL;
	buf[1] = MUX_MODE_ACCRESET;

	ret = jtag_libusb_bulk_write(g_sess->usb_handle, BULK_EP_OUT,
			(char *)buf, 2, BULK_EP_TIMEOUT);
	if (ret <= 0) {
		LOG_ERROR("Bulk write failed");
		return ERROR_FAIL;
	}

	/* Select SDQ SWD sequence */

	buf[0] = CMD_SDQ_SEL;
	buf[1] = SDQ_MODE_SWD;

	ret = jtag_libusb_bulk_write(g_sess->usb_handle, BULK_EP_OUT,
			(char *)buf, 2, BULK_EP_TIMEOUT);
	if (ret <= 0) {
		LOG_ERROR("Bulk write failed");
		goto err_init;
	}

	/* MUX select SDQ */

	buf[0] = CMD_MUX_SEL;
	buf[1] = MUX_MODE_SDQ;

	ret = jtag_libusb_bulk_write(g_sess->usb_handle, BULK_EP_OUT,
			(char *)buf, 2, BULK_EP_TIMEOUT);
	if (ret <= 0) {
		LOG_ERROR("Bulk write failed");
		goto err_init;
	}

	/* Check SDQ sequence status */

	buf[0] = CMD_SDQ_RESULT;

	ret = jtag_libusb_bulk_write(g_sess->usb_handle, BULK_EP_OUT,
			(char *)buf, 1, BULK_EP_TIMEOUT);
	if (ret <= 0) {
		LOG_ERROR("Bulk write failed");
		goto err_init;
	}

	ret = jtag_libusb_bulk_read(g_sess->usb_handle, BULK_EP_IN,
			(char *)buf, 1, BULK_EP_TIMEOUT);
	if (ret <= 0) {
		LOG_ERROR("Bulk read failed");
		goto err_init;
	}

	if (!(buf[0] & SDQ_STATUS_DONE)) {
		LOG_ERROR("No SDQ, phone not plugged/powered ?");
		goto err_init;
	}

	/* MUX select SWD */

	buf[0] = CMD_MUX_SEL;
	buf[1] = MUX_MODE_SWD;

	ret = jtag_libusb_bulk_write(g_sess->usb_handle, BULK_EP_OUT,
			(char *)buf, 2, BULK_EP_TIMEOUT);
	if (ret <= 0) {
		LOG_ERROR("Bulk write failed");
		goto err_init;
	}

	return ERROR_OK;

err_init:
	return ERROR_FAIL;
}

static int bonobo_initialize(void)
{
	int ret;
	uint8_t buf[4];
	int retries;

	g_sess = malloc(sizeof(struct bonobo_s));
	if (g_sess == NULL) {
		LOG_ERROR("Failed to allocate memory");
		return ERROR_FAIL;
	}
	memset(g_sess, 0, sizeof(struct bonobo_s));

	/* Allocate queue buffer */

	g_sess->queue = calloc(4, sizeof(struct queue_s));
	if (g_sess->queue == NULL) {
		LOG_ERROR("Failed to allocate memory");
		goto err_malloc;
	}
	g_sess->queue_len = 0;
	g_sess->queue_alloc = 4;

	/* Open USB device */

	if (bonobo_usb_open() != ERROR_OK) {
		LOG_ERROR("Can't find a Bonobo device! Please check device connections and permissions.");
		goto err_open;
	}

	/* Set SWD clock */

	buf[0] = CMD_SWD_FREQ;
	buf[1] = 6; // T High
	buf[2] = 25; // T Low

	ret = jtag_libusb_bulk_write(g_sess->usb_handle, BULK_EP_OUT,
			(char *)buf, 3, BULK_EP_TIMEOUT);
	if (ret <= 0) {
		LOG_ERROR("Bulk write failed");
		goto err_init;
	}

	/* Init / Reset sequence */

	retries = 0;
	do {
		ret = bonobo_acc_reset();
	} while ((ret != ERROR_OK) && (retries++ < 50));

	if (ret != ERROR_OK) {
		goto err_init;
	}

	return ERROR_OK;

err_init:
	bonobo_usb_close();
err_open:
	free(g_sess->queue);
err_malloc:
	free(g_sess);
	return ERROR_FAIL;
}

static int bonobo_quit(void)
{
	bonobo_usb_close();

	if (g_sess != NULL) {
		if (g_sess->queue != NULL) {
			free(g_sess->queue);
		}
		free(g_sess);
	}

	return ERROR_OK;
}

static int bonobo_speed(int speed)
{
	return ERROR_OK;
}

static int bonobo_speed_div(int speed, int *khz)
{
	*khz = speed / 1000;
	return ERROR_OK;
}

static int bonobo_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz * 1000;
	return ERROR_OK;
}

static int bonobo_reset_target(void)
{
	int ret;
	uint8_t buf[4];

	/* Reset the ACC line */

	buf[0] = CMD_MUX_SEL;
	buf[1] = MUX_MODE_ACCRESET;

	ret = jtag_libusb_bulk_write(g_sess->usb_handle, BULK_EP_OUT,
			(char *)buf, 2, BULK_EP_TIMEOUT);
	if (ret <= 0) {
		LOG_ERROR("Bulk write failed");
		return ERROR_FAIL;
	}

	/* Select SDQ RESET sequence */

	buf[0] = CMD_SDQ_SEL;
	buf[1] = SDQ_MODE_RESET;

	ret = jtag_libusb_bulk_write(g_sess->usb_handle, BULK_EP_OUT,
			(char *)buf, 2, BULK_EP_TIMEOUT);
	if (ret <= 0) {
		LOG_ERROR("Bulk write failed");
		goto err;
	}

	/* MUX select SDQ */

	buf[0] = CMD_MUX_SEL;
	buf[1] = MUX_MODE_SDQ;

	ret = jtag_libusb_bulk_write(g_sess->usb_handle, BULK_EP_OUT,
			(char *)buf, 2, BULK_EP_TIMEOUT);
	if (ret <= 0) {
		LOG_ERROR("Bulk write failed");
		goto err;
	}

	/* Check SDQ sequence status */

	buf[0] = CMD_SDQ_RESULT;

	ret = jtag_libusb_bulk_write(g_sess->usb_handle, BULK_EP_OUT,
			(char *)buf, 1, BULK_EP_TIMEOUT);
	if (ret <= 0) {
		LOG_ERROR("Bulk write failed");
		goto err;
	}

	ret = jtag_libusb_bulk_read(g_sess->usb_handle, BULK_EP_IN,
			(char *)buf, 1, BULK_EP_TIMEOUT);
	if (ret <= 0) {
		LOG_ERROR("Bulk read failed");
		goto err;
	}

	if (!(buf[0] & SDQ_STATUS_DONE)) {
		LOG_ERROR("No SDQ, phone not plugged/powered ?");
		goto err;
	}

	return ERROR_OK;

err:
	return ERROR_FAIL;
}

static int bonobo_execute_reset(struct jtag_command *cmd)
{
	int ret = ERROR_OK;

	if (cmd->cmd.reset->srst == 1) { /* Assert */
		tap_set_state(TAP_RESET);
		ret = bonobo_reset_target();
	}

	return ret;
}

static int bonobo_execute_command(struct jtag_command *cmd)
{
	int ret;

	switch (cmd->type) {
		case JTAG_RESET:
			ret = bonobo_execute_reset(cmd);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type encountered: %d", cmd->type);
			ret = ERROR_FAIL;
			break;
	}

	return ret;
}

static int bonobo_execute_queue(void)
{
	int ret;

	for (struct jtag_command *cmd = jtag_command_queue; cmd; cmd = cmd->next) {
		/* fill the write buffer with the desired command */
		ret = bonobo_execute_command(cmd);
		if (ret != ERROR_OK) {
			return ret;
		}
	}

	return ERROR_OK;
}

/* queue functions */

static void bonobo_swd_queue(uint8_t cmd, uint32_t *data_r, uint32_t data_w)
{
	int idx;
	struct queue_s *new;

	if (g_sess->queue_len == g_sess->queue_alloc) {

		LOG_DEBUG("New queue size: %d", 2*g_sess->queue_alloc);

		new = realloc(g_sess->queue, 2*g_sess->queue_alloc * sizeof(struct queue_s));
		if (new == NULL) {
			LOG_ERROR("Failed to reallocate memory");
			return;
		}
		g_sess->queue = new;
		g_sess->queue_alloc = 2*g_sess->queue_alloc;
	}

	idx = g_sess->queue_len;

	g_sess->queue[idx].cmd = cmd;
	g_sess->queue[idx].data_w = data_w;
	g_sess->queue[idx].data_r = data_r;

	g_sess->queue_len++;
}

/* swd functions */

static int bonobo_swd_init(void)
{
	return ERROR_OK;
}

static int bonobo_swd_switch_seq(enum swd_special_seq seq)
{
	int ret;
	int retries;
	int loops = 0;
	uint8_t buf[8];
	uint8_t status;

	do {
		/* SWD switch */

		buf[0] = CMD_SWD_JTAG2SWD;

		/* Read IDCODE */

		buf[1] = CMD_SWD_EXEC;
		buf[2] = swd_cmd(true, false, DP_DPIDR) | SWD_CMD_START | SWD_CMD_PARK;

		ret = jtag_libusb_bulk_write(g_sess->usb_handle, BULK_EP_OUT,
				(char *)buf, 1 + 2, BULK_EP_TIMEOUT);
		if (ret <= 0) {
			LOG_ERROR("Bulk write failed");
			return ERROR_FAIL;
		}

		ret = jtag_libusb_bulk_read(g_sess->usb_handle, BULK_EP_IN,
				(char *)buf, 1 + 5, BULK_EP_TIMEOUT);
		if (ret <= 0) {
			LOG_ERROR("Bulk read failed");
			return ERROR_FAIL;
		}

		/* Read IDCODE status */

		status = buf[1];
		if (status == (SWDCORE_DONE | SWDCORE_OK | SWDCORE_PARITYOK)) {
			return ERROR_OK;
		}

		retries = 0;
		do {
			ret = bonobo_acc_reset();
		} while ((ret != ERROR_OK) && (retries++ < 5));

	} while ((loops++) < 1000);

	LOG_ERROR("Max retries, switch failed");

	return ERROR_FAIL;
}

static void bonobo_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RnW);
	bonobo_swd_queue(cmd, value, 0);
}

static void bonobo_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RnW));
	bonobo_swd_queue(cmd, NULL, value);
}

static int bonobo_swd_clear(void)
{
	int ret;
	uint8_t buf[8];
	uint32_t data;

	/* Write ABORT */

	buf[0] = CMD_SWD_EXEC;
	buf[1] = swd_cmd(false, false, DP_ABORT) | SWD_CMD_START | SWD_CMD_PARK;

	data = STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR;
	buf[2] = (data) & 0xff;
	buf[3] = (data >> 8) & 0xff;
	buf[4] = (data >> 16) & 0xff;
	buf[5] = (data >> 24) & 0xff;

	ret = jtag_libusb_bulk_write(g_sess->usb_handle, BULK_EP_OUT,
			(char *)buf, 2 + 4, BULK_EP_TIMEOUT);
	if (ret <= 0) {
		LOG_ERROR("Bulk write failed");
		return ERROR_FAIL;
	}

	ret = jtag_libusb_bulk_read(g_sess->usb_handle, BULK_EP_IN,
			(char *)buf, 1, BULK_EP_TIMEOUT);
	if (ret <= 0) {
		LOG_ERROR("Bulk read failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int bonobo_swd_run(void)
{
	int i;
	int ret;
	uint8_t cmd;
	uint32_t data;
	uint8_t ack;
	int read_index;
	int queue_index = 0;
	int queue_next;
	int read_count;
	int write_count;
	int will_read;
	int will_write;
	uint8_t *buffer;

	if (g_sess->queue_len <= 0) {
		return ERROR_OK;
	}

	buffer = malloc(BULK_TRANSFER_SIZE);
	if (buffer == NULL) {
		LOG_ERROR("Failed to allocate memory");
		ret = ERROR_FAIL;
		goto err;
	}

	do {

		/* Prepare the USB buffer */

		write_count = 0;
		read_count = 0;

		for (i = queue_index; i < g_sess->queue_len; i++) {
			cmd = g_sess->queue[i].cmd;
			data = g_sess->queue[i].data_w;

			/* Check available length */

			if (!(cmd & SWD_CMD_RnW)) {
				will_write = 2 + 4;
				will_read = 1;
			} else {
				will_write = 2;
				will_read = 1 + 4;
			}

			if ( ((write_count + will_write) > BULK_TRANSFER_SIZE) || \
				 ((read_count + will_read) > BULK_TRANSFER_SIZE) ) {

				/* Big enough, execute now */

				break;
			}

			/* Push the next command into the buffer */

			buffer[write_count++] = CMD_SWD_EXEC;
			buffer[write_count++] = cmd | SWD_CMD_START | SWD_CMD_PARK;
			read_count++;

			if (!(cmd & SWD_CMD_RnW)) {
				buffer[write_count++] = (data) & 0xff;
				buffer[write_count++] = (data >> 8) & 0xff;
				buffer[write_count++] = (data >> 16) & 0xff;
				buffer[write_count++] = (data >> 24) & 0xff;
			} else {
				read_count += 4;
			}
		}

		queue_next = i;

		assert(write_count <= BULK_TRANSFER_SIZE);
		assert(read_count <= BULK_TRANSFER_SIZE);

		/* Send the SWD commands */

		ret = jtag_libusb_bulk_write(g_sess->usb_handle, BULK_EP_OUT,
				(char *)buffer, write_count, BULK_EP_TIMEOUT);
		if (ret <= 0) {
			LOG_ERROR("Bulk write failed");
			ret = ERROR_FAIL;
			goto err;
		}

		/* Receive the results */

		ret = jtag_libusb_bulk_read(g_sess->usb_handle, BULK_EP_IN,
				(char *)buffer, BULK_TRANSFER_SIZE + 1, BULK_EP_TIMEOUT);
		if (ret <= 0) {
			LOG_ERROR("Bulk read failed");
			ret = ERROR_FAIL;
			goto err;
		}

		assert(ret == read_count);

		/* Process responses */

		ret = ERROR_OK;

		read_index = 0;

		for (i = queue_index; i < queue_next; i++) {
			cmd = g_sess->queue[i].cmd;
			ack = buffer[read_index++] & SWDCORE_ACK_MASK;
			ack = ((ack & 1) << 2) | ((ack & 2)) | ((ack & 4) >> 2); /* Reverse bit order */
			g_sess->queue[i].ack = ack;

			if (ack != SWD_ACK_OK) {
				LOG_DEBUG("SWD ack not OK: %d %s", i,
						(ack == SWD_ACK_WAIT) ? "WAIT" : (ack == SWD_ACK_FAULT) ? "FAULT" : "JUNK");
				ret = (ack == SWD_ACK_WAIT) ? ERROR_WAIT : ERROR_FAIL;
			}

			if (cmd & SWD_CMD_RnW) {
				data = le_to_h_u32(&buffer[read_index]);

				g_sess->queue[i].data_rlog = data;
				if (g_sess->queue[i].data_r) {
					*g_sess->queue[i].data_r = data;
				}
				read_index += 4;
			}

			debug_queue(&g_sess->queue[i]);
		}

		if (ret != ERROR_OK) {
			bonobo_swd_clear();
			goto err;
		}

		queue_index = queue_next;

	} while(queue_index < g_sess->queue_len); /* Until the queue is exhausted */

err:
	if (buffer)
		free(buffer);

	g_sess->queue_len = 0;
	return ret;
}

static const struct swd_driver bonobo_swd = {
	.init = bonobo_swd_init,
	.switch_seq = bonobo_swd_switch_seq,
	.read_reg = bonobo_swd_read_reg,
	.write_reg = bonobo_swd_write_reg,
	.run = bonobo_swd_run,
};

static const char * const bonobo_transports[] = { "jtag", "swd", NULL };

struct jtag_interface bonobo_interface = {
	.name = "bonobo",
	.supported = DEBUG_CAP_TMS_SEQ,
	.commands = NULL,
	.transports = bonobo_transports,
	.swd = &bonobo_swd,

	.init = bonobo_initialize,
	.quit = bonobo_quit,
	.speed = bonobo_speed,
	.speed_div = bonobo_speed_div,
	.khz = bonobo_khz,
	.execute_queue = bonobo_execute_queue,
};

/* ========== don't read past this line for your own sake ========== */

static int jim_dirty_yolo_hack_command(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	int retval = JIM_ERR;
	int r;
	uint32_t trash;
	uint32_t pidr[4];

	if (argc > 1)
		goto out;

	if (bonobo_initialize() != ERROR_OK)
		goto out;

	r = bonobo_swd_switch_seq(JTAG_TO_SWD);
	if (r != ERROR_OK) {
		LOG_ERROR("dirty_yolo_hack: bonobo_swd_switch_seq failed: %d", r);
		goto out;
	}

	bonobo_swd_write_reg(swd_cmd(false, false, DP_ABORT), STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR, 0);
	bonobo_swd_write_reg(swd_cmd(false, false, DP_CTRL_STAT), CDBGPWRUPREQ | CSYSPWRUPREQ | CORUNDETECT, 0);
	bonobo_swd_write_reg(swd_cmd(false, false, DP_SELECT), 0x1000000, 0); // APBANK 0
	// All bank 0
	bonobo_swd_write_reg(swd_cmd(false, true, MEM_AP_REG_CSW), 0x80000052, 0); // AddrInc single, access size = 32
	bonobo_swd_write_reg(swd_cmd(false, true, MEM_AP_REG_TAR), 0x80000fe0, 0); // ROMTABLE PIDR0
	bonobo_swd_read_reg(swd_cmd(true, true, MEM_AP_REG_DRW), &trash, 0);
	bonobo_swd_read_reg(swd_cmd(true, true, MEM_AP_REG_DRW), &pidr[0], 0);
	bonobo_swd_read_reg(swd_cmd(true, true, MEM_AP_REG_DRW), &pidr[1], 0);
	bonobo_swd_read_reg(swd_cmd(true, true, MEM_AP_REG_DRW), &pidr[2], 0);
	bonobo_swd_read_reg(swd_cmd(true, true, DP_RDBUFF), &pidr[3], 0);

	r = bonobo_swd_run();
	if (r != ERROR_OK) {
		LOG_ERROR("dirty_yolo_hack: bonobo_swd_run failed: %d", r);
		goto out;
	}

/*	struct jtag_libusb_device_handle *usb_handle = NULL;
	const uint16_t vids[] = { VID, 0 };
	const uint16_t pids[] = { PID, 0 };
	if (jtag_libusb_open(vids, pids, NULL, &usb_handle) != ERROR_OK) {
		LOG_ERROR("Failed to open or find Bonobo");
		goto out;
	}
	if (jtag_libusb_claim_interface(usb_handle, 0) != ERROR_OK) {
		LOG_ERROR("Failed to claim Bonobo (bulk transfer) interface");
		goto out;
	}

#define ACKMASK(x) ((((x) & 1) << 2) | ((x) & 2) | (((x) & 4) >> 2))
	uint8_t buf[8];
	uint8_t ack;
	buf[0] = CMD_SWD_FREQ;
	buf[1] = 6; // T High
	buf[2] = 25; // T Low
	if (jtag_libusb_bulk_write(usb_handle, BULK_EP_OUT, (char*)buf, 3, BULK_EP_TIMEOUT) <= 0) {
		LOG_ERROR("Bulk write failed (1)");
		goto out;
	}

	buf[0] = CMD_MUX_SEL;
	buf[1] = MUX_MODE_ACCRESET;
	if (jtag_libusb_bulk_write(usb_handle, BULK_EP_OUT, (char*)buf, 2, BULK_EP_TIMEOUT) <= 0) {
		LOG_ERROR("Bulk write failed (2)");
		goto out;
	}

	buf[0] = CMD_SDQ_SEL;
	buf[1] = SDQ_MODE_SWD;
	if (jtag_libusb_bulk_write(usb_handle, BULK_EP_OUT, (char*)buf, 2, BULK_EP_TIMEOUT) <= 0) {
		LOG_ERROR("Bulk write failed (3)");
		goto out;
	}

	buf[0] = CMD_MUX_SEL;
	buf[1] = MUX_MODE_SDQ;
	if (jtag_libusb_bulk_write(usb_handle, BULK_EP_OUT, (char*)buf, 2, BULK_EP_TIMEOUT) <= 0) {
		LOG_ERROR("Bulk write failed (4)");
		goto out;
	}

	buf[0] = CMD_SDQ_RESULT;
	if (jtag_libusb_bulk_write(usb_handle, BULK_EP_OUT, (char*)buf, 1, BULK_EP_TIMEOUT) <= 0) {
		LOG_ERROR("Bulk write failed (5)");
		goto out;
	}

	if (jtag_libusb_bulk_read(usb_handle, BULK_EP_IN, (char*)buf, 1, BULK_EP_TIMEOUT) <= 0) {
		LOG_ERROR("Bulk read failed (6)");
		goto out;
	}
	if (!(buf[0] & SDQ_STATUS_DONE)) {
		LOG_ERROR("No SDQ, phone not plugged/powered?");
		goto out;
	}

	buf[0] = CMD_MUX_SEL;
	buf[1] = MUX_MODE_SWD;
	if (jtag_libusb_bulk_write(usb_handle, BULK_EP_OUT, (char*)buf, 2, BULK_EP_TIMEOUT) <= 0) {
		LOG_ERROR("Bulk write failed (7)");
		goto out;
	}

	buf[0] = CMD_SWD_EXEC;
	buf[1] = swd_cmd(false, false, 0x8) | SWD_CMD_START | SWD_CMD_PARK;
	buf[2] = 0x0;
	buf[3] = 0x0;
	buf[4] = 0x0;
	buf[5] = 0x1; // AP1
	if (jtag_libusb_bulk_write(usb_handle, BULK_EP_OUT, (char*)buf, 6, BULK_EP_TIMEOUT) <= 0) {
		LOG_ERROR("Bulk write failed (8)");
		goto out;
	}
	if (jtag_libusb_bulk_read(usb_handle, BULK_EP_IN, (char*)buf, 1, BULK_EP_TIMEOUT) <= 0) {
		LOG_ERROR("Bulk read failed (9)");
		goto out;
	}
	ack = ACKMASK(buf[0]);
	if (ack != SWD_ACK_OK) {
		LOG_ERROR("SWD ack not OK: %s", (ack == SWD_ACK_WAIT) ? "WAIT" : (ack == SWD_ACK_FAULT) ? "FAULT" : "JUNK");
		goto out;
	}

	Jim_Obj *const *elements

	//Jim_SetResult(interp, Jim_NewStringObj(interp, version_str, -1));
	Jim_SetResult(interp, Jim_NewListObj(interp, version_str, -1));*/

	Jim_Obj *list = Jim_NewListObj(interp, NULL, 0);
	Jim_ListAppendElement(interp, list, Jim_NewIntObj(interp, pidr[0]));
	Jim_ListAppendElement(interp, list, Jim_NewIntObj(interp, pidr[1]));
	Jim_ListAppendElement(interp, list, Jim_NewIntObj(interp, pidr[2]));
	Jim_ListAppendElement(interp, list, Jim_NewIntObj(interp, pidr[3]));
	Jim_SetResult(interp, list);

	retval = JIM_OK;
out:
	/*if (usb_handle)
		jtag_libusb_close(usb_handle);*/
	bonobo_quit();

	return retval;
}

static const struct command_registration dirty_yolo_hack_command_handlers[] =
{
	{
		.name = "dirty_yolo_hack",
		.jim_handler = jim_dirty_yolo_hack_command,
		.mode = COMMAND_CONFIG,
		.help = "run while you still can",
	},
	COMMAND_REGISTRATION_DONE
};

int dirty_yolo_hack_register_commands(struct command_context *ctx)
{
	return register_commands(ctx, NULL, dirty_yolo_hack_command_handlers);
}
