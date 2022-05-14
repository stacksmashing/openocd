/***************************************************************************
 *   Copyright (C) 2022 by Thomas Roth                                     *
 *   code@stacksmashing.net                                                *
 *                                                                         *
 *   Based on picoprobe.c by Liam Fraser                                   *
 *                                                                         *
 *   Also based on: kitprog.c, ftdi.c, mpsse.c                             *
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


#include <jtag/interface.h>
#include <jtag/swd.h>
#include <jtag/commands.h>
#include <helper/command.h>

#include "libusb_common.h"

enum TAMARIN_CMDS {
    TAMARIN_INVALID      = 0, // Invalid command
    TAMARIN_READ = 1,
    TAMARIN_WRITE = 2,
    TAMARIN_LINE_RESET = 3,
    TAMARIN_SET_FREQ = 4
};

// This struct is the direct struct that is sent to
// the probe.
struct __attribute__((__packed__)) tamarin_cmd_hdr {
    // Currently unused
	uint8_t id;
    // One of TAMARIN_CMDS
    uint8_t cmd;
    // The full (incl. start/stop/parity) SWD command
    // Unused for LINE_RESET and SET_FREQ.
    uint8_t request;
    // The data for writes (unused otherwise)
    uint32_t data;
    // Number of (8 bit) idle cycles to perform after this op
	uint8_t idle_cycles;
};

// This is the struture returned by the probe for each command
struct __attribute__((__packed__)) tamarin_res_hdr {
    // Unused
	uint8_t id;
    // The (3 bit) result: OK/WAIT/FAULT
    uint8_t res;
    // The data for reads (undefined otherwise)
    uint32_t data;
};

// This struct is just an encapsulation for the
// cmd_hdr that also keeps the pointer for reads.
struct tamarin_cmd_hdr_enc {
	struct tamarin_cmd_hdr cmd;
	uint32_t *data;
};

// Random value that performs well enough
#define TAMARIN_QUEUE_SIZE 64


#define VID 0x2E8A /* Raspberry Pi */
#define PID 0x0004 /* Picoprobe */

#define BULK_EP_OUT 4
#define BULK_EP_IN  5
#define PICOPROBE_INTERFACE 2

#define PICOPROBE_MAX_PACKET_LENGTH 512
#define LIBUSB_TIMEOUT 1000



struct tamarin {
	jtag_libusb_device_handle *usb_handle;
	int freq;
	struct tamarin_cmd_hdr_enc queue[TAMARIN_QUEUE_SIZE];
	size_t queue_length;
};

static int queued_retval;


static struct tamarin *tamarin_handle;

static int tamarin_init(void);
static int tamarin_quit(void);


struct __attribute__((__packed__)) probe_cmd_hdr {
	uint8_t id;
	uint8_t cmd;
	uint32_t bits;
};

struct __attribute__((__packed__)) probe_pkt_hdr {
	uint32_t total_packet_length;
};




/* Separate queue to swd_cmd_queue because we sometimes insert idle cycles not described
 * there */
#define PICOPROBE_QUEUE_SIZE 64
struct tamarin_queue_entry {
	uint8_t id;
	uint8_t cmd; /* PROBE_CMDS */
	unsigned bits;
	unsigned offset;
	const uint8_t *buf;
};

static int tamarin_swd_run_queue(void)
{
	LOG_DEBUG_IO("Executing %zu queued transactions", tamarin_handle->queue_length);


	// yes this is slow as fuck
	for(size_t i = 0; i < tamarin_handle->queue_length; i++) {
		// sleep(0.1);
		struct tamarin_res_hdr result;
		struct tamarin_cmd_hdr_enc *command_enc = &tamarin_handle->queue[i];
		struct tamarin_cmd_hdr *command = &command_enc->cmd;
		int ret;
		// write the SINGLE command
		// LOG_DEBUG("USB BULK WRITE");
		ret = jtag_libusb_bulk_write(tamarin_handle->usb_handle, BULK_EP_OUT, (char*)command, sizeof(struct tamarin_cmd_hdr), LIBUSB_TIMEOUT);
		if(ret < 0) {
			LOG_DEBUG("BULK WRITE FAILED");
			return ERROR_JTAG_DEVICE_ERROR;
		}
		// LOG_DEBUG("DONE, LETS REAAAD");
		ret = jtag_libusb_bulk_read(tamarin_handle->usb_handle,
		BULK_EP_IN | LIBUSB_ENDPOINT_IN, (char *)&result,
			sizeof(struct tamarin_res_hdr), LIBUSB_TIMEOUT);
		if(ret < 0) {
			LOG_DEBUG("BULK READ FAILED");
			tamarin_handle->queue_length = 0;
			return ERROR_JTAG_DEVICE_ERROR;
		}
		// LOG_DEBUG("READ DONE");
		// LOG_DEBUG("Result is: %d %d %d\n", result.id, result.res, result.data);

        // TODO: Handle errors better here.
		if(result.res == 4) {
			LOG_DEBUG("FAIL FAIL FAIL\n");
			tamarin_handle->queue_length = 0;
			return ERROR_TARGET_FAILURE;
		}
		if((command->cmd == TAMARIN_READ) && (command_enc->data != NULL)) {
			*command_enc->data = result.data;
		}
		
		keep_alive();
	}


	tamarin_handle->queue_length = 0;

	queued_retval = ERROR_OK;
	return ERROR_OK;
}

static void tamarin_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RnW);
	// Et hätt noch immer jot jejange.
	assert(tamarin_handle->queue_length < TAMARIN_QUEUE_SIZE-1);

	struct tamarin_cmd_hdr command = {
		.id = 0,
		.cmd = TAMARIN_READ,
		.request = cmd | 0x81,
		.data =  0,
		.idle_cycles = ap_delay_clk
	};

	struct tamarin_cmd_hdr_enc command_enc = {
		.cmd = command,
		.data = value
	};

	LOG_DEBUG("Enqueue read: 0x%02X", cmd);
	tamarin_handle->queue[tamarin_handle->queue_length] = command_enc;
	tamarin_handle->queue_length++;
}

static void tamarin_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RnW));
	assert(tamarin_handle->queue_length < TAMARIN_QUEUE_SIZE-1);

	struct tamarin_cmd_hdr command = {
		.id = 0,
		.cmd = TAMARIN_WRITE,
		.request = cmd | 0x81,
		.data =  value,
		.idle_cycles = ap_delay_clk
	};

	struct tamarin_cmd_hdr_enc command_enc = {
		.cmd = command,
		.data = NULL
	};

	LOG_DEBUG("Enqueue write: 0x%02X - 0x%08X", cmd, value);
	tamarin_handle->queue[tamarin_handle->queue_length] = command_enc;
	tamarin_handle->queue_length++;
}

static int_least32_t tamarin_set_frequency(int_least32_t hz)
{

	struct tamarin_cmd_hdr command = {
		.id = 0,
		.cmd = TAMARIN_SET_FREQ,
		.request = 0,
		.data =  hz / 1000,
		.idle_cycles = 0
	};
	struct tamarin_cmd_hdr_enc command_enc = {
		.cmd = command,
		.data = NULL
	};

	LOG_DEBUG("Enqueue set frequency: %d", hz/1000);
	tamarin_handle->queue[tamarin_handle->queue_length] = command_enc;
	tamarin_handle->queue_length++;


	// TODO: Flush
	// TODO: handle error
	// if (ret < 0)
	// 	return ERROR_JTAG_DEVICE_ERROR;	

	return hz;
}

static int_least32_t tamarin_speed(int_least32_t hz)
{
	int ret = tamarin_set_frequency(hz);

	if (ret < 0)
		LOG_ERROR("Couldn't set tamarin cable speed");
	else
		tamarin_handle->freq = ret;

	return ERROR_OK;
}

static int tamarin_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz * 1000;
	return ERROR_OK;
}

static int tamarin_speed_div(int speed, int *khz)
{
	*khz = speed / 1000;
	return ERROR_OK;
}

static int tamarin_swd_init(void)
{
	return ERROR_OK;
}

static int tamarin_line_reset() {
	struct tamarin_cmd_hdr command = {
		.id = 0,
		.cmd = TAMARIN_LINE_RESET,
		.request = 0,
		.data =  0,
		.idle_cycles = 0
	};
	struct tamarin_cmd_hdr_enc command_enc = {
		.cmd = command,
		.data = NULL
	};


	LOG_DEBUG("Enqueue line reset / SWD-to-JTAG sequence");
	tamarin_handle->queue[tamarin_handle->queue_length] = command_enc;
	tamarin_handle->queue_length++;
	return ERROR_OK;
}

static int tamarin_swd_switch_seq(enum swd_special_seq seq)
{
	int ret = ERROR_OK;

	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG_IO("SWD line reset");
		tamarin_line_reset();
		// ret = tamarin_write_bits(swd_seq_line_reset, 0, swd_seq_line_reset_len);
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		tamarin_line_reset();
		// ret = tamarin_write_bits(swd_seq_jtag_to_swd, 0, swd_seq_jtag_to_swd_len);
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		LOG_DEBUG("NOT IMPLEMENTED");
		assert(false);
		// ret = tamarin_write_bits(swd_seq_swd_to_jtag, 0, swd_seq_swd_to_jtag_len);
		break;
	case DORMANT_TO_SWD:
		LOG_DEBUG("DORMANT-to-SWD");

		LOG_DEBUG("NOT IMPLEMENTED");
		// assert(false);		// ret = tamarin_write_bits(swd_seq_dormant_to_swd, 0, swd_seq_dormant_to_swd_len);
		break;
	case SWD_TO_DORMANT:
		LOG_DEBUG("SWD-to-DORMANT");
		LOG_DEBUG("NOT IMPLEMENTED");
		// ret = tamarin_write_bits(swd_seq_swd_to_dormant, 0, swd_seq_swd_to_dormant_len);
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	return ret;
}

static const struct swd_driver tamarin_swd = {
	.init = tamarin_swd_init,
	.switch_seq = tamarin_swd_switch_seq,
	.read_reg = tamarin_swd_read_reg,
	.write_reg = tamarin_swd_write_reg,
	.run = tamarin_swd_run_queue,
};

const char *tamarin_serial_number = NULL;


static const struct command_registration serialnum_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};
static const char * const tamarin_transports[] = { "swd", NULL };

struct jtag_interface tamarin_adapter_driver = {
	.name = "tamarin",
	.commands = serialnum_command_handlers,
	.transports = tamarin_transports,
	.swd = &tamarin_swd,
	.init = tamarin_init,
	.quit = tamarin_quit,
	// .reset = tamarin_reset,
    // .reset = NULL, // TODO: Is this really supported by OpenOCD or will it null deref?
	.speed = tamarin_speed,
	.speed_div = tamarin_speed_div,
	.khz = tamarin_khz,
};

static int tamarin_usb_open(void)
{
	const uint16_t vids[] = { VID, 0 };
	const uint16_t pids[] = { PID, 0 };

	if (jtag_libusb_open(vids, pids, tamarin_serial_number,
			&tamarin_handle->usb_handle) != ERROR_OK) {
		LOG_ERROR("Failed to open or find the device");
		return ERROR_FAIL;
	}

	if (jtag_libusb_claim_interface(tamarin_handle->usb_handle, PICOPROBE_INTERFACE) != ERROR_OK) {
		LOG_ERROR("Failed to claim tamarin cable interface");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static void tamarin_usb_close(void)
{
	jtag_libusb_close(tamarin_handle->usb_handle);
}

static int tamarin_init(void)
{
	tamarin_handle = malloc(sizeof(struct tamarin));
	if (tamarin_handle == NULL) {
		LOG_ERROR("Failed to allocate memory");
		return ERROR_FAIL;
	}

	if (tamarin_usb_open() != ERROR_OK) {
		LOG_ERROR("Can't find a tamarin device! Please check device connections and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}


	tamarin_handle->queue_length = 0;
    
    return ERROR_OK;
}


static int tamarin_quit(void)
{
	tamarin_usb_close();
	return ERROR_OK;
}
