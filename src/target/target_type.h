/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008,2009 Øyvind Harboe                            *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef TARGET_TYPE_H
#define TARGET_TYPE_H

#include "types.h"

struct target;

/**
 * This holds methods shared between all instances of a given target
 * type.  For example, all Cortex-M3 targets on a scan chain share
 * the same method table.
 */
struct target_type
{
	/**
	 * Name of this type of target.  Do @b not access this
	 * field directly, use target_type_name() instead.
	 */
	char *name;

	/* poll current target status */
	int (*poll)(struct target *target);
	/* Invoked only from target_arch_state().
	 * Issue USER() w/architecture specific status.  */
	int (*arch_state)(struct target *target);

	/* target request support */
	int (*target_request_data)(struct target *target, uint32_t size, uint8_t *buffer);

	/* halt will log a warning, but return ERROR_OK if the target is already halted. */
	int (*halt)(struct target *target);
	int (*resume)(struct target *target, int current, uint32_t address, int handle_breakpoints, int debug_execution);
	int (*step)(struct target *target, int current, uint32_t address, int handle_breakpoints);

	/* target reset control. assert reset can be invoked when OpenOCD and
	 * the target is out of sync.
	 *
	 * A typical example is that the target was power cycled while OpenOCD
	 * thought the target was halted or running.
	 *
	 * assert_reset() can therefore make no assumptions whatsoever about the
	 * state of the target
	 *
	 * Before assert_reset() for the target is invoked, a TRST/tms and
	 * chain validation is executed. TRST should not be asserted
	 * during target assert unless there is no way around it due to
	 * the way reset's are configured.
	 *
	 */
	int (*assert_reset)(struct target *target);
	int (*deassert_reset)(struct target *target);
	int (*soft_reset_halt_imp)(struct target *target);
	int (*soft_reset_halt)(struct target *target);

	/**
	 * Target register access for GDB.  Do @b not call this function
	 * directly, use target_get_gdb_reg_list() instead.
	 *
	 * Danger! this function will succeed even if the target is running
	 * and return a register list with dummy values.
	 *
	 * The reason is that GDB connection will fail without a valid register
	 * list, however it is after GDB is connected that monitor commands can
	 * be run to properly initialize the target
	 */
	int (*get_gdb_reg_list)(struct target *target, struct reg **reg_list[], int *reg_list_size);

	/* target memory access
	* size: 1 = byte (8bit), 2 = half-word (16bit), 4 = word (32bit)
	* count: number of items of <size>
	*/
	int (*read_memory_imp)(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);
	/**
	 * Target memory read callback.  Do @b not call this function
	 * directly, use target_read_memory() instead.
	 */
	int (*read_memory)(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);
	int (*write_memory_imp)(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);
	/**
	 * Target memory write callback.  Do @b not call this function
	 * directly, use target_write_memory() instead.
	 */
	int (*write_memory)(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);

	/**
	 * Write target memory in multiples of 4 bytes, optimized for
	 * writing large quantities of data.  Do @b not call this
	 * function directly, use target_bulk_write_memory() instead.
	 */
	int (*bulk_write_memory)(struct target *target, uint32_t address, uint32_t count, uint8_t *buffer);

	int (*checksum_memory)(struct target *target, uint32_t address, uint32_t count, uint32_t* checksum);
	int (*blank_check_memory)(struct target *target, uint32_t address, uint32_t count, uint32_t* blank);

	/*
	 * target break-/watchpoint control
	 * rw: 0 = write, 1 = read, 2 = access
	 *
	 * Target must be halted while this is invoked as this
	 * will actually set up breakpoints on target.
	 *
	 * The breakpoint hardware will be set up upon adding the first breakpoint.
	 *
	 * Upon GDB connection all breakpoints/watchpoints are cleared.
	 */
	int (*add_breakpoint)(struct target *target, struct breakpoint *breakpoint);

	/* remove breakpoint. hw will only be updated if the target is currently halted.
	 * However, this method can be invoked on unresponsive targets.
	 */
	int (*remove_breakpoint)(struct target *target, struct breakpoint *breakpoint);
	int (*add_watchpoint)(struct target *target, struct watchpoint *watchpoint);
	/* remove watchpoint. hw will only be updated if the target is currently halted.
	 * However, this method can be invoked on unresponsive targets.
	 */
	int (*remove_watchpoint)(struct target *target, struct watchpoint *watchpoint);

	/* target algorithm support */
	int (*run_algorithm_imp)(struct target *target, int num_mem_params, struct mem_param *mem_params, int num_reg_params, struct reg_param *reg_param, uint32_t entry_point, uint32_t exit_point, int timeout_ms, void *arch_info);
	/**
	 * Target algorithm support.  Do @b not call this method directly,
	 * use target_run_algorithm() instead.
	 */
	int (*run_algorithm)(struct target *target, int num_mem_params, struct mem_param *mem_params, int num_reg_params, struct reg_param *reg_param, uint32_t entry_point, uint32_t exit_point, int timeout_ms, void *arch_info);

	const struct command_registration *commands;

	/* called when target is created */
	int (*target_create)(struct target *target, Jim_Interp *interp);

	/* called for various config parameters */
	/* returns JIM_CONTINUE - if option not understood */
	/* otherwise: JIM_OK, or JIM_ERR, */
	int (*target_jim_configure)(struct target *target, Jim_GetOptInfo *goi);

	/* target commands specifically handled by the target */
	/* returns JIM_OK, or JIM_ERR, or JIM_CONTINUE - if option not understood */
	int (*target_jim_commands)(struct target *target, Jim_GetOptInfo *goi);

	/**
	 * This method is used to perform target setup that requires
	 * JTAG access.
	 *
	 * This may be called multiple times.  It is called after the
	 * scan chain is initially validated, or later after the target
	 * is enabled by a JRC.  It may also be called during some
	 * parts of the reset sequence.
	 *
	 * For one-time initialization tasks, use target_was_examined()
	 * and target_set_examined().  For example, probe the hardware
	 * before setting up chip-specific state, and then set that
	 * flag so you don't do that again.
	 */
	int (*examine)(struct target *target);

	/* Set up structures for target.
	 *
	 * It is illegal to talk to the target at this stage as this fn is invoked
	 * before the JTAG chain has been examined/verified
	 * */
	int (*init_target)(struct command_context *cmd_ctx, struct target *target);

	/* translate from virtual to physical address. Default implementation is successful
	 * no-op(i.e. virtual==physical).
	 */
	int (*virt2phys)(struct target *target, uint32_t address, uint32_t *physical);

	/* read directly from physical memory. caches are bypassed and untouched.
	 *
	 * If the target does not support disabling caches, leaving them untouched,
	 * then minimally the actual physical memory location will be read even
	 * if cache states are unchanged, flushed, etc.
	 *
	 * Default implementation is to call read_memory.
	 */
	int (*read_phys_memory)(struct target *target, uint32_t phys_address, uint32_t size, uint32_t count, uint8_t *buffer);

	/*
	 * same as read_phys_memory, except that it writes...
	 */
	int (*write_phys_memory)(struct target *target, uint32_t phys_address, uint32_t size, uint32_t count, uint8_t *buffer);

	int (*mmu)(struct target *target, int *enabled);

	/* Read coprocessor - arm specific. Default implementation returns error. */
	int (*mrc)(struct target *target, int cpnum, uint32_t op1, uint32_t op2, uint32_t CRn, uint32_t CRm, uint32_t *value);

	/* Write coprocessor. Default implementation returns error.  */
	int (*mcr)(struct target *target, int cpnum, uint32_t op1, uint32_t op2, uint32_t CRn, uint32_t CRm, uint32_t value);
};

#endif // TARGET_TYPE_H
