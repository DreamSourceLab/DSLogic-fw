/*
 * This file is part of the DSLogic-fw project.
 *
 * Copyright (C) 2014 DreamSourceLab <support@dreamsourcelab.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <fx2regs.h>
#include <fx2macros.h>
#include <setupdat.h>
#include <eputils.h>
#include <DSLogic.h>

/* ... */
extern volatile __bit recv_setup;
extern BYTE command;

BOOL handle_vendorcommand(BYTE cmd)
{
	/* Protocol implementation */
	switch (cmd) {
	case CMD_GET_FW_VERSION:
		EP0BUF[0] = DSLOGICFW_VER_MAJOR;
		EP0BUF[1] = DSLOGICFW_VER_MINOR;
		EP0BCH = 0;
		EP0BCL = 2;
		return TRUE;
		break;
	case CMD_GET_REVID_VERSION:
		EP0BUF[0] = REVID;
		EP0BCH = 0;
		EP0BCL = 1;
		return TRUE;
		break;
	case CMD_START:
	case CMD_SETTING:
	case CMD_CONTROL:
		command = cmd;
		EP0BCL = 0;
		return TRUE;
		break;
	case CMD_CONFIG:
		IOA |= 0x03;
		command = cmd;
		EP0BCL = 0;
		return TRUE;
		break;
	}
	return FALSE;
}

BOOL handle_get_interface(BYTE ifc, BYTE *alt_ifc)
{
	/* We only support interface 0, alternate interface 0. */
	if (ifc != 0)
		return FALSE;

	*alt_ifc = 0;
	return TRUE;
}

BOOL handle_set_interface(BYTE ifc, BYTE alt_ifc)
{
	/* We only support interface 0, alternate interface 0. */
	if (ifc != 0 || alt_ifc != 0)
		return FALSE;

	/* Perform procedure from TRM, section 2.3.7: */

	/* (1) TODO. */

	/* (2) Reset data toggles of the EPs in the interface. */
	/* Note: RESETTOGGLE() gets the EP number WITH bit 7 set/cleared. */
	RESETTOGGLE(0x86);
	RESETTOGGLE(0x02);

	/* (3) Restore EPs to their default conditions. */
	/* Note: RESETFIFO() gets the EP number WITHOUT bit 7 set/cleared. */
	RESETFIFO(0x06);
	RESETFIFO(0x02);
	/* TODO */

	/* (4) Clear the HSNAK bit. Not needed, fx2lib does this. */

	return TRUE;
}

BYTE handle_get_configuration(void)
{
	/* Only support configuration 1. */
	return 1;
}

BOOL handle_set_configuration(BYTE cfg)
{
	/* Only support configuration 1. */
	return (cfg == 1) ? TRUE : FALSE;
}

void sudav_isr(void) __interrupt SUDAV_ISR
{
	recv_setup = TRUE;
	CLEAR_SUDAV();
}

void sof_isr(void) __interrupt SOF_ISR __using 1
{
	CLEAR_SOF();
}

void usbreset_isr(void) __interrupt USBRESET_ISR
{
	handle_hispeed(FALSE);
	CLEAR_USBRESET();
}

void hispeed_isr(void) __interrupt HISPEED_ISR
{
	handle_hispeed(TRUE);
	CLEAR_HISPEED();
}
