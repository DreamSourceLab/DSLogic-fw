/*
 * This file is part of the DSLogic-fw project.
 *
 * Copyright (C) 2012 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>
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


#include <eputils.h>
#include <fx2regs.h>
#include <fx2macros.h>
#include <delay.h>
#include <gpif.h>
#include <DSLogic.h>
#include <interface.h>

/* ... */
#define MSBW(word)      (BYTE)(((WORD)(word) >> 8) & 0xff)
#define LSBW(word)      (BYTE)((WORD)(word) & 0xff)

/* ... */
__bit capturing = FALSE;

const char __xdata WaveData[128] =     
{                                      
// Wave 0 
/* LenBr */ 0x08,     0x02,     0x93,     0x38,     0x01,     0x01,     0x01,     0x07,
/* Opcode*/ 0x01,     0x01,     0x03,     0x01,     0x00,     0x00,     0x00,     0x00,
/* Output*/ 0x07,     0x07,     0x05,     0x07,     0x07,     0x07,     0x07,     0x07,
/* LFun  */ 0x87,     0x36,     0xF0,     0x3F,     0x00,     0x00,     0x00,     0x3F,
// Wave 1 
///* LenBr */ 0xF0,     0x01,     0x3F,     0x01,     0x01,     0x01,     0x00,     0x07,
/* LenBr */ 0xE8,     0x01,     0x3F,     0x01,     0x01,     0x00,     0x00,     0x07,
/* Opcode*/ 0x07,     0x06,     0x01,     0x00,     0x00,     0x00,     0x00,     0x00,
/* Output*/ 0x04,     0x04,     0x06,     0x06,     0x06,     0x06,     0x06,     0x06,
/* LFun  */ 0x6E,     0x00,     0x2D,     0x00,     0x00,     0x00,     0x00,     0x3F,
// Wave 2 
/* LenBr */ 0x01,     0x01,     0x01,     0x01,     0x01,     0x01,     0x01,     0x07,
/* Opcode*/ 0x00,     0x00,     0x00,     0x00,     0x00,     0x00,     0x00,     0x00,
/* Output*/ 0x06,     0x06,     0x06,     0x06,     0x06,     0x06,     0x06,     0x06,
/* LFun  */ 0x00,     0x00,     0x00,     0x00,     0x00,     0x00,     0x00,     0x3F,
// Wave 3 
/* LenBr */ 0x01,     0x01,     0x01,     0x01,     0x01,     0x01,     0x01,     0x07,
/* Opcode*/ 0x00,     0x00,     0x00,     0x00,     0x00,     0x00,     0x00,     0x00,
/* Output*/ 0x06,     0x06,     0x06,     0x06,     0x06,     0x06,     0x06,     0x06,
/* LFun  */ 0x00,     0x00,     0x00,     0x00,     0x00,     0x00,     0x00,     0x3F,
};                     

void setup_gpif_waveforms(void)
{
	int i;

	// use dual autopointer feature... 
	AUTOPTRSETUP = 0x07;          // inc both pointers, 
	                              // ...warning: this introduces pdata hole(s)
	                              // ...at E67B (XAUTODAT1) and E67C (XAUTODAT2)
	// source
	AUTOPTRH1 = MSBW(&WaveData);
	AUTOPTRL1 = LSBW(&WaveData);
	// destination
	AUTOPTRH2 = 0xE4;
	AUTOPTRL2 = 0x00;
	// transfer
	for ( i = 0x00; i < 128; i++ )
	{
	  EXTAUTODAT2 = EXTAUTODAT1;
	}	
}

void init_capture_intf(void)
{
	IFCONFIG = 0xb6;
	// 7	IFCLKSRC=1   , FIFOs executes on internal clk source
	// 6	xMHz=0       , 30MHz internal clk rate
	// 5	IFCLKOE=1    , Drive IFCLK pin signal at 30MHz
	// 4	IFCLKPOL=1   , Invert IFCLK pin signal from internal clk
	// 3	ASYNC=0      , master samples synchronously
	// 2	GSTATE=1     , Drive GPIF states out on PORTE[2:0], debug WF
	// 1:0	IFCFG=10     , FX2 in GPIF master mode

	/* Abort currently executing GPIF waveform (if any). */
	GPIFABORT = 0xff;

	/* Setup the GPIF registers. */
	GPIFREADYCFG = 0x00;
	GPIFCTLCFG = 0x00;
	GPIFIDLECS = 0x00;
	/* CTL2 = 1; CTL1 = 1; CTL0 = 0*/
	GPIFIDLECTL = 0x06;
	/*
	 * Map index 0 in WAVEDATA to FIFORD.
	 * Map index 1 in WAVEDATA to FIFOWR.
	 * GPIFWFSELECT: [7:6] = SINGLEWR index, [5:4] = SINGLERD index,
	 *               [3:2] = FIFOWR index, [1:0] = FIFORD index
	 */
	GPIFWFSELECT = (0x3 << 6) | (0x2 << 4) | (0x1 << 2) | (0x0 << 0);
	GPIFREADYSTAT = 0x00;

	/* Reset the status. */
	capturing = FALSE;
}

void init_config_intf(void)
{
	IFCONFIG = 0xa6;
	// 7	IFCLKSRC=1   , FIFOs executes on internal clk source
	// 6	xMHz=0       , 30MHz internal clk rate
	// 5	IFCLKOE=1    , Drive IFCLK pin signal at 30MHz
	// 4	IFCLKPOL=0   , Don't invert IFCLK pin signal from internal clk
	// 3	ASYNC=0      , master samples synchronously
	// 2	GSTATE=1     , Drive GPIF states out on PORTE[2:0], debug WF
	// 1:0	IFCFG=10     , FX2 in GPIF master mode

	/* Abort currently executing GPIF waveform (if any). */
	GPIFABORT = 0xff;

	/* Setup the GPIF registers. */
	/* Internal RDY = 1; asynchronous RDY signals*/
	GPIFREADYCFG = 0xE0;
	GPIFCTLCFG = 0x00;
	GPIFIDLECS = 0x00;
	/* CTL2 (PROG_B) = 1; CTL1 (CSI_B) = 1; CTL0(RDWR_B) = 0*/
	GPIFIDLECTL = 0x06;
	/*
	 * Map index 0 in WAVEDATA to FIFORD.
	 * Map index 1 in WAVEDATA to FIFOWR.
	 * GPIFWFSELECT: [7:6] = SINGLEWR index, [5:4] = SINGLERD index,
	 *               [3:2] = FIFOWR index, [1:0] = FIFORD index
	 */
	GPIFWFSELECT = (0x3 << 6) | (0x2 << 4) | (0x1 << 2) | (0x0 << 0);
	GPIFREADYSTAT = 0x00;

        /**/
	EP2FIFOPFH = 0x40;
	EP2FIFOPFL = 0x01;
	EP2GPIFFLGSEL = 0x00;	

	/* */
	IOA &= 0xfd;
	IOA |= 0x02;
}


bool start_capture()
{
	/* Sample clear before each acquistion */
	IOA &= 0xf7;

	/* Abort currently executing GPIF waveform (if any). */
	GPIFABORT = 0xff;

	/* ... */
 	FIFORESET = 0x80;  // set NAKALL bit to NAK all transfers from host
	SYNCDELAY();
	FIFORESET = 0x06;  // reset EP6 FIFO
	SYNCDELAY();
	FIFORESET = 0x00;  // clear NAKALL bit to resume normal operation
	SYNCDELAY();	 

	/* Ensure GPIF is idle before reconfiguration. */
	while (!(GPIFTRIG & 0x80));

	/* Clear the stop flag */
	GPIFREADYCFG &= 0x7f;

	/* Configure the EP6 FIFO. */
	EP6FIFOCFG = bmAUTOIN | bmWORDWIDE;
	SYNCDELAY();

	/* sample clear end */
	IOA |= 0x08;

	/* Execute the whole GPIF waveform once. */
	gpif_set_tc16(1);

	/* Perform the initial GPIF read. */
	gpif_fifo_read(GPIF_EP6);

	/* Update the status. */
	capturing = TRUE;
	IOA |= 0x04;

	return true;
}

bool stop_capture()
{
	GPIFREADYCFG |= bmBIT7;
	IOA &= 0xfb;
	return true;
}

void poll_intf(void)
{
	/* Polling capture status. */
	if (capturing && (GPIFTRIG & 0x80)) {
		/* Activate NAK-ALL to avoid race conditions. */
		FIFORESET = 0x80;
		SYNCDELAY();

		/* Switch to manual mode. */
		EP6FIFOCFG = 0;
		SYNCDELAY();

		/* Reset EP6. */
		FIFORESET = 0x06;
		SYNCDELAY();

		/* Return to auto mode. */
		EP6FIFOCFG = bmAUTOIN;
		SYNCDELAY();

		/* Release NAK-ALL. */
		FIFORESET = 0x00;
		SYNCDELAY();

		capturing = FALSE;
	}
}
