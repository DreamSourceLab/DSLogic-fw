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
#include <delay.h>
#include <setupdat.h>
#include <eputils.h>
#include <gpif.h>
#include <i2c.h>
#include <DSLogic.h>
#include <interface.h>

/* ... */
#define GPIFTRIGWR 0
#define GPIFTRIGRD 4

/* ... */
volatile __bit recv_setup;
BYTE command;
BOOL cfg_enable = FALSE;
BOOL set_enable = FALSE;
BOOL set_init = FALSE;
BOOL set_dso_ctrl = FALSE;
BYTE setting_count_b0;
BYTE setting_count_b1;
BYTE setting_count_b2;
char dsoConfig[4] = {0x00, 0xc8, 0x61, 0x55};

static void setup_endpoints(void)
{
	/* Setup EP2 (out). */
	EP2CFG = (1 << 7) |		  /* EP is valid/activated */
		 (0 << 6) |		  /* EP direction: OUT */
		 (1 << 5) | (0 << 4) |	  /* EP Type: bulk */
		 (0 << 3) |		  /* EP buffer size: 1024 */
		 (0 << 2) |		  /* Reserved. */
		 (0 << 1) | (0 << 0);	  /* EP buffering: double buffering */
	SYNCDELAY();

	/* Setup EP6 (IN). */
	EP6CFG = (1 << 7) |		  /* EP is valid/activated */
		 (1 << 6) |		  /* EP direction: IN */
		 (1 << 5) | (0 << 4) |	  /* EP Type: bulk */
		 (0 << 3) |		  /* EP buffer size: 1024 */
		 (0 << 2) |		  /* Reserved. */
		 (0 << 1) | (0 << 0);	  /* EP buffering: double buffering */
	SYNCDELAY();


	/* Disable all other EPs (EP1, EP4, and EP8). */
	EP1INCFG &= ~bmVALID;
	SYNCDELAY();
	EP1OUTCFG &= ~bmVALID;
	SYNCDELAY();
	EP4CFG &= ~bmVALID;
	SYNCDELAY();
	EP8CFG &= ~bmVALID;
	SYNCDELAY();

	RESETFIFOS();
	
	/* EP2 */
 	EP2FIFOCFG = 0x00; // allow core to see zero to one transition of auto out bit
	SYNCDELAY();
 	EP2FIFOCFG = 0x10; // auto out mode, disable PKTEND zero length send, byte operation
	SYNCDELAY();
	EP2GPIFFLGSEL = 0x01; // For EP2OUT, GPIF uses emtpy flag 
	SYNCDELAY();
	

	/* EP6 */
	/* EP6: Enable AUTOIN mode. Set FIFO width to 8bits. */
	EP6FIFOCFG = bmAUTOIN;
	SYNCDELAY();
	/* EP6: Auto-commit 512 (0x200) byte packets (due to AUTOIN = 1). */
	EP6AUTOINLENH = 0x02;
	SYNCDELAY();
	EP6AUTOINLENL = 0x00;
	SYNCDELAY();
	/* EP6: Set the GPIF flag to 'full'. */
	EP6GPIFFLGSEL = (1 << 1) | (0 << 1);
	SYNCDELAY();

	/* PA1~PA0: LED; PA2: sample enable; PA3: sample clear */
	OEA = 0x0f;
	IOA = 0x00;
}

void DSLogic_init(void)
{
	recv_setup = FALSE;
	command = 0;

	/* Renumerate. */
	RENUMERATE_UNCOND();

	/* Set 8051 run @48MHz */
	SETCPUFREQ(CLK_48M);

	/* USB init */
	USE_USB_INTS();
	ENABLE_SOF();
	ENABLE_HISPEED();
	ENABLE_USBRESET();
	ENABLE_SUDAV();

	/* Global (8051) interrupt enable. */
	EA = 1;

	/* Setup the endpoints. */
	setup_endpoints();

	/* GPIF Wavedata Init*/
	setup_gpif_waveforms();
}

void DSLogic_poll(void)
{
	if (recv_setup) {
		handle_setupdata();
		recv_setup = FALSE;
	}

	switch (command) {
	case CMD_START:
		if ((EP0CS & bmEPBUSY) != 0)
			break;

		if (EP0BCL == sizeof(struct cmd_start)) {
			if ((*(uint8_t *)EP0BUF) & CMD_START_FLAGS_STOP)
				stop_capture();
			else
				start_capture();
		}

		/* Acknowledge the command. */
		command = 0;
		break;

	case CMD_CONFIG:
		if ((EP0CS & bmEPBUSY) != 0)
			break;

		if (EP0BCL == sizeof(struct cmd_cfg_count)) {
      			init_config_intf();   
  			FIFORESET = 0x80;  // set NAKALL bit to NAK all transfers from host
  			SYNCDELAY();
  			FIFORESET = 0x02;  // reset EP2 FIFO
  			SYNCDELAY();
  			FIFORESET = 0x06;  // reset EP6 FIFO
  			SYNCDELAY();
  			FIFORESET = 0x00;  // clear NAKALL bit to resume normal operation
  			SYNCDELAY();	 

  			EP2FIFOCFG = 0x00; // allow core to see zero to one transition of auto out bit
  			SYNCDELAY();
  			EP2FIFOCFG = 0x10; // auto out mode, disable PKTEND zero length send, word ops
  			SYNCDELAY();
  			EP6FIFOCFG = 0x08; // auto in mode, disable PKTEND zero length send, word ops
  			SYNCDELAY();   	 
			
			GPIFIDLECTL &= 0xFB;	//PROG_B signal low
       			delay(1);		//PROG_B signal kept asserted for 1 ms
			GPIFIDLECTL |= 0x04;	//PROG_B signal high
  			SYNCDELAY();

			// setup transaction count
	    		GPIFTCB0 = ((const struct cmd_cfg_count *)EP0BUF)->byte0;   
			SYNCDELAY();
	    		GPIFTCB1 = ((const struct cmd_cfg_count *)EP0BUF)->byte1;            		
			SYNCDELAY();
	    		GPIFTCB2 = ((const struct cmd_cfg_count *)EP0BUF)->byte2;
			SYNCDELAY();

			cfg_enable = TRUE;			
		}

		/* Acknowledge the command. */
		command = 0;
		break;

	case CMD_SETTING:
		if ((EP0CS & bmEPBUSY) != 0)
			break;

		if (EP0BCL == sizeof(struct cmd_setting_count)) {
			GPIFABORT = 0xff;
  			SYNCDELAY();	 
			EP2FIFOCFG = 0x11; // auto out mode, disable PKTEND zero length send, word operation
			SYNCDELAY();
			setting_count_b0 = ((const struct cmd_setting_count *)EP0BUF)->byte0;
			setting_count_b1 = ((const struct cmd_setting_count *)EP0BUF)->byte1;
			setting_count_b2 = ((const struct cmd_setting_count *)EP0BUF)->byte2;
			set_enable = TRUE;			
		}

		/* Acknowledge the command. */
		command = 0;
		break;

	case CMD_CONTROL:
		if ((EP0CS & bmEPBUSY) != 0)
			break;

		if (EP0BCL == sizeof(struct cmd_control)) {
			dsoConfig[0] = ((const struct cmd_control *)EP0BUF)->byte0;
			dsoConfig[1] = ((const struct cmd_control *)EP0BUF)->byte1;
			dsoConfig[2] = ((const struct cmd_control *)EP0BUF)->byte2;
			dsoConfig[3] = ((const struct cmd_control *)EP0BUF)->byte3;
			set_dso_ctrl = TRUE;
		}

		/* Acknowledge the command. */
		command = 0;
		break;
	default:
		/* Unimplemented command. */
		command = 0;
		break;
	}
	if (cfg_enable && (GPIFTRIG & 0x80)) {        		// if GPIF interface IDLE
		if ( (EP24FIFOFLGS & 0x01) && (GPIFREADYSTAT & 0x01)) {
			// if there's a packet in the peripheral domain for EP2
		 	// and FPGA is ready to receive the configuration bitstream
			IFCONFIG = 0xA6;
				// 7	IFCLKSRC=1   , FIFOs executes on internal clk source
				// 6	xMHz=0       , 30MHz internal clk rate
				// 5	IFCLKOE=1    , Drive IFCLK pin signal at 30MHz
				// 4	IFCLKPOL=0   , Don't invert IFCLK pin signal from internal clk
				// 3	ASYNC=0      , master samples asynchronous
				// 2	GSTATE=1     , Drive GPIF states out on PORTE[2:0], debug WF
				// 1:0	IFCFG=10, FX2 in GPIF master mode
			SYNCDELAY();
		
	       		//delay(1);				//avoid CSI_B deasserted during sync words
			GPIFTRIG = GPIFTRIGWR | GPIF_EP2;  	// launch GPIF FIFO WRITE Transaction from EP2 FIFO
			SYNCDELAY();
		 
		    	while( !( GPIFTRIG & 0x80 ) );      	// poll GPIFTRIG.7 GPIF Done bit
			SYNCDELAY();
		 	cfg_enable= FALSE;                 	//end of configuration

			/* Put the FX2 into GPIF master mode and setup the GPIF. */
			init_capture_intf();

			if (GPIFREADYSTAT & 0x02) {	// FPGA Configure Done
				IOA |= 0x01;
				IOA &= 0xf5;
				delay(1);
				IOA |= 0x08;
			} else {
				IOA &= 0xfc;
			}
		 }	
	}
	if (set_enable && (GPIFTRIG & 0x80)) {        		// if GPIF interface IDLE
		if (!(EP24FIFOFLGS & 0x02)) {
			SYNCDELAY();
		    	GPIFTCB2 = setting_count_b2;   
			SYNCDELAY();
		    	GPIFTCB1 = setting_count_b1;            // fpga setting count
			SYNCDELAY();
		    	GPIFTCB0 = setting_count_b0;
			SYNCDELAY();
		
		    	GPIFTRIG = GPIFTRIGWR | GPIF_EP2;  	// launch GPIF FIFO WRITE Transaction from EP2 FIFO
			SYNCDELAY();
		 
		    	while( !( GPIFTRIG & 0x80 ) );      	// poll GPIFTRIG.7 GPIF Done bit
			SYNCDELAY();
		 	set_enable= FALSE;                 	//end of configuration

			/* Put the FX2 into GPIF master mode and setup the GPIF. */
			init_capture_intf();
		 }	
	}

	if (set_dso_ctrl) {
    		i2c_write (0x51, 0, NULL, 4, dsoConfig);
		set_dso_ctrl = FALSE;
	}

	poll_intf();
}

void main(void)
{
	DSLogic_init();
	while (1)
		DSLogic_poll();
}

