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

#ifndef DSLOGICFW_INCLUDE_DSLOGIC_H
#define DSLOGICFW_INCLUDE_DSLOGIC_H

#include <autovector.h>
#include <stdint.h>

#define SYNCDELAY() SYNCDELAY4

/*
 * Major and minor DSLogic-fw version numbers.
 * These can be queried by the host via CMD_GET_FW_VERSION.
 */
#define DSLOGICFW_VER_MAJOR	1
#define DSLOGICFW_VER_MINOR	0

/* Protocol commands */
#define CMD_GET_FW_VERSION		0xb0
#define CMD_GET_REVID_VERSION		0xb1
#define CMD_START			0xb2
#define CMD_CONFIG			0xb3
#define CMD_SETTING			0xb4
#define CMD_CONTROL			0xb5

#define CMD_START_FLAGS_WIDE_POS	5
#define CMD_START_FLAGS_CLK_SRC_POS	6
#define CMD_START_FLAGS_STOP_POS	7

#define CMD_START_FLAGS_SAMPLE_8BIT	(0 << CMD_START_FLAGS_WIDE_POS)
#define CMD_START_FLAGS_SAMPLE_16BIT	(1 << CMD_START_FLAGS_WIDE_POS)

#define CMD_START_FLAGS_CLK_30MHZ	(0 << CMD_START_FLAGS_CLK_SRC_POS)
#define CMD_START_FLAGS_CLK_48MHZ	(1 << CMD_START_FLAGS_CLK_SRC_POS)

#define CMD_START_FLAGS_STOP		(1 << CMD_START_FLAGS_STOP_POS)

struct cmd_start {
	uint8_t flags;
	uint8_t param0;
	uint8_t param1;
};

struct cmd_setting_count {
	uint8_t byte0;
	uint8_t byte1;
	uint8_t byte2;
};

struct cmd_cfg_count {
	uint8_t byte0;
	uint8_t byte1;
	uint8_t byte2;
};

struct cmd_control {
    	uint8_t byte0;
    	uint8_t byte1;
    	uint8_t byte2;
    	uint8_t byte3;
};

#endif
