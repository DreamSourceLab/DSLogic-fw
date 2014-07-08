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

#ifndef DSLOGICFW_INCLUDE_INTERFACE_H
#define DSLOGICFW_INCLUDE_INTERFACE_H

#include <stdbool.h>
#include <DSLogic.h>

void setup_gpif_waveforms(void);
void init_capture_intf(void);
void init_config_intf(void);
bool start_capture(void);
bool stop_capture();
void poll_intf(void);

#endif
