###
## This file is part of the DSLogic-fw project.
##
## Copyright (C) 2013 Uwe Hermann <uwe@hermann-uwe.de>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, write to the Free Software
## Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
##

SUBDIRS = interrupts

SRC = delay.c eputils.c gpif.c i2c.c serial.c delay.c setupdat.c

REL = delay.rel eputils.rel gpif.rel i2c.rel serial.rel delay.rel setupdat.rel

EXTRA_DIST = $(SRC) int4av.a51 usbav.a51

all: fx2.lib

fx2.lib: $(REL) int4av.rel usbav.rel
	$(AM_V_GEN)$(SDCCLIB) fx2.lib $?

usbav.rel: usbav.a51
	$(AM_V_at)if test "x${abs_top_srcdir}" != "x${abs_top_builddir}"; then \
		$(INSTALL_DATA) ${srcdir}/usbav.a51 ${builddir}; \
	fi
	$(AM_V_GEN)$(SDAS8051) -logs -I${top_srcdir}/include usbav.a51

int4av.rel: int4av.a51
	$(AM_V_at)if test "x${abs_top_srcdir}" != "x${abs_top_builddir}"; then \
		$(INSTALL_DATA) ${srcdir}/int4av.a51 ${builddir}; \
	fi
	$(AM_V_GEN)$(SDAS8051) -logs -I${top_srcdir}/include int4av.a51

.c.rel:
	$(AM_V_GEN)$(SDCC) -mmcs51 -I${top_srcdir}/fx2lib/include -c $< -o $@

clean-local:
	$(AM_V_at)-rm -f *.asm *.lst *.rel *.sym fx2.lib

