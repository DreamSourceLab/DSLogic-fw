##
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

SRC = ep0ack_isr.c ep0in_isr.c ep0out_isr.c ep0ping_isr.c ep1in_isr.c \
      ep1out_isr.c ep1ping_isr.c ep2ef_isr.c ep2ff_isr.c ep2isoerr_isr.c \
      ep2_isr.c ep2pf_isr.c ep2ping_isr.c ep4ef_isr.c ep4ff_isr.c \
      ep4isoerr_isr.c ep4_isr.c ep4pf_isr.c ep4ping_isr.c ep6ef_isr.c \
      ep6ff_isr.c ep6isoerr_isr.c ep6_isr.c ep6pf_isr.c ep6ping_isr.c \
      ep8ef_isr.c ep8ff_isr.c ep8isoerr_isr.c ep8_isr.c ep8pf_isr.c \
      ep8ping_isr.c errlimit_isr.c gpifdone_isr.c gpifwf_isr.c hispeed_isr.c \
      ibn_isr.c sof_isr.c spare_isr.c sudav_isr.c suspend_isr.c sutok_isr.c \
      usbreset_isr.c

REL = ep0ack_isr.rel ep0in_isr.rel ep0out_isr.rel ep0ping_isr.rel \
      ep1in_isr.rel ep1out_isr.rel ep1ping_isr.rel ep2ef_isr.rel \
      ep2ff_isr.rel ep2isoerr_isr.rel ep2_isr.rel ep2pf_isr.rel \
      ep2ping_isr.rel ep4ef_isr.rel ep4ff_isr.rel ep4isoerr_isr.rel \
      ep4_isr.rel ep4pf_isr.rel ep4ping_isr.rel ep6ef_isr.rel ep6ff_isr.rel \
      ep6isoerr_isr.rel ep6_isr.rel ep6pf_isr.rel ep6ping_isr.rel \
      ep8ef_isr.rel ep8ff_isr.rel ep8isoerr_isr.rel ep8_isr.rel ep8pf_isr.rel \
      ep8ping_isr.rel errlimit_isr.rel gpifdone_isr.rel gpifwf_isr.rel \
      hispeed_isr.rel ibn_isr.rel sof_isr.rel spare_isr.rel sudav_isr.rel \
      suspend_isr.rel sutok_isr.rel usbreset_isr.rel

EXTRA_DIST = $(SRC)

all-local: ints.lib

ints.lib: $(REL)
	$(AM_V_GEN)$(SDCCLIB) ints.lib $?

.c.rel:
	$(AM_V_GEN)$(SDCC) -mmcs51 -I${top_srcdir}/fx2lib/include -c $< -o $@

clean-local:
	$(AM_V_at)-rm -f *.asm *.lst *.rel *.sym ints.lib

