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

BASENAME = DSLogic

all-local: $(BASENAME).fw

EXTRA_DIST = dscr.a51

DSLogic:
	$(AM_V_at)cd ${top_builddir} && $(MAKE) DSLogic

if FOUND_OBJCOPY
$(BASENAME).fw: $(BASENAME).ihx
	$(AM_V_GEN)$(OBJCOPY) -Iihex $(BASENAME).ihx -Obinary $@
else
if FOUND_MAKEBIN
$(BASENAME).fw: $(BASENAME).ihx
	$(AM_V_GEN)$(MAKEBIN) -p < $(BASENAME).ihx > $@
endif
endif

RELS = ${top_builddir}/DSLogic.rel ${top_builddir}/interface.rel ${top_builddir}/usb.rel\
       ${builddir}/dscr.rel

$(RELS): DSLogic

$(builddir)/dscr.rel: dscr.a51
	$(AM_V_at)if test "x${abs_top_srcdir}" != "x${abs_top_builddir}"; then \
		$(INSTALL_DATA) ${srcdir}/dscr.a51 ${builddir}; \
	fi
	$(AM_V_GEN)$(SDAS8051) -logs -I${top_srcdir}/include dscr.a51

$(BASENAME).ihx: DSLogic $(RELS) $(top_builddir)/fx2lib/lib/fx2.lib $(top_builddir)/fx2lib/lib/interrupts/ints.lib
	$(AM_V_GEN)$(SDCC) -mmcs51 $(SDCC_FLAGS) -o $@ $(RELS) -L$(top_builddir)/fx2lib/lib fx2.lib interrupts/ints.lib

install-data-local: $(BASENAME).fw
	$(AM_V_at)$(MKDIR_P) $(DESTDIR)$(FIRMWARE_DIR)
	$(AM_V_at)$(INSTALL_DATA) $(BASENAME).fw $(DESTDIR)$(FIRMWARE_DIR)

uninstall-local:
	$(AM_V_at)-rm -f $(DESTDIR)$(FIRMWARE_DIR)/$(BASENAME).fw

clean-local:
	$(AM_V_at)-rm -f *.lst *.rel *.rst *.sym *.lnk *.map *.mem *.ihx *.fw
	$(AM_V_at)-rm -f *.cdb *.lk *.omf
