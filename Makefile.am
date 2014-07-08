##
## This file is part of the DSLogic-fw project.
##
## Copyright (C) 2014 DreamSourceLab <dreamsourcelab@dreamsourcelab.com>
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

ACLOCAL_AMFLAGS = -I autostuff

SUBDIRS = include fx2lib hw

EXTRA_DIST = DSLogic.c interface.c usb.c

.PHONY: DSLogic
DSLogic: DSLogic.rel interface.rel usb.rel

.c.rel:
	$(AM_V_GEN)$(SDCC) -mmcs51 -I$(srcdir)/include -I${srcdir}/fx2lib/include -c $< -o $@

MAINTAINERCLEANFILES = ChangeLog

.PHONY: ChangeLog
ChangeLog:
	$(AM_V_at)git --git-dir $(top_srcdir)/.git log > ChangeLog || touch ChangeLog

dist-hook: ChangeLog

# Creates a convenience tarball with just the generated *.fw files, so that
# the users don't have to build them manually.
TARBALL=$(PACKAGE)-bin-$(VERSION)
distbin: all ChangeLog
	$(AM_V_at)$(MKDIR_P) $(TARBALL)
	$(AM_V_at)$(INSTALL_DATA) hw/*.fw $(TARBALL)
	$(AM_V_at)$(INSTALL_DATA) README NEWS ChangeLog COPYING* $(TARBALL)
	$(AM_V_at)tar -c -z -f $(TARBALL).tar.gz $(TARBALL)
	$(AM_V_at)-rm -rf $(TARBALL)
	$(AM_V_at)-rm -f ChangeLog

clean-local:
	$(AM_V_at)-rm -f *.asm *.lst *.rel *.rst *.sym

