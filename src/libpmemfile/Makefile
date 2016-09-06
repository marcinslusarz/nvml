# Copyright 2016, Intel Corporation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#
#     * Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#
# src/libpmemfile/Makefile -- Makefile for libpmemfile project
#

TARGETS = corelib
CLEAN_TARGETS = $(TARGETS)
CLOBBER_TARGETS = $(TARGETS)
CSTYLE_TARGETS = $(TARGETS)
INSTALL_TARGETS = $(TARGETS)

all: $(TARGETS)
install: $(INSTALL_TARGETS:=-install)
uninstall: $(INSTALL_TARGETS:=-uninstall)
clean: $(CLEAN_TARGETS:=-clean)
clobber: $(CLOBBER_TARGETS:=-clobber)
cstyle: $(CSTYLE_TARGETS:=-cstyle)

%-install: %
	$(MAKE) -C $* install

%-uninstall: %
	$(MAKE) -C $* uninstall

%-clean:
	$(MAKE) -C $* clean

%-clobber:
	$(MAKE) -C $* clobber

%-cstyle:
	$(MAKE) -C $* cstyle

preload tools fuse: corelib

$(TARGETS):
	$(MAKE) -C $@

.PHONY: all install uninstall clean clobber cstyle $(TARGETS)
