#
#
#  This file is provided under a dual BSD/GPLv2 license.  When using or
#  redistributing this file, you may do so under either license.
#
#  GPL LICENSE SUMMARY
#
#  Copyright(c) 2007-2012 Intel Corporation. All rights reserved.
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of version 2 of the GNU General Public License as
#  published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful, but
#  WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
#  The full GNU General Public License is included in this distribution
#  in the file called LICENSE.GPL.
#
#  Contact Information:
#  intel.com
#  Intel Corporation
#  2200 Mission College Blvd.
#  Santa Clara, CA  95052
#  USA
#  (408) 765-8080
#
#
#  BSD LICENSE
#
#  Copyright(c) 2007-2012 Intel Corporation. All rights reserved.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in
#      the documentation and/or other materials provided with the
#      distribution.
#    * Neither the name of Intel Corporation nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#

export COMPONENT := thermal
export PWD := $(shell pwd)
ifndef THERMAL_BASE 
export THERMAL_BASE := $(PWD)
endif

include Makefile.inc

SUB_DIRS := driver Applications

<default>:
	@echo "below targets are supported:"
	@echo "all: build $(COMPONENT)"
	@echo "install: install $(COMPONENT)"
	@echo "clean: clean $(COMPONENT)"

.PHONY: all clean install $(COMPONENT) doc test debug

all: $(COMPONENT) 

$(COMPONENT): 
	@echo ">>>Building Components $(COMPONENT)"
	@echo build base is $(THERMAL_BASE)
	@$(foreach SUBDIR, $(SUB_DIRS), $(MAKE) -C $(SUBDIR) -f Makefile &&) exit 0;

.PHONY: install
install: install_dev install_target
	@echo ">>>Installed development and target files"

.PHONY: install_dev
install_dev: all
	@echo ">>>Copying all development files to $(BUILD_DEST)"
	mkdir -p $(BUILD_DEST)/bin
	mkdir -p $(BUILD_DEST)/lib
	mkdir -p $(BUILD_DEST)/lib/modules
	cp -f Applications/thermal/thermal $(BUILD_DEST)/bin/
	cp -f driver/thermal.ko $(BUILD_DEST)/lib/modules/
	if [ -e init_thermal ]; then mkdir -p $(BUILD_DEST)/etc/init.d/; cp -f init_thermal $(BUILD_DEST)/etc/init.d/thermal; fi

.PHONY: install_target
install_target: all
	@echo ">>>Copying all target files to $(FSROOT)"
	mkdir -p $(FSROOT)/bin
	mkdir -p $(FSROOT)/lib
	mkdir -p $(FSROOT)/lib/modules
	cp -f Applications/thermal/thermal $(FSROOT)/bin/
	if [ -e driver/thermal.ko ]; then cp -f driver/thermal.ko $(FSROOT)/lib/modules/; fi
	if [ -e init_thermal ]; then mkdir -p $(FSROOT)/etc/init.d/; cp -f init_thermal $(FSROOT)/etc/init.d/thermal; fi

debug: dbld doc test

.PHONY: dbld
dbld:
	@echo ">>>Do nothing"
#$(foreach SUBDIR, $(SUB_DIRS), $(MAKE) -e DEBUG=1 -C $(SUBDIR) -f Makefile  &&) exit 0;

doc:
	@echo ">>>Do nothing"
#make -C docs

test:
	@echo ">>>Do nothing"

clean: uninstall
	@echo ">>>Cleaning up Components $(COMPONENT)"
	$(foreach SUBDIR, $(SUB_DIRS), $(MAKE) -C $(SUBDIR) -f Makefile clean;)

.PHONY: uninstall
uninstall: uninstall_dev uninstall_target
	@echo ">>>Uninstalled development and target files"

.PHONY: uninstall_dev
uninstall_dev:
	@echo ">>>Removing edl development files from $(BUILD_DEST)"
	rm -f $(BUILD_DEST)/bin/thermal
	rm -f $(BUILD_DEST)/lib/modules/thermal.ko

.PHONY: uninstall_target
uninstall_target:
	@echo ">>>Removing edl target files from $(FSROOT)"
	rm -f $(FSROOT)/bin/thermal
	rm -f $(FSROOT)/lib/modules/thermal.ko

