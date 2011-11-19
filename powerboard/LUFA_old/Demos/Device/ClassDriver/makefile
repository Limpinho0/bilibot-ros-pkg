#
#             LUFA Library
#     Copyright (C) Dean Camera, 2011.
#
#  dean [at] fourwalledcubicle [dot] com
#           www.lufa-lib.org
#

# Makefile to build all the LUFA Device Demos. Call with "make all" to
# rebuild all Device demos.

# Projects are pre-cleaned before each one is built, to ensure any
# custom LUFA library build options are reflected in the compiled
# code.

all:


	$(MAKE) -C DualVirtualSerial clean
	$(MAKE) -C DualVirtualSerial all


	$(MAKE) -C Joystick clean
	$(MAKE) -C Joystick all

	$(MAKE) -C VirtualSerial clean
	$(MAKE) -C VirtualSerial all

%:
	$(MAKE) -C DualVirtualSerial $@
	$(MAKE) -C Joystick $@
	$(MAKE) -C VirtualSerial $@

