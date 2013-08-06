#******************************************************************************
#
# Makefile - Rules for building the Marble Maze Reciever.
#
# Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
# TI Information - Selective Disclosure
#
#******************************************************************************

#
# Defines the part type that this project uses.
#
PART=TM4C123GH6PM

#
# The base directory for TivaWare.
#
ROOT=../../../..

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find source files that do not live in this directory.
#
VPATH=../drivers
VPATH+=../../../../utils

#
# Where to find header files that do not live in the source directory.
#
IPATH=..
IPATH+=../../../..

#
# The default rule, which causes the Marble Maze Reciever to be built.
#
all: ${COMPILER}
all: ${COMPILER}/maze_receiver.axf
##### INTERNAL BEGIN #####
all_comp:
	@for i in `grep COMPILER= ${ROOT}/makedefs | awk -F= '{print $$2}'`; \
	 do                                                                  \
	     ${MAKE} COMPILER=$${i};                                            \
	 done

#
# A rule to regenerate the Makefile if config.in has changed.
#
Makefile: config.in
	@perl ${ROOT}/scripts/buildproj.pl -s -f config.in
##### INTERNAL END #####

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}
##### INTERNAL BEGIN #####
	@rm -rf settings ${wildcard *.Bak} ${wildcard *.dep}
clean_comp:
	@for i in `grep COMPILER= ${ROOT}/makedefs | awk -F= '{print $$2}'`; \
	 do                                                                  \
	     ${MAKE} COMPILER=$${i} clean;                                      \
	 done

#
# Cleans out everything, including the Makefile.  Should return this directory
# to fresh from subversion status.
#
realclean:: clean
	@rm -rf ccs codered ewarm gcc rvmdk sourcerygxx ccstempws
	@rm -f ${wildcard *.Uv2} ${wildcard *.Opt}
	@rm -f ${wildcard *.uvopt} ${wildcard *.uvproj}
	@rm -f ${wildcard *.ew?} ${wildcard *.xcl}
	@rm -f ${wildcard *.sgxx}
	@rm -f ccs_proj_args.txt
	@rm -f maze_receiver_ccs.cmd
	@rm -f maze_receiver_codered.ld
	@rm -f maze_receiver.ld
	@rm -f maze_receiver_sourcerygxx.ld
	@rm -f ${wildcard *-rom.ld}
	@rm -f maze_receiver.icf
	@rm -f maze_receiver.sct
	@rm -f ${wildcard *.xml}
	@rm -f ${wildcard startup_*}
	@rm -f readme.txt
	@rm -f Makefile cr_project.xml
##### INTERNAL END #####

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

#
# Rules for building the Marble Maze Reciever.
#
${COMPILER}/maze_receiver.axf: ${COMPILER}/buttons.o
${COMPILER}/maze_receiver.axf: ${COMPILER}/lprf.o
${COMPILER}/maze_receiver.axf: ${COMPILER}/maze_receiver.o
${COMPILER}/maze_receiver.axf: ${COMPILER}/motion.o
${COMPILER}/maze_receiver.axf: ${COMPILER}/remoti_npi.o
${COMPILER}/maze_receiver.axf: ${COMPILER}/remoti_rtis.o
${COMPILER}/maze_receiver.axf: ${COMPILER}/remoti_uart.o
${COMPILER}/maze_receiver.axf: ${COMPILER}/rgb.o
${COMPILER}/maze_receiver.axf: ${COMPILER}/ringbuf.o
${COMPILER}/maze_receiver.axf: ${COMPILER}/startup_${COMPILER}.o
${COMPILER}/maze_receiver.axf: ${COMPILER}/uartstdio.o
${COMPILER}/maze_receiver.axf: ${ROOT}/sensorlib/${COMPILER}/libsensor.a
${COMPILER}/maze_receiver.axf: ${ROOT}/driverlib/${COMPILER}/libdriver.a
##### INTERNAL BEGIN #####
ifeq (${COMPILER}, ccs)
${COMPILER}/maze_receiver.axf: maze_receiver_ccs.cmd
endif
SCATTERccs_maze_receiver=maze_receiver_ccs.cmd
ifeq (${COMPILER}, codered)
${COMPILER}/maze_receiver.axf: maze_receiver_codered.ld
endif
SCATTERcodered_maze_receiver=maze_receiver_codered.ld
ifeq (${COMPILER}, ewarm)
${COMPILER}/maze_receiver.axf: maze_receiver.icf
endif
SCATTERewarm_maze_receiver=maze_receiver.icf
ifeq (${COMPILER}, gcc)
##### INTERNAL END #####
${COMPILER}/maze_receiver.axf: maze_receiver.ld
##### INTERNAL BEGIN #####
endif
##### INTERNAL END #####
SCATTERgcc_maze_receiver=maze_receiver.ld
##### INTERNAL BEGIN #####
ifeq (${COMPILER}, rvmdk)
${COMPILER}/maze_receiver.axf: maze_receiver.sct
endif
SCATTERrvmdk_maze_receiver=maze_receiver.sct
ifeq (${COMPILER}, sourcerygxx)
${COMPILER}/maze_receiver.axf: maze_receiver_sourcerygxx.ld
endif
SCATTERsourcerygxx_maze_receiver=lm4f120h5qr-rom.ld -T maze_receiver_sourcerygxx.ld
##### INTERNAL END #####
ENTRY_maze_receiver=ResetISR
##### INTERNAL BEGIN #####
CFLAGSccs=-DTARGET_IS_BLIZZARD_RB1 -DUART_BUFFERED
##### INTERNAL END #####
##### INTERNAL BEGIN #####
CFLAGScodered=-DTARGET_IS_BLIZZARD_RB1 -DUART_BUFFERED
##### INTERNAL END #####
##### INTERNAL BEGIN #####
CFLAGSewarm=-DTARGET_IS_BLIZZARD_RB1 -DUART_BUFFERED
##### INTERNAL END #####
CFLAGSgcc=-DTARGET_IS_BLIZZARD_RB1 -DUART_BUFFERED
##### INTERNAL BEGIN #####
CFLAGSrvmdk=-DTARGET_IS_BLIZZARD_RB1 -DUART_BUFFERED
##### INTERNAL END #####
##### INTERNAL BEGIN #####
CFLAGSsourcerygxx=-DTARGET_IS_BLIZZARD_RB1 -DUART_BUFFERED
##### INTERNAL END #####

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
##### INTERNAL BEGIN #####
ifneq (${MAKECMDGOALS},realclean)
##### INTERNAL END #####
-include ${wildcard ${COMPILER}/*.d} __dummy__
##### INTERNAL BEGIN #####
endif
##### INTERNAL END #####
endif
