
include ../Makefile.param

CFLAGS += -I$(PWD)/sample
CFLAGS += -I/$(PWD)/../SDL2-2.26.1/install/include/SDL2 -I/$(PWD)/../SDL2_ttf-2.0.18/install/include/SDL2/ -L/$(PWD)/../SDL2-2.26.1/install/lib -L/$(PWD)/../SDL2_ttf-2.0.18/install/lib -lSDL2 -lSDL2_ttf -lpthread -Wl,--rpath-link=/$(PWD)/../freetype-2.9/install/lib
CFLAGS += -I/$(PWD)/../../../out/include/rtsp_lib

SMP_SRCS := $(wildcard *.c)
SMP_SRCS += $(wildcard ./sample/*.c)
SMP_SRCS += $(wildcard $(PWD)/../common/*.c)

TARGET := sample_ive_main

TARGET_PATH := $(PWD)
# compile linux or LITEOS
include $(PWD)/../../$(ARM_ARCH)_$(OSTYPE).mak
