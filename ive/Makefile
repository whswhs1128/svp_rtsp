
include ../Makefile.param

CFLAGS += -I$(PWD)/sample
CFLAGS += -I/home/ginger/SDL2-2.26.1/install/include/SDL2 -I/home/ginger/SDL2_ttf-2.0.18/install/include/SDL2/ -L/home/ginger/SDL2-2.26.1/install/lib -L/home/ginger/SDL2_ttf-2.0.18/install/lib -lSDL2 -lSDL2_ttf -lpthread -Wl,--rpath-link=/home/ginger/freetype-2.9/install/lib
CFLAGS += -I/home/ginger/sdk/SS928V100R001C02SPC021/01.software/board/SS928V100_SDK_V2.0.2.1/package/smp/a55_linux/mpp/out/include/rtsp_lib

SMP_SRCS := $(wildcard *.c)
SMP_SRCS += $(wildcard ./sample/*.c)
SMP_SRCS += $(wildcard $(PWD)/../common/*.c)

TARGET := sample_ive_main

TARGET_PATH := $(PWD)
# compile linux or LITEOS
include $(PWD)/../../$(ARM_ARCH)_$(OSTYPE).mak
