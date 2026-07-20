# Makefile for RK3568 aarch64

# =========================
# RK3568 SDK toolchain
# =========================
TOOLCHAIN := /home/mcloud/rk3568/prebuilts/gcc/linux-x86/aarch64/gcc-buildroot-9.3.0-2020.03-x86_64_aarch64-rockchip-linux-gnu


export LD_LIBRARY_PATH := $(TOOLCHAIN)/lib:$(LD_LIBRARY_PATH)

CROSS   := $(TOOLCHAIN)/bin/aarch64-rockchip-linux-gnu-
CC      := $(CROSS)gcc
CXX     := $(CROSS)g++

# =========================
# Project paths
# =========================
TOP_DIR     := $(shell pwd)
BUILD_DIR   := $(TOP_DIR)/build

# Windows 共享目录
WIN_DIR     := /mnt/d/rk3568/ubuntu2win

# =========================
# RK3568 sysroot / Ubuntu rootfs
# =========================
SYSROOT := $(TOOLCHAIN)/aarch64-rockchip-linux-gnu/sysroot

RK3568_UBUNTU_ROOT    := /home/mcloud/rk3568/ubuntu/binary
RK3568_UBUNTU_LIB_DIR := $(RK3568_UBUNTU_ROOT)/usr/lib/aarch64-linux-gnu

# RGA headers
RGA_INC_DIR := $(RK3568_UBUNTU_ROOT)/usr/include/rga

# RK3568 SDK-toolchain OpenCV
OPENCV_DIR := /home/mcloud/opencv-aarch64-rk/install

# =========================
# Include paths
# =========================
INC := -I$(SYSROOT)/usr/include \
       -I$(SYSROOT)/usr/include/libdrm \
       -I$(RK3568_UBUNTU_ROOT)/usr/include/libdrm \
       -I$(RGA_INC_DIR) \
       -I$(TOP_DIR)/app \
       -I$(TOP_DIR)/drivers/camera \
       -I$(TOP_DIR)/drivers/rga \
       -I$(TOP_DIR)/drivers/dmaHeap \
       -I$(TOP_DIR)/drivers/propValve \
       -I$(TOP_DIR)/middlewares/bufferPool \
       -I$(TOP_DIR)/middlewares/threadSafeQueue \
       -I$(TOP_DIR)/vision/frameCapture \
       -I$(TOP_DIR)/vision/frameAcc \
       -I$(TOP_DIR)/vision/frameProc \
       -I$(TOP_DIR)/control \
       -I$(OPENCV_DIR)/include/opencv4

# =========================
# Compile flags
# =========================
CFLAGS   := --sysroot=$(SYSROOT) $(INC) -Wall -O0 -g
CXXFLAGS := $(CFLAGS) -std=c++11 -DNO_OPENCV_TEST

# =========================
# Library paths
# =========================
LIBDIR := -L$(SYSROOT)/usr/lib \
          -L$(SYSROOT)/usr/lib/aarch64-linux-gnu \
          -L$(RK3568_UBUNTU_LIB_DIR) \
          -L$(OPENCV_DIR)/lib

# =========================
# Link flags
# =========================
LDFLAGS := --sysroot=$(SYSROOT) $(LIBDIR) \
           -Wl,-rpath-link,$(SYSROOT)/usr/lib \
           -Wl,-rpath-link,$(SYSROOT)/usr/lib/aarch64-linux-gnu \
           -Wl,-rpath-link,$(RK3568_UBUNTU_LIB_DIR) \
           -Wl,-rpath-link,$(OPENCV_DIR)/lib \
           $(RK3568_UBUNTU_LIB_DIR)/libdrm.so.2 \
           -lrga \
		   -lpthread \
           -ldl \
           -lm \
           -lopencv_core \
           -lopencv_imgproc \
           -lopencv_imgcodecs \
           

# =========================
# Source files
# =========================
SRCS_CPP = $(shell find app drivers middlewares vision -name "*.cpp")
SRCS_C   = $(shell find app drivers middlewares vision -name "*.c")

OBJS  = $(addprefix $(BUILD_DIR)/, $(SRCS_CPP:.cpp=.o))
OBJS += $(addprefix $(BUILD_DIR)/, $(SRCS_C:.c=.o))

TARGET = $(BUILD_DIR)/program3

# =========================
# Rules
# =========================
all: $(BUILD_DIR) $(TARGET)

$(TARGET): $(OBJS)
	@echo "Linking: $@"
	@$(CXX) $(OBJS) -o $@ $(LDFLAGS)
	@echo "Build Complete: $@"
	@mkdir -p $(WIN_DIR)
	@cp $@ $(WIN_DIR)/
	@echo "已成功拷贝到: $(WIN_DIR)/program3"

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo "Compiling C++: $<"
	@$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "Compiling C: $<"
	@$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)
	@echo "Cleaned build directory."

.PHONY: all clean