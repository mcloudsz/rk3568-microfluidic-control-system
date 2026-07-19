# Makefile
CROSS   := arm-linux-gnueabihf-
CC      := $(CROSS)gcc
CXX     := $(CROSS)g++

TOP_DIR     := $(shell pwd)
BUILD_DIR   := $(TOP_DIR)/build

# Windows 共享目录
WIN_DIR     ?= /mnt/d/rk3568/output

SYSROOT     ?= /opt/rk3568-sdk/sysroot
OPENCV_DIR  ?= /opt/opencv

INC     := -I$(SYSROOT)/usr/include \
           -I$(SYSROOT)/usr/include/libdrm \
           -I$(SYSROOT)/usr/include/rga \
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

CFLAGS      := --sysroot=$(SYSROOT) $(INC) -Wall -O0 -g
CXXFLAGS    := $(CFLAGS)

LIBDIR  := -L$(SYSROOT)/usr/lib
LDFLAGS := --sysroot=$(SYSROOT) $(LIBDIR) -ldrm -lrga
LDFLAGS += -L$(OPENCV_DIR)/lib -lopencv_core -lopencv_imgproc

SRCS_CPP = $(shell find app drivers middlewares vision -name "*.cpp")
SRCS_C   = $(shell find app drivers middlewares vision -name "*.c")

OBJS  = $(addprefix $(BUILD_DIR)/, $(SRCS_CPP:.cpp=.o))
OBJS += $(addprefix $(BUILD_DIR)/, $(SRCS_C:.c=.o))

TARGET = $(BUILD_DIR)/program3

all: $(BUILD_DIR) $(TARGET)

# 链接
$(TARGET): $(OBJS)
	@echo "Linking: $@"
	@$(CXX) $(OBJS) -o $@ $(LDFLAGS)
	@echo "Build Complete: $@"
	@mkdir -p $(WIN_DIR)
	@cp $@ $(WIN_DIR)/
	@echo "已成功拷贝到: $(WIN_DIR)/program3"

# 编译 .cpp 文件
$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo "Compiling C++: $<"
		$(CXX) $(CXXFLAGS) -c $< -o $@

# 编译 .c 文件
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "Compiling C: $<"
	@$(CC) $(CFLAGS) -c $< -o $@

# 创建 build 目录
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)
	@echo "Cleaned build directory."

.PHONY: all clean