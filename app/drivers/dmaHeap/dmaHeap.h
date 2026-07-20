#pragma once

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <rga.h>
#include <cstddef>
#include <im2d.h> 
#include "dma_heap_compat.h"
#define DMA_HEAP_PATH "/dev/dma_heap/linux,cma"

class dmaHeap{
    private:
        int _fd = -1; 
        void* _ptr = MAP_FAILED; 
        int _size = 0; 

        int dmaBufferAlloc(int size);
        void* dmaBufferMmap(int fd, int size);
    public:
        dmaHeap(int size);
        ~dmaHeap();
        int dmaHeapGetFd() const;
        void* dmaHeapGetPtr() const;
};

class dmaHeapBuffer{
private:
    dmaHeap _dmaHeapInfo;
    int _srcWidth;
    int _srcHeight;
    RgaSURF_FORMAT _fmt;
    int _index;
    rga_buffer_handle_t _rgaHandle;

    struct v4l2_buffer _v4l2Buffer;
    struct v4l2_plane _v4l2Planes[VIDEO_MAX_PLANES];

    static int calcSize(RgaSURF_FORMAT fmt, int srcWidth, int srcHeight);
    void v4l2BufferSet();

public:
    dmaHeapBuffer(RgaSURF_FORMAT fmt, int srcWidth, int srcHeight, int index);
    ~dmaHeapBuffer();

    rga_buffer_handle_t getBufferHandle() const;
    int getWidth() const;
    int getHeight() const;
    RgaSURF_FORMAT getFmt() const;
    void* getPtr() const;

    int getFd() const;
    int getSize() const;

    struct v4l2_buffer* getV4l2Ptr();
};
