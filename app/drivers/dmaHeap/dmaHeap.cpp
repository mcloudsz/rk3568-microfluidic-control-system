#include "dmaHeap.h"

int dmaHeap::dmaBufferAlloc(int size)
{
    if(size == 0)
    {
        std::cout << "size can not be 0" << std::endl;
        return -1;
    }

    int fd = open(DMA_HEAP_PATH, O_RDWR | O_CLOEXEC);
    if (fd < 0) 
    {
        std::cout << "dma buffer alloc failed. path is:" << DMA_HEAP_PATH << std::endl;
        return -1;
    }

    struct dma_heap_allocation_data data = {
        .len = size,
        .fd_flags = O_RDWR | O_CLOEXEC,
    };

    if (ioctl(fd, DMA_HEAP_IOCTL_ALLOC, &data) < 0) 
    {
        std::cout << "dma buffer ioctl failed. path is:" << DMA_HEAP_PATH << std::endl;
        close(fd);
        return -1;
    }

    close(fd);
    return data.fd; 
}

void* dmaHeap::dmaBufferMmap(int fd, int size)
{
    if(fd < 0 || size == 0)
    {
        std::cout << "invaild param" << std::endl;
        return nullptr; 
    }

    void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if(ptr == MAP_FAILED)
    {
        std::cout << "dma buffer mmap failed" << std::endl;                
        return nullptr;
    }
    return ptr;
}


dmaHeap::dmaHeap(int size)
{
    int fd = dmaBufferAlloc(size);
    if(fd < 0)
        throw std::runtime_error("dmaHeap creation failed at allocation");

    _fd = fd;
    _size = size;

    void* ptr = dmaBufferMmap(fd, size);
    if(ptr == nullptr)
    {
        close(_fd);
        throw std::runtime_error("dmaBuffer mmap failed");
    }
    _ptr = ptr;
}

// 析构函数
dmaHeap::~dmaHeap()
{
    if (_ptr != MAP_FAILED) 
        munmap(_ptr, _size);

    if (_fd >= 0) 
        close(_fd);
}

int dmaHeap::dmaHeapGetFd() const
{
    return _fd;
}

void* dmaHeap::dmaHeapGetPtr() const
{
    return _ptr;
}


int dmaHeapBuffer::calcSize(RgaSURF_FORMAT fmt, int srcWidth, int srcHeight)
{
    if(fmt == RK_FORMAT_YVYU_422 || fmt == RK_FORMAT_YUYV_422)
        return srcWidth * srcHeight * 2;
    else if(fmt == RK_FORMAT_YCbCr_420_SP)
        return srcWidth * srcHeight * 3 / 2;
    else if(fmt == RK_FORMAT_RGB_888)
        return srcWidth * srcHeight * 3;
    else 
        return 0;
}

void dmaHeapBuffer::v4l2BufferSet()
{
    _v4l2Buffer.index = _index;
    _v4l2Buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    _v4l2Buffer.memory = V4L2_MEMORY_DMABUF;
    _v4l2Buffer.m.fd = _dmaHeapInfo.dmaHeapGetFd();
    _v4l2Buffer.length = calcSize(_fmt, _srcWidth, _srcHeight);
}


dmaHeapBuffer::dmaHeapBuffer(RgaSURF_FORMAT fmt, int srcWidth, int srcHeight, int index)
    :_dmaHeapInfo(calcSize(fmt, srcWidth, srcHeight)),
        _srcWidth(srcWidth),
        _srcHeight(srcHeight),
        _fmt(fmt),
        _index(index)
{
    // 走到这里，_dmaHeapInfo 一定是被构造了的
    // 将 buffer 导入到 rga 中
    rga_buffer_handle_t rgaHandle = importbuffer_fd(
        _dmaHeapInfo.dmaHeapGetFd(),
        srcWidth, 
        srcHeight, 
        fmt
    );
    if (rgaHandle == 0) 
        throw std::runtime_error("dma heap buffer import rga fd failed");
    
    _rgaHandle = rgaHandle;
    memset(&_v4l2Buffer, 0, sizeof(_v4l2Buffer));
    v4l2BufferSet();
}

dmaHeapBuffer::~dmaHeapBuffer()
{
    releasebuffer_handle(_rgaHandle);
}

rga_buffer_handle_t dmaHeapBuffer::getBufferHandle() const
{
    return _rgaHandle;
}

int dmaHeapBuffer::getWidth() const 
{
    return _srcWidth;
}

int dmaHeapBuffer::getHeight() const
{
    return _srcHeight;
}

RgaSURF_FORMAT dmaHeapBuffer::getFmt() const
{
    return _fmt;
}

void* dmaHeapBuffer::getPtr() const
{
    return _dmaHeapInfo.dmaHeapGetPtr();
}

struct v4l2_buffer* dmaHeapBuffer::getV4l2Ptr()
{
    return &_v4l2Buffer;
}
