#include "frameCapture.h"

void frameCapture::frameCaptureLoop()
{
    std::cout << "frame capture thread is running" << std::endl;
    
    // ============================
    // timer
    struct timespec lastTime;
    clock_gettime(CLOCK_MONOTONIC, &lastTime);
    // ===========================
    try
    {
        while(_isRunning)
        {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_DMABUF;
            
            // 线程会在这个 ioctl 上阻塞。v4l2 处理完数据之后, 会往下执行并且拿到数据包下标 
            if (ioctl(_fd, VIDIOC_DQBUF, &buf) < 0) 
            {
                if (!_isRunning) 
                    break; 

                throw std::runtime_error("frameCapture start error. ioctl failed");
            }

            // ==================================
            // timer
            // 计算距上一帧的间隔
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            long ms = (now.tv_sec  - lastTime.tv_sec)  * 1000
                    + (now.tv_nsec - lastTime.tv_nsec) / 1000000;
            std::cout << "[TIMER] capture interval: " << ms << " ms" << std::endl;
            lastTime = now;
            // =================================

            // 获取处理完成的那一帧下标
            dmaHeapBuffer* pBuf = _v[buf.index];
            _outQueue.push(pBuf);
        }
    }
    catch(const std::exception& e)
    {
        fatalErrorMsg = std::string("frameCapture") + e.what();
        fatalError.store(true);
    }
}


frameCapture::frameCapture(const std::vector<dmaHeapBuffer*>& v, threadSafeQueue<dmaHeapBuffer*>& outQueue, int fd)
    :_v(v),
     _outQueue(outQueue),
    _fd(fd)
{
}

frameCapture::~frameCapture()
{
    stop();
}

        
void frameCapture::start()
{
    if(!_isRunning)
    {
        _isRunning = true;
        _workerThread = std::thread(&frameCapture::frameCaptureLoop, this);
    }
}


void frameCapture::stop()
{
    if (_isRunning) 
    {
        _isRunning = false; 
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(_fd, VIDIOC_STREAMOFF, &type);
        _outQueue.close();
        if (_workerThread.joinable()) 
            _workerThread.join();
    }
}
