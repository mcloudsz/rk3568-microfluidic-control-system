#include "frameAcc.h"

frameAcc::frameAcc(int fd, int clipXPos, int clipYPos, int clipWidth, int clipHeight, 
    threadSafeQueue<dmaHeapBuffer*>& inQueue, threadSafeQueue<dmaHeapBuffer*>& outQueue, bufferPool& inPool)
    :_fd(fd),
        _rga(clipXPos, clipYPos, clipWidth, clipHeight),
        _inQueue(inQueue),
        _outQueue(outQueue),
        _inPool(inPool)
{

}

frameAcc::~frameAcc()
{
    stop();
}

#include "scopeTimer.hpp"
void frameAcc::frameAccLoop()
{
    std::cout << "frame Acc thread is running" << std::endl;

    try {
        while (_isRunning) {
            dmaHeapBuffer* srcBuf = _inQueue.pop();
            if (srcBuf == nullptr)
                break;

            dmaHeapBuffer* dstBuf = _inPool.pop();
            if (dstBuf == nullptr) 
            {
                ioctl(_fd, VIDIOC_QBUF, srcBuf->getV4l2Ptr());
                break;
            }

            {
                ScopeTimer t("rgaExec");
                _rga.rgaExec(*srcBuf, *dstBuf);
            }

            _outQueue.push(dstBuf);

            if (ioctl(_fd, VIDIOC_QBUF, srcBuf->getV4l2Ptr()) < 0)
                throw std::runtime_error("VIDIOC_QBUF failed");
        }
    }
    catch (const std::exception& e) {
        fatalErrorMsg = std::string("frameAcc: ") + e.what();
        fatalError.store(true);
    }
}

void frameAcc::start()
{
    if(_isRunning == false)
    {
        _isRunning = true;
        _workerThread = std::thread(&frameAcc::frameAccLoop, this);
    }
}

void frameAcc::stop()
{
    if(_isRunning)
    {
        _isRunning = false;
        _inQueue.close();  // 退出可能阻塞的队列
        _outQueue.close();
        if(_workerThread.joinable())
            _workerThread.join();
    }
}

