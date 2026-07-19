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
    try
    {
        while (_isRunning)
        {
            // 从 inQueue 中取出准备好的 v4l2 数据。如果没有，会阻塞在此处。
            dmaHeapBuffer* srcBuf;
            srcBuf = _inQueue.pop();

            // 从 inPool 中取出 opencv 处理好的 dstBuffer。如果没有，会阻塞在此处
            dmaHeapBuffer* dstBuf;
            dstBuf = _inPool.pop();

            {
                ScopeTimer t("rgaExec");
                // 走到这里，src 和 dst 缓冲区都已经准备好了。交由 rga 处理数据
                _rga.rgaExec(*srcBuf, *dstBuf);
            }
            

            // 处理完成之后。dstBuf 传给 opencv。srcBuf 传给 v4l2 中(QUE)。
            _outQueue.push(dstBuf);
            if (ioctl(_fd, VIDIOC_QBUF, srcBuf->getV4l2Ptr()) < 0) 
                throw std::runtime_error("frameAcc loop error. ioctl failed");
        }
    }
    catch(const std::exception& e)
    {
        fatalErrorMsg = std::string("frameCapture") + e.what();
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

