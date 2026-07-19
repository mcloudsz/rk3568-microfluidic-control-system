#pragma once

#include <linux/videodev2.h>
#include <iostream>
#include <cstring>
#include <queue>
#include <vector>
#include <thread>
#include "dmaHeap.h"
#include "rgaAcc.h"
#include "bufferPool.h"
#include "threadSafeQueue.h"
#include "gError.h"

class frameAcc{
    private:
        int _fd = -1;
        bool _isRunning = false;
        std::thread _workerThread;
        threadSafeQueue<dmaHeapBuffer*>& _inQueue;
        threadSafeQueue<dmaHeapBuffer*>& _outQueue;
        bufferPool& _inPool;
        rgaAcc _rga;
    public:
        frameAcc(int fd, int clipXPos, int clipYPos, int clipWidth, int clipHeight, 
            threadSafeQueue<dmaHeapBuffer*>& inQueue, threadSafeQueue<dmaHeapBuffer*>& outQueue, bufferPool& inPool);
        ~frameAcc();
        void frameAccLoop();
        void start();
        void stop();
};

