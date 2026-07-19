#pragma once

#include <linux/videodev2.h>
#include <iostream>
#include <cstring>
#include <queue>
#include <vector>
#include <thread>
#include "dmaHeap.h"
#include "bufferPool.h"
#include "threadSafeQueue.h"
#include "gError.h"

class frameCapture{
    private:
        const std::vector<dmaHeapBuffer*>& _v;
        threadSafeQueue<dmaHeapBuffer*>& _outQueue;
        int _fd = -1;
        bool _isRunning = false;
        std::thread _workerThread;

        void frameCaptureLoop();
    public:
        frameCapture(const std::vector<dmaHeapBuffer*>& v, threadSafeQueue<dmaHeapBuffer*>& outQueue, int fd);
        ~frameCapture();
        void start();
        void stop();
};