#pragma once

#include <linux/videodev2.h>
#include <iostream>
#include <cstring>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include "dmaHeap.h"

class bufferPool{
    private:
        std::vector<dmaHeapBuffer*> _v;
        std::deque<dmaHeapBuffer*> _q;
        std::condition_variable _cv;
        std::mutex _mtx;
        int _capacity = 0;  //
    public:
        bufferPool(int capacity);
        ~bufferPool();
        void init(int width, int height, RgaSURF_FORMAT fmt);
        void push(dmaHeapBuffer* buffer);
        dmaHeapBuffer* pop();
        dmaHeapBuffer* index2pBuf(int index);
};