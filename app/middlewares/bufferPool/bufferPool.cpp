#include "bufferPool.h"


bufferPool::bufferPool(int capacity)
{
    _capacity = capacity;
}
bufferPool::~bufferPool()
{
    std::unique_lock<std::mutex> lock(_mtx);
    int cap = _v.size();
    for(int i = 0; i < cap; i++)
        delete _v[i];
}
void bufferPool::init(int width, int height, RgaSURF_FORMAT fmt)
{
    if(_capacity == 0)
        throw std::runtime_error("capacity can not be 0");

    dmaHeapBuffer* pBuf;
    for(int i = 0; i < _capacity; i++)
    {
        // 创建 src buffer。new 失败的场合直接穿透到上层。
        pBuf = new dmaHeapBuffer(
            fmt,                  // fmt
            width,                // width
            height,               // height
            i                     // index
        );
        _v.push_back(pBuf);  // 推到链表中
        _q.push_back(pBuf);  // 推到队列中
    }
}

void bufferPool::push(dmaHeapBuffer* buffer)
{
    if(buffer == nullptr)
        return;

    std::unique_lock<std::mutex> lock(_mtx);
    if (_q.size() == _capacity) 
        throw std::runtime_error("duplicate release exception");

    _q.push_back(buffer);
    _cv.notify_one();
}

dmaHeapBuffer* bufferPool::pop()
{
    dmaHeapBuffer* outBuffer;
    // 进入临界区
    std::unique_lock<std::mutex> lock(_mtx);
    while(_q.size() == 0)
        _cv.wait(lock, [this]{return _q.size() > 0;});

    outBuffer = _q.front();
    _q.pop_front();
    return outBuffer;
}

dmaHeapBuffer* bufferPool::index2pBuf(int index)
{
    return _v[index];
}


