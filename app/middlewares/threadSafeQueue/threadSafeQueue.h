#pragma once

#include <deque>
#include <mutex>
#include <condition_variable>

template<typename T>
class threadSafeQueue {
public:
    threadSafeQueue(int capacity) : _size(0), _capacity(capacity) {}

    ~threadSafeQueue() {
        std::unique_lock<std::mutex> lock(_mtx);
        _q.clear();
    }

    void push(T item) {
        std::unique_lock<std::mutex> lock(_mtx);
        _cv.wait(lock, [this]{ return _closed || _size < _capacity; });
        if(_closed)
            return;

        _q.push_back(item);
        _size++;
        _cv.notify_one();
    }

    T pop() 
    {
        std::unique_lock<std::mutex> lock(_mtx);

        _cv.wait(lock, [this]{ return _closed || _size > 0; });

        if (_closed && _size == 0)
            return T{};

        T item = _q.front();
        _q.pop_front();
        _size--;

        _cv.notify_one();
        return item;
    }

    void empty()  // 清空队列
    {
        std::unique_lock<std::mutex> lock(_mtx);
        _q.clear();
        _size = 0;
        _cv.notify_all();
    }

    void close()
    {
        {
            std::unique_lock<std::mutex> lock(_mtx);
            _closed = true;
            _q.clear();
            _size = 0;
        }
        // 唤醒所有线程
        _cv.notify_all();
    }
    
private:
    std::deque<T> _q;
    std::condition_variable _cv;
    std::mutex _mtx;
    int  _size;
    int  _capacity;
    bool _closed = false;
};
