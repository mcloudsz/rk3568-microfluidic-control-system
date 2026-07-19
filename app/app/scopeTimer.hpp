#pragma once
#include <iostream>

class ScopeTimer {
public:
    ScopeTimer(const std::string& name) : _name(name)
    {
        clock_gettime(CLOCK_MONOTONIC, &_start);
    }
    ~ScopeTimer()
    {
        struct timespec end;
        clock_gettime(CLOCK_MONOTONIC, &end);
        long ms = (end.tv_sec  - _start.tv_sec)  * 1000
                + (end.tv_nsec - _start.tv_nsec) / 1000000;
        std::cout << "[TIMER] " << _name << ": " << ms << " ms" << std::endl;
    }
private:
    std::string     _name;
    struct timespec _start;
};
