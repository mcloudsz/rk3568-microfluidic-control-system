#pragma once

#include <sys/ioctl.h>
#include <thread>
#include <fstream>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/eventfd.h>
#include <poll.h>
#include <linux/iio/events.h>
#include <linux/iio/types.h>
#include "gError.h"

#define PRE_SHRESHOLD (1.2f)
#define ERR_SUM_SHRESHOLD (3)

#define SENSOR_VOL_SCALE_PATH "/sys/bus/iio/devices/iio:device0/in_voltage0_scale"
#define SENSOR_EVENT_VALUE_PATH "/sys/bus/iio/devices/iio:device0/events/in_voltage0_thresh_rising_value"
#define SENSOR_EVENT_ENABLE_PATH "/sys/bus/iio/devices/iio:device0/events/in_voltage0_thresh_rising_en"

#define SENSOR_IIO_DEV_PATH "/dev/iio:device0"
#define IIO_EVENT_POLL_TIMEOUT_MS 500

class preMonitor
{
public:
    preMonitor()
    {

    }

    ~preMonitor()
    {
        stop();
    }

    void start()
    {
        if(_isRunning == false)
        {
            _isRunning = true;
            _workThread = std::thread(&preMonitor::preMonitorLoop, this);
        }
    }

    void stop()
    {
        if(_isRunning == true)
        {
            // 写 _stopEventFd
            uint64_t value = 1;
            ssize_t ret = write(_stopEventFd, &value, sizeof(value));
            if (ret < 0 && errno != EAGAIN)
                std::cout << "write stop event fd failed" << std::endl;

            _isRunning = false;
            if(_workThread.joinable())
                _workThread.join();
        }
    }
private:
    std::thread _workThread;
    bool _isRunning = false;
    int _errTimes = 0;
    int _eventFd = -1;
    int _stopEventFd = -1;

    void preMonitorLoop()
    {
        try
        {
            int devFd;
            int ret;

            // 打开 IIO 设备
            devFd = open(SENSOR_IIO_DEV_PATH, O_RDONLY);
            if(devFd < 0)
            {
                _isRunning = false;
                throw std::runtime_error("pressure monitor open failed");
            }

            // 获取 IIO 事件 fd
            ret = ioctl(devFd, IIO_GET_EVENT_FD_IOCTL, &_eventFd);
            if(ret < 0 || _eventFd < 0)
            {
                close(devFd);
                _isRunning = false;
                throw std::runtime_error("get IIO event fd failed");
            }

            // 先关闭原有 IIO Event，确保新的阈值可以重新写入硬件
            std::ofstream eventDisableFile(SENSOR_EVENT_ENABLE_PATH);
            if(!eventDisableFile.is_open())
            {
                close(_eventFd);
                close(devFd);
                _isRunning = false;
                throw std::runtime_error("open IIO event enable file failed");
            }
            eventDisableFile << 0;
            eventDisableFile.close();

            // 读取 channel0 的电压比例
            float scale;
            std::ifstream scaleFile(SENSOR_VOL_SCALE_PATH);
            if(!scaleFile.is_open())
            {
                close(_eventFd);
                close(devFd);
                _isRunning = false;
                throw std::runtime_error("open IIO scale file failed");
            }
            scaleFile >> scale;
            scaleFile.close();

            if(scale <= 0)
            {
                close(_eventFd);
                close(devFd);
                _isRunning = false;
                throw std::runtime_error("invalid IIO scale");
            }

            // 将电压阈值转换成 ADS1115 原始值
            int thresholdRaw = static_cast<int>(PRE_SHRESHOLD * 1000.0f / scale);

            // 设置 channel0 高阈值
            std::ofstream thresholdFile(SENSOR_EVENT_VALUE_PATH);
            if(!thresholdFile.is_open())
            {
                close(_eventFd);
                close(devFd);
                _isRunning = false;
                throw std::runtime_error("open IIO event value file failed");
            }
            thresholdFile << thresholdRaw;
            thresholdFile.close();

            // 使能 channel0 高阈值事件
            std::ofstream eventEnableFile(SENSOR_EVENT_ENABLE_PATH);
            if(!eventEnableFile.is_open())
            {
                close(_eventFd);
                close(devFd);
                _isRunning = false;
                throw std::runtime_error("open IIO event enable file failed");
            }
            eventEnableFile << 1;
            eventEnableFile.close();

            // 获取 _eventFd 后，原始 IIO 设备 fd 可以关闭
            close(devFd);

            // 获取退出事件的 fd
            _stopEventFd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
            if (_stopEventFd < 0) 
            {
                close(_eventFd);
                _isRunning = false;
                throw std::runtime_error("get IIO stop event fd failed");
            }

            struct pollfd pollFds[2];
            pollFds[0].fd = _eventFd;
            pollFds[0].events = POLLIN;

            pollFds[1].fd = _stopEventFd;
            pollFds[1].events = POLLIN;

            while(_isRunning)
            {
                // 阻塞在 IIO 事件上
                ret = poll(pollFds, 2, IIO_EVENT_POLL_TIMEOUT_MS);
                if(ret < 0)  // poll 失败
                {
                    close(_eventFd);
                    close(_stopEventFd);
                    throw std::runtime_error("pressure monitor loop error");
                }

                // poll 到了其他数据. 如错误
                if(pollFds[0].revents & (POLLERR | POLLHUP | POLLNVAL))
                {
                    close(_eventFd);
                    close(_stopEventFd);
                    throw std::runtime_error("pressure monitor loop error");
                }
                
                // 检查是不是触发了退出事件
                if (pollFds[1].revents & POLLIN)
                {
                    uint64_t value;
                    (void)read(_stopEventFd, &value, sizeof(value));
                    break;
                }

                // 如果走到这里, 说明 poll 成功等到了数据. 需要通过 read 读取.
                if(!(pollFds[0].revents & POLLIN))
                    continue;

                // 走到这里, 说明是 IIO event 的事件
                struct iio_event_data eventData;
                ret = read(_eventFd, &eventData, sizeof(eventData));
                if(ret < 0)
                {
                    close(_eventFd);
                    close(_stopEventFd);
                    throw std::runtime_error("pressure monitor loop error");
                }

                if(ret != sizeof(eventData))
                {
                    std::cerr << "pressure monitor: invalid IIO event size: " << std::endl;
                    continue;
                }

                // 驱动中上报的事件为: IIO_VOLTAGE/IIO_EV_TYPE_THRESH/IIO_EV_DIR_RISING
                enum iio_chan_type channelType;
                enum iio_event_type eventType;
                enum iio_event_direction direction;
                int channel;
                channelType = static_cast<enum iio_chan_type>(IIO_EVENT_CODE_EXTRACT_CHAN_TYPE(eventData.id));
                eventType = static_cast<enum iio_event_type>(IIO_EVENT_CODE_EXTRACT_TYPE(eventData.id));
                direction = static_cast<enum iio_event_direction>(IIO_EVENT_CODE_EXTRACT_DIR(eventData.id));
                channel = static_cast<int>(IIO_EVENT_CODE_EXTRACT_CHAN(eventData.id));

                // 处理电压通道为 0 的高阈值事件
                if(channelType != IIO_VOLTAGE || eventType != IIO_EV_TYPE_THRESH || direction != IIO_EV_DIR_RISING || channel != 0)
                    continue;

                std::cout << "pressure abnormal event" << std::endl;
                // 重置标志位
                resetFlag.store(true);
            }
            close(_eventFd);
            close(_stopEventFd);
        }
        catch(const std::exception& e)
        {
            fatalErrorMsg = std::string("preMonitor") + e.what();
            fatalError.store(true);
        }
    }
};
