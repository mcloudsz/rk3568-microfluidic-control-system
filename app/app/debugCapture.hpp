#pragma once

#include <atomic>
#include <thread>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <cstring>
#include <cerrno>
#include <stdint.h>

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <linux/videodev2.h>

#include "gError.h"

#define DEBUG_VIDEO_PATH      "/dev/video1"
#define DEBUG_SAVE_DIR        "/mnt/sdcard/debug_img"
#define DEBUG_SD_MOUNT_POINT  "/mnt/sdcard"

// 保存图片的宽高参数
#define DEBUG_WIDTH 320
#define DEBUG_HEIGHT 240
#define DEBUG_BUFFER_NUM 4
#define DEBUG_SAVE_INTERVAL 5
#define DEBUG_MAX_SAVE_COUNT 200

class debugCapture {
private:
    typedef struct debugMmapBuffer {
        void* ptr;
        size_t len;
    }debugMmapBuffer_t;

private:
    int _fd = -1;
    int _width = DEBUG_WIDTH;
    int _height = DEBUG_HEIGHT;
    int _stride = DEBUG_WIDTH;

    int _frameCount = 0;
    int _saveCount = 0;

    bool _prepared = false;
    bool _streamOn = false;

    std::atomic<bool> _isRunning{false};
    std::thread _workerThread;
    std::vector<debugMmapBuffer_t> _buffers;

private:
    // 检查 SD 卡是否已经挂载. 防止 SD 卡未挂载时, 文件被写入 eMMC/rootfs.
    bool isSdMounted()
    {
        std::ifstream f("/proc/mounts");
        std::string dev, mountPoint, fs, opt;
        int dump, pass;

        while (f >> dev >> mountPoint >> fs >> opt >> dump >> pass) {
            if (mountPoint == DEBUG_SD_MOUNT_POINT)
                return true;
        }

        return false;
    }

    // 创建保存目录. 
    void makeSaveDir()
    {
        if (!isSdMounted())
            throw std::runtime_error("SD card is not mounted at /mnt/sdcard");

        if (mkdir(DEBUG_SAVE_DIR, 0755) < 0 && errno != EEXIST)
            throw std::runtime_error("mkdir debug save dir failed");
    }

    // 配置 selfpath 输出格式. 
    void setFormat()
    {
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));

        // 调试流使用 NV12 320x240.
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.fmt.pix_mp.width = DEBUG_WIDTH;  // 320
        fmt.fmt.pix_mp.height = DEBUG_HEIGHT;  // 240
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.fmt.pix_mp.num_planes = 1;

        if (ioctl(_fd, VIDIOC_S_FMT, &fmt) < 0)
            throw std::runtime_error("debug VIDIOC_S_FMT failed");

        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

        if (ioctl(_fd, VIDIOC_G_FMT, &fmt) < 0)
            throw std::runtime_error("debug VIDIOC_G_FMT failed");

        if (fmt.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_NV12)
            throw std::runtime_error("debug stream is not NV12");

        _width = fmt.fmt.pix_mp.width;
        _height = fmt.fmt.pix_mp.height;
        _stride = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;

        if (_stride <= 0)
            _stride = _width;

        std::cout << "[debugCapture] fmt " << _width << "x" << _height << " stride=" << _stride << std::endl;
    }

    // 申请 V4L2 MMAP buffer. 
    void requestMmapBuffers()
    {
        // 调试流使用 mmap, 不走主采集链路
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));

        req.count = DEBUG_BUFFER_NUM;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        req.memory = V4L2_MEMORY_MMAP;

        // req
        if (ioctl(_fd, VIDIOC_REQBUFS, &req) < 0)
            throw std::runtime_error("debug VIDIOC_REQBUFS failed");

        _buffers.resize(req.count);

        // mmap vb2 申请的 buffer
        for (unsigned int i = 0; i < req.count; ++i) 
        {
            struct v4l2_buffer buf;
            struct v4l2_plane planes[VIDEO_MAX_PLANES];

            memset(&buf, 0, sizeof(buf));
            memset(planes, 0, sizeof(planes));

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            buf.length = 1;
            buf.m.planes = planes;

            if (ioctl(_fd, VIDIOC_QUERYBUF, &buf) < 0)
                throw std::runtime_error("debug VIDIOC_QUERYBUF failed");

            _buffers[i].len = planes[0].length;
            _buffers[i].ptr = mmap(nullptr, planes[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, planes[0].m.mem_offset);

            if (_buffers[i].ptr == MAP_FAILED)
                throw std::runtime_error("debug mmap failed");
        }
    }

    // 将所有 MMAP buffer 入队.
    void queueAllBuffers()
    {
        for (size_t i = 0; i < _buffers.size(); ++i) 
        {
            struct v4l2_buffer buf;
            struct v4l2_plane planes[VIDEO_MAX_PLANES];

            memset(&buf, 0, sizeof(buf));
            memset(planes, 0, sizeof(planes));

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            buf.length = 1;
            buf.m.planes = planes;

            if (ioctl(_fd, VIDIOC_QBUF, &buf) < 0)
                throw std::runtime_error("debug VIDIOC_QBUF failed");
        }
    }

    // 保存 NV12 的 Y 平面为 PGM 灰度图.
    void savePgm(const uint8_t* data, int index)
    {
        std::ostringstream path;
        path << DEBUG_SAVE_DIR << "/debug_" << std::setw(6) << std::setfill('0') << index << "_y.pgm";

        std::ofstream ofs(path.str(), std::ios::binary);
        if (!ofs.is_open())
            throw std::runtime_error("open debug pgm failed");

        ofs << "P5\n" << _width << " " << _height << "\n255\n";

        for (int y = 0; y < _height; ++y)
            ofs.write(reinterpret_cast<const char*>(data + y * _stride), _width);

        ofs.close();

        std::cout << "[debugCapture] save " << path.str() << std::endl;
    }

    // 调试线程主循环. 每 5 帧保存一次 selfpath 的 Y 平面.
    void debugCaptureLoop()
    {
        try {
            while (_isRunning.load()) {
                struct pollfd pfd;
                memset(&pfd, 0, sizeof(pfd));

                pfd.fd = _fd;
                pfd.events = POLLIN;

                int ret = poll(&pfd, 1, 500);
                if (ret < 0) {
                    if (errno == EINTR)
                        continue;
                    throw std::runtime_error("debug poll failed");
                }

                if (ret == 0)
                    continue;

                struct v4l2_buffer buf;
                struct v4l2_plane planes[VIDEO_MAX_PLANES];

                memset(&buf, 0, sizeof(buf));
                memset(planes, 0, sizeof(planes));

                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.length = 1;
                buf.m.planes = planes;

                if (ioctl(_fd, VIDIOC_DQBUF, &buf) < 0) {
                    if (!_isRunning.load() || errno == EAGAIN)
                        continue;
                    throw std::runtime_error("debug VIDIOC_DQBUF failed");
                }

                _frameCount++;

                if ((_frameCount % DEBUG_SAVE_INTERVAL) == 0 && _saveCount < DEBUG_MAX_SAVE_COUNT) {
                    _saveCount++;
                    savePgm(static_cast<uint8_t*>(_buffers[buf.index].ptr), _saveCount);
                }

                if (ioctl(_fd, VIDIOC_QBUF, &buf) < 0)
                    throw std::runtime_error("debug VIDIOC_QBUF failed");
            }
        }
        catch (const std::exception& e) {
            fatalErrorMsg = std::string("debugCapture: ") + e.what();
            fatalError.store(true);
        }
    }

public:
    debugCapture()
    {

    }

    ~debugCapture()
    {
        stop();
    }

    debugCapture(const debugCapture&) = delete;
    debugCapture& operator=(const debugCapture&) = delete;

    // 初始化调试流. 必须在主链路 STREAMON 之前完成 prepare.
    void prepare()
    {
        if (_prepared)
            return;

        makeSaveDir();

        _fd = open(DEBUG_VIDEO_PATH, O_RDWR | O_NONBLOCK);
        if (_fd < 0)
            throw std::runtime_error("open debug video failed");

        setFormat();
        requestMmapBuffers();
        queueAllBuffers();

        _prepared = true;
    }

    // 启动调试流. 建议在主链路 STREAMON 成功之后调用.
    void start()
    {
        if (_isRunning.load())
            return;

        if (!_prepared)
            prepare();

        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        if (ioctl(_fd, VIDIOC_STREAMON, &type) < 0)
            throw std::runtime_error("debug VIDIOC_STREAMON failed");

        _streamOn = true;
        _isRunning.store(true);
        _workerThread = std::thread(&debugCapture::debugCaptureLoop, this);
    }

    // 停止调试流并释放 MMAP buffer.
    void stop()
    {
        _isRunning.store(false);

        if (_streamOn && _fd >= 0) {
            enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            ioctl(_fd, VIDIOC_STREAMOFF, &type);
            _streamOn = false;
        }

        if (_workerThread.joinable())
            _workerThread.join();

        for (auto& b : _buffers) {
            if (b.ptr != MAP_FAILED) {
                munmap(b.ptr, b.len);
                b.ptr = MAP_FAILED;
                b.len = 0;
            }
        }

        _buffers.clear();

        if (_fd >= 0) {
            close(_fd);
            _fd = -1;
        }

        _prepared = false;
    }
};
