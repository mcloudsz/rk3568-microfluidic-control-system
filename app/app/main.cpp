#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <im2d.h>
#include <rga.h>
#include <linux/dma-heap.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "frameCapture.h"
#include "frameAcc.h"
#include "frameProc.h"
#include "threadSafeQueue.h"
#include "bufferPool.h"
#include "dropletCtrl.hpp"
#include "gError.h"
#include "preMonitor.hpp"

#define VIDEO_PATH "/dev/video0"
#define PUMP_DEV_PATH "/dev/rk_pump"

// v4l2 使用的 buffer 缓冲区个数
#define V4L2_BUFFER_NUM 4
// 默认的摄像头输出大小(宽)
#define DEFAULT_WIDTH 640
// 默认的摄像头输出大小(高)
#define DEFAULT_HEIGHT 480

// 液滴消息队列容量
#define INFO_BUFFER_NUM 5

int testFd;

int main()
{
    // 1. 打开视频设备
    int fd = open(VIDEO_PATH, O_RDWR);
    if (fd < 0) 
    { 
        perror("open video"); 
        return -1; 
    }

    // 2. 查询能力
    struct v4l2_capability cap;
    memset((void*)&cap, 0, sizeof(struct v4l2_capability));
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) 
    { 
        perror("QUERYCAP failed"); 
        return -1; 
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) 
    {
        printf("not a capture device\n"); 
        return -1;
    }

    // 3. 设置格式
    struct v4l2_format fmt;
    memset((void*)&fmt, 0, sizeof(struct v4l2_format));
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
    fmt.fmt.pix.width       = DEFAULT_WIDTH;
    fmt.fmt.pix.height      = DEFAULT_HEIGHT;

    // 申请格式
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) 
    { 
        perror("S_FMT failed"); 
        return -1; 
    }

    // 返回的实际格式
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) < 0) 
    { 
        perror("G_FMT failed"); 
        return -1; 
    }

    if(fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_NV12)
    {
        std::cerr << "video node does not return NV12 format" << std::endl;
        return -1;
    }

    int width  = (int)fmt.fmt.pix.width;
    int height = (int)fmt.fmt.pix.height;
    std::cout << "fmt size:" << std::to_string(width) << "x" << std::to_string(height) << std::endl;

    // 创建一个目标 buffer 内存池。
    // 构造函数必定成功
    bufferPool detBufferPool(V4L2_BUFFER_NUM);
    detBufferPool.init(width, height, RK_FORMAT_RGB_888);

    // 创建两个线程安全队列, 用于存放 dmaHeapBuffer* 缓冲区指针
    threadSafeQueue<dmaHeapBuffer*> rawFrameQueue(V4L2_BUFFER_NUM);
    threadSafeQueue<dmaHeapBuffer*> processedFrameQueue(V4L2_BUFFER_NUM);

    // 创建一个线程安全队列, 用于存放 dropletInfo
    threadSafeQueue<dropletInfo> dropletInfoQueue(INFO_BUFFER_NUM);

    // 创建 V4L2_BUFFER_NUM 个供 v4l2 管理的 buffer
    std::vector<dmaHeapBuffer*> v;
    dmaHeapBuffer* pBuf;
    for(int i = 0; i < V4L2_BUFFER_NUM; i++)
    {
        pBuf = new dmaHeapBuffer(
            RK_FORMAT_YCbCr_420_SP,   // fmt
            width, // width
            height, // height
            i // index
        );
        v.push_back(pBuf);  // 推到链表中
    }

    // 启动 v4l2 stream。
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count  = V4L2_BUFFER_NUM;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_DMABUF;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) 
    {
        perror("REQBUFS failed");
        return -1;
    }

    for(int i = 0; i < V4L2_BUFFER_NUM ;i++)
    {
        if (ioctl(fd, VIDIOC_QBUF, v[i]->getV4l2Ptr()) < 0) 
        { 
            perror("QBUF init failed"); 
            return -1; 
        }
    }

    // ==================================
    // 测试逻辑
#define TEST_PATH "/oem/frame.rgb"
    testFd = open(TEST_PATH, O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (testFd < 0) 
    { 
        perror("open test file failed"); 
        return -1; 
    }
    // ==================================

    // 打开 pump 对应的 ioctl 文件
    int pumpFd = open(PUMP_DEV_PATH, O_RDWR);
    if (pumpFd < 0) 
    { 
        perror("open pump device"); 
        return -1; 
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) 
    { 
        perror("STREAMON"); 
        return -1; 
    }

    // 初始化线程
    frameCapture threadCapture(v, rawFrameQueue, fd);
    frameAcc threadAcc(fd, 0, 0, width, height, rawFrameQueue, processedFrameQueue, detBufferPool);
    frameProc threadProc(processedFrameQueue, detBufferPool, dropletInfoQueue);
    dropletCtrl threadCtrl(dropletInfoQueue, pumpFd);
    preMonitor threadPre;

    // 启动流
    threadCapture.start();
    threadAcc.start();
    threadProc.start();
    threadCtrl.start();
    threadPre.start();

    std::cout << "System is running ..." << std::endl;
    while (!fatalError.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (fatalError.load())
        std::cerr << "[FATAL] " << fatalErrorMsg << std::endl;

    // 退出逻辑
    threadCapture.stop();
    threadAcc.stop();
    threadProc.stop();
    threadCtrl.stop();
    threadPre.stop();
}
