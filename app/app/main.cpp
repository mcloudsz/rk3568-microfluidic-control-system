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
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "frameCapture.h"
#include "frameAcc.h"
#include "frameProc.h"
#include "threadSafeQueue.h"
#include "debugCapture.hpp"
#include "bufferPool.h"
#include "dropletCtrl.hpp"
#include "gError.h"
#include "preMonitor.hpp"
#include "dma_heap_compat.h"

#define VIDEO_PATH "/dev/video0"
#define PUMP_DEV_PATH "/dev/rk_pump"

// v4l2 使用的 buffer 缓冲区个数
#define V4L2_BUFFER_NUM 4

// 视窗位置参数
#define ROI_X 440
#define ROI_Y 350
#define ROI_WIDTH 100
#define ROI_HEIGHT 64

// 液滴消息队列容量
#define INFO_BUFFER_NUM 5

#define V4L2_CAPTURE_TYPE V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
#define V4L2_CAPTURE_FMT  V4L2_PIX_FMT_NV12

static int set_isp_crop_roi(int fd, int x, int y, int w, int h)
{
    struct v4l2_selection sel;
    memset(&sel, 0, sizeof(sel));

    sel.type = V4L2_CAPTURE_TYPE;
    sel.target = V4L2_SEL_TGT_CROP;
    sel.r.left = x;
    sel.r.top = y;
    sel.r.width = w;
    sel.r.height = h;

    if (ioctl(fd, VIDIOC_S_SELECTION, &sel) < 0) {
        perror("VIDIOC_S_SELECTION crop failed");
        return -1;
    }

    memset(&sel, 0, sizeof(sel));
    sel.type = V4L2_CAPTURE_TYPE;
    sel.target = V4L2_SEL_TGT_CROP;

    if (ioctl(fd, VIDIOC_G_SELECTION, &sel) < 0) {
        perror("VIDIOC_G_SELECTION crop failed");
        return -1;
    }

    printf("actual crop ROI: left=%d top=%d width=%d height=%d\n", sel.r.left, sel.r.top, sel.r.width, sel.r.height);

    if (sel.r.left != x || sel.r.top != y || sel.r.width != w || sel.r.height != h) 
    {
        printf("[WARN] driver adjusted ROI, requested %d,%d %dx%d, actual %d,%d %dx%d\n",
               x, y, w, h, sel.r.left, sel.r.top, sel.r.width, sel.r.height);
    }

    return 0;
}

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
    uint32_t caps = cap.capabilities;

    if (cap.capabilities & V4L2_CAP_DEVICE_CAPS)
        caps = cap.device_caps;
    if (!(caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE)) {
        printf("not a multiplanar capture device\n");
        return -1;
    }
    if (!(caps & V4L2_CAP_STREAMING)) {
        printf("not support streaming\n");
        return -1;
    }

    // 设置 ROI 区域
    if (set_isp_crop_roi(fd, ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT) < 0)
        return -1;

    // 3. 设置格式
    struct v4l2_format fmt;
    memset((void*)&fmt, 0, sizeof(struct v4l2_format));
    fmt.type = V4L2_CAPTURE_TYPE;
    fmt.fmt.pix_mp.pixelformat = V4L2_CAPTURE_FMT;
    fmt.fmt.pix_mp.width = ROI_WIDTH;
    fmt.fmt.pix_mp.height = ROI_HEIGHT;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    fmt.fmt.pix_mp.num_planes = 1;

    // 申请格式
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) 
    { 
        perror("S_FMT failed"); 
        return -1; 
    }
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_CAPTURE_TYPE;

    // 返回的实际格式
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) < 0) 
    { 
        perror("G_FMT failed"); 
        return -1; 
    }

    int width  = fmt.fmt.pix_mp.width;
    int height = fmt.fmt.pix_mp.height;
    int bytesperline = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
    int sizeimage = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
    printf("actual fmt: %dx%d, pixfmt=NV12, bytesperline=%d, sizeimage=%d\n", width, height, bytesperline, sizeimage);

    // 创建一个目标 buffer 内存池。
    // 构造函数必定成功
    bufferPool dstBufferPool(V4L2_BUFFER_NUM);
    dstBufferPool.init(width, height, RK_FORMAT_BGR_888);

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
        pBuf = new dmaHeapBuffer(RK_FORMAT_YCbCr_420_SP, width, height, i);
        v.push_back(pBuf);  // 推到链表中
    }

    // 启动 v4l2 stream。
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count  = V4L2_BUFFER_NUM;
    req.type   = V4L2_CAPTURE_TYPE;
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

    // 打开 pump 对应的 ioctl 文件
    int pumpFd = open(PUMP_DEV_PATH, O_RDWR);
    if (pumpFd < 0) 
    { 
        perror("open pump device"); 
        return -1; 
    }

    enum v4l2_buf_type type = V4L2_CAPTURE_TYPE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) 
    { 
        perror("STREAMON"); 
        return -1; 
    }

    // 初始化线程
    frameCapture threadCapture(v, rawFrameQueue, fd);
    frameAcc threadAcc(fd, 0, 0, width, height, rawFrameQueue, processedFrameQueue, dstBufferPool);
    frameProc threadProc(processedFrameQueue, dstBufferPool, dropletInfoQueue);
    dropletCtrl threadCtrl(dropletInfoQueue, pumpFd);
    preMonitor threadPre;
    debugCapture threadDebug;

    threadDebug.prepare();

    threadDebug.start();
    threadCapture.start();
    threadAcc.start();
    threadProc.start();
    threadCtrl.start();
    threadPre.start();

    std::cout << "system is running..." << std::endl;
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
    threadDebug.stop();
}
