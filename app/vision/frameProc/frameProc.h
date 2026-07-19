#pragma once
#include <linux/videodev2.h>
#include <iostream>
#include <cstring>
#include <vector>
#include <queue>
#include <mutex>
#include <algorithm>
#include <condition_variable>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>
#include "dmaHeap.h"
#include "threadSafeQueue.h"
#include "bufferPool.h"
#include "gError.h"

enum class procState {
    STATE_IDLE = 0,
    STATE_SAMPLING,
    STATE_LEAVE
};

enum class initState {
    INIT_NO_CALIBRATE = 0,  // 未校准
    INIT_NO_INIT,           // 未初始化
    INIT_OK
};

struct dropletInfo {
    int size = 0;       // 液滴平均像素面积（每帧白色像素数的均值）
    float variance = 0.0f;    // 混合均匀度
    float mean  = 0.0f;    // 混合比例
};

class frameProc{
    private:
        int _globalFrameCount = 0;   // 全局帧数
        int _lastBackgroundUpdateFrame = 0;  // 上一次触发背景更新时的全局帧数
        int _frameCount = 0;  // 初始化时的帧数
        int _sumPixels = 0;
        bool _isRunning = false;
        std::thread _workerThread;
        threadSafeQueue<dmaHeapBuffer*>& _inQueue;
        bufferPool& _outPool;
        initState _initState = initState::INIT_NO_INIT;
        threadSafeQueue<dropletInfo>& _infoQueue;

        // proc state 处理一个液滴过程中用到的临时变量
        double _sumc;               // 计算这个液滴的均值
        double _sumcSquare;         // 计算这个液滴的平方和(用于计算方差)
        int _dropletFrameCount;          // 这个液滴的帧数(用于反映其大小)
        procState _procState;
        cv::Mat _backgroundRoi;     // 初始化的像素矩阵
        cv::Mat _tempRoi;           // 用于计算的矩阵 
        // proc state

        void cvProcReset();
        void filterBubblePixels(const cv::Mat& currentRoi, cv::Mat& mask);
        int backgroundRoiInit(const cv::Mat& currentRoi);
        int cvProc(const cv::Mat& currentRoi);
    public:
        frameProc(threadSafeQueue<dmaHeapBuffer*>& inQueue, bufferPool& outPool, threadSafeQueue<dropletInfo>& infoQueue);
        ~frameProc();
        void frameProcLoop();
        void start();
        void stop();
};

