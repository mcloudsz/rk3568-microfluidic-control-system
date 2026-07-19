#include "frameProc.h"

#define TOTAL_INIT_FRAME_NUM 10
#define THRESHOLD_VAL 30          // 小于这个值的像素点会被当做噪声处理
#define SAMPLE_THRESHOLD_ENTER 80   // 大于这个值会被认作 ENTER 的开始
#define SAMPLE_THRESHOLD_LEAVE 20  // 大于这个值会被认作 LEAVE 的开始

// 气泡滤除相关.这个宏的含义是气泡像素阈值.如果 RGB 三个通道的值都高于这个值, 则可能会被判定为气泡
#define BUBBLE_RGB_MIN 220
// 气泡滤除相关.这个宏的含义是在 BUBBLE_RGB_MIN 满足的情况下, 如果三个通道的差值均小于 BUBBLE_CHANNEL_DIFF, 则会被判定为气泡
#define BUBBLE_CHANNEL_DIFF 25

// 背景更新相关
#define BACKGROUND_UPDATE_FRAME_NUM  300  // 当前帧数 - 上一次背景更新帧数如果大于这个阈值, 需要进行背景更新
#define BACKGROUND_UPDATE_ALPHA 0.05  // 
#define BACKGROUND_UPDATE_MAX_COUNT  10 // 背景中的噪点小于这个值, 则判断是一个干净的背景. 允许更新.

void frameProc::cvProcReset()
{
    if(_initState != initState::INIT_OK && !_tempRoi.empty()) 
        _tempRoi.setTo(cv::Scalar(0, 0, 0));
    _sumc = 0;
    _sumcSquare = 0;
    _dropletFrameCount = 0;
    _frameCount = 0;
    _sumPixels = 0;
    _procState = procState::STATE_IDLE;
}

int frameProc::backgroundRoiInit(const cv::Mat& currentRoi)
{    
    // 第一次进入时, 初始化
    if(_frameCount == 0)
    {
        _tempRoi = cv::Mat::zeros(currentRoi.size(), CV_32FC3);
        _backgroundRoi = cv::Mat::zeros(currentRoi.size(), CV_8UC3);
    }

    cv::Mat tempFloatRoi;  // 存放转换后的(8位 -> 32位)的数据帧
    currentRoi.convertTo(tempFloatRoi, CV_32FC3); 
    _tempRoi += tempFloatRoi; 
    _frameCount++;
    _globalFrameCount = 0;
    _lastBackgroundUpdateFrame = 0;
    
    // 收集完指定的帧数, 计算平均后返回
    if(_frameCount == TOTAL_INIT_FRAME_NUM)
    {
        _tempRoi.convertTo(_backgroundRoi, CV_8UC3, 1.0 / TOTAL_INIT_FRAME_NUM);
        // 清零缓冲区
        cvProcReset();
        _initState = initState::INIT_OK;  // 设置标志位

        // 清除 reinit 标志位
        needReinit.store(false);
        // 再清除一遍 _infoQueue 队列, 防止因为进程时序调度问题的旧数据残留
        _infoQueue.empty(); 
    }

    return 0;
}

void frameProc::filterBubblePixels(const cv::Mat& currentRoi, cv::Mat& mask)
{
    for (int y = 0; y < mask.rows; ++y)
    {
        uchar* maskRow = mask.ptr<uchar>(y);
        const cv::Vec3b* rgbRow = currentRoi.ptr<cv::Vec3b>(y);

        for (int x = 0; x < mask.cols; ++x)
        {
            // 当前像素原本就不是前景，不需要处理
            if (maskRow[x] == 0)
                continue;

            const cv::Vec3b& pixel = rgbRow[x];

            int r = pixel[0];
            int g = pixel[1];
            int b = pixel[2];

            int maxChannel = std::max({r, g, b});
            int minChannel = std::min({r, g, b});

            // 判定条件一: 所有通道的像素值均大于 BUBBLE_RGB_MIN
            bool highBrightness = r >= BUBBLE_RGB_MIN && g >= BUBBLE_RGB_MIN && b >= BUBBLE_RGB_MIN;
            // 判定条件二: 两个通道的差值小于 BUBBLE_CHANNEL_DIFF
            bool similarChannels = (maxChannel - minChannel) <= BUBBLE_CHANNEL_DIFF;

            // 条件一和条件二全部满足, 则可以判定为气泡
            if (highBrightness && similarChannels)
                // 从前景 mask 中删除气泡像素
                maskRow[x] = 0;
        }
    }
}

int frameProc::cvProc(const cv::Mat& currentRoi)
{
    // 检查: 是否初始化完成
    if(_initState != initState::INIT_OK)
        return -1;

    _globalFrameCount++;  // 总帧数

    // 计算 currentRoi 和 backgroundRoi 的差异，得到 diffRoi
    cv::Mat diffRoi;
    cv::absdiff(_backgroundRoi, currentRoi, diffRoi);

    // 计算 diffRoi 的灰度图
    cv::Mat diffGray;
    cv::cvtColor(diffRoi, diffGray, cv::COLOR_RGB2GRAY);

    // 根据灰度图 diffGray 计算二值化掩码图 mask
    cv::Mat mask;
    cv::threshold(diffGray, mask, THRESHOLD_VAL, 255, cv::THRESH_BINARY);

    // 滤除气泡
    filterBubblePixels(currentRoi, mask);

    // 计算图像中的白色点个数
    int count = cv::countNonZero(mask);

    switch(_procState)
    {
        case procState::STATE_IDLE:
            // 开始采集
            if(count >= SAMPLE_THRESHOLD_ENTER)
            {
                _procState = procState::STATE_SAMPLING;
                break;
            }
            // 尝试更新背景
            // 更新背景的条件: 帧数相差超过 BACKGROUND_UPDATE_FRAME_NUM, 且当前背景中的白点 count 小于 BACKGROUND_UPDATE_MAX_COUNT
            // 则可以认为背景是干净的
            if((_globalFrameCount - _lastBackgroundUpdateFrame) >= BACKGROUND_UPDATE_FRAME_NUM && count <= BACKGROUND_UPDATE_MAX_COUNT)
            {
                cv::addWeighted(_backgroundRoi, 1.0 - BACKGROUND_UPDATE_ALPHA, currentRoi, BACKGROUND_UPDATE_ALPHA, 0.0, _backgroundRoi);

                _lastBackgroundUpdateFrame = _globalFrameCount;
            }
            break;
        case procState::STATE_SAMPLING:
        {
            // 如果画框内液滴大小小于阈值，认为液滴已经全部移走
            if(count <= SAMPLE_THRESHOLD_LEAVE)
                _procState = procState::STATE_LEAVE;  // 穿透
            else 
            {
                // 获取白色点区域的 RGB 三通道均值
                // 我们只关心 R 通道 [0] 和 B 通道 [2]
                cv::Scalar meanVal = cv::mean(currentRoi, mask);
                double c = (meanVal[0] - meanVal[2]) / (meanVal[0] + meanVal[2] + 0.0001);
                _sumc += c; // 累加 c
                _sumcSquare += c * c; 

                _dropletFrameCount++;  // 累加计数
                _sumPixels += count;
                break;
            }   
        }

        case procState::STATE_LEAVE:
        {
            // 没有采集到有效液滴帧，直接复位
            if(_dropletFrameCount == 0)
            {
                cvProcReset();
                break;
            }

            // 求出平均的 c 值
            double avec = _sumc / _dropletFrameCount;

            // 计算平方和的期望 E(X^2)
            double avecSquare = _sumcSquare / _dropletFrameCount;
                    
            // 计算方差: 方差 = E(X^2) - [E(X)]^2
            double varc = avecSquare - (avec * avec);
            if (varc < 0) 
                varc = 0;

            // 表示一个液滴信息的三元组: 
            // 1. _dropletFrameCount 反映液滴的尺寸
            // 2. varc 反映液滴混合的均匀程度
            // 3. avec 反映液滴混合的比例

            std::cout << "droplet info. count:" + std::to_string(_dropletFrameCount) + "var:" + std::to_string(varc) + 
                "ave:" + std::to_string(avec) << std::endl;

            // 构造一个液滴信息结构体, 发送给处理线程
            dropletInfo info;
            info.size = _dropletFrameCount;
            info.mean = avec;
            info.variance = varc;
            _infoQueue.push(info);  // 值拷贝

            cvProcReset();
            break;
        }
        default:
            break;
    }
    return 0;
}

frameProc::frameProc(threadSafeQueue<dmaHeapBuffer*>& inQueue, bufferPool& outPool, threadSafeQueue<dropletInfo>& infoQueue)
    :_inQueue(inQueue),
    _outPool(outPool),
    _infoQueue(infoQueue)
{

}

frameProc::~frameProc()
{
    stop();
}

#include "scopeTimer.hpp"
void frameProc::frameProcLoop()
{   
    try
    {
        while (_isRunning)
        {
            // 从 inQueue 中获取处理好的帧
            dmaHeapBuffer* pBuf = _inQueue.pop();
            
            // 只会执行一次
            static int width = pBuf->getWidth();
            static int height = pBuf->getHeight();
            cv::Mat currentRoi(height, width, CV_8UC3, pBuf->getPtr());

            // 判断是否初始化, 如果未初始化, 就调用 background_roi_init 函数进行初始化
            if(_initState == initState::INIT_NO_INIT || needReinit.load())
            {
                ScopeTimer t("backgroundRoiInit");
                if(backgroundRoiInit(currentRoi) < 0)
                    throw std::runtime_error("frameProc unexpected error. proc failed");
            }

            // 如果已初始化, 就调用 cv_proc 接管代码逻辑
            else
            {
                ScopeTimer t("cvProc");
                if(cvProc(currentRoi) < 0)
                    throw std::runtime_error("frameProc unexpected error. proc failed");
            }
            _outPool.push(pBuf);
        }
    }
    catch(const std::exception& e)
    {
        fatalErrorMsg = std::string("frameCapture") + e.what();
        fatalError.store(true);
    }
}

void frameProc::start()
{
    if(_isRunning == false)
    {
        _isRunning = true;
        _workerThread = std::thread(&frameProc::frameProcLoop, this);
    }
}

void frameProc::stop()
{
    if(_isRunning)
    {
        _isRunning = false;
        _inQueue.close();
        if(_workerThread.joinable())
            _workerThread.join();
    }
}

