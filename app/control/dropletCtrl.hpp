#pragma once

#include "threadSafeQueue.h"
#include "frameProc.h"
#include <sys/ioctl.h>
#include <thread>
#include <cmath>
#include <chrono>
#include "rk_pump.h"
#include "gError.h"

#define DROPLET_COUNT_MAX       10
#define DROPLET_COUNT_MIN       6
#define MAX_ERROR_FRAME_NR      10

#define RATIO_BUFFER_BASE_FREQ 2100.0f // 初始的缓冲液基准频率

// 比例环参数
#define RATIO_SETPOINT 0.0f
#define RATIO_KP 22.4f
#define RATIO_KI 5.1f
#define RATIO_CONVERGE_THRESH 0.05f

// 均匀度环参数（等比例缩放两泵总频率）
#define VAR_SETPOINT 0.02f
#define VAR_KP 150.0f
#define VAR_KI 30.0f

#define FREQ_MIN 50.0f   // 输出脉冲频率需要限幅. 因为过低的频率对应脉冲的死区.
#define FREQ_MAX 3000.0f // 泵不支持 3000 以上的频率.

// 均匀度环回退参数. 是较为宽松的参数限制. 如果不满足, 需要回退至比例环
#define RATIO_FALLBACK_THRESH 0.08f
#define RATIO_FALLBACK_COUNT_MAX 2
#define RATIO_CORRECT_COUNT_MAX 3

// 流量相关, 用于冻结 PI 控制
#define BLOOD_FLOW_PER_HZ   0.001f
#define BUFFER_FLOW_PER_HZ  0.001f
#define PIPE_EFFECTIVE_VOLUME  100.0f // 从控制作用位置到视觉 ROI 的等效管路容积
#define TRANSITION_WINDOW_VOLUME 20.0f // 确保液滴稳定, 可以增加一个等待窗口. 这里直接将窗口增加在管理容积处
#define CTRL_FREEZE_VOLUME (PIPE_EFFECTIVE_VOLUME + TRANSITION_WINDOW_VOLUME) // 总冻结目标容积

enum class ctrlStage
{
    RATIO_STAGE,
    VAR_STAGE
};

// ---------------------------------------------------------------
// 配比环前馈查找表
// 二元组: {目标配比, 实际输出频率}
typedef struct ratioMap {
    float ratio_setpoint;   // 目标配比
    float blood_freq_hz;    // 实际输出频率
}ratioMap_t;

const static ratioMap_t ratioFreqMap[] = {
    { -0.50f,  952.0f },
    { -0.30f, 1246.0f },
    { -0.10f, 1785.0f },
    {  0.00f, 2150.0f },
    {  0.10f, 2542.0f },
    {  0.20f, 2788.0f },
    {  0.30f, 3153.0f },
};

static const int ratioFreqMapSize = sizeof(ratioFreqMap) / sizeof(ratioFreqMap[0]);

// 配比环前馈查找函数. 输入目标配比, 返回
static float ratioFreqLookup(float ratio_setpoint)
{
    if (ratio_setpoint <= ratioFreqMap[0].ratio_setpoint)
        return ratioFreqMap[0].blood_freq_hz;

    if (ratio_setpoint >= ratioFreqMap[ratioFreqMapSize - 1].ratio_setpoint)
    {
        return ratioFreqMap[ratioFreqMapSize - 1].blood_freq_hz;
    }

    for (int i = 0; i < ratioFreqMapSize - 1; ++i)
    {
        const ratioMap_t& left  = ratioFreqMap[i];
        const ratioMap_t& right = ratioFreqMap[i + 1];

        if (ratio_setpoint >= left.ratio_setpoint &&
            ratio_setpoint < right.ratio_setpoint)
        {
            float t = (ratio_setpoint - left.ratio_setpoint) / (right.ratio_setpoint - left.ratio_setpoint);
            return left.blood_freq_hz + t * (right.blood_freq_hz - left.blood_freq_hz);
        }
    }

    // should not be here
    return ratioFreqMap[0].blood_freq_hz;
}
// ---------------------------------------------------------------


// ---------------------------------------------------------------
// 均匀度环前馈查找表
// 二元组: {目标均匀度, 实际输出频率}
typedef struct VarMap
{
    float variance_setpoint;
    float total_freq_hz;
}VarMap_t;

static const VarMap_t varFreqMap[] = {
    { 0.005f, 1645.0f },
    { 0.010f, 2242.0f },
    { 0.020f, 3189.0f },
    { 0.040f, 4255.0f },
    { 0.060f, 4960.0f },
    { 0.080f, 5776.0f },
    { 0.100f, 6014.0f },
};

static const int varFreqMapSize = sizeof(varFreqMap) / sizeof(varFreqMap[0]);

static float varFreqLookup(float variance_setpoint)
{
    if (variance_setpoint <= varFreqMap[0].variance_setpoint)
        return varFreqMap[0].total_freq_hz;

    if (variance_setpoint >= varFreqMap[varFreqMapSize - 1].variance_setpoint)
    {
        return varFreqMap[varFreqMapSize - 1].total_freq_hz;
    }

    for (int i = 0; i < varFreqMapSize - 1; ++i)
    {
        const VarMap_t& left  = varFreqMap[i];
        const VarMap_t& right = varFreqMap[i + 1];

        if (variance_setpoint >= left.variance_setpoint && variance_setpoint < right.variance_setpoint)
        {
            float t = (variance_setpoint - left.variance_setpoint) / (right.variance_setpoint - left.variance_setpoint);
            return left.total_freq_hz + t * (right.total_freq_hz - left.total_freq_hz);
        }
    }
    return varFreqMap[0].total_freq_hz;
}
// ---------------------------------------------------------------

class dropletCtrl {
private:
    threadSafeQueue<dropletInfo>& _inQueue;
    int         _fd            = -1;
    bool        _isRunning     = false;
    std::thread _workerThread;
    int         _errorFrameCount = 0;

    // 增量式 PI
    float _ratioLastError = 0.0f;  
    float _varLastError   = 0.0f;
    bool  _ratioConverged = false;

    // 当前正在控制比例环 / 均匀度环
    ctrlStage _ctrlStage = ctrlStage::RATIO_STAGE;  // 初始时刻控制比例环

    // 当前下发给两路泵的频率
    float _currentBloodFreq  = ratioFreqLookup(RATIO_SETPOINT); // 血液基准频率: 根据缓冲液基准频率计算得出
    float _currentBufferFreq = RATIO_BUFFER_BASE_FREQ;     // 缓冲液基准频率. 设定好的 RATIO_BUFFER_BASE_FREQ

    int _ratioCorrectCount = 0;  // 均匀度环正常液滴计数
    int _ratioFallbackCount = 0;  // 均匀度环比例异常计数

    bool _piFrozen = false;  // 标志位: 是否需要冻结 PI
    float _passedVolume = 0.0f;  // 距离上一次 PI 下发, 流过了多少流量
    float _estimatedTotalFlow = 0.0f;
    std::chrono::steady_clock::time_point _lastVolumeUpdateTime;  // 上一次更新的时间点

    // 限幅函数: 限制 val 在 [low, height] 之间. 
    static float clamp(float val, float low, float height)
    {
        return val < low ? low : (val > height ? height : val);
    }

    static float bloodFreqToFlow(float freq)
    {
        return BLOOD_FLOW_PER_HZ * freq;
    }   

    static float bufferFreqToFlow(float freq)
    {
        return BUFFER_FLOW_PER_HZ * freq;
    }

    float estimateTotalFlow()
    {
        float bloodFlow = bloodFreqToFlow(_currentBloodFreq);
        float bufferFlow = bufferFreqToFlow(_currentBufferFreq);
        return bloodFlow + bufferFlow;
    }

    int setFreq(int pump_id, int freq_hz)
    {
        struct pump_freq arg;
        arg.pump_id = pump_id;
        arg.freq_hz = freq_hz;
        return ioctl(_fd, PUMP_SET_FREQ, &arg);
    }

    // 重新根据前馈查找表计算频率. 在初始化时期/RESET 时期/配比环从稳定-不稳定的时期 可能会调用该函数
    void ratioFeedforward()
    {
        float ratioBloodFreq = ratioFreqLookup(RATIO_SETPOINT);
        int bloodFreqInt = static_cast<int>(clamp(ratioFreqLookup(RATIO_SETPOINT), FREQ_MIN, FREQ_MAX));
        int bufferFreqInt = static_cast<int>(clamp(RATIO_BUFFER_BASE_FREQ, FREQ_MIN, FREQ_MAX));

        if(setFreq(0, bloodFreqInt) < 0)
            throw std::runtime_error("set blood frequency failed");
        if(setFreq(1, bufferFreqInt) < 0)
            throw std::runtime_error("set buffer frequency failed");

        _currentBloodFreq = static_cast<float>(bloodFreqInt);
        _currentBufferFreq = static_cast<float>(bufferFreqInt);
        // PI 从新的前馈工作点重新开始修正
        _ratioLastError = 0.0f;
        _ratioConverged = false;

        // 每次触发反馈之后, 冻结 PI
        beginPiFreeze();
    }


    // 当前两相泵频率来自已经收敛的配比环. 
    // 先计算当前两相频率比例，再保持该比例分配前馈总频率。
    void varFeedforward()
    {
        float currentTotalFreq = _currentBloodFreq + _currentBufferFreq;

        // 正常情况下两路频率均大于零。
        // 如果内部状态异常，则退化为 1:1 分配。
        float bloodRatio = 0.5f;

        if (currentTotalFreq > 0.0f)
            bloodRatio = _currentBloodFreq / currentTotalFreq;

        float bufferRatio = 1.0f - bloodRatio;

        // 根据目标方差查询前馈总频率
        float totalFfFreq = varFreqLookup(VAR_SETPOINT);
        totalFfFreq = clamp(totalFfFreq, FREQ_MIN * 2.0f, FREQ_MAX * 2.0f);

        // 保持配比环已经稳定的两相比例
        float newBloodFreq = totalFfFreq * bloodRatio;
        float newBufferFreq = totalFfFreq * bufferRatio;

        newBloodFreq = clamp(newBloodFreq, FREQ_MIN, FREQ_MAX);
        newBufferFreq = clamp(newBufferFreq, FREQ_MIN, FREQ_MAX);

        int bloodFreqInt = static_cast<int>(newBloodFreq);
        int bufferFreqInt = static_cast<int>(newBufferFreq);
        if(setFreq(0, bloodFreqInt) < 0)
            throw std::runtime_error("set blood frequency failed");
        if(setFreq(1, bufferFreqInt) < 0)
            throw std::runtime_error("set buffer frequency failed");

        _currentBloodFreq = static_cast<float>(bloodFreqInt);
        _currentBufferFreq = static_cast<float>(bufferFreqInt);

        // 均匀度 PI 从前馈工作点附近重新开始修正
        _varLastError = 0.0f;

        // 冻结 PI
        beginPiFreeze();
    }

    void beginPiFreeze()
    {
        _estimatedTotalFlow = estimateTotalFlow();
        _passedVolume = 0.0f;
        _lastVolumeUpdateTime = std::chrono::steady_clock::now();
        _piFrozen = true;
    }

    bool updatePiFreeze()
    {
        if (!_piFrozen)
            return false;
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        float elapsedSeconds = std::chrono::duration<float>(now - _lastVolumeUpdateTime).count();

        _lastVolumeUpdateTime = now;

        if (_estimatedTotalFlow > 0.0f && elapsedSeconds > 0.0f)
            _passedVolume += _estimatedTotalFlow * elapsedSeconds;

        if (_passedVolume >= CTRL_FREEZE_VOLUME)
        {
            _piFrozen = false;
            _passedVolume = 0.0f;
            return false;
        }

        return true;
    }
    
    // 比例环
    void ratioLoop(float mean)
    {
        float e = RATIO_SETPOINT - mean;  // c < 0 表示蓝色偏多、血液偏少，需要提高血液泵频率

        float deltaFreq = RATIO_KP * (e - _ratioLastError) + RATIO_KI * e;
        float bloodFreq = _currentBloodFreq + deltaFreq;
    
        bloodFreq = clamp(bloodFreq, FREQ_MIN, FREQ_MAX);

        int oldFreq = _currentBloodFreq;
        int newFreq = bloodFreq;
        if(newFreq != oldFreq)
        {
            if(setFreq(0, (int)bloodFreq) < 0)
                throw std::runtime_error("set blood frequency failed");

            _currentBloodFreq = newFreq;
            beginPiFreeze();
        }
        _ratioLastError = e;    
        _ratioConverged = (fabsf(e) < RATIO_CONVERGE_THRESH);
    }

    // 均匀度环：等比例缩放两泵总频率
    void varLoop(float variance)
    {
        float e = variance - VAR_SETPOINT;  // e>0 方差偏大，需降速

        float deltaTotalFreq = VAR_KP * (e - _varLastError) + VAR_KI * e;
        float currentTotalFreq = _currentBloodFreq + _currentBufferFreq;
        float totalFreq = currentTotalFreq - deltaTotalFreq;

        totalFreq = clamp(totalFreq, FREQ_MIN * 2, FREQ_MAX * 2);

        // 使用当前两路泵的频率计算比例 bloodRatio 和 bufferRatio. 
        float bloodRatio = 0.5f;
        if (currentTotalFreq > 0.0f)
            bloodRatio = _currentBloodFreq / currentTotalFreq;

        float bufferRatio = 1.0f - bloodRatio;

        // 保持比例不变, 只修改总频率
        // 血液泵新频率
        float newBloodFreq = totalFreq * bloodRatio;
        newBloodFreq = clamp(newBloodFreq, FREQ_MIN, FREQ_MAX);

        // 缓冲液泵新频率
        float newBufferFreq = totalFreq * bufferRatio;
        newBufferFreq = clamp(newBufferFreq, FREQ_MIN, FREQ_MAX);

        // 不能使用浮点比较, 需要使用整形比较.
        // PI 计算出来的浮点数需要换成 freq 的整形. 否则会出现无意义的冻结
        int oldBloodFreq = static_cast<int>(_currentBloodFreq);
        int oldBufferFreq = static_cast<int>(_currentBufferFreq);
        int newBloodFreqInt = static_cast<int>(newBloodFreq);
        int newBufferFreqInt = static_cast<int>(newBufferFreq);
        // 是否需要修改
        bool frequencyChanged = newBloodFreqInt != oldBloodFreq || newBufferFreqInt != oldBufferFreq;
        if(frequencyChanged)
        {
            if(setFreq(0, newBloodFreqInt) < 0)
                throw std::runtime_error("set blood frequency failed");
            if(setFreq(1, newBufferFreqInt) < 0)
                throw std::runtime_error("set buffer frequency failed");

            // 保存当前下发频率
            _currentBloodFreq  = static_cast<float>(newBloodFreq);
            _currentBufferFreq = static_cast<float>(newBufferFreq);
            // 冻结 PI
            beginPiFreeze();
        }
        _varLastError = e;
    }

    void doReset()
    {
        // 这个函数中直接上抛异常, 由 dropletCtrlLoop 循环接住异常.
        if (ioctl(_fd, PUMP_CMD_FLUSH) < 0) {
            throw std::runtime_error("doReset start error. ioctl failed");
        }
        // 等待硬件重洗
        for(int i = 0; i < 10; i++)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::cout << "do reset, waiting " + std::to_string(10 - i) + "seconds" << std::endl;
        }
        if (ioctl(_fd, PUMP_CMD_FLUSH_DONE) < 0) {
            throw std::runtime_error("doReset start error. ioctl failed");
        }

        // 重置系统状态,重新启动混合
        _ratioConverged = false;
        _errorFrameCount = 0;

        // 清除误差
        _ratioLastError = 0.0f;
        _varLastError = 0.0f;

        // 清除冻结 PI 相关标志位
        _piFrozen = false;
        _passedVolume = 0.0f;
        _estimatedTotalFlow = 0.0f;

        _ratioCorrectCount = 0;
        _ratioFallbackCount = 0;
      
        if (ioctl(_fd, PUMP_CMD_START_MIX) < 0) { 
            throw std::runtime_error("dropletCtrl mix start error. ioctl failed");
        }
        
        _ctrlStage = ctrlStage::RATIO_STAGE;
        ratioFeedforward();

        // 修改 needReinit 标志位
        needReinit.store(true);
        _inQueue.empty();  // 清空队列
    }

public:
    dropletCtrl(threadSafeQueue<dropletInfo>& inQueue, int fd)
        : _inQueue(inQueue),
        _fd(fd)
    {}

    ~dropletCtrl()
    {
        stop();
    }

    void dropletCtrlLoop()
    {
        try
        {
            if (ioctl(_fd, PUMP_CMD_START_MIX) < 0) {
                throw std::runtime_error("dropletCtrl mix start error. ioctl failed");
            }
            _ctrlStage = ctrlStage::RATIO_STAGE;
            
            // 比例环前馈. 确定初始状态下的两相流速
            ratioFeedforward();

            while (_isRunning) 
            {
                if (resetFlag.load()) 
                {
                    doReset();
                    resetFlag.store(false);
                    continue;  // 触发复位之后直接 doReset. 后续这个线程会阻塞在 _inQueue.pop() 上.
                }
                dropletInfo info = _inQueue.pop();
                if (!_isRunning)
                    break; 
                             
                // 在前一次 PI 控制指令下发 - 这一轮液滴出现在视窗内. PI 会被冻结
                if (_piFrozen)
                {
                    if(updatePiFreeze())
                        continue;
                }

                // 液滴大小检查（门控）
                if (info.size > DROPLET_COUNT_MAX || info.size < DROPLET_COUNT_MIN) 
                {
                    if (++_errorFrameCount > MAX_ERROR_FRAME_NR) 
                    {
                        _errorFrameCount = 0;
                        doReset();
                        continue; 
                    }
                    continue;
                }
                _errorFrameCount = 0;

                switch (_ctrlStage)
                {
                    // 配比环. 如果配比不满足, 优先调整配比.
                    case ctrlStage::RATIO_STAGE:
                    {
                        // 比例阶段
                        ratioLoop(info.mean);

                        // 比例达到阈值后，下一滴开始进入均匀度阶段
                        if (_ratioConverged)
                        {
                            _ratioCorrectCount++;  // 比例环收敛计数++
                            if (_ratioCorrectCount >= RATIO_CORRECT_COUNT_MAX)
                            {
                                varFeedforward();  // 均匀度前馈
                                _ctrlStage = ctrlStage::VAR_STAGE;
                                // 清除两个窗口计数
                                _ratioCorrectCount = 0;
                                _ratioFallbackCount = 0;
                            }
                        }
                        else 
                            _ratioCorrectCount = 0;
                        break;
                    }

                    case ctrlStage::VAR_STAGE:
                    {
                        float ratioError = info.mean - RATIO_SETPOINT;  // 先计算配比误差, 如果配比误差不满足, 需要回退
                        if (fabsf(ratioError) > RATIO_FALLBACK_THRESH)
                        {
                            _ratioFallbackCount++;  // 增加计数值
                            // 超过计数值, 回退
                            if (_ratioFallbackCount >= RATIO_FALLBACK_COUNT_MAX)
                            {
                                _ctrlStage = ctrlStage::RATIO_STAGE;

                                _ratioFallbackCount = 0;
                                _ratioCorrectCount = 0;
                                ratioFeedforward();
                                _ratioLastError = 0.0f;
                                break;
                            }
                        }
                        else
                            _ratioFallbackCount = 0;

                        // 均匀度阶段只执行均匀度环
                        varLoop(info.variance);
                        break;
                    }

                    default:
                        break;
                }
            }
        }
        catch(const std::exception& e)
        {
            fatalErrorMsg = std::string("dropletCtrl") + e.what();
            fatalError.store(true);
        }
    }

    void start()
    {
        if (!_isRunning) 
        {
            _isRunning = true;
            _workerThread = std::thread(&dropletCtrl::dropletCtrlLoop, this);
        }
    }

    void stop()
    {
        if (_isRunning) 
        {
            _isRunning = false;
            _inQueue.close();  // 如果任务阻塞在队列上, 就先唤醒
            if (_workerThread.joinable())
                _workerThread.join();
        }
    }
};
