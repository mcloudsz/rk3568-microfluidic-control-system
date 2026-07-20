# rk3568-microfluidic-control-system

## 1. 项目简介

### 1.1 应用场景
### 1.2 核心功能
### 1.3 系统组成
### 1.4 系统运行流程

## 2. 系统设计与关键实现

### 2.1 系统整体框架

系统以 RK3568 为主控，围绕摄像头图像采集、液滴图像处理、泵阀控制和压力监测构建，软件整体分为 Linux 内核驱动层和用户态应用层。

系统主要包含以下模块：

- MIPI CSI 摄像头与 ISP 图像处理链路；
- V4L2 双路图像采集；
- DMA-BUF 与 RGA 图像处理链路；
- 液滴图像处理；
- 血液、缓冲液和冲洗液三路泵阀控制；
- 压力监测与超压处理；
- 调试图像采集与保存。

系统整体架构如下：
<img width="1859" height="663" alt="image" src="https://github.com/user-attachments/assets/ce81b854-cb26-4df0-87df-113c22def40a" />

摄像头图像经过 Sensor、CSI 和 ISP 后分为主图像流和调试流。两路图像共享摄像头输入链路，并分别通过独立的 V4L2 Video Node 完成输出格式配置、缓冲区管理和图像采集。

| 图像流 | 设备节点 | 输出格式 | 主要用途 |
|---|---|---|---|
| 主图像流 | `/dev/video0` | NV12，100×64 | 输出目标区域图像，供 RGA 转换和后续液滴图像处理 |
| 调试流 | `/dev/video1` | NV12，320×240 | 按帧间隔保存灰度图像，用于图像链路和处理结果定位 |

RGA 导入 V4L2 使用的 DMA-BUF，将 NV12 图像转换为 BGR 图像。V4L2 与 RGA 共享采集缓冲区，避免两者之间的 CPU 侧图像拷贝。

调试流使用 V4L2 MMAP 缓冲区独立采集图像。程序按照设定的帧间隔保存 NV12 图像的 Y 平面，并生成 PGM 灰度图。调试流不参与主图像处理和泵阀控制流程。


图像采集线程、RGA 处理线程和图像处理线程之间传递缓冲区指针，不直接复制完整图像。RGA 输出缓冲区由缓冲池统一分配和回收，处理完成后重新归还缓冲池，供后续图像帧继续使用。

图像处理结果通过 `dropletInfoQueue` 提交给液路控制线程。控制线程根据目标状态、当前液滴状态和控制阶段计算泵阀调节量，并通过字符设备接口控制血液和缓冲液两路泵阀。

压力监测链路独立于图像处理链路运行。当检测到液路压力超过设定阈值时，压力监测线程停止正常液路，并控制冲洗液路执行超压处理。

### 2.2 DMA-BUF 与 V4L2 采集

主图像流通过 `/dev/video0` 进行采集，输出格式配置为 NV12 100×64。程序在启动采集前查询设备能力，并通过 `VIDIOC_S_FMT` 设置图像格式，再使用 `VIDIOC_G_FMT` 获取驱动实际返回的宽度、高度、行跨度和缓冲区大小。流程如下：
1. 通过 `dma_heap` 分配 DMA-BUF；
2. 使用 `VIDIOC_REQBUFS` 申请 V4L2 Buffer 槽位；
3. 使用 `VIDIOC_QBUF` 将 DMA-BUF 加入采集队列；
4. 使用 `VIDIOC_STREAMON` 启动连续采集；
5. 使用 `VIDIOC_DQBUF` 获取已经完成采集的图像帧；
6. 将缓冲区指针写入 `rawFrameQueue`，交由后续线程处理；
7. 图像处理完成后重新执行 `VIDIOC_QBUF`，将缓冲区加入采集队列。

采集缓冲区由 `dma_heap` 从 CMA 内存池中分配，并以 DMA-BUF 文件描述符的形式提交给 V4L2。程序使用 `V4L2_MEMORY_DMABUF` 申请 4 个 Buffer 槽位，将每个 DMA-BUF 通过 `VIDIOC_QBUF` 加入采集队列，最后通过 `VIDIOC_STREAMON` 启动连续采集。


V4L2 完成一帧采集后，采集线程将对应缓冲区指针写入 `rawFrameQueue`。RGA 直接导入同一 DMA-BUF 完成后续格式转换，避免在 V4L2 与 RGA 之间进行 CPU 侧整帧图像拷贝。

### 2.3 RGA 图像链路

V4L2 主图像流输出 NV12 格式图像，后续图像处理需要使用 BGR 格式。系统通过 RGA 硬件完成 NV12 到 BGR 的格式转换，避免由 CPU 逐像素执行颜色空间转换。

RGA 处理流程如下：

1. `frameCapture` 线程从 V4L2 获取已经完成采集的 DMA-BUF；
2. 将采集缓冲区指针写入 `rawFrameQueue`；
3. `frameAcc` 线程从队列中取得 NV12 缓冲区；
4. 从 `bufferPool` 获取一个 BGR 输出缓冲区；
5. RGA 导入输入和输出缓冲区，完成 NV12 到 BGR 的格式转换；
6. 将处理后的缓冲区写入 `processedFrameQueue`；
7. 将原始采集缓冲区重新加入 V4L2 采集队列；
8. BGR 图像处理完成后，输出缓冲区重新归还 `bufferPool`。

RGA 输入端直接使用 V4L2 采集阶段分配的 DMA-BUF，不需要将 NV12 图像复制到新的用户态缓冲区。输出端由 `bufferPool` 预先分配固定数量的 BGR 缓冲区，并在图像处理线程之间循环复用。

| 缓冲区 | 图像格式 | 管理方式 | 主要用途 |
|---|---|---|---|
| 输入缓冲区 | NV12 | V4L2 DMA-BUF | 保存摄像头采集图像 |
| 输出缓冲区 | BGR888 | `bufferPool` | 保存 RGA 转换结果，供后续图像处理 |

通过将图像采集、RGA 转换和图像处理拆分到不同线程，并使用线程安全队列传递缓冲区指针，系统可以在处理当前图像时继续采集后续图像，减少各处理阶段之间的相互等待。
### 2.4 液滴图像分析

`frameProc` 线程从 `processedFrameQueue` 获取 RGA 转换后的 BGR 图像，对目标区域内的液滴状态进行分析，并将结果封装为 `dropletInfo` 写入 `dropletInfoQueue`，供液路控制线程使用。

图像处理流程如下：
1. 在系统初始化时，先开启缓冲液的阀门，在管道内部充满了蓝色缓冲液时，采集 10 帧 RGB 图像取均值作为 background_ROI。
2. 将当前帧的 current_ROI 与 background_ROI 求绝对差值，超过设定阈值即判定为前景（白），否则为背景（黑），生成二值化的掩码 mask。
3. 在 mask 中针对原像素图作气泡剔除的处理，得到剔除气泡之后的 mask。
4. 在裁剪区域内利用掩码中白色像素的数量触发状态切换。
   - 空闲态（IDLE）。掩码中几乎没有白色像素，说明无液滴经过。
   - 采样态（SAMPLING）。在采样态中，提取当前掩码区域内的像素，并将结果累加。
   - 离开态（LEAVE）。白色像素数量骤降至阈值以下，说明液滴已离开。统计结果并计算。计算完成之后，将状态机切换为 IDLE。
   - 由于液滴在进入或离开时白色像素数据会在阈值附近反复横跳，导致状态机频繁切换。因此加入迟滞参数，进入阈值和离开阈值设置不同的值（例如白色像素数 > 200 才进入 SAMPLING，白色像素数 < 80 才退出 SAMPLING）。
5. 在 SAMPLING 状态下，针对每一帧的有效像素直接计算归一化色差指数：C = (R - B)/(R + B)
6. 在一个完整液滴通过（LEAVE 态触发）后，计算该液滴的帧数，以及该液滴所有采样点 c 值的数学期望（均值）和方差。
   - 帧数代表液滴的大小。
   - 均值代表整体混合后的宏观颜色。
   - 方差代表混合的均匀程度（极小值代表完全混匀，较大值代表出现红蓝分层）。
7. 在获得液滴的帧数、均值和方差后，执行控制逻辑。控制逻辑见 2.7 章节。


液滴检测采用 `STATE_IDLE`、`STATE_SAMPLING` 和 `STATE_LEAVE` 三状态模型。状态机默认处于`STATE_IDLE`空闲态，当检测到的某帧（二值化后的）白色像素数量达到 80 时，清空本次统计数据并进入`STATE_SAMPLING`采样态。在采样态下
持续累计液滴像素数量、颜色分量、颜色平方和及有效帧数。当白色像素数量下降到 20 以下时，认为液滴离开主要检测区域并进入`STATE_LEAVE`E离开态。在离开态下：根据累计数据计算液滴尺寸、平均颜色和颜色方差，将结果封装为 `dropletInfo` 写入 `dropletInfoQueue`，随后清空统计数据并返回空闲态。


在图像处理线程中实现了背景ROI的动态更新。背景如果长时间不更新，可能会导致液滴大小逐渐增大。原因是：系统长时间运行之后会在管壁和管底附着一层灰色的颗粒。如果这些颗粒附着在相机的 ROI 区域，会导致识别图像时将这些灰色杂质也识别成液滴。因为背景图像使用的还是初始化时的纯净图像。
解决方案是定期动态更新背景图像。更新背景图像的策略是：
  1. 维护一个全局的帧数 _globalFrameCount 和上一次更新背景图像时的帧数 _lastBackgroundUpdateFrame。如果二者的差值到达一定阈值，就需要进行更新。
  2. 更新必须在状态机为 STATE_IDLE 的时候进行，并且更新时背景 mask 中的白色点必须小于一定阈值。这样更新出来的图像才是纯净的。

更新背景图像的代码如下。首先在 STATE_IDLE 中如果不进入 SAMPLE_THRESHOLD_ENTER，说明可能没有出现液滴，进入更新逻辑。在更新逻辑中检查：帧数是否符合要求,且 mask 中白色点个数是否小于阈值。如果这两个判断都满足，则可以进行更新。
```cpp
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
        // 更新背景的条件: 帧数相差超过 BACKGROUND_UPDATE_FRAME_NUM,
        // 且当前背景中的白点 count 小于 BACKGROUND_UPDATE_MAX_COUNT
        // 则可以认为背景是干净的
        if((_globalFrameCount - _lastBackgroundUpdateFrame) >= BACKGROUND_UPDATE_FRAME_NUM
             && count <= BACKGROUND_UPDATE_MAX_COUNT)
        {
            cv::addWeighted(_backgroundRoi, 1.0 - BACKGROUND_UPDATE_ALPHA, 
                currentRoi, BACKGROUND_UPDATE_ALPHA, 0.0, _backgroundRoi);

            _lastBackgroundUpdateFrame = _globalFrameCount;
        }
        break;
    case procState::STATE_SAMPLING:
    {
        // ... ...
```
另外还实现了气泡的滤除。分析气泡的像素特征：首先气泡的大小比较小。并且气泡的像素特征一般是接近白色，如 (235, 250, 245)。而普通液滴的像素特征一般是 红色液滴 (200, 80, 70)、蓝色液滴 (70, 90, 190)。

因此可以通过以下策略过滤掉气泡。先将原始 RGB 图像转化为单通道灰度图。其次对灰度图二值化处理提取出二值化掩码图 mask。随后遍历 mask 中的所有像素点，对于每一个像素点去原 RGB 图像中检查其像素值。如果原 RGB 图像的每个通道的像素值均大于 225 这个阈值，并且通道像素差值小于 25，则可以判断这个像素点是一个气泡。因此从 mask 中剔除这个像素点。

在获取 mask 之后调用 filterBubblePixels 函数滤除气泡。
```cpp
int frameProc::cvProc(const cv::Mat& currentRoi)
{
    // 检查: 是否初始化完成
    if(_initState != initState::INIT_OK)
        return -1;

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
    
    // ... ...
    // 进入状态机
```
剔除气泡的算法如下（filterBubblePixels 函数）：
```cpp
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
```
### 2.5 调试图像保存

系统通过 `/dev/video1` 获取独立的调试图像流，输出格式配置为 NV12 320×240。调试流使用 `V4L2_MEMORY_MMAP` 管理采集缓冲区，不参与主图像处理和液路控制流程。
PGM 文件只保存 NV12 图像中的亮度分量，不需要进行颜色格式转换，便于快速检查摄像头画面、目标区域位置和液滴处理结果。通过限制保存间隔和最大文件数量，可以减少调试图像写入对系统持续运行的影响。
### 2.6 泵阀字符设备驱动

系统包含血液、缓冲液和冲洗液三路液路，每路均由步进电机蠕动泵和电磁阀组成。内核驱动将三路泵阀统一封装为字符设备，用户态应用通过 `open()` 和 `ioctl()` 完成液路控制，不直接操作底层硬件接口。

驱动支持的主要控制命令如下：

| 命令 | 作用 |
|---|---|
| PUMP_CMD_START_MIX | 启动指定液路的蠕动泵和电磁阀 |
| PUMP_CMD_STOP_MIX | 停止指定液路的蠕动泵和电磁阀 |
| PUMP_SET_FREQ | 修改指定蠕动泵的运行频率 |
| PUMP_CMD_FLUSH | 控制冲洗液路执行冲洗或复位流程 |

泵阀联动遵循“先开阀、再开泵；先停泵、再关阀”的顺序，避免蠕动泵在阀门关闭时继续工作。

PUMP_CMD_START_MIX 命令的执行顺序如下：

1. 根据液路编号获取对应的泵阀设备；
2. 拉高电磁阀 GPIO，打开液路；
3. 延时 20 ms，等待电磁阀完成动作；
4. 拉高蠕动泵使能 GPIO；
5. 配置蠕动泵 PWM 频率；
6. 使能 PWM，启动蠕动泵。

PUMP_CMD_STOP_MIX 命令的执行顺序如下：

1. 禁用蠕动泵 PWM；
2. 拉低蠕动泵使能 GPIO；
3. 拉低电磁阀 GPIO，关闭液路；
4. 延时 20 ms，等待电磁阀完成动作。

PUMP_CMD_FLUSH 的底层执行顺序如下：

1. 关闭血液泵：
2. 关闭缓冲液泵：
3. 打开冲洗液电磁阀；
4. 延时 20 ms，等待冲洗液路导通；
5. 将冲洗泵正向控制 GPIO 拉高、反向控制 GPIO 拉低，启动直流泵；

对应的驱动逻辑可以概括为：

```c
pump_start(pump)
{
    valve_on(pump->valve);
    msleep(20);

    pump_enable_gpio = 1;
    pump_set_freq(pump->frequency);
    pwm_enable(pump->pwm);
}

pump_stop(pump)
{
    pwm_disable(pump->pwm);
    pump_enable_gpio = 0;

    valve_off(pump->valve);
    msleep(20);
}

cmd_flush()
{
    servo_pump_off(&pump_descs[0]);  // 关闭血液泵和阀
    servo_pump_off(&pump_descs[1]);  // 关闭缓冲液泵和阀
    dc_pump_on(&pump_descs[2]);      // 打开冲洗液阀并启动直流泵
}
```
字符设备驱动屏蔽了不同泵阀的底层控制差异，使图像处理、闭环控制和压力监测模块均通过统一接口操作液路设备，避免用户态业务代码直接依赖具体的硬件控制方式。

### 2.7 液路闭环控制

液路控制线程从 `dropletInfoQueue` 获取液滴大小、颜色均值和颜色方差，并按照“液滴大小门控—混合比例控制—混合均匀度控制”的顺序调节血液泵和缓冲液泵。

| 液滴信息 | 控制用途 |
|---|---|
| `size` | 判断液滴大小是否处于有效范围 |
| `mean` | 表征血液与缓冲液的混合比例 |
| `variance` | 表征液滴内部的混合均匀度 |

由于项目需要同时检测液滴的大小、混合均匀度和混合比例。因此设计了如下的控制架构：
  1. 使用液滴大小作为门控：如果液滴大小不合要求（太大或太小），返回并累计错误。如果出现连续的 10 帧液滴大小错误，直接退出。
  2. 混合比例环控制：使用带前馈的 PI 算法控制液滴混合比例。首先根据当前混合的目标配比查找标定表（标定的是配比 - 血液相流速的关系，保持缓冲液流速不变）。计算出血液相流速。随后调用底层 ioctl SET_FREQ 接口设置血液相蠕动泵频率，从而调控血液相流速。
  3. 使用 PI 算法持续微调血液相流速，直到视觉反馈中液滴的混合配比符合要求（持续 3 帧作为窗口）。
  4. 混合均匀度环控制：在比例环控制确认无误后，使用带前馈的 PI 算法控制液滴混合比例。首先根据当前的两相流速比查找标定表（标定的是在目标混合均匀度下，两相流速比 - 总流速的关系）。获取总流速，根据总流速计算出两相分别的流速（维持两相流速比不变）。随后调用底层 ioctl SET_FREQ 接口设置两相相蠕动泵频率，从而调控两相流速。
  5. 使用 PI 算法持续微调两相流速，直到视觉反馈中液滴的混合均匀度符合要求。
  6. 在混合均匀度环中，需要检查混合配比是否满足要求。如果混合配比不满足，会回滚到混合比例环。重新执行混合比例环的前馈逻辑。

控制状态机包含两个主要阶段：

| 控制阶段 | 前馈控制 | PI 微调 |
|---|---|---|
| `RATIO_STAGE` | 根据目标配比查询标定表，得到血液泵频率，并使用固定的缓冲液泵基础频率 | 根据 `mean` 与目标值的偏差，只调整血液泵频率 |
| `VAR_STAGE` | 根据目标方差查询标定表得到总频率，并按照当前两路泵频率比分配 | 根据 `variance` 与目标值的偏差调节总频率，同时保持两路泵频率比不变 |

比例环采用增量式 PI 控制：

```cpp
error = RATIO_SETPOINT - mean;

deltaFreq = RATIO_KP * (error - lastError) + RATIO_KI * error;

bloodFreq = clamp(currentBloodFreq + deltaFreq, FREQ_MIN, FREQ_MAX);
```

比例环固定缓冲液泵频率，只调整血液泵频率。当比例误差连续 3 次处于收敛范围内时，执行均匀度环前馈并切换到 `VAR_STAGE`。

均匀度环同样采用增量式 PI 控制：

```cpp
error = variance - VAR_SETPOINT;

deltaTotalFreq = VAR_KP * (error - lastError) + VAR_KI * error;

totalFreq = clamp(currentTotalFreq - deltaTotalFreq, FREQ_MIN * 2, FREQ_MAX * 2);
```

新的总频率按照比例环已经确定的两相频率比分配：

```cpp
bloodRatio = currentBloodFreq / currentTotalFreq;
bufferRatio = 1.0f - bloodRatio;

newBloodFreq = totalFreq * bloodRatio;
newBufferFreq = totalFreq * bufferRatio;
```

由于泵速调整到液滴进入视觉检测区域之间存在流体传输滞后，每次下发前馈或 PI 调节结果后，系统会暂时冻结闭环更新。

冻结期间的处理流程如下：

1. 记录当前两路泵的估算总流量和控制下发时刻；
2. 暂停使用新液滴数据更新 PI；
3. 根据总流量和经过时间累计控制指令下发后的流体通过量；
4. 累计流量达到 `CTRL_FREEZE_VOLUME` 后解除冻结；
5. 从下一颗有效液滴开始继续闭环调节。

该机制避免控制器使用仍由旧泵速产生的液滴反馈再次调节，从而减少流体滞后造成的重复修正和频率振荡。
### 2.8 压力监测与冲洗保护

压力传感器通过 ADS1115 采集液路压力信号，并利用硬件阈值比较器产生超压中断。驱动将中断转换为 IIO Event 上报，用户态压力监测线程通过事件文件描述符阻塞等待，不需要周期性轮询压力数据。

压力监测线程打开 `/dev/iio:device0`，获取 IIO Event FD，并通过 `poll()` 等待事件。收到事件后，只处理以下类型：

| 事件字段 | 有效值 |
|---|---|
| 通道类型 | `IIO_VOLTAGE` |
| 事件类型 | `IIO_EV_TYPE_THRESH` |
| 事件方向 | `IIO_EV_DIR_RISING` |
| 通道编号 | `0` |

超压保护流程如下：

1. ADS1115 检测到采样值超过硬件比较阈值；
2. 驱动在中断处理中上报 IIO 阈值事件；
3. 压力监测线程读取并校验事件类型；
4. 确认超压后设置 `resetFlag`；
5. 液路控制线程检测到复位标志并执行 `doReset()`；
6. 停止血液和缓冲液两路泵阀；
7. 打开冲洗液电磁阀并启动冲洗泵；
8. 清理当前闭环控制状态，等待后续重新启动。

为避免压力持续处于阈值以上时重复产生中断，驱动在上报一次超压事件后暂时关闭比较器，并在 10 秒后恢复阈值检测。该方式将压力异常检测和重复事件抑制放在驱动侧完成，同时使用户态线程在正常状态下保持阻塞等待。
### 2.9 系统故障处理

系统将异常分为可恢复业务异常和不可恢复系统故障。压力超限、液滴连续检测异常等情况通过复位流程处理；设备访问失败、图像采集异常或线程内部异常则触发统一故障退出。

| 异常类型 | 处理方式 |
|---|---|
| 液滴大小连续异常 | 执行控制复位，清除当前闭环状态 |
| 压力超过阈值 | 设置 `resetFlag`，停止正常液路并启动冲洗 |
| V4L2 采集失败 | 设置全局故障状态并退出程序 |
| RGA 转换失败 | 设置全局故障状态并退出程序 |
| 泵阀 `ioctl()` 失败 | 设置全局故障状态并退出程序 |
| IIO 事件读取失败 | 设置全局故障状态并退出程序 |
| 线程发生未处理异常 | 记录异常原因并触发统一退出 |

各业务线程使用 `try-catch` 捕获运行期间的不可恢复错误，并通过 `fatalError` 和 `fatalErrorMsg` 向主线程上报：

```cpp
try
{
    // 线程业务循环
}
catch (const std::exception& e)
{
    fatalErrorMsg = std::string("[模块名称] ") + e.what();
    fatalError.store(true);
}
```

统一故障处理流程如下：

1. 设置全局原子标志 `fatalError`；
2. 主线程检测到故障状态后停止各业务线程；
3. 唤醒仍阻塞在队列、`poll()` 或设备读取接口上的线程；
4. 停止 V4L2 主图像流和调试图像流；
5. 停止血液、缓冲液和冲洗液泵阀；
6. 等待各业务线程退出；
4. 释放资源后退出。

可恢复异常不会直接设置 `fatalError`。液滴连续检测异常或压力事件只设置 `resetFlag`，由液路控制线程清除比例环、均匀度环、PI 冻结状态和累计计数，并进入冲洗或等待重新启动状态。


## 3. 软件结构

