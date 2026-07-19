#ifndef __RK_PUMP_H__
#define __RK_PUMP_H__

#include <linux/ioctl.h>

#define DEFAULT_PUMP_FREQ 1050

struct pump_status {
    bool is_running;
};

// 低层参数结构体：设置指定蠕动泵频率 
struct pump_freq {
    int pump_id; 
    int freq_hz;
};

#define PUMP_IOC_MAGIC 'P'

#define PUMP_CMD_START_MIX       _IO(PUMP_IOC_MAGIC, 1)  // 启动混合：开血液 + 缓冲液阀和泵 
#define PUMP_CMD_STOP_MIX        _IO(PUMP_IOC_MAGIC, 2)  // 停止混合：安全关闭血液 + 缓冲液泵和阀 
#define PUMP_CMD_FLUSH           _IO(PUMP_IOC_MAGIC, 3)  // 启动冲洗：停混合 + 开冲洗阀和泵 
#define PUMP_CMD_FLUSH_DONE      _IO(PUMP_IOC_MAGIC, 4)  // 冲洗完毕：关冲洗泵和阀 
#define PUMP_CMD_EMERGENCY_STOP  _IO(PUMP_IOC_MAGIC, 5)  // 紧急停止：立即关闭所有泵和阀 

#define PUMP_SET_FREQ            _IOW(PUMP_IOC_MAGIC, 6, struct pump_freq)    // 设置蠕动泵频率 
#define PUMP_GET_STATUS          _IOR(PUMP_IOC_MAGIC, 7, struct pump_status)  // 查询当前泵状态 

#endif /* __RK_PUMP_H__ */
