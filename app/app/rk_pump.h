#pragma once

#ifndef __RK_PUMP_H__
#define __RK_PUMP_H__

#include <linux/ioctl.h>

#define DEFAULT_PUMP_FREQ 1050

struct pump_status {
    bool is_running;
};

struct pump_freq {
    int pump_id;   
    int freq_hz;   
};

#define PUMP_IOC_MAGIC 'P'

#define PUMP_CMD_START_MIX       _IO(PUMP_IOC_MAGIC, 1)  
#define PUMP_CMD_STOP_MIX        _IO(PUMP_IOC_MAGIC, 2)  
#define PUMP_CMD_FLUSH           _IO(PUMP_IOC_MAGIC, 3)  
#define PUMP_CMD_FLUSH_DONE      _IO(PUMP_IOC_MAGIC, 4)  
#define PUMP_CMD_EMERGENCY_STOP  _IO(PUMP_IOC_MAGIC, 5)  

#define PUMP_SET_FREQ            _IOW(PUMP_IOC_MAGIC, 6, struct pump_freq)   
#define PUMP_GET_STATUS          _IOR(PUMP_IOC_MAGIC, 7, struct pump_status) 

#endif /* __RK_PUMP_H__ */
