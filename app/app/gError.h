#pragma once
#include <iostream>

#include <atomic>
#include <string>

// 全局错误标志位
inline std::atomic<bool> fatalError{false};
inline std::string       fatalErrorMsg;

// 全局复位标志位(proc 进程/ctrl 进程)
inline std::atomic<bool> resetFlag{false};

// 背景图重新采集通知
inline std::atomic<bool> needReinit{false};   // 通知 frameProc 重置背景帧
