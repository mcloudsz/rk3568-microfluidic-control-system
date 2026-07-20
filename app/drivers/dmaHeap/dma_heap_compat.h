#ifndef DMA_HEAP_COMPAT_H
#define DMA_HEAP_COMPAT_H

#include <stdint.h>
#include <sys/ioctl.h>

struct dma_heap_allocation_data {
    uint64_t len;        // input: allocation size
    uint32_t fd;         // output: allocated dma-buf fd
    uint32_t fd_flags;   // input: O_CLOEXEC / O_RDWR
    uint64_t heap_flags; // input: usually 0
};

#define DMA_HEAP_IOC_MAGIC 'H'

#define DMA_HEAP_IOCTL_ALLOC \
    _IOWR(DMA_HEAP_IOC_MAGIC, 0x0, struct dma_heap_allocation_data)

#endif
