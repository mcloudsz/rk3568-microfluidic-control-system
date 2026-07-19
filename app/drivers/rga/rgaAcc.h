#pragma once
#include <cstddef>
#include <im2d.h> 
#include <rga.h>
#include <iostream>
#include "dmaHeap.h"

class rgaAcc{
    private:
        int _clipXPos;
        int _clipYPos;
        int _clipWidth;
        int _clipHeight;
    public:
        rgaAcc(int clipXPos, int clipYPos, int clipWidth, int clipHeight);
        ~rgaAcc();
        void rgaExec(dmaHeapBuffer& srcBuffer, dmaHeapBuffer& dstBuffer) const;
};
