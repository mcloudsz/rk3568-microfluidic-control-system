#include "rgaAcc.h"


rgaAcc::rgaAcc(int clipXPos, int clipYPos, int clipWidth, int clipHeight)
{
    _clipXPos = clipXPos;
    _clipYPos = clipYPos;
    _clipWidth = clipWidth;
    _clipHeight = clipHeight;
}
rgaAcc::~rgaAcc()
{

}

void rgaAcc::rgaExec(dmaHeapBuffer& srcBuffer, dmaHeapBuffer& dstBuffer) const
{
    if (srcBuffer.getWidth() != dstBuffer.getWidth() || srcBuffer.getHeight() != dstBuffer.getHeight())
        throw std::invalid_argument("src and dst size must be same in current RKISP ROI mode");

    if (dstBuffer.getWidth() != _clipWidth || dstBuffer.getHeight() != _clipHeight) 
        throw std::invalid_argument("dstBuffer size must match output region");

    rga_buffer_handle_t srcHandle = srcBuffer.getBufferHandle();
    rga_buffer_handle_t dstHandle = dstBuffer.getBufferHandle();

    rga_buffer_t srcBuf = wrapbuffer_handle(srcHandle, srcBuffer.getWidth(),srcBuffer.getHeight(),srcBuffer.getFmt());

    rga_buffer_t dstBuf = wrapbuffer_handle(dstHandle, dstBuffer.getWidth(), dstBuffer.getHeight(),dstBuffer.getFmt());

    im_rect srcRect;
    srcRect.x = 0;
    srcRect.y = 0;
    srcRect.width = srcBuffer.getWidth();
    srcRect.height = srcBuffer.getHeight();

    IM_STATUS ret = imcrop(srcBuf, dstBuf, srcRect);

    if (ret != IM_STATUS_SUCCESS) 
        throw std::runtime_error("rga exec failed, IM_STATUS is " + std::to_string(ret));
}


