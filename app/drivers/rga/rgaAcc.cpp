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
    if (dstBuffer.getWidth() != _clipWidth || dstBuffer.getHeight() != _clipHeight)
          throw std::invalid_argument("dstBuffer size must match clip region");
    
          if (_clipXPos + _clipWidth > srcBuffer.getWidth() ||
          _clipYPos + _clipHeight > srcBuffer.getHeight())
          throw std::invalid_argument("clip region out of src bounds");
          
    rga_buffer_handle_t srcHandle = srcBuffer.getBufferHandle();
    rga_buffer_handle_t dstHandle = dstBuffer.getBufferHandle();

    rga_buffer_t srcBuf = wrapbuffer_handle(srcHandle, srcBuffer.getWidth(), srcBuffer.getHeight(), srcBuffer.getFmt());
    rga_buffer_t dstBuf = wrapbuffer_handle(dstHandle, dstBuffer.getWidth(), dstBuffer.getHeight(), dstBuffer.getFmt());

    // 如果后续需要裁剪, 使用 imcrop 接口.
    // 目前的实现, 裁剪直接在 ISP subdev 中进行了, 因此使用 imcvtcolor 接口实现颜色转换.
    IM_STATUS ret = imcvtcolor(srcBuf, dstBuf, srcBuffer.getFmt(), dstBuffer.getFmt());
    if(ret != IM_STATUS_SUCCESS)
        throw std::runtime_error("rga exec failed, IM_STATUS is" + std::to_string(ret));
}


