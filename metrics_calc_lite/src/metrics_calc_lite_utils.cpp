// Copyright (c) 2017 Intel Corporation
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "metrics_calc_lite_utils.h"

#ifndef NO_IPP
EErrorStatus stsIPPtoMCL(IppStatus sts)
{
    if (ippStsNoErr == sts)       return MCL_ERR_NONE;
    if (ippStsSizeErr == sts)     return MCL_ERR_INVALID_PARAM;
    if (ippStsNullPtrErr == sts)  return MCL_ERR_NULL_PTR;
    if (ippStsMemAllocErr == sts) return MCL_ERR_MEMORY_ALLOC;

    return MCL_ERR_UNKNOWN;
}
#endif

bool is_interlaced(ESequenceType st) {
    switch (st) {
        case I420I:
        case YV12I:
        case NV12I:
        case YUY2I:
        case NV16I:
        case I422I:
        case AYUVI:
        case Y410I:
        case I444I:
        case I410I:
        case RGB32I:
        case A2RGB10I:
            return true;
        default:
            return false;
    }
}

bool is_rgb(ESequenceType st) {
    switch (st) {
        case RGB32P:
        case RGB32I:
        case A2RGB10P:
        case A2RGB10I:
            return true;
        default:
            return false;
    }
}

EChromaType get_chromaclass(ESequenceType st) {
    switch (st) {
        case AYUVP:
        case AYUVI:
        case Y410P:
        case Y410I:
        case I444P:
        case I444I:
        case I410P:
        case I410I:
        case RGB32I:
        case RGB32P:
        case A2RGB10I:
        case A2RGB10P:
            return C444;
        case YUY2P:
        case YUY2I:
        case NV16P:
        case NV16I:
        case I422P:
        case I422I:
            return C422;
        default:
            return C420;
    }
}

uint64_t _file_fseek(FILE *fd, int64_t position, int32_t mode)
{
#if defined(WIN32) || defined(WIN64)
    return _fseeki64(fd, position, mode);
#elif defined(__APPLE__) || defined(LINUX64) || defined(ANDROID)
    return fseeko(fd, (off_t)position, mode);
#else
    return fseeko64(fd, (__off64_t)position, mode);
#endif
}

uint64_t _file_ftell(FILE *fd)
{
#if defined(WIN32) || defined(WIN64)
    return (uint64_t) _ftelli64(fd);
#elif defined(__APPLE__) || defined(LINUX64) || defined(ANDROID)
    return (uint64_t) ftello(fd);
#else
    return (uint64_t)ftello64(fd);
#endif
}

uint8_t* mclMalloc(uint32_t size, EBitDepth bd)
{
#ifdef NO_IPP
    if      (bd == D008) return (uint8_t*)new uint8_t[size];
    else if (bd == D010) return (uint8_t*)new uint16_t[size];
#else
    if      (bd == D008) return (uint8_t*)ippsMalloc_8u(size);
    else if (bd == D010) return (uint8_t*)ippsMalloc_16u(size);
#endif
    return NULL;
}

float* mclMalloc_32f_C1(int32_t widthPixels, int32_t heightPixels, int32_t* pStepBytes)
{
#ifdef NO_IPP
    *pStepBytes = widthPixels << 2;
    return new float[widthPixels * heightPixels];
#else
    return ippiMalloc_32f_C1(widthPixels, heightPixels, pStepBytes);
#endif
}

EErrorStatus mclYCbCr420ToYCrCb420_8u_P2P3R(const uint8_t* pSrcY, int32_t srcYStep, const uint8_t* pSrcUV, int32_t srcUVStep, uint8_t* PDst[3], int32_t dstStep[3], ImageSize roiSize)
{
    if (!pSrcY || !pSrcUV || !PDst || !dstStep)  return MCL_ERR_NULL_PTR;
    if (!PDst[0] || !PDst[1] || !PDst[2])        return MCL_ERR_NULL_PTR;
    if (roiSize.width < 2 || roiSize.height < 2) return MCL_ERR_INVALID_PARAM;

    int32_t h,w;
    uint8_t* pDst[3];
    int32_t width  = roiSize.width ;
    int32_t height = roiSize.height;
    pDst[0] = PDst[0]; pDst[2] = PDst[1]; pDst[1] = PDst[2];

    for( h = 0;h < height; h++ )
    {
        const uint8_t* srcy;
        uint8_t*  dst0;
        srcy = pSrcY;
        dst0 = pDst[0];
        for( w = 0; w < width; w++ )
        {
            dst0[0] = srcy[0];
            dst0++;srcy++;
        }
        pSrcY += srcYStep;pDst[0] += dstStep[0];
    }

    height>>=1;width>>=1;

    for( h = 0;h < height; h ++)
    {
        const uint8_t*  srcu = pSrcUV + h * srcUVStep;
        uint8_t*  dst2 = pDst[1] + h * dstStep[2];
        uint8_t*  dst1 = pDst[2] + h * dstStep[1];
        srcu = pSrcUV + h * srcUVStep;
        for( w = 0; w < width; w++ )
        {
            dst1[0] = srcu[0];
            dst2[0] = srcu[1];
            dst1++;dst2++;srcu+=2;
        }
    }

    return MCL_ERR_NONE;
}

EErrorStatus mclYCbCr420ToYCrCb420_16u_P2P3R(const uint16_t* pSrcY, int32_t srcYStep, const uint16_t* pSrcUV, int32_t srcUVStep, uint16_t* PDst[3], int32_t dstStep[3], ImageSize roiSize)
{
    if (!pSrcY || !pSrcUV || !PDst || !dstStep)  return MCL_ERR_NULL_PTR;
    if (!PDst[0] || !PDst[1] || !PDst[2])        return MCL_ERR_NULL_PTR;
    if (roiSize.width < 2 || roiSize.height < 2) return MCL_ERR_INVALID_PARAM;

    srcYStep   = srcYStep   >> 1;
    srcUVStep  = srcUVStep  >> 1;
    dstStep[0] = dstStep[0] >> 1;
    dstStep[1] = dstStep[1] >> 1;
    dstStep[2] = dstStep[2] >> 1;

    int32_t h,w;
    uint16_t* pDst[3];
    int32_t width  = roiSize.width ;
    int32_t height = roiSize.height;
    pDst[0] = PDst[0]; pDst[2] = PDst[1]; pDst[1] = PDst[2];

    for( h = 0;h < height; h++ )
    {
        const uint16_t* srcy;
        uint16_t*  dst0;
        srcy = pSrcY;
        dst0 = pDst[0];
        for( w = 0; w < width; w++ )
        {
            dst0[0] = srcy[0];
            dst0++;srcy++;
        }
        pSrcY += srcYStep;pDst[0] += dstStep[0];
    }

    height>>=1;width>>=1;

    for( h = 0;h < height; h ++)
    {
        const uint16_t*  srcu = pSrcUV + h * srcUVStep;
        uint16_t*  dst2 = pDst[1] + h * dstStep[2];
        uint16_t*  dst1 = pDst[2] + h * dstStep[1];
        srcu = pSrcUV + h * srcUVStep;
        for( w = 0; w < width; w++ )
        {
            dst1[0] = srcu[0];
            dst2[0] = srcu[1];
            dst1++;dst2++;srcu+=2;
        }
    }

    return MCL_ERR_NONE;
}

EErrorStatus mclYCbCr420ToYCrCb420_P2P3R(const uint8_t* pSrcY, int32_t srcYStep, const uint8_t* pSrcUV, int32_t srcUVStep, uint8_t* PDst[3], int32_t dstStep[3], ImageSize roiSize, EBitDepth bd)
{
#ifdef NO_IPP
    if      (bd == D008) mclYCbCr420ToYCrCb420_8u_P2P3R( (uint8_t*)pSrcY,  srcYStep, (uint8_t*)pSrcUV,  srcUVStep, (uint8_t**)PDst,  dstStep, roiSize);
#else
    if      (bd == D008) ippiYCbCr420ToYCrCb420_8u_P2P3R((uint8_t*)pSrcY,  srcYStep, (uint8_t*)pSrcUV,  srcUVStep, (uint8_t**)PDst,  dstStep, roiSize);
#endif
    else if (bd == D010) mclYCbCr420ToYCrCb420_16u_P2P3R((uint16_t*)pSrcY, srcYStep, (uint16_t*)pSrcUV, srcUVStep, (uint16_t**)PDst, dstStep, roiSize);
    return MCL_ERR_INVALID_PARAM;
}

EErrorStatus mclYCbCr422_8u_C2P3R(uint8_t* pSrc, int32_t srcStep, uint8_t* pDst[3], int32_t dstStep[3], ImageSize roiSize)
{
    if (!pSrc || !pDst || !dstStep)              return MCL_ERR_NULL_PTR;
    if (!pDst[0] || !pDst[1] || !pDst[2])        return MCL_ERR_NULL_PTR;
    if (roiSize.width < 2 || roiSize.height < 1) return MCL_ERR_INVALID_PARAM;

    int32_t w,h;
    uint8_t* pDstU = pDst[1];
    uint8_t* pDstV = pDst[2];
    int32_t dstStepU = dstStep[1];
    int32_t dstStepV = dstStep[2];
    int32_t width  = roiSize.width;
    int32_t height = roiSize.height;

    for( h = 0; h < height; h ++ )
    {
        const uint8_t* src;
        uint8_t* dsty;
        uint8_t* dstu;
        uint8_t* dstv;

        src = pSrc + h * srcStep;

        dsty = pDst[0] + h * dstStep[0];
        dstu = pDstU + h * dstStepU;
        dstv = pDstV + h * dstStepV;

        for( w = 0; w < width; w += 2 )
        {
            *dsty++ = *src++;
            *dstu++ = *src++;
            *dsty++ = *src++;
            *dstv++ = *src++;
        }
    }

    return MCL_ERR_NONE;
}

EErrorStatus mclYCbCr422_16u_C2P3R(uint16_t* pSrc, int32_t srcStep, uint16_t* pDst[3], int32_t dstStep[3], ImageSize roiSize)
{
    if (!pSrc || !pDst || !dstStep)              return MCL_ERR_NULL_PTR;
    if (!pDst[0] || !pDst[1] || !pDst[2])        return MCL_ERR_NULL_PTR;
    if (roiSize.width < 2 || roiSize.height < 1) return MCL_ERR_INVALID_PARAM;

    srcStep    = srcStep    >> 1;
    dstStep[0] = dstStep[0] >> 1;
    dstStep[1] = dstStep[1] >> 1;
    dstStep[2] = dstStep[2] >> 1;

    int32_t w,h;
    uint16_t* pDstU = pDst[1];
    uint16_t* pDstV = pDst[2];
    int32_t dstStepU = dstStep[1];
    int32_t dstStepV = dstStep[2];
    int32_t width  = roiSize.width;
    int32_t height = roiSize.height;

    for( h = 0; h < height; h ++ )
    {
        const uint16_t* src;
        uint16_t* dsty;
        uint16_t* dstu;
        uint16_t* dstv;

        src = pSrc + h * srcStep;

        dsty = pDst[0] + h * dstStep[0];
        dstu = pDstU + h * dstStepU;
        dstv = pDstV + h * dstStepV;

        for( w = 0; w < width; w += 2 )
        {
            *dsty++ = *src++;
            *dstu++ = *src++;
            *dsty++ = *src++;
            *dstv++ = *src++;
        }
    }

    return MCL_ERR_NONE;
}

EErrorStatus mclYCbCr422_C2P3R(uint8_t* pSrc, int32_t srcStep, uint8_t* pDst[3], int32_t dstStep[3], ImageSize roiSize, EBitDepth bd)
{
#ifdef NO_IPP
    if      (bd == D008) mclYCbCr422_8u_C2P3R( (uint8_t*)pSrc,  srcStep, (uint8_t**)pDst,  dstStep, roiSize);
#else
    if      (bd == D008) ippiYCbCr422_8u_C2P3R((uint8_t*)pSrc,  srcStep, (uint8_t**)pDst,  dstStep, roiSize);
#endif
    else if (bd == D010) mclYCbCr422_16u_C2P3R((uint16_t*)pSrc, srcStep, (uint16_t**)pDst, dstStep, roiSize);
    return MCL_ERR_INVALID_PARAM;
}

EErrorStatus mclA2RGB10ToRGB_10u16u_C4P4R(const uint8_t* pSrc, int32_t srcStep, uint16_t* const pDst[4], int32_t dstStep, ImageSize roiSize)
{
    if (!pSrc || !pDst )                              return MCL_ERR_NULL_PTR;
    if (!pDst[0] || !pDst[1] || !pDst[2] || !pDst[3]) return MCL_ERR_NULL_PTR;
    if (!srcStep || !dstStep)                         return MCL_ERR_INVALID_PARAM;
    if (roiSize.width < 2 || roiSize.height < 2)      return MCL_ERR_INVALID_PARAM;

    srcStep = srcStep >> 1;
    dstStep = dstStep >> 1;

    uint32_t A_mask = 0x0003 << 30,
             R_mask = 0x03ff << 20,
             G_mask = 0x03ff << 10,
             B_mask = 0x03ff;

    int32_t h,w;
    const uint32_t* src = (uint32_t*) pSrc;

    uint16_t* dstR = pDst[0];
    uint16_t* dstG = pDst[1];
    uint16_t* dstB = pDst[2];
    uint16_t* dstA = pDst[3];

    uint32_t A, R, G, B;

    for(h = 0; h < roiSize.height; h++ )
    {
        for(w = 0; w < roiSize.width; w++)
        {
            A = (uint32_t)(src[w] & A_mask) >> 30;
            R = (uint32_t)(src[w] & R_mask) >> 20;
            G = (uint32_t)(src[w] & G_mask) >> 10;
            B = (uint32_t)(src[w] & B_mask);
            dstA[w] = A;
            dstR[w] = R;
            dstG[w] = G;
            dstB[w] = B;
        }
        src+=srcStep;dstA+=dstStep;dstR+=dstStep;dstG+=dstStep;dstB+=dstStep;
    }

    return MCL_ERR_NONE;
}

EErrorStatus mclY410ToYUV_10u16u_C4P4R(const uint8_t* pSrc, int32_t srcStep, uint16_t* const pDst[4], int32_t dstStep, ImageSize roiSize)
{
    if (!pSrc || !pDst )                              return MCL_ERR_NULL_PTR;
    if (!pDst[0] || !pDst[1] || !pDst[2] || !pDst[3]) return MCL_ERR_NULL_PTR;
    if (!srcStep || !dstStep)                         return MCL_ERR_INVALID_PARAM;
    if (roiSize.width < 2 || roiSize.height < 2)      return MCL_ERR_INVALID_PARAM;

    srcStep = srcStep >> 1;
    dstStep = dstStep >> 1;

    uint32_t A_mask = 0x0003 << 30,
             V_mask = 0x03ff << 20,
             Y_mask = 0x03ff << 10,
             U_mask = 0x03ff;

    int32_t h,w;
    const uint32_t* src = (uint32_t*) pSrc;

    uint16_t* dstY = pDst[0];
    uint16_t* dstU = pDst[1];
    uint16_t* dstV = pDst[2];
    uint16_t* dstA = pDst[3];

    uint32_t A, Y, U, V;

    for(h = 0; h < roiSize.height; h++ )
    {
        for(w = 0; w < roiSize.width; w++)
        {
            A = (uint32_t)(src[w] & A_mask) >> 30;
            V = (uint32_t)(src[w] & V_mask) >> 20;
            Y = (uint32_t)(src[w] & Y_mask) >> 10;
            U = (uint32_t)(src[w] & U_mask);
            dstA[w] = A;
            dstV[w] = V;
            dstY[w] = Y;
            dstU[w] = U;
        }
        src+=srcStep;dstA+=dstStep;dstY+=dstStep;dstU+=dstStep;dstV+=dstStep;
    }

    return MCL_ERR_NONE;
}

EErrorStatus mclY410ToYUV_C4P4R(const uint8_t* pSrc, int32_t srcStep, uint8_t* const pDst[4], int32_t dstStep, ImageSize roiSize, EBitDepth bd)
{
    if (D010 == bd) return mclY410ToYUV_10u16u_C4P4R((uint8_t*)pSrc, srcStep, (uint16_t**)pDst, dstStep, roiSize);
    return MCL_ERR_INVALID_PARAM;
}

EErrorStatus mclA2RGB10ToRGB_C4P4R(const uint8_t* pSrc, int32_t srcStep, uint8_t* const pDst[4], int32_t dstStep, ImageSize roiSize, EBitDepth bd)
{
    if (D010 == bd) return mclA2RGB10ToRGB_10u16u_C4P4R((uint8_t*)pSrc, srcStep, (uint16_t**)pDst, dstStep, roiSize);
    return MCL_ERR_INVALID_PARAM;
}

EErrorStatus mclNV16ToYCbCr422_8u_P2P3R(const uint8_t *pSrcY, int32_t srcYStep, const uint8_t *pSrcUV, int32_t srcUVStep, uint8_t *pDst[3], int32_t dstStep[3], ImageSize roiSize)
{
    if (!pSrcY || !pSrcUV || !pDst || !dstStep)  return MCL_ERR_NULL_PTR;
    if (!pDst[0] || !pDst[1] || !pDst[2])        return MCL_ERR_NULL_PTR;
    if (roiSize.width < 2 || roiSize.height < 2) return MCL_ERR_INVALID_PARAM;

    int32_t h,w;
    int32_t width  = roiSize.width ;
    int32_t height = roiSize.height;

    for( h = 0;h < height; h++ )
    {
        const uint8_t *srcy;
        uint8_t * dst0;
        srcy = pSrcY   + h * srcYStep;
        dst0 = pDst[0] + h * dstStep[0];
        for( w = 0; w < width; w++ )
        {
            dst0[0] = srcy[0];
            dst0++;srcy++;
        }
    }

    width>>=1;

    for( h = 0;h < height; h ++)
    {
        const uint8_t * srcu = pSrcUV + h * srcUVStep;
        uint8_t * dst1 = pDst[1] + h * dstStep[1];
        uint8_t * dst2 = pDst[2] + h * dstStep[2];
        srcu = pSrcUV + h * srcUVStep;
        for( w = 0; w < width; w++ )
        {
            dst1[0] = srcu[0];
            dst2[0] = srcu[1];
            dst1++;dst2++;srcu+=2;
        }
    }

    return MCL_ERR_NONE;
}

EErrorStatus mclNV16ToYCbCr422_16u_P2P3R(const uint16_t *pSrcY, int32_t srcYStep, const uint16_t *pSrcUV, int32_t srcUVStep, uint16_t *pDst[3], int32_t dstStep[3], ImageSize roiSize)
{
    if (!pSrcY || !pSrcUV || !pDst || !dstStep)  return MCL_ERR_NULL_PTR;
    if (!pDst[0] || !pDst[1] || !pDst[2])        return MCL_ERR_NULL_PTR;
    if (roiSize.width < 2 || roiSize.height < 2) return MCL_ERR_INVALID_PARAM;

    srcYStep   = srcYStep   >> 1;
    srcUVStep  = srcUVStep  >> 1;
    dstStep[0] = dstStep[0] >> 1;
    dstStep[1] = dstStep[1] >> 1;
    dstStep[2] = dstStep[2] >> 1;

    int32_t h,w;
    int32_t width  = roiSize.width ;
    int32_t height = roiSize.height;

    for( h = 0;h < height; h++ )
    {
        const uint16_t *srcy;
        uint16_t * dst0;
        srcy = pSrcY;
        dst0 = pDst[0];
        for( w = 0; w < width; w++ )
        {
            dst0[0] = srcy[0];
            dst0++;srcy++;
        }
        pSrcY += srcYStep;pDst[0] += dstStep[0];
    }

    width>>=1;

    for( h = 0;h < height; h ++)
    {
        const uint16_t * srcu = pSrcUV + h * srcUVStep;
        uint16_t * dst1 = pDst[1] + h * dstStep[1];
        uint16_t * dst2 = pDst[2] + h * dstStep[2];
        srcu = pSrcUV + h * srcUVStep;
        for( w = 0; w < width; w++ )
        {
            dst1[0] = srcu[0];
            dst2[0] = srcu[1];
            dst1++;dst2++;srcu+=2;
        }
    }

    return MCL_ERR_NONE;
}

EErrorStatus mclNV16ToYCbCr422_P2P3R(const uint8_t *pSrcY, int32_t srcYStep, const uint8_t *pSrcUV, int32_t srcUVStep, uint8_t *pDst[3], int32_t dstStep[3], ImageSize roiSize, EBitDepth bd)
{
    if      (bd == D008) mclNV16ToYCbCr422_8u_P2P3R( (uint8_t*)pSrcY,  srcYStep, (uint8_t*)pSrcUV,  srcUVStep, (uint8_t**)pDst,  dstStep, roiSize);
    else if (bd == D010) mclNV16ToYCbCr422_16u_P2P3R((uint16_t*)pSrcY, srcYStep, (uint16_t*)pSrcUV, srcUVStep, (uint16_t**)pDst, dstStep, roiSize);
    return MCL_ERR_INVALID_PARAM;
}

EErrorStatus mclCopy_8u_C4P4R(const uint8_t* pSrc, int32_t srcStep, uint8_t* const pDst[4], int32_t dstStep, ImageSize roiSize)
{
    if (!pSrc || !pDst || !dstStep)                   return MCL_ERR_NULL_PTR;
    if (!pDst[0] || !pDst[1] || !pDst[2] || !pDst[3]) return MCL_ERR_NULL_PTR;
    if (roiSize.width < 1 || roiSize.height < 1)      return MCL_ERR_INVALID_PARAM;

    const uint8_t* src;
    uint8_t *dst0 = pDst[0],
            *dst1 = pDst[1],
            *dst2 = pDst[2],
            *dst3 = pDst[3];

    for(int32_t h = 0; h < roiSize.height; h++)
    {
        src = pSrc + h * srcStep;

        dst0 = pDst[0] + h *dstStep;
        dst1 = pDst[1] + h *dstStep;
        dst2 = pDst[2] + h *dstStep;
        dst3 = pDst[3] + h *dstStep;

        for(int32_t w = 0; w < roiSize.width; w++)
        {
            *dst0++ = *src++;
            *dst1++ = *src++;
            *dst2++ = *src++;
            *dst3++ = *src++;
        }
    }

    return MCL_ERR_NONE;
}

EErrorStatus mclCopy_16u_C4P4R(const uint16_t* pSrc, int32_t srcStep, uint16_t* const pDst[4], int32_t dstStep, ImageSize roiSize)
{
    if (!pSrc || !pDst)                               return MCL_ERR_NULL_PTR;
    if (!pDst[0] || !pDst[1] || !pDst[2] || !pDst[3]) return MCL_ERR_NULL_PTR;
    if (roiSize.width < 1 || roiSize.height < 1)      return MCL_ERR_INVALID_PARAM;

    srcStep = srcStep >> 1;
    dstStep = dstStep >> 1;

    const uint16_t* src;
    uint16_t *dst0 = pDst[0],
             *dst1 = pDst[1],
             *dst2 = pDst[2],
             *dst3 = pDst[3];

    for(int32_t h = 0; h < roiSize.height; h++)
    {
        src = pSrc + h * srcStep;

        dst0 = pDst[0] + h *dstStep;
        dst1 = pDst[1] + h *dstStep;
        dst2 = pDst[2] + h *dstStep;
        dst3 = pDst[3] + h *dstStep;

        for(int32_t w = 0; w < roiSize.width; w++)
        {
            *dst0++ = *src++;
            *dst1++ = *src++;
            *dst2++ = *src++;
            *dst3++ = *src++;
        }
    }

    return MCL_ERR_NONE;
}

EErrorStatus mclCopy_C4P4R(const uint8_t* pSrc, int32_t srcStep, uint8_t* const pDst[4], int32_t dstStep, ImageSize roiSize, EBitDepth bd)
{
#ifdef NO_IPP
    if      (D008 == bd) return mclCopy_8u_C4P4R( (uint8_t*)pSrc,  srcStep, (uint8_t**)pDst,  dstStep, roiSize);
    else if (D010 == bd) return mclCopy_16u_C4P4R((uint16_t*)pSrc, srcStep, (uint16_t**)pDst, dstStep, roiSize);
#else
    if      (D008 == bd) return stsIPPtoMCL(ippiCopy_8u_C4P4R( (uint8_t*)pSrc,  srcStep, (uint8_t**)pDst,  dstStep, roiSize));
    else if (D010 == bd) return stsIPPtoMCL(ippiCopy_16u_C4P4R((uint16_t*)pSrc, srcStep, (uint16_t**)pDst, dstStep, roiSize));
#endif
    return MCL_ERR_INVALID_PARAM;
}

EErrorStatus mclRShiftC_8u_C1IR(uint32_t value, uint8_t* pSrcDst, int32_t srcDstStep, ImageSize roiSize)
{
    if (!pSrcDst)                                return MCL_ERR_NULL_PTR;
    if (value >= 8)                              return MCL_ERR_INVALID_PARAM;
    if (roiSize.width < 1 || roiSize.height < 1) return MCL_ERR_INVALID_PARAM;

    uint8_t* srcDst;

    for(int32_t h = 0; h < roiSize.height; h++)
    {
        srcDst = pSrcDst + h * srcDstStep;

        for(int32_t w = 0; w < roiSize.width; w++)
        {
            srcDst[w] = srcDst[w] >> value;
        }
    }

    return MCL_ERR_NONE;
}

EErrorStatus mclRShiftC_16u_C1IR(uint32_t value, uint16_t* pSrcDst, int32_t srcDstStep, ImageSize roiSize)
{
    if (!pSrcDst)                                return MCL_ERR_NULL_PTR;
    if (value >= 16)                             return MCL_ERR_INVALID_PARAM;
    if (roiSize.width < 1 || roiSize.height < 1) return MCL_ERR_INVALID_PARAM;

    srcDstStep = srcDstStep >> 1;

    uint16_t* srcDst;

    for(int32_t h = 0; h < roiSize.height; h++)
    {
        srcDst = pSrcDst + h * srcDstStep;

        for(int32_t w = 0; w < roiSize.width; w++)
        {
            srcDst[w] = srcDst[w] >> value;
        }
    }

    return MCL_ERR_NONE;
}

EErrorStatus mclRShiftC_C1IR(uint32_t value, uint8_t* pSrcDst, int32_t srcDstStep, ImageSize roiSize, EBitDepth bd)
{
    if (0 == value) return MCL_ERR_NONE;
#ifdef NO_IPP
    if      (D008 == bd) return mclRShiftC_8u_C1IR (value, (uint8_t*) pSrcDst, srcDstStep,  roiSize);
    else if (D010 == bd) return mclRShiftC_16u_C1IR(value, (uint16_t*) pSrcDst, srcDstStep, roiSize);
#else
    if      (D008 == bd) return stsIPPtoMCL(ippiRShiftC_8u_C1IR (value, (uint8_t*) pSrcDst, srcDstStep,  roiSize));
    else if (D010 == bd) return stsIPPtoMCL(ippiRShiftC_16u_C1IR(value, (uint16_t*) pSrcDst, srcDstStep, roiSize));
#endif
    return MCL_ERR_INVALID_PARAM;
}

EErrorStatus mclNormDiff_L2_8u_C1R(const uint8_t* pSrc1, int32_t src1Step, const uint8_t* pSrc2, int32_t src2Step, ImageSize roiSize, double* pValue)
{
    if (!pSrc1 || !pSrc2 || !pValue)             return MCL_ERR_NULL_PTR;
    if (roiSize.width < 1 || roiSize.height < 1) return MCL_ERR_INVALID_PARAM;

    uint8_t *src1, *src2;

    *pValue = 0.0;

    for(int32_t h = 0; h < roiSize.height; h++)
    {
        src1 = (uint8_t*)pSrc1 + h * src1Step;
        src2 = (uint8_t*)pSrc2 + h * src2Step;

        for(int32_t w = 0; w < roiSize.width; w++)
        {
            *pValue += (src1[w] - src2[w]) * (src1[w] - src2[w]);
        }
    }

    *pValue = sqrt(*pValue);

    return MCL_ERR_NONE;
}

EErrorStatus mclNormDiff_L2_16u_C1R(const uint16_t* pSrc1, int32_t src1Step, const uint16_t* pSrc2, int32_t src2Step, ImageSize roiSize, double* pValue)
{
    if (!pSrc1 || !pSrc2 || !pValue)             return MCL_ERR_NULL_PTR;
    if (roiSize.width < 1 || roiSize.height < 1) return MCL_ERR_INVALID_PARAM;

    src1Step = src1Step >> 1;
    src2Step = src2Step >> 1;

    uint16_t *src1, *src2;

    *pValue = 0.0;

    for(int32_t h = 0; h < roiSize.height; h++)
    {
        src1 = (uint16_t*)pSrc1 + h * src1Step;
        src2 = (uint16_t*)pSrc2 + h * src2Step;

        for(int32_t w = 0; w < roiSize.width; w++)
        {
            *pValue += (src1[w] - src2[w]) * (src1[w] - src2[w]);
        }
    }

    *pValue = sqrt(*pValue);

    return MCL_ERR_NONE;
}

EErrorStatus mclNormDiff_L2_C1R(const uint8_t* pSrc1, int32_t src1Step, const uint8_t* pSrc2, int32_t src2Step, ImageSize roiSize, double& value, EBitDepth bd)
{
#ifdef NO_IPP
    if      (D008 == bd) return mclNormDiff_L2_8u_C1R( (uint8_t*) pSrc1, src1Step, (uint8_t*) pSrc2, src2Step, roiSize, &value);
    else if (D010 == bd) return mclNormDiff_L2_16u_C1R((uint16_t*)pSrc1, src1Step, (uint16_t*)pSrc2, src2Step, roiSize, &value);
#else
    if      (D008 == bd) return stsIPPtoMCL(ippiNormDiff_L2_8u_C1R( (uint8_t*) pSrc1, src1Step, (uint8_t*) pSrc2, src2Step, roiSize, &value));
    else if (D010 == bd) return stsIPPtoMCL(ippiNormDiff_L2_16u_C1R((uint16_t*)pSrc1, src1Step, (uint16_t*)pSrc2, src2Step, roiSize, &value));
#endif
    return MCL_ERR_INVALID_PARAM;
}

EErrorStatus mclConvert_8u32f_C1R(const uint8_t* pSrc, int32_t srcStep, float* pDst, int32_t dstStep, ImageSize roiSize)
{
    if (!pSrc || !pDst)                          return MCL_ERR_NULL_PTR;
    if (roiSize.width < 1 || roiSize.height < 1) return MCL_ERR_INVALID_PARAM;

    dstStep = dstStep >> 2;

    uint8_t *src;
    float   *dst;

    for(int32_t h = 0; h < roiSize.height; h++)
    {
        src = (uint8_t*)pSrc + h * srcStep;
        dst = (float*  )pDst + h * dstStep;

        for(int32_t w = 0; w < roiSize.width; w++)
        {
            dst[w] = (float)src[w];
        }
    }

    return MCL_ERR_NONE;
}

EErrorStatus mclConvert_16u32f_C1R(const uint16_t* pSrc, int32_t srcStep, float* pDst, int32_t dstStep, ImageSize roiSize)
{
    if (!pSrc || !pDst)                          return MCL_ERR_NULL_PTR;
    if (roiSize.width < 1 || roiSize.height < 1) return MCL_ERR_INVALID_PARAM;

    srcStep = srcStep >> 1;
    dstStep = dstStep >> 2;

    uint16_t *src;
    float    *dst;

    for(int32_t h = 0; h < roiSize.height; h++)
    {
        src = (uint16_t*)pSrc + h * srcStep;
        dst = (float*   )pDst + h * dstStep;

        for(int32_t w = 0; w < roiSize.width; w++)
        {
            dst[w] = (float)src[w];
        }
    }

    return MCL_ERR_NONE;
}

EErrorStatus mclConvert__u32f_C1R(const uint8_t* pSrc, int32_t srcStep, float* pDst, int32_t dstStep, ImageSize roiSize, EBitDepth bd)
{
#ifdef NO_IPP
    if      (D008 == bd) return mclConvert_8u32f_C1R( (uint8_t*) pSrc, srcStep, pDst, dstStep, roiSize);
    else if (D010 == bd) return mclConvert_16u32f_C1R((uint16_t*)pSrc, srcStep, pDst, dstStep, roiSize);
#else
    if      (D008 == bd) return stsIPPtoMCL(ippiConvert_8u32f_C1R( (uint8_t*) pSrc, srcStep, pDst, dstStep, roiSize));
    else if (D010 == bd) return stsIPPtoMCL(ippiConvert_16u32f_C1R((uint16_t*)pSrc, srcStep, pDst, dstStep, roiSize));
#endif
    return MCL_ERR_INVALID_PARAM;
}

EErrorStatus mclSqr_32f_C1R(const float* pSrc, int srcStep, float* pDst, int dstStep, ImageSize roiSize)
{
#ifdef NO_IPP
    if (!pSrc || !pDst)                          return MCL_ERR_NULL_PTR;
    if (roiSize.width < 1 || roiSize.height < 1) return MCL_ERR_INVALID_PARAM;

    srcStep = srcStep >> 2;
    dstStep = dstStep >> 2;

    float *src, *dst;

    for(int32_t h = 0; h < roiSize.height; h++)
    {
        src = (float*)pSrc + h * srcStep;
        dst = (float*)pDst + h * dstStep;

        for(int32_t w = 0; w < roiSize.width; w++)
        {
            dst[w] = src[w] * src[w];
        }
    }

    return MCL_ERR_NONE;
#else
    return stsIPPtoMCL(ippiSqr_32f_C1R(pSrc, srcStep, pDst, dstStep, roiSize));
#endif
}

EErrorStatus mclMul_32f_C1R(const float* pSrc1, int srcStep1, const float* pSrc2, int srcStep2, float* pDst, int dstStep, ImageSize roiSize)
{
#ifdef NO_IPP
    if (!pSrc1 || !pSrc2 || !pDst)               return MCL_ERR_NULL_PTR;
    if (roiSize.width < 1 || roiSize.height < 1) return MCL_ERR_INVALID_PARAM;

    srcStep1 = srcStep1 >> 2;
    srcStep2 = srcStep2 >> 2;
    dstStep  = dstStep  >> 2;

    float *src1, *src2, *dst;

    for(int32_t h = 0; h < roiSize.height; h++)
    {
        src1 = (float*)pSrc1 + h * srcStep1;
        src2 = (float*)pSrc2 + h * srcStep2;
        dst  = (float*)pDst  + h * dstStep;

        for(int32_t w = 0; w < roiSize.width; w++)
        {
            dst[w] = src1[w] * src2[w];
        }
    }

    return MCL_ERR_NONE;
#else
    return stsIPPtoMCL(ippiMul_32f_C1R(pSrc1, srcStep1, pSrc2, srcStep2, pDst, dstStep, roiSize));
#endif
}

EErrorStatus mclMean_32f_C1R(const float* pSrc, int srcStep, ImageSize roiSize, double& value)
{
#ifdef NO_IPP
    if (!pSrc)                                   return MCL_ERR_NULL_PTR;
    if (roiSize.width < 1 || roiSize.height < 1) return MCL_ERR_INVALID_PARAM;

    srcStep = srcStep >> 2;

    float *src;

    value = 0.0;

    for(int32_t h = 0; h < roiSize.height; h++)
    {
        src = (float*)pSrc + h * srcStep;

        for(int32_t w = 0; w < roiSize.width; w++)
        {
            value += src[w];
        }
    }

    value /= roiSize.width * roiSize.height;

    return MCL_ERR_NONE;
#else
    return stsIPPtoMCL(ippiMean_32f_C1R(pSrc, srcStep, roiSize, &value, ippAlgHintAccurate));
#endif
}

EErrorStatus mclFilterRow_32f_C1R(const float* pSrc, int32_t srcStep, float* pDst, int32_t dstStep, ImageSize dstRoiSize, const float* pKernel, int32_t kernelSize, int32_t xAnchor)
{
#ifdef NO_IPP
    if (!pSrc || !pDst || !pKernel)                    return MCL_ERR_NULL_PTR;
    if (dstRoiSize.width < 1 || dstRoiSize.height < 1) return MCL_ERR_INVALID_PARAM;
    if (kernelSize < 1 || !(kernelSize&0x1))           return MCL_ERR_INVALID_PARAM;

    srcStep = srcStep >> 2;
    dstStep = dstStep >> 2;

    float *src, *dst;
    double value;

    for(int32_t h = 0; h < dstRoiSize.height; h++)
    {
        src = (float*)pSrc + h * srcStep;
        dst = (float*)pDst + h * dstStep;

        for(int32_t w = 0; w < dstRoiSize.width; w++)
        {
            value = 0.0f;

            for(int32_t i = 0; i < kernelSize; i++)
            {
                value += (double)pKernel[i] * (double)src[w - (kernelSize >> 1) + i];
            }

            dst[w] = (float)value;
        }
    }

    return MCL_ERR_NONE;
#else
    return stsIPPtoMCL(ippiFilterRow_32f_C1R(pSrc, srcStep, pDst, dstStep, dstRoiSize, pKernel, kernelSize, xAnchor));
#endif
}

EErrorStatus mclFilterColumn_32f_C1R(const float* pSrc, int32_t srcStep, float* pDst, int32_t dstStep, ImageSize dstRoiSize, const float* pKernel, int32_t kernelSize, int32_t xAnchor)
{
#ifdef NO_IPP
    if (!pSrc || !pDst || !pKernel)                    return MCL_ERR_NULL_PTR;
    if (dstRoiSize.width < 1 || dstRoiSize.height < 1) return MCL_ERR_INVALID_PARAM;
    if (kernelSize < 1 || !(kernelSize&0x1))           return MCL_ERR_INVALID_PARAM;

    srcStep = srcStep >> 2;
    dstStep = dstStep >> 2;

    float *src, *dst;
    double value;

    for(int32_t h = 0; h < dstRoiSize.height; h++)
    {
        src = (float*)pSrc + h * srcStep;
        dst = (float*)pDst + h * dstStep;

        for(int32_t w = 0; w < dstRoiSize.width; w++)
        {
            value = 0.0f;

            for(int32_t i = 0; i < kernelSize; i++)
            {
                value += (double)pKernel[i] * (double)src[w - (kernelSize >> 1) * srcStep + i * srcStep];
            }

            dst[w] = (float)value;
        }
    }

    return MCL_ERR_NONE;
#else
    return stsIPPtoMCL(ippiFilterColumn_32f_C1R(pSrc, srcStep, pDst, dstStep, dstRoiSize, pKernel, kernelSize, xAnchor));
#endif
}
