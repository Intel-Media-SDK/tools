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

#ifndef __METRICS_CALC_LITE_UTILS_H__
#define __METRICS_CALC_LITE_UTILS_H__

#include <cctype>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <float.h>
#include <map>
#include <string>
#include <iostream>
#include <iomanip>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <memory>
#include <vector>
#include <algorithm>

//#define NO_IPP

#ifndef NO_IPP
#include "ipp.h"
#endif

typedef enum {
    MCL_ERR_NONE          =  0,
    MCL_ERR_UNKNOWN       = -1,
    MCL_ERR_NULL_PTR      = -2,
    MCL_ERR_INVALID_PARAM = -3,
    MCL_ERR_MEMORY_ALLOC  = -4
} EErrorStatus;

typedef enum { UNKNOWN,
               I420P, I420I, YV12P, YV12I, NV12P, NV12I,               // 4:2:0
               YUY2P, YUY2I, NV16P, NV16I, I422P, I422I,               // 4:2:2
               AYUVP, AYUVI, Y410P, Y410I, I444P, I444I, I410P, I410I, // 4:4:4
               RGB32I, RGB32P, A2RGB10I, A2RGB10P        // R:G:B
} ESequenceType;

typedef enum { C420, C422, C444 } EChromaType;
typedef enum { D008, D010       } EBitDepth;

#ifdef NO_IPP
typedef struct {
    int32_t width;
    int32_t height;
} ImageSize;
#else
typedef IppiSize ImageSize;
#endif

#ifdef NO_IPP
typedef struct {
    int32_t x;
    int32_t y;
} ImagePoint;
#else
typedef IppiPoint ImagePoint;
#endif

typedef struct {
    uint8_t  *data;
    uint32_t  step;
    ImageSize roi;
} SImage;

bool is_interlaced(ESequenceType st);
bool is_rgb(ESequenceType st);
EChromaType get_chromaclass(ESequenceType st);

/* File operations with portability issues */
uint64_t _file_fseek(FILE *fd, int64_t position, int32_t mode);
uint64_t _file_ftell(FILE *fd);

/* Memory allocation/deletion */
uint8_t* mclMalloc(uint32_t size, EBitDepth bd);
float* mclMalloc_32f_C1(int32_t widthPixels, int32_t heightPixels, int32_t* pStepBytes);
#ifdef NO_IPP
#define mclFree(ptr) if (ptr) delete[] ptr; ptr = NULL;
#else
#define mclFree(ptr) ippsFree(ptr); ptr = NULL;
#endif

/* Packed to planar formats conversions */
EErrorStatus mclYCbCr420ToYCrCb420_P2P3R(const uint8_t* pSrcY, int32_t srcYStep, const uint8_t* pSrcUV, int32_t srcUVStep, uint8_t* PDst[3], int32_t dstStep[3], ImageSize roiSize, EBitDepth bd);
EErrorStatus mclYCbCr422_C2P3R(uint8_t* pSrc, int32_t srcStep, uint8_t* pDst[3], int32_t dstStep[3], ImageSize roiSize, EBitDepth bd);
EErrorStatus mclA2RGB10ToRGB_C4P4R(const uint8_t* pSrc, int32_t srcStep, uint8_t* const pDst[4], int32_t dstStep, ImageSize roiSize, EBitDepth bd);
EErrorStatus mclY410ToYUV_C4P4R(const uint8_t* pSrc, int32_t srcStep, uint8_t* const pDst[4], int32_t dstStep, ImageSize roiSize, EBitDepth bd);
EErrorStatus mclNV16ToYCbCr422_P2P3R(const uint8_t *pSrcY, int32_t srcYStep, const uint8_t *pSrcUV, int32_t srcUVStep, uint8_t *pDst[3], int32_t dstStep[3], ImageSize roiSize, EBitDepth bd);
EErrorStatus mclCopy_C4P4R(const uint8_t* pSrc, int32_t srcStep, uint8_t* const pDst[4], int32_t dstStep, ImageSize roiSize, EBitDepth bd);

EErrorStatus mclRShiftC_C1IR(uint32_t value, uint8_t* pSrcDst, int32_t srcDstStep, ImageSize roiSize, EBitDepth bd);

/* PSNR */
EErrorStatus mclNormDiff_L2_C1R(const uint8_t* pSrc1, int32_t src1Step, const uint8_t* pSrc2, int32_t src2Step, ImageSize roiSize, double& pValue, EBitDepth bd);

/* SSIM */
EErrorStatus mclConvert__u32f_C1R(const uint8_t* pSrc, int32_t srcStep, float* pDst, int32_t dstStep, ImageSize roiSize, EBitDepth bd);
EErrorStatus mclSqr_32f_C1R(const float* pSrc, int32_t srcStep, float* pDst, int32_t dstStep, ImageSize roiSize);
EErrorStatus mclMul_32f_C1R(const float* pSrc1, int32_t srcStep1, const float* pSrc2, int32_t srcStep2, float* pDst, int32_t dstStep, ImageSize roiSize);
EErrorStatus mclMean_32f_C1R(const float* pSrc, int32_t srcStep, ImageSize roiSize, double& value);
EErrorStatus mclFilterRow_32f_C1R(const float* pSrc, int32_t srcStep, float* pDst, int32_t dstStep, ImageSize dstRoiSize, const float* pKernel, int32_t kernelSize, int32_t xAnchor);
EErrorStatus mclFilterColumn_32f_C1R(const float* pSrc, int32_t srcStep, float* pDst, int32_t dstStep, ImageSize dstRoiSize, const float* pKernel, int32_t kernelSize, int32_t xAnchor);

#endif // __METRICS_CALC_LITE_UTILS_H__