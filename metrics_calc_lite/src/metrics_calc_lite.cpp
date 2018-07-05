// Copyright (c) 2017-2018 Intel Corporation
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

#if defined(_OPENMP)
#include <omp.h>
#endif

#include "metrics_calc_lite_utils.h"

#define MASK_PSNR      (1 << 0)
#define MASK_APSNR     (1 << 1)
#define MASK_MSE       (1 << 2)
#define MASK_SSIM      (1 << 3)

#if !defined(NO_IPP)
#define MASK_ARTIFACTS (1 << 4)
#define MASK_MWDVQM    (1 << 5)
#define MASK_UQI       (1 << 6)
#define MASK_MSSIM     (1 << 7)
#endif

#define INIT_YUV(a)        (a).push_back(std::make_pair('Y', 0)); \
                           (a).push_back(std::make_pair('U', 0)); \
                           (a).push_back(std::make_pair('V', 0)); \
                           (a).push_back(std::make_pair('O', 0));

#define INIT_RGB(a, alpha) (a).push_back(std::make_pair('B', 0)); \
                           (a).push_back(std::make_pair('G', 0)); \
                           (a).push_back(std::make_pair('R', 0)); \
                 if(alpha) (a).push_back(std::make_pair('A', 0)); \
                           (a).push_back(std::make_pair('O', 0));

const double MAX_PSNR = 1000.0;

double MaxError(EBitDepth bd) {
    switch (bd)
    {
    case D010:
        return 1023.0;
    case D012:
        return 4095.0;
    case D016:
        return 65535.0;
    default:
        return 255.0;
    }
}

double MSEToPSNR ( double p_mse, double MaxErr )
{
    if ( p_mse < 0 )    return -1.0;
    if ( p_mse == 0.0 ) return MAX_PSNR;

    return (std::min) ( 10.0 * log10 ( MaxErr * MaxErr / p_mse ), MAX_PSNR );
}

typedef std::vector < std::pair <char, uint32_t> > Component;

class CReader {
protected:
    FILE         *m_file;
    uint32_t      m_num_fields;
    int32_t       m_cur_frame;
    bool          m_intl;
    ESequenceType m_type;
    int32_t       m_field_order;
    int32_t       m_bottom;
    EBitDepth     m_bd;
    uint32_t      m_RShift;
    uint32_t      m_source_pixel_size;

public:
    CReader() :
        m_file(0),
        m_num_fields(0),
        m_cur_frame(-1),
        m_intl(0),
        m_type(UNKNOWN),
        m_field_order(0),
        m_bottom(0),
        m_bd(D008),
        m_RShift(0),
        m_source_pixel_size(0)
    {};

    virtual ~CReader() {
        if (m_file) { fclose(m_file); m_file = 0; }
    };

    int32_t       GetFramesCount(void) const   { return m_num_fields; };
    bool          GetInterlaced(void) const    { return m_intl; };
    EBitDepth     GetBitDepth() const          { return m_bd; };
    ESequenceType GetSqType(void) const        { return m_type; };

    virtual EErrorStatus OpenReadFile(std::string name, uint32_t w, uint32_t h, ESequenceType type, int32_t order, EBitDepth bd, uint32_t RShift) = 0;
    virtual bool ReadRawFrame(uint32_t field) = 0;
    virtual void GetFrame(int32_t idx, void *dst) = 0;
};

class CRGBReader : public CReader {
private:
    SImage m_Meta;
    SImage m_planes[4]; /* BRGA order */

public:
    CRGBReader() {
        memset(&m_Meta,   0, sizeof(m_Meta));
        memset(&m_planes, 0, sizeof(m_planes));
    };

    virtual ~CRGBReader() {
        if (m_Meta.data) {
            mclFree(m_Meta.data);
        }
        if (m_planes[0].data) {
            mclFree(m_planes[0].data);
        }
    };

    EErrorStatus OpenReadFile(std::string name, uint32_t w, uint32_t h, ESequenceType type, int32_t order, EBitDepth bd, uint32_t RShift) {
        m_type = type; m_field_order = order; m_bd = bd; m_RShift = RShift;
        if(m_file) {
            mclFree(m_Meta.data); fclose(m_file);
        }
        if(NULL != (m_file = fopen(name.c_str(),"rb"))) {
            m_source_pixel_size = ( bd == D008 || m_type == A2RGB10P || m_type == A2RGB10I ) ? 1 : 2;
            m_Meta.step         = w * h * 4;
            m_Meta.roi.width    = w;
            m_Meta.roi.height   = h;
            m_Meta.data         = mclMalloc(m_Meta.step, bd);
            if (!m_Meta.data)  return MCL_ERR_MEMORY_ALLOC;

            _file_fseek(m_file, 0, SEEK_END);
            m_num_fields = (uint32_t)(_file_ftell(m_file) / (m_Meta.step*m_source_pixel_size));
            _file_fseek(m_file, 0, SEEK_SET);

            if(0 != (m_intl = is_interlaced(type))) m_num_fields <<= 1;

            m_planes[0].data = mclMalloc(m_Meta.step, bd);
            if (!m_planes[0].data) return MCL_ERR_MEMORY_ALLOC;

            m_planes[0].roi.width  = m_planes[1].roi.width   = m_planes[2].roi.width  = m_planes[3].roi.width  = w;
            m_planes[0].roi.height = m_planes[1].roi.height  = m_planes[2].roi.height = m_planes[3].roi.height = h;
            m_planes[0].step       = m_planes[1].step        = m_planes[2].step       = m_planes[3].step       = w * ((m_type == ARGB16P || m_type == A2RGB10P || m_type == A2RGB10I) ? 2 : m_source_pixel_size);

            m_planes[1].data = m_planes[0].data + m_planes[0].roi.height*m_planes[0].step;
            m_planes[2].data = m_planes[1].data + m_planes[1].roi.height*m_planes[1].step;
            m_planes[3].data = m_planes[2].data + m_planes[2].roi.height*m_planes[2].step;
            return MCL_ERR_NONE;
        } else {
            return MCL_ERR_INVALID_PARAM;
        }
    };

    bool ReadRawFrame(uint32_t field) {
        uint8_t  *planes[4];
        ImageSize roi = m_planes[0].roi;

        for(int32_t i=0; i<4; i++) { planes[i] = m_planes[i].data; }
        if(m_intl) { m_bottom = (m_field_order)^(field&0x1); field >>= 1; }
        if(m_cur_frame != (int32_t)field) {
            _file_fseek(m_file, ((uint64_t)field)*(m_Meta.step*m_source_pixel_size), SEEK_SET);
            size_t res = fread( m_Meta.data, m_source_pixel_size, m_Meta.step, m_file );
            switch (m_type) {
                case RGB32P:
                case RGB32I:
                case ARGB16P:
                    mclCopy_C4P4R(m_Meta.data, m_planes[0].step<<2, planes, m_planes[0].step, roi, m_bd);
                    break;
                case A2RGB10P:
                case A2RGB10I:
                    planes[0] = m_planes[2].data; planes[2] = m_planes[0].data;
                    mclA2RGB10ToRGB_C4P4R(m_Meta.data, m_planes[0].step, planes, m_planes[0].step, roi, m_bd);
                    break;
                default:
                    break;
            }
            mclRShiftC_C1IR(m_RShift, m_planes[0].data, m_planes[0].step, m_planes[0].roi, m_bd);
            mclRShiftC_C1IR(m_RShift, m_planes[1].data, m_planes[1].step, m_planes[1].roi, m_bd);
            mclRShiftC_C1IR(m_RShift, m_planes[2].data, m_planes[2].step, m_planes[2].roi, m_bd);
            mclRShiftC_C1IR(m_RShift, m_planes[3].data, m_planes[3].step, m_planes[3].roi, m_bd);
            m_cur_frame = (int32_t)field;
            return (res != m_Meta.step);
        } else {
            return false;
        }
    };

    void GetFrame(int32_t idx, void *dst) {
        memcpy((uint8_t*)dst, (uint8_t*)(m_planes+idx), sizeof(SImage));
        SImage *destination = (SImage*) dst;
        if(m_intl) {
            if(m_bottom) destination->data += destination->step;
            destination->step <<= 1; destination->roi.height >>= 1;
        }
    };
};

class CYUVReader : public CReader {
private:
    SImage     m_Meta;
    SImage     m_planes[4]; /* YUV order */

public:
    CYUVReader() {
        memset(&m_Meta, 0, sizeof(m_Meta));
        memset(&m_planes, 0, sizeof(m_planes));
    };

    virtual ~CYUVReader() {
        if (m_Meta.data == m_planes[0].data) {
            if (m_Meta.data) {
                mclFree(m_Meta.data);
                m_planes[0].data = 0;
            }
        } else {
            if (m_Meta.data) {
                mclFree(m_Meta.data);
            }
            if (m_planes[0].data) {
                mclFree(m_planes[0].data);
            }
        }
    };

    EErrorStatus OpenReadFile(std::string name, uint32_t w, uint32_t h, ESequenceType type, int32_t order, EBitDepth bd, uint32_t RShift) {
        m_type = type; m_field_order = order; m_bd = bd; m_RShift = RShift;
        if(m_file) {
            mclFree(m_Meta.data); fclose(m_file);
            if(m_planes[0].data!=m_Meta.data) mclFree(m_planes[0].data);
        }
        if(NULL != (m_file = fopen(name.c_str(),"rb"))) {
            m_source_pixel_size = ( bd == D008 || m_type == Y410P || m_type == Y410I ) ? 1 : 2;
            if(get_chromaclass(type) == C420) {
                m_Meta.step        = w * h * 3 / 2;
                m_Meta.data        = mclMalloc(m_Meta.step, bd);
                m_Meta.roi.width   = 1;
                m_Meta.roi.height  = 1;
                if (!m_Meta.data) return MCL_ERR_MEMORY_ALLOC;

                _file_fseek(m_file, 0, SEEK_END);
                m_num_fields = (uint32_t)(_file_ftell(m_file) / (m_Meta.step*m_source_pixel_size));
                _file_fseek(m_file, 0, SEEK_SET);

                if(0 != (m_intl = is_interlaced(type))) m_num_fields <<= 1;

                m_planes[0].data        = m_Meta.data;
                m_planes[0].roi.width   = w;
                m_planes[0].roi.height  = h;
                m_planes[0].step        = w * m_source_pixel_size;

                m_planes[1].roi.width   = m_planes[2].roi.width  = m_planes[0].roi.width >> 1;
                m_planes[1].roi.height  = m_planes[2].roi.height = m_planes[0].roi.height >> 1;
                m_planes[1].step        = m_planes[2].step       = m_planes[0].step >> 1;

                switch(type) {
                    case YV12P:
                    case YV12I:
                        m_planes[2].data = m_planes[0].data + m_planes[0].roi.height*m_planes[0].step;
                        m_planes[1].data = m_planes[2].data + m_planes[2].roi.height*m_planes[2].step;
                        break;
                    case I420P:
                    case I420I:
                        m_planes[1].data = m_planes[0].data + m_planes[0].roi.height*m_planes[0].step;
                        m_planes[2].data = m_planes[1].data + m_planes[1].roi.height*m_planes[1].step;
                        break;
                    case NV12P:
                    case NV12I:
                        m_planes[0].data = mclMalloc(m_Meta.step, bd);
                        if (!m_planes[0].data) return MCL_ERR_MEMORY_ALLOC;
                        m_planes[2].data = m_planes[0].data + m_planes[0].roi.height*m_planes[0].step;
                        m_planes[1].data = m_planes[2].data + m_planes[2].roi.height*m_planes[2].step;
                        break;
                }
            } else if(get_chromaclass(type) == C422) {
                m_Meta.step        = w * h * 2;
                m_Meta.data        = mclMalloc(m_Meta.step, bd);
                m_Meta.roi.width   = 1;
                m_Meta.roi.height  = 1;
                if (!m_Meta.data) return MCL_ERR_MEMORY_ALLOC;

                _file_fseek(m_file, 0, SEEK_END);
                m_num_fields = (uint32_t)(_file_ftell(m_file) / (m_Meta.step*m_source_pixel_size));
                _file_fseek(m_file, 0, SEEK_SET);

                if(0 != (m_intl = is_interlaced(type))) m_num_fields <<= 1;

                m_planes[0].data        = m_Meta.data;
                m_planes[0].roi.width   = w;
                m_planes[0].roi.height  = h;
                m_planes[0].step        = w * m_source_pixel_size;

                m_planes[1].roi.width   = m_planes[2].roi.width  = m_planes[0].roi.width >> 1;
                m_planes[1].roi.height  = m_planes[2].roi.height = m_planes[0].roi.height;
                m_planes[1].step        = m_planes[2].step       = m_planes[0].step >> 1;

                switch(type) {
                    case I422P:
                    case I422I:
                        m_planes[1].data = m_planes[0].data + m_planes[0].roi.height*m_planes[0].step;
                        m_planes[2].data = m_planes[1].data + m_planes[1].roi.height*m_planes[1].step;
                        break;
                    case YUY2P:
                    case YUY2I:
                    case NV16P:
                    case NV16I:
                        m_planes[0].data = mclMalloc(m_Meta.step, bd);
                        if (!m_planes[0].data) return MCL_ERR_MEMORY_ALLOC;
                        m_planes[1].data = m_planes[0].data + m_planes[0].roi.height*m_planes[0].step;
                        m_planes[2].data = m_planes[1].data + m_planes[1].roi.height*m_planes[1].step;
                        break;
                }
            } else if(get_chromaclass(type) == C444) {
                if (   type == I444P || type == I444I
                    || type == I410P || type == I410I)
                    m_Meta.step        = w * h * 3;
                else
                    m_Meta.step        = w * h * 4;
                m_Meta.data        = mclMalloc(m_Meta.step, bd);
                m_Meta.roi.width   = 1;
                m_Meta.roi.height  = 1;
                if (!m_Meta.data) return MCL_ERR_MEMORY_ALLOC;

                _file_fseek(m_file, 0, SEEK_END);
                m_num_fields = (uint32_t)(_file_ftell(m_file) / (m_Meta.step*m_source_pixel_size));
                _file_fseek(m_file, 0, SEEK_SET);

                if(0 != (m_intl = is_interlaced(type))) m_num_fields <<= 1;

                m_planes[0].data        = m_Meta.data;
                m_planes[0].roi.width   = m_planes[1].roi.width   = m_planes[2].roi.width  = m_planes[3].roi.width  = w;
                m_planes[0].roi.height  = m_planes[1].roi.height  = m_planes[2].roi.height = m_planes[3].roi.height = h;
                m_planes[0].step        = m_planes[1].step        = m_planes[2].step       = m_planes[3].step       = w * ((m_type == Y410P || m_type == Y410I) ? 2 : m_source_pixel_size);

                switch(type) {
                    case AYUVP:
                    case AYUVI:
                    case Y416P:
                    case Y416I:
                        m_planes[0].data = mclMalloc(m_Meta.step, bd);
                        if (!m_planes[0].data) return MCL_ERR_MEMORY_ALLOC;
                        m_planes[1].data = m_planes[0].data + m_planes[0].roi.height*m_planes[0].step;
                        m_planes[2].data = m_planes[1].data + m_planes[1].roi.height*m_planes[1].step;
                        m_planes[3].data = m_planes[2].data + m_planes[2].roi.height*m_planes[2].step;
                        break;
                    case I444P:
                    case I444I:
                    case I410P:
                    case I410I:
                        m_planes[1].data = m_planes[0].data + m_planes[0].roi.height*m_planes[0].step;
                        m_planes[2].data = m_planes[1].data + m_planes[1].roi.height*m_planes[1].step;
                        m_planes[3].roi.width  = 0;
                        m_planes[3].roi.height = 0;
                        m_planes[3].step = 0;
                        break;
                    case Y410P:
                    case Y410I:
                        m_planes[0].data = mclMalloc(m_Meta.step, bd);
                        if (!m_planes[0].data) return MCL_ERR_MEMORY_ALLOC;
                        m_planes[1].data = m_planes[0].data + m_planes[0].roi.height*m_planes[0].step;
                        m_planes[2].data = m_planes[1].data + m_planes[1].roi.height*m_planes[1].step;
                        m_planes[3].data = m_planes[2].data + m_planes[2].roi.height*m_planes[2].step;
                        break;
                }
            }

            return MCL_ERR_NONE;
        } else {
            return MCL_ERR_INVALID_PARAM;
        }
    };

    bool ReadRawFrame(uint32_t field) {
        uint8_t *planes[4];
        int32_t  steps[4];

        for(int32_t i=0; i<4; i++) { planes[i] = m_planes[i].data; steps[i] = m_planes[i].step; }
        if(m_intl) { m_bottom = (m_field_order)^(field&0x1); field >>= 1; }
        if(m_cur_frame != (int32_t)field) {
            _file_fseek(m_file, ((uint64_t)field)*(m_Meta.step*m_source_pixel_size), SEEK_SET);
            size_t res = fread( m_Meta.data, m_source_pixel_size, m_Meta.step, m_file );
            switch (m_type) {
                case NV12P:
                case NV12I:
                    mclYCbCr420ToYCrCb420_P2P3R(m_Meta.data, m_planes[0].step, m_Meta.data+m_planes[0].step*m_planes[0].roi.height, m_planes[0].step, planes, steps, m_planes[0].roi, m_bd);
                    break;
                case YUY2P:
                case YUY2I:
                    mclYCbCr422_C2P3R(m_Meta.data, m_planes[0].step<<1, planes, steps, m_planes[0].roi, m_bd);
                    break;
                case NV16P:
                case NV16I:
                    mclNV16ToYCbCr422_P2P3R(m_Meta.data, m_planes[0].step, m_Meta.data+m_planes[0].step*m_planes[0].roi.height, m_planes[0].step, planes, steps, m_planes[0].roi, m_bd);
                    break;
                case AYUVP:
                case AYUVI:
                    planes[2] = m_planes[0].data;
                    planes[0] = m_planes[2].data;
                    mclCopy_C4P4R(m_Meta.data, m_planes[0].step<<2, planes, m_planes[0].step, m_planes[0].roi, m_bd);
                    break;
                case Y416P:
                case Y416I:
                    planes[1] = m_planes[0].data;
                    planes[0] = m_planes[1].data;
                    mclCopy_C4P4R(m_Meta.data, m_planes[0].step << 2, planes, m_planes[0].step, m_planes[0].roi, m_bd);
                    break;
                case Y410P:
                case Y410I:
                    mclY410ToYUV_C4P4R(m_Meta.data, m_planes[0].step, planes, m_planes[0].step, m_planes[0].roi, m_bd);
                    break;
                default:
                    break;
            }
            mclRShiftC_C1IR(m_RShift, m_planes[0].data, m_planes[0].step, m_planes[0].roi, m_bd);
            mclRShiftC_C1IR(m_RShift, m_planes[1].data, m_planes[1].step, m_planes[1].roi, m_bd);
            mclRShiftC_C1IR(m_RShift, m_planes[2].data, m_planes[2].step, m_planes[2].roi, m_bd);
            m_cur_frame = (int32_t)field;
            return (res != m_Meta.step);
        } else {
            return false;
        }
    };

    void GetFrame(int32_t idx, void *dst) {
        memcpy((uint8_t*)dst, (uint8_t*)(m_planes+idx), sizeof(SImage));
        SImage *destination = (SImage*) dst;
        if(m_intl) {
            if(m_bottom) destination->data += destination->step;
            destination->step <<= 1; destination->roi.height >>= 1;
        }
    };
};

class CMetricEvaluator {
protected:
    std::vector< std::pair< std::string, std::pair<uint32_t, uint32_t> > > metrics;
    uint32_t   m_num_planes, c_mask[5];
    CReader       *m_i1, *m_i2;
public:
    virtual ~CMetricEvaluator(void) {};
    void InitFrameParams(CReader *i1, CReader *i2) { m_i1 = i1; m_i2 = i2; };
    void InitComputationParams(Component cmps,
        std::vector< std::string > &st, std::vector< bool > &oflag, std::vector< double > &avg)
    {
        m_num_planes = (uint32_t)cmps.size() - 1;

        for(uint32_t i=0; i < m_num_planes; i++)
            c_mask[i] = cmps[i].second|cmps[m_num_planes].second;

        c_mask[m_num_planes] = cmps[m_num_planes].second;

        for(uint32_t i=0; i < metrics.size(); i++) {
            for(uint32_t j=0; j < m_num_planes; j++) {
                if(metrics[i].second.second&c_mask[j]) {
                    c_mask[j] |= metrics[i].second.second;
                    st.push_back(std::string(1, cmps[j].first) + std::string("-") + metrics[i].first);
                    oflag.push_back((metrics[i].second.first&cmps[j].second)!=0);
                    avg.push_back(0.0);
                }
            }
            if(metrics[i].second.first&c_mask[m_num_planes]) {
                st.push_back(metrics[i].first);
                oflag.push_back(true);
                avg.push_back(0.0);
            }
        }
    };
    virtual int32_t AllocateResourses(void) = 0;
    virtual void ComputeMetrics(std::vector< double > &val, std::vector< double > &avg) = 0;
};

class CPSNREvaluator: public CMetricEvaluator {
public:
    CPSNREvaluator() {
        std::pair< std::string, std::pair<uint32_t, uint32_t> >   metric_pair;
        metric_pair.first = "MSE";   metric_pair.second.first = MASK_MSE;   metric_pair.second.second = MASK_MSE;   metrics.push_back(metric_pair);
        metric_pair.first = "PSNR";  metric_pair.second.first = MASK_PSNR;  metric_pair.second.second = MASK_PSNR;  metrics.push_back(metric_pair);
        metric_pair.first = "APSNR"; metric_pair.second.first = MASK_APSNR; metric_pair.second.second = MASK_APSNR; metrics.push_back(metric_pair);
    };
    ~CPSNREvaluator(void) {};
    int32_t AllocateResourses(void) { return 0; };
    void ComputeMetrics(std::vector< double > &val, std::vector< double > &avg) {
        SImage i1_p, i2_p;
        double sum[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
        size_t i, j = val.size();
        ESequenceType sqtype = m_i1->GetSqType();
        ImageSize a = {2,2};

        for(i=0; i<m_num_planes; i++) {
            if(c_mask[i]&MASK_MSE) {
                m_i1->GetFrame((int32_t)i, &i1_p); m_i2->GetFrame((int32_t)i, &i2_p);
                mclNormDiff_L2_C1R(i1_p.data,  i1_p.step, i2_p.data,  i2_p.step, i1_p.roi, sum[i], m_i1->GetBitDepth());
                sum[i] = sum[i]*sum[i]/(double)(i1_p.roi.width*i1_p.roi.height);
                val.push_back(sum[i]); avg[j++] += sum[i];
            }
        }

        switch(get_chromaclass(sqtype)) {
            case C444:
                sum[m_num_planes] = (sum[0]+sum[1]+sum[2]+sum[3])/(double)m_num_planes; break;
            case C422:
                sum[3] = (2.0*sum[0]+sum[1]+sum[2])/4.0; break;
            case C420:
            default:
                sum[3] = (4.0*sum[0]+sum[1]+sum[2])/6.0; break;
        }

        if(c_mask[m_num_planes]&MASK_MSE) { val.push_back(sum[m_num_planes]); avg[j++] += sum[m_num_planes]; }
        for(i=0; i<=m_num_planes; i++) { if(c_mask[i]&MASK_PSNR) { val.push_back(MSEToPSNR(sum[i], MaxError(m_i1->GetBitDepth()))); avg[j++] += sum[i]; } }
        for(i=0; i<=m_num_planes; i++) { if(c_mask[i]&MASK_APSNR) { val.push_back(MSEToPSNR(sum[i], MaxError(m_i1->GetBitDepth()))); avg[j++] += MSEToPSNR(sum[i], MaxError(m_i1->GetBitDepth())); } }
    };
};

class CSSIMEvaluator: public CMetricEvaluator {
private:
    float  *m_mu1, *m_mu2, *m_mu1_sq, *m_mu2_sq, *m_mu1_mu2, *m_tmp;
    int32_t m_step, mc_ksz[3], m_xkidx[4], m_ykidx[4];
    float   m_ssim_c1, m_ssim_c2, m_kernel_values[11+7+5], *mc_krn[3];

    int32_t GaussianKernel(int32_t KernelSize, float sigma, float* pKernel) {
        int32_t i;

        float val, sum = 0.0f;
        for (i = 0; i < KernelSize; i++) {
            val = (float)(i - KernelSize / 2);
            pKernel[i] = (float)exp(-(float)(val * val) / (float)(2.0f * sigma * sigma));
            sum += pKernel[i];
        }

        for (i = 0; i < KernelSize; i++) pKernel[i] /= sum;

        return 0;
    }

    void testFastSSIM_32f(
        const float* pSrc1, int32_t src1Step, const float* pSrc2, int32_t src2Step, const float* pSrc3, int32_t src3Step,
        const float* pSrc4, int32_t src4Step, const float* pSrc5, int32_t src5Step, float* pDst, int32_t dstStep,
        ImageSize roiSize, float C1, float C2)
    {
        int32_t     i, j;
        float  t1, t2, t3, t4, *pMx, *pMy, *pSx2, *pSy2, *pSxy, *pDi;

        C2 += C1;

        for (j = 0; j<roiSize.height; j++) {
            pMx = (float*)((uint8_t*)(pSrc1)+j * src1Step); pMy = (float*)((uint8_t*)(pSrc2)+j * src2Step);
            pSx2 = (float*)((uint8_t*)(pSrc3)+j * src3Step); pSy2 = (float*)((uint8_t*)(pSrc4)+j * src4Step);
            pSxy = (float*)((uint8_t*)(pSrc5)+j * src5Step); pDi = (float*)((uint8_t*)(pDst)+j * dstStep);
            for (i = 0; i<roiSize.width; i++) {
                t1 = (*pMx)*(*pMy); t1 = t1 + t1 + C1; t2 = (*pSxy) + (*pSxy) - t1 + C2;
                t3 = (*pMx)*(*pMx) + (*pMy)*(*pMy) + C1; t4 = (*pSx2) + (*pSy2) - t3 + C2;
                t2 *= t1; t4 *= t3;
                *pDi = (t4 >= FLT_EPSILON) ? (t2 / t4) : ((t3 >= FLT_EPSILON) ? (t1 / t3) : (1.0f));
                pMx++; pMy++; pSx2++; pSy2++; pSxy++; pDi++;
            }
        }
    }

public:
    CSSIMEvaluator(): m_ssim_c1(0), m_ssim_c2(0), m_step(0) {
        std::pair< std::string, std::pair<uint32_t, uint32_t> >   metric_pair;
        metric_pair.first = "SSIM"; metric_pair.second.first = MASK_SSIM; metric_pair.second.second = MASK_SSIM; metrics.push_back(metric_pair);
        m_mu1 = m_mu2 = m_mu1_sq = m_mu2_sq = m_mu1_mu2 = m_tmp = 0;
        GaussianKernel(11, 1.5,   m_kernel_values);
        GaussianKernel( 7, 0.75,  m_kernel_values+11);
        GaussianKernel( 5, 0.375, m_kernel_values+18);
        mc_krn[0] = m_kernel_values; mc_krn[1] = m_kernel_values+11; mc_krn[2] = m_kernel_values+18;
        mc_ksz[0] = 11; mc_ksz[1] = 7; mc_ksz[2] = 5;
        m_xkidx[0] = m_xkidx[1] = m_xkidx[2] = m_xkidx[3] = m_ykidx[0] = m_ykidx[1] = m_ykidx[2] = m_ykidx[3] = 0;
    };
    ~CSSIMEvaluator(void) {
        mclFree(m_mu1); mclFree(m_mu2); mclFree(m_mu1_sq); mclFree(m_mu2_sq); mclFree(m_mu1_mu2); mclFree(m_tmp);
    };

    int32_t AllocateResourses(void) {
        SImage ref;

        m_i1->GetFrame(0, &ref);

        m_mu1     = mclMalloc_32f_C1(ref.roi.width, ref.roi.height, &m_step);
        m_mu2     = mclMalloc_32f_C1(ref.roi.width, ref.roi.height, &m_step);
        m_mu1_sq  = mclMalloc_32f_C1(ref.roi.width, ref.roi.height, &m_step);
        m_mu2_sq  = mclMalloc_32f_C1(ref.roi.width, ref.roi.height, &m_step);
        m_mu1_mu2 = mclMalloc_32f_C1(ref.roi.width, ref.roi.height, &m_step);
        m_tmp     = mclMalloc_32f_C1(ref.roi.width, ref.roi.height, &m_step);

        if(!m_mu1 || !m_mu2 || !m_mu1_sq || !m_mu2_sq || !m_mu1_mu2 || !m_tmp)
            return MCL_ERR_MEMORY_ALLOC;

        for(int32_t i=0; i<4; i++) {
            if(i!=0) {
                switch (get_chromaclass(m_i1->GetSqType())) {
                case C444:
                    break;
                case C422:
                    m_xkidx[i]++; break;
                case C420:
                default:
                    m_xkidx[i]++; m_ykidx[i]++; break;
                }
            }
            if(m_i1->GetInterlaced()) m_ykidx[i]++;
        }

        float max_e = (float)MaxError(m_i1->GetBitDepth());
        m_ssim_c1 = 0.0001f*max_e*max_e;
        m_ssim_c2 = 0.0009f*max_e*max_e;
        
        return MCL_ERR_NONE;
    };

    void ComputeMetrics(std::vector< double > &val, std::vector< double > &avg) {
        double     idx[5]  = {0.0, 0.0, 0.0, 0.0, 0.0};
        double     aidx[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
        ImageSize  flt, flt_h;
        uint32_t   i, j = (uint32_t)val.size(), shift, shift_h;
        SImage     i1_p, i2_p;

        for(i=0; i<m_num_planes; i++) {
            if(c_mask[i]&MASK_SSIM) {
                m_i1->GetFrame(i, &i1_p); m_i2->GetFrame(i, &i2_p);

                mclConvert__u32f_C1R(i1_p.data, i1_p.step, m_mu1, m_step, i1_p.roi, m_i1->GetBitDepth());
                mclConvert__u32f_C1R(i2_p.data, i2_p.step, m_mu2, m_step, i2_p.roi, m_i1->GetBitDepth());

                mclSqr_32f_C1R(m_mu1, m_step, m_mu1_sq, m_step, i1_p.roi);
                mclSqr_32f_C1R(m_mu2, m_step, m_mu2_sq, m_step, i1_p.roi);
                mclMul_32f_C1R(m_mu1, m_step, m_mu2, m_step, m_mu1_mu2, m_step, i1_p.roi);

                flt.width  = i1_p.roi.width  - (mc_ksz[m_xkidx[i]]&~1);
                flt.height = i1_p.roi.height - (mc_ksz[m_ykidx[i]]&~1);
                shift = (mc_ksz[m_xkidx[i]]>>1) + (mc_ksz[m_ykidx[i]]>>1)*m_step/sizeof(float);

                flt_h.width  = i1_p.roi.width  - (mc_ksz[m_xkidx[i]]&~1);
                flt_h.height = i1_p.roi.height;
                shift_h = (mc_ksz[m_xkidx[i]]>>1);

                mclFilterRow_32f_C1R(m_mu1+shift_h,     m_step, m_tmp+shift_h,   m_step, flt_h, mc_krn[m_xkidx[i]], mc_ksz[m_xkidx[i]], mc_ksz[m_xkidx[i]]>>1);
                mclFilterColumn_32f_C1R(m_tmp+shift,    m_step, m_mu1+shift,     m_step, flt,   mc_krn[m_ykidx[i]], mc_ksz[m_ykidx[i]], mc_ksz[m_ykidx[i]]>>1);
                mclFilterRow_32f_C1R(m_mu2+shift_h,     m_step, m_tmp+shift_h,   m_step, flt_h, mc_krn[m_xkidx[i]], mc_ksz[m_xkidx[i]], mc_ksz[m_xkidx[i]]>>1);
                mclFilterColumn_32f_C1R(m_tmp+shift,    m_step, m_mu2+shift,     m_step, flt,   mc_krn[m_ykidx[i]], mc_ksz[m_ykidx[i]], mc_ksz[m_ykidx[i]]>>1);
                mclFilterRow_32f_C1R(m_mu1_sq+shift_h,  m_step, m_tmp+shift_h,   m_step, flt_h, mc_krn[m_xkidx[i]], mc_ksz[m_xkidx[i]], mc_ksz[m_xkidx[i]]>>1);
                mclFilterColumn_32f_C1R(m_tmp+shift,    m_step, m_mu1_sq+shift,  m_step, flt,   mc_krn[m_ykidx[i]], mc_ksz[m_ykidx[i]], mc_ksz[m_ykidx[i]]>>1);
                mclFilterRow_32f_C1R(m_mu2_sq+shift_h,  m_step, m_tmp+shift_h,   m_step, flt_h, mc_krn[m_xkidx[i]], mc_ksz[m_xkidx[i]], mc_ksz[m_xkidx[i]]>>1);
                mclFilterColumn_32f_C1R(m_tmp+shift,    m_step, m_mu2_sq+shift,  m_step, flt,   mc_krn[m_ykidx[i]], mc_ksz[m_ykidx[i]], mc_ksz[m_ykidx[i]]>>1);
                mclFilterRow_32f_C1R(m_mu1_mu2+shift_h, m_step, m_tmp+shift_h,   m_step, flt_h, mc_krn[m_xkidx[i]], mc_ksz[m_xkidx[i]], mc_ksz[m_xkidx[i]]>>1);
                mclFilterColumn_32f_C1R(m_tmp+shift,    m_step, m_mu1_mu2+shift, m_step, flt,   mc_krn[m_ykidx[i]], mc_ksz[m_ykidx[i]], mc_ksz[m_ykidx[i]]>>1);

                testFastSSIM_32f(m_mu1+shift, m_step, m_mu2+shift, m_step, m_mu1_sq+shift, m_step,
                   m_mu2_sq+shift, m_step, m_mu1_mu2+shift, m_step, m_tmp+shift, m_step, flt, m_ssim_c1, m_ssim_c2);

                mclMean_32f_C1R(m_tmp+shift, m_step, flt, idx[i]);

                val.push_back(idx[i]); avg[j++] += idx[i];
            }
        }

        if(c_mask[m_num_planes]&MASK_SSIM) {
            switch(get_chromaclass(m_i1->GetSqType())) {
                case C444:
                    idx[m_num_planes] = (idx[0]+idx[1]+idx[2]+idx[3])/(double)m_num_planes; break;
                case C422:
                    idx[3] = (2.0*idx[0]+idx[1]+idx[2])/4.0; break;
                case C420:
                default:
                    idx[3] = (4.0*idx[0]+idx[1]+idx[2])/6.0; break;
            }
            val.push_back(idx[m_num_planes]); avg[j++] += idx[m_num_planes];
        }
    };
};

#if !defined(NO_IPP) && !defined(LEGACY_IPP)
typedef struct {
    Ipp32f **ppMu1;     // Placeholder for line buffer pointers for filtered Mu1 
    Ipp32f **ppMu2;     // Placeholder for line buffer pointers for filtered Mu2
    Ipp32f **ppMu1S;    // Placeholder for line buffer pointers for filtered Mu1*Mu1
    Ipp32f **ppMu2S;    // Placeholder for line buffer pointers for filtered Mu2*Mu2
    Ipp32f **ppMu12;    // Placeholder for line buffer pointers for filtered Mu1*Mu2

    Ipp32f **ppData;    // Line buffer pointers data (allocated for max filter size)
    Ipp32f  *pData;     // Filtered data (allocated for max filter size)
    Ipp32s   step;      // Filtered data step

    Ipp8u   *pRowBuf;   // Buffer for filtering along the rows
    Ipp8u   *pColBuf;   // Buffer for filtering along the columns
} ssim_context;

const int mmsim_depth = 5;
const int min_patch_height = 64;
#if defined(_OPENMP)
const int ssim_ctx_cnt = 8; // Support up to 8 threads if compiled with enabled OpenMP
#else
const int ssim_ctx_cnt = 1; // Default to sequential MS-SSIM evaluation
#endif
const float artifacts_threshold = 0.3f;

class CMSSIMEvaluator : public CMetricEvaluator {
private:
    Ipp32f * m_kernel_values;
    ssim_context  m_ssim_ctx[ssim_ctx_cnt];

    Ipp32f  *m_im1, *m_im2, *m_imt;
    Ipp32f   m_ssim_c1, m_ssim_c2;

    Ipp32s   mc_ksz[3], m_xkidx[4], m_ykidx[4], m_step;
    Ipp32f  *mc_krn[3];

    IppiResizeSpec_32f *m_pSpec;
    Ipp8u *m_pBuffer;

    int allocateSSIMContext(Ipp32s xs, Ipp32s ys, Ipp32s width, ssim_context* pCtx) {
        IppiSize roi;
        int      bsize, tsize = 5 * ys + 5;

        pCtx->pData = ippiMalloc_32f_C1(width, tsize, &pCtx->step);
        pCtx->ppData = (Ipp32f**)ippsMalloc_8s(sizeof(Ipp32f*) * 2 * tsize);

        roi.width = width; roi.height = ys;
        ippiFilterRowBorderPipelineGetBufferSize_32f_C1R(roi, xs, &bsize);
        pCtx->pRowBuf = ippsMalloc_8u(bsize);

        roi.width = width; roi.height = 1;
        ippiFilterColumnPipelineGetBufferSize_32f_C1R(roi, ys, &bsize);
        pCtx->pColBuf = ippsMalloc_8u(bsize);

        return 0;
    }

    void freeSSIMContext(ssim_context* pCtx) {
        ippsFree(pCtx->pRowBuf);
        ippsFree(pCtx->pColBuf);
        ippsFree(pCtx->ppData);
        ippiFree(pCtx->pData);
    }

    inline void getSSIMIndexes_32f_R(Ipp32f* pSrc1, Ipp32f* pSrc2, const int srcStep, const int x_offset, const int y_offset, const IppiSize ds_roi, ssim_context *pCtx,
        const Ipp32f* xknl, const int xsz, const Ipp32f* yknl, const int ysz, const Ipp32f C1, const Ipp32f C2, double &mssim, double &mcs, int &artf)
    {
        int      i, j;
        Ipp32f **pTmp;
        IppiSize i_roi, o_roi;
        IppStatus status;
        const int tsize = 5 * ysz + 5;

        /* Build-up filtering pipeline*/
        pSrc1 = (Ipp32f*)((Ipp8u*)pSrc1 + (y_offset - (ysz >> 1))*srcStep);
        pSrc2 = (Ipp32f*)((Ipp8u*)pSrc2 + (y_offset - (ysz >> 1))*srcStep);

        for (int i = 0; i < tsize; i++)
            pCtx->ppData[i + tsize] = pCtx->ppData[i] = (Ipp32f*)((Ipp8u*)pCtx->pData + i * pCtx->step);

        pCtx->ppMu1 = pCtx->ppData;
        pCtx->ppMu2 = pCtx->ppMu1 + ysz;
        pCtx->ppMu1S = pCtx->ppMu2 + ysz;
        pCtx->ppMu2S = pCtx->ppMu1S + ysz;
        pCtx->ppMu12 = pCtx->ppMu2S + ysz;
        pTmp = pCtx->ppMu12 + ysz;

        i_roi.width = ds_roi.width - xsz + 1; i_roi.height = ysz;
        status = ippiFilterRowBorderPipeline_32f_C1R(pSrc1 + x_offset, srcStep, pCtx->ppMu1, i_roi, xknl, xsz, xsz >> 1, ippBorderInMem, 0.0f, pCtx->pRowBuf);
        status = ippiFilterRowBorderPipeline_32f_C1R(pSrc2 + x_offset, srcStep, pCtx->ppMu2, i_roi, xknl, xsz, xsz >> 1, ippBorderInMem, 0.0f, pCtx->pRowBuf);

        i_roi.width = ds_roi.width; i_roi.height = 1;
        for (i = 0; i < ysz - 1; i++) {
            status = ippsSqr_32f(pSrc1, *pTmp, ds_roi.width);
            status = ippiFilterRowBorderPipeline_32f_C1R(*pTmp + x_offset, srcStep, pCtx->ppMu1S + i, i_roi, xknl, xsz, xsz >> 1, ippBorderInMem, 0.0f, pCtx->pRowBuf);
            status = ippsSqr_32f(pSrc2, *pTmp, ds_roi.width);
            status = ippiFilterRowBorderPipeline_32f_C1R(*pTmp + x_offset, srcStep, pCtx->ppMu2S + i, i_roi, xknl, xsz, xsz >> 1, ippBorderInMem, 0.0f, pCtx->pRowBuf);
            status = ippsMul_32f(pSrc1, pSrc2, *pTmp, ds_roi.width);
            status = ippiFilterRowBorderPipeline_32f_C1R(*pTmp + x_offset, srcStep, pCtx->ppMu12 + i, i_roi, xknl, xsz, xsz >> 1, ippBorderInMem, 0.0f, pCtx->pRowBuf);

            pSrc1 = (Ipp32f*)((Ipp8u*)pSrc1 + srcStep);
            pSrc2 = (Ipp32f*)((Ipp8u*)pSrc2 + srcStep);
        }

        /* Process ROI */
        o_roi.width = ds_roi.width - xsz + 1; o_roi.height = 1;
        for (i = 0; i < ds_roi.height; i++) {
            status = ippiFilterRowBorderPipeline_32f_C1R(pSrc1 + x_offset, srcStep, pCtx->ppMu1 + ysz - 1, i_roi, xknl, xsz, xsz >> 1, ippBorderInMem, 0.0f, pCtx->pRowBuf);
            status = ippiFilterRowBorderPipeline_32f_C1R(pSrc2 + x_offset, srcStep, pCtx->ppMu2 + ysz - 1, i_roi, xknl, xsz, xsz >> 1, ippBorderInMem, 0.0f, pCtx->pRowBuf);
            status = ippsSqr_32f(pSrc1, *pTmp, ds_roi.width);
            status = ippiFilterRowBorderPipeline_32f_C1R(*pTmp + x_offset, srcStep, pCtx->ppMu1S + ysz - 1, i_roi, xknl, xsz, xsz >> 1, ippBorderInMem, 0.0f, pCtx->pRowBuf);
            status = ippsSqr_32f(pSrc2, *pTmp, ds_roi.width);
            status = ippiFilterRowBorderPipeline_32f_C1R(*pTmp + x_offset, srcStep, pCtx->ppMu2S + ysz - 1, i_roi, xknl, xsz, xsz >> 1, ippBorderInMem, 0.0f, pCtx->pRowBuf);
            status = ippsMul_32f(pSrc1, pSrc2, *pTmp, ds_roi.width);
            status = ippiFilterRowBorderPipeline_32f_C1R(*pTmp + x_offset, srcStep, pCtx->ppMu12 + ysz - 1, i_roi, xknl, xsz, xsz >> 1, ippBorderInMem, 0.0f, pCtx->pRowBuf);
            status = ippiFilterColumnPipeline_32f_C1R((const Ipp32f**)pCtx->ppMu1, pTmp[0], srcStep, o_roi, yknl, ysz, pCtx->pColBuf);
            status = ippiFilterColumnPipeline_32f_C1R((const Ipp32f**)pCtx->ppMu2, pTmp[1], srcStep, o_roi, yknl, ysz, pCtx->pColBuf);
            status = ippiFilterColumnPipeline_32f_C1R((const Ipp32f**)pCtx->ppMu1S, pTmp[2], srcStep, o_roi, yknl, ysz, pCtx->pColBuf);
            status = ippiFilterColumnPipeline_32f_C1R((const Ipp32f**)pCtx->ppMu2S, pTmp[3], srcStep, o_roi, yknl, ysz, pCtx->pColBuf);
            status = ippiFilterColumnPipeline_32f_C1R((const Ipp32f**)pCtx->ppMu12, pTmp[4], srcStep, o_roi, yknl, ysz, pCtx->pColBuf);

            Ipp32f  t1, t2, t3, t4, *pMx, *pMy, *pSx2, *pSy2, *pSxy;
            Ipp64f  tmp;
            pMx = pTmp[0]; pMy = pTmp[1]; pSx2 = pTmp[2]; pSy2 = pTmp[3]; pSxy = pTmp[4];
            
            for (j = 0; j<o_roi.width; j++) {
                t1 = (*pMx)*(*pMy); t1 = t1 + t1 + C1;
                t2 = (*pSxy) + (*pSxy) - t1 + C2;
                t3 = (*pMx)*(*pMx) + (*pMy)*(*pMy) + C1;
                t4 = (*pSx2) + (*pSy2) - t3 + C2;

                mcs += t2 / (Ipp64f)t4;
                tmp = (t1*t2) / (Ipp64f)(t3*t4);
                mssim += tmp;
                if (tmp < artifacts_threshold) (artf)++;
                pMx++; pMy++; pSx2++; pSy2++; pSxy++;
            }
            
            pSrc1 = (Ipp32f*)((Ipp8u*)pSrc1 + srcStep);
            pSrc2 = (Ipp32f*)((Ipp8u*)pSrc2 + srcStep);

            if ((pCtx->ppMu1 - pCtx->ppData) == (tsize - 1)) {
                pCtx->ppMu1 = pCtx->ppData;
                pCtx->ppMu2 = pCtx->ppMu1 + ysz;
                pCtx->ppMu1S = pCtx->ppMu2 + ysz;
                pCtx->ppMu2S = pCtx->ppMu1S + ysz;
                pCtx->ppMu12 = pCtx->ppMu2S + ysz;
                pTmp = pCtx->ppMu12 + ysz;
            }
            else {
                pCtx->ppMu1++; pCtx->ppMu2++; pCtx->ppMu1S++; pCtx->ppMu2S++; pCtx->ppMu12++; pTmp++;
            }
        }
    }

public:
    CMSSIMEvaluator() : m_ssim_c1(0), m_ssim_c2(0) {
        std::pair< std::string, std::pair<unsigned int, unsigned int> >   metric_pair;
        metric_pair.first = "MSSIM"; metric_pair.second.first = MASK_MSSIM; metric_pair.second.second = MASK_MSSIM; metrics.push_back(metric_pair);
        metric_pair.first = "SSIM"; metric_pair.second.first = MASK_SSIM; metric_pair.second.second = MASK_SSIM; metrics.push_back(metric_pair);
        metric_pair.first = "ARTIFACTS"; metric_pair.second.first = MASK_ARTIFACTS; metric_pair.second.second = MASK_ARTIFACTS; metrics.push_back(metric_pair);

        m_xkidx[0] = m_xkidx[1] = m_xkidx[2] = m_xkidx[3] = m_ykidx[0] = m_ykidx[1] = m_ykidx[2] = m_ykidx[3] = 0;

        m_pBuffer = 0; m_pSpec = 0;
    };

    ~CMSSIMEvaluator(void) {
        for (int i = 0; i < ssim_ctx_cnt; i++) freeSSIMContext(m_ssim_ctx + i);
        ippsFree(m_kernel_values);
        ippiFree(m_im1);
        ippsFree(m_pBuffer);
        ippsFree(m_pSpec);
    };

    int GetGaussianSize(Ipp32f sigma, Ipp32f accuracy, Ipp32f* p, int maxsz) {
        const int MaxKernelSize = 1023;
        Ipp32f filter_tmp[MaxKernelSize];
        int    i, gfsz = 0;
        Ipp32f *tp;

        Ipp32f val, sum = 0.0f;
        for (i = 0; i < MaxKernelSize; i++) {
            val = (Ipp32f)(i - MaxKernelSize / 2);
            filter_tmp[i] = (Ipp32f)exp(-(Ipp32f)(val * val) / (Ipp32f)(2.0f * sigma * sigma));
            sum += filter_tmp[i];
        }
        accuracy *= sum; sum = 0.0f; tp = p;
        for (i = 0; i < MaxKernelSize; i++) {
            if (filter_tmp[i] > accuracy) {
                if (maxsz--) {
                    gfsz++; *tp++ = (Ipp32f)filter_tmp[i]; sum += filter_tmp[i];
                } else {
                    return -1;
                }
            }
        }
        for (i = 0; i < gfsz; i++) *p++ /= (Ipp32f)sum;

        return gfsz;
    }

    int AllocateResourses(void) {
        SImage  ref;
        int     asz = 0;
        float   sigma = 1.5f;

        m_i1->GetFrame(0, &ref);

        m_kernel_values = ippsMalloc_32f(1024);

        mc_krn[0] = m_kernel_values;
        mc_ksz[0] = GetGaussianSize(sigma, 0.0001f, mc_krn[0], 1024); asz += mc_ksz[0];
        mc_krn[1] = m_kernel_values + asz;
        mc_ksz[1] = GetGaussianSize(sigma / 2.0f, 0.0001f, mc_krn[1], 1024 - asz); asz += mc_ksz[1];
        mc_krn[2] = m_kernel_values + asz;
        mc_ksz[2] = GetGaussianSize(sigma / 4.0f, 0.0001f, mc_krn[2], 1024 - asz);

        for (int i = 0; i<(int)m_num_planes; i++) {
            if (i != 0) {
                switch (get_chromaclass(m_i1->GetSqType())) {
                case C444:
                    break;
                case C422:
                    m_xkidx[i]++; break;
                case C420:
                default:
                    m_xkidx[i]++; m_ykidx[i]++; break;
                }
            }
            if (m_i1->GetInterlaced()) m_ykidx[i]++;
        }

        for (int i = 0; i < ssim_ctx_cnt; i++)
            allocateSSIMContext(mc_ksz[m_xkidx[0]], mc_ksz[m_ykidx[0]], ref.roi.width, m_ssim_ctx + i);

        m_im1 = ippiMalloc_32f_C1(ref.roi.width, 5 * ref.roi.height / 2, &m_step);
        if (!m_im1)  return -2;
        m_im2 = (Ipp32f*)((Ipp8s*)m_im1 + ref.roi.height*m_step);
        m_imt = (Ipp32f*)((Ipp8s*)m_im2 + ref.roi.height*m_step);

        float max_e = (float)MaxError(m_i1->GetBitDepth());
        m_ssim_c1 = 0.0001f*max_e*max_e;
        m_ssim_c2 = 0.0009f*max_e*max_e;

        IppiSize sSize, dSize;
        int specSize = 0, initSize = 0, bufSize = 0;
        int specSizeMax = 0, bufSizeMax = 0;

        // Minimal frame size requirements check
        IppiSize mSize = { 176, 176 };
        if ((c_mask[1] & (MASK_MSSIM | MASK_SSIM | MASK_ARTIFACTS)) || (c_mask[2] & (MASK_MSSIM | MASK_SSIM | MASK_ARTIFACTS))) {
            if (get_chromaclass(m_i1->GetSqType()) == C422) {
                mSize.width *= 2;
            }
            else if (get_chromaclass(m_i1->GetSqType()) == C444) {
                mSize.width *= 2;
                mSize.height *= 2;
            }
        }
        if ((ref.roi.width < mSize.width || ref.roi.height < mSize.height)) return -3;

        // Over-allocate temporary buffers for resize to cover both 420/422/444
        sSize = ref.roi;
        for (int l = 0; l<mmsim_depth; l++) {
            dSize.width = sSize.width >> 1;
            dSize.height = sSize.height >> 1;
            ippiResizeGetSize_32f(sSize, dSize, ippSuper, 0, &specSize, &initSize);
            if (specSize > specSizeMax) specSizeMax = specSize;
            sSize = dSize;
        }
        m_pSpec = (IppiResizeSpec_32f*)ippsMalloc_8u(2 * specSizeMax);
        if (!m_pSpec)  return -2;

        sSize = ref.roi;
        for (int l = 0; l<mmsim_depth; l++) {
            dSize.width = sSize.width >> 1;
            dSize.height = sSize.height >> 1;
            ippiResizeSuperInit_32f(sSize, dSize, m_pSpec);
            ippiResizeGetBufferSize_32f(m_pSpec, dSize, 1, &bufSize);
            if (bufSize > bufSizeMax) bufSizeMax = bufSize;
            sSize = dSize;
        }
        m_pBuffer = ippsMalloc_8u(2 * bufSizeMax);
        if (!m_pBuffer)  return -2;

        return 0;
    };

    void ComputeMetrics(std::vector< double > &val, std::vector< double > &avg) {
        const double pwrs[mmsim_depth] = { 0.0448, 0.2856, 0.3001, 0.2363, 0.1333 };
        double      ms_idx[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
        double      ss_idx[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
        double      af_idx[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
        double      mssim[mmsim_depth], mcs[mmsim_depth], artcnt[mmsim_depth];
        int         depth, r, k, i, j = (int)val.size();
        SImage      i1_p, i2_p;
        IppiSize    pr_roi, ds_roi;
        IppiPoint   dstOffset = { 0, 0 };
        Ipp32f     *pSrc1, *pSrc2, *pTmp, *pTmp1;

        for (i = 0; i<(int)m_num_planes; i++) {
            if (c_mask[i] & (MASK_MSSIM | MASK_SSIM | MASK_ARTIFACTS)) {
                m_i1->GetFrame(i, &i1_p); m_i2->GetFrame(i, &i2_p);
                pSrc1 = m_im1; pSrc2 = m_im2; pTmp = m_imt;
                depth = (c_mask[i] & (MASK_MSSIM | MASK_ARTIFACTS)) ? mmsim_depth : 1;
                for (k = 0; k<depth; k++) {
                    if (k) {
                        pr_roi = ds_roi; ds_roi.width = pr_roi.width >> 1; ds_roi.height = pr_roi.height >> 1;
                        pr_roi.width &= ~0x1; pr_roi.height &= ~0x1; // Possibly discard last column/row to match reference MS-SSIM code
                        ippiResizeSuperInit_32f(pr_roi, ds_roi, m_pSpec);
                        ippiResizeSuper_32f_C1R(pSrc2, m_step, pTmp, m_step, dstOffset, ds_roi, m_pSpec, m_pBuffer);
                        ippiResizeSuper_32f_C1R(pSrc1, m_step, pSrc2, m_step, dstOffset, ds_roi, m_pSpec, m_pBuffer);
                        pTmp1 = pSrc1; pSrc1 = pSrc2; pSrc2 = pTmp; pTmp = pTmp1;
                    }
                    else {
                        if (m_i1->GetBitDepth() == D008) {
                            ippiConvert_8u32f_C1R((Ipp8u*)i1_p.data, i1_p.step, pSrc1, m_step, i1_p.roi);
                            ippiConvert_8u32f_C1R((Ipp8u*)i2_p.data, i2_p.step, pSrc2, m_step, i2_p.roi);
                        }
                        else {
                            ippiConvert_16u32f_C1R((Ipp16u*)i1_p.data, i1_p.step, pSrc1, m_step, i1_p.roi);
                            ippiConvert_16u32f_C1R((Ipp16u*)i2_p.data, i2_p.step, pSrc2, m_step, i2_p.roi);
                        }
                        ds_roi = i1_p.roi;
                    }

                    int i_width = ds_roi.width - mc_ksz[m_xkidx[i]] + 1;
                    int i_height = ds_roi.height - mc_ksz[m_ykidx[i]] + 1;
                    int patch_cnt = i_height / min_patch_height;

                    if (patch_cnt == 0) patch_cnt = 1;
                    if (patch_cnt > ssim_ctx_cnt) patch_cnt = ssim_ctx_cnt;
#if defined(_OPENMP)
                    if (patch_cnt > omp_get_max_threads()) patch_cnt = omp_get_max_threads();
#endif
                    int patch_height = i_height / patch_cnt;

                    double mssim_a = 0.0, mcs_a = 0.0, artcnt_a = 0.0;
#if defined(_OPENMP)
                    #pragma omp parallel for reduction( + : mssim_a, mcs_a, artcnt_a )
#endif
                    for (r = 0; r < patch_cnt; r++) {
                        IppiSize p_roi = ds_roi;
                        int x_offset = (mc_ksz[m_xkidx[i]] >> 1);
                        int y_offset = (mc_ksz[m_ykidx[i]] >> 1) + r * patch_height;
                        double mssim_l = 0.0, mcs_l = 0.0;
                        int    artcnt_l = 0;

                        p_roi.width = ds_roi.width;
                        p_roi.height = (r != (patch_cnt - 1)) ? patch_height : ds_roi.height - mc_ksz[m_ykidx[i]] + 1 - r * patch_height;

                        getSSIMIndexes_32f_R(pSrc1, pSrc2, m_step, x_offset, y_offset, p_roi, m_ssim_ctx + r, mc_krn[m_xkidx[i]], mc_ksz[m_xkidx[i]], mc_krn[m_ykidx[i]], mc_ksz[m_ykidx[i]], m_ssim_c1, m_ssim_c2 + m_ssim_c1, mssim_l, mcs_l, artcnt_l);
                        mssim_a += mssim_l; mcs_a += mcs_l; artcnt_a += artcnt_l;
                    }

                    mssim[k] = mssim_a/(double)(i_width * i_height);
                    mcs[k] = mcs_a/(double)(i_width * i_height);
                    artcnt[k] = artcnt_a/(double)(i_width * i_height);

                    if (mcs[k] < 0.0f) mcs[k] = 0.0f;
                    if (mssim[k] < 0.0f) mssim[k] = 0.0f;
                }

                if (c_mask[i] & MASK_MSSIM) {
                    double f_mssim = pow(mssim[mmsim_depth - 1], pwrs[mmsim_depth - 1]);
                    for (k = 0; k<mmsim_depth - 1; k++) f_mssim *= pow(mcs[k], pwrs[k]);

                    ms_idx[i] = f_mssim;
                }
                if (c_mask[i] & MASK_SSIM) ss_idx[i] = mssim[0];
                if (c_mask[i] & MASK_ARTIFACTS) af_idx[i] = 0.5*(artcnt[3] + artcnt[4]);
            }
        }

        for (i = 0; i<(int)m_num_planes; i++) {
            if (c_mask[i] & MASK_MSSIM) {
                val.push_back(ms_idx[i]); avg[j++] += ms_idx[i];
            }
        }

        if (c_mask[m_num_planes] & MASK_MSSIM) {
            switch (get_chromaclass(m_i1->GetSqType())) {
            case C444:
                ms_idx[m_num_planes] = (ms_idx[0] + ms_idx[1] + ms_idx[2] + ms_idx[3]) / (double)m_num_planes; break;
            case C422:
                ms_idx[3] = (2.0*ms_idx[0] + ms_idx[1] + ms_idx[2]) / 4.0; break;
            case C420:
            default:
                ms_idx[3] = (4.0*ms_idx[0] + ms_idx[1] + ms_idx[2]) / 6.0; break;
            }
            val.push_back(ms_idx[m_num_planes]); avg[j++] += ms_idx[m_num_planes];
        }

        for (i = 0; i<(int)m_num_planes; i++) {
            if (c_mask[i] & MASK_SSIM) {
                val.push_back(ss_idx[i]); avg[j++] += ss_idx[i];
            }
        }
        if (c_mask[m_num_planes] & MASK_SSIM) {
            switch (get_chromaclass(m_i1->GetSqType())) {
            case C444:
                ss_idx[m_num_planes] = (ss_idx[0] + ss_idx[1] + ss_idx[2] + ss_idx[3]) / (double)m_num_planes; break;
            case C422:
                ss_idx[3] = (2.0*ss_idx[0] + ss_idx[1] + ss_idx[2]) / 4.0; break;
            case C420:
            default:
                ss_idx[3] = (4.0*ss_idx[0] + ss_idx[1] + ss_idx[2]) / 6.0; break;
            }
            val.push_back(ss_idx[m_num_planes]); avg[j++] += ss_idx[m_num_planes];
        }

        for (i = 0; i<(int)m_num_planes; i++) {
            if (c_mask[i] & MASK_ARTIFACTS) {
                val.push_back(af_idx[i]); avg[j++] += af_idx[i];
            }
        }
        if (c_mask[m_num_planes] & MASK_ARTIFACTS) {
            switch (get_chromaclass(m_i1->GetSqType())) {
            case C444:
                af_idx[m_num_planes] = (af_idx[0] + af_idx[1] + af_idx[2] + af_idx[3]) / (double)m_num_planes; break;
            case C422:
                af_idx[3] = (2.0*af_idx[0] + af_idx[1] + af_idx[2]) / 4.0; break;
            case C420:
            default:
                af_idx[3] = (4.0*af_idx[0] + af_idx[1] + af_idx[2]) / 6.0; break;
            }
            val.push_back(af_idx[m_num_planes]); avg[j++] += af_idx[m_num_planes];
        }
    }
};

const short mpegmatrix [ 64 ] = {
    8,  16, 19, 22, 26, 27, 29, 34,
    16, 16, 22, 21, 27, 29, 34, 37,
    19, 22, 26, 27, 29, 31, 34, 38,
    22, 22, 26, 27, 29, 34, 37, 40,
    22, 26, 27, 29, 32, 35, 40, 48,
    26, 27, 29, 32, 35, 40, 48, 58,
    26, 27, 29, 34, 38, 46, 56, 69,
    27, 29, 35, 38, 46, 56, 69, 83 };

class CMWDVQMEvaluator: public CMetricEvaluator {
private:
    float  fb1[64], fb2[64], impm[64];
    ImageSize sz8x8;
public:
    CMWDVQMEvaluator() {
        std::pair< std::string, std::pair<uint32_t, uint32_t> >   metric_pair;
        metric_pair.first = "MWDVQM"; metric_pair.second.first = MASK_MWDVQM; metric_pair.second.second = MASK_MWDVQM; metrics.push_back(metric_pair);
        sz8x8.width = sz8x8.height = 8;
        for(int32_t i=0; i<64; i++) impm[i] = 1.0f/(float)mpegmatrix[i];
    };
    ~CMWDVQMEvaluator(void) {};
    int32_t AllocateResourses(void) { return 0; };
    void ComputeMetrics(std::vector< double > &val, std::vector< double > &avg) {
        SImage      i1_p, i2_p;
        double      sum[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
        uint32_t    i, j = (int)val.size(), k, m;
        float       f1c, f2c, max_bdif, avg_bdif, bmean, bmax;

        for(i=0; i<m_num_planes; i++) {
            if(c_mask[i]&MASK_MWDVQM) {
                m_i1->GetFrame(i, &i1_p); m_i2->GetFrame(i, &i2_p);

                bmean = 0; bmax = 0;
                for(m = 0; m < (uint32_t)(i1_p.roi.height>>3); m++) {
                    for(k = 0; k < (uint32_t)(i1_p.roi.width>>3); k++) {
                        if (m_i1->GetBitDepth() == D008) {
                            ippiConvert_8u32f_C1R((uint8_t*)(i1_p.data+((m*i1_p.step+k)<<3)), i1_p.step, fb1, 8*sizeof(float), sz8x8);
                            ippiConvert_8u32f_C1R((uint8_t*)(i2_p.data+((m*i2_p.step+k)<<3)), i2_p.step, fb2, 8*sizeof(float), sz8x8);
                        } else {
                            ippiConvert_16u32f_C1R((uint16_t*)(i1_p.data+((m*i1_p.step+k)<<3)), i1_p.step, fb1, 8*sizeof(float), sz8x8);
                            ippiConvert_16u32f_C1R((uint16_t*)(i2_p.data+((m*i2_p.step+k)<<3)), i2_p.step, fb2, 8*sizeof(float), sz8x8);
                        }
                        ippiDCT8x8Fwd_32f_C1I(fb1); ippiDCT8x8Fwd_32f_C1I(fb2);
                        f1c = (fb1[0]>0.0f) ? powf((fb1[0] / 1024.0f), 0.65f) / fb1[0] : 1.0f;
                        f2c = (fb2[0]>0.0f) ? powf((fb2[0] / 1024.0f), 0.65f) / fb2[0] : 1.0f;
                        ippsMulC_32f_I(f1c,fb1,64); ippsMulC_32f_I(f2c,fb2,64); ippsSub_32f_I(fb2,fb1,64); ippsMul_32f_I(impm,fb1,64); ippsAbs_32f_I(fb1,64);
                        ippsMax_32f(fb1,64,&max_bdif); ippsMean_32f(fb1,64,&avg_bdif,ippAlgHintAccurate);
                        bmean += avg_bdif; bmax = std::max(bmax, max_bdif);
                    }
                }
                sum[i] = 50.0f * ((12800.0f * bmean / ((double)(i1_p.roi.height * i1_p.roi.width)))+ bmax);
                val.push_back(sum[i]); avg[j++] += sum[i];
            }
        }
        switch(get_chromaclass(m_i1->GetSqType())) {
            case C444:
                sum[m_num_planes] = (sum[0]+sum[1]+sum[2]+sum[3])/(double)m_num_planes; break;
            case C422:
                sum[3] = (2.0*sum[0]+sum[1]+sum[2])/4.0; break;
            case C420:
            default:
                sum[3] = (4.0*sum[0]+sum[1]+sum[2])/6.0; break;
        }
        if(c_mask[m_num_planes]&MASK_MWDVQM) { val.push_back(sum[m_num_planes]); avg[j++] += sum[m_num_planes]; }
    };
};

class CUQIEvaluator : public CMetricEvaluator {
private:
    Ipp8u * pBuf;
public:
    CUQIEvaluator() {
        std::pair< std::string, std::pair<unsigned int, unsigned int> >   metric_pair;
        metric_pair.first = "UQI"; metric_pair.second.first = MASK_UQI; metric_pair.second.second = MASK_UQI; metrics.push_back(metric_pair);
        pBuf = 0;
    };
    ~CUQIEvaluator(void) { ippsFree(pBuf); };
    int AllocateResourses(void) {
        SImage  i1_p;
        int     bsize;

        m_i1->GetFrame(0, &i1_p);

        if (m_i1->GetBitDepth() == D008) ippiQualityIndexGetBufferSize(ipp8u, ippC1, i1_p.roi, &bsize);
        else ippiQualityIndexGetBufferSize(ipp16u, ippC1, i1_p.roi, &bsize);

        pBuf = ippsMalloc_8u(bsize);

        return 0;
    };
    void ComputeMetrics(std::vector< double > &val, std::vector< double > &avg) {
        SImage  i1_p, i2_p;
        float   sum[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
        int     i, j = (int)val.size();

        for (i = 0; i<3; i++) {
            if (c_mask[i] & MASK_UQI) {
                m_i1->GetFrame(i, &i1_p); m_i2->GetFrame(i, &i2_p);
                if (m_i1->GetBitDepth() == D008) ippiQualityIndex_8u32f_C1R((Ipp8u*)i1_p.data, i1_p.step, (Ipp8u*)i2_p.data, i2_p.step, i1_p.roi, &(sum[i]), pBuf);
                else if (m_i1->GetBitDepth() == D010 || m_i1->GetBitDepth() == D012 || m_i1->GetBitDepth() == D016) ippiQualityIndex_16u32f_C1R((Ipp16u*)i1_p.data, i1_p.step, (Ipp16u*)i2_p.data, i2_p.step, i1_p.roi, &(sum[i]), pBuf);
                val.push_back((double)(sum[i])); avg[j++] += sum[i];
            }
        }

        switch (get_chromaclass(m_i1->GetSqType())) {
        case C444:
            sum[m_num_planes] = (sum[0] + sum[1] + sum[2] + sum[3]) / (float)m_num_planes; break;
        case C422:
            sum[3] = (2.0f*sum[0] + sum[1] + sum[2]) / 4.0f; break;
        case C420:
        default:
            sum[3] = (4.0f*sum[0] + sum[1] + sum[2]) / 6.0f; break;
        }
        if (c_mask[m_num_planes] & MASK_UQI) { val.push_back((double)(sum[m_num_planes])); avg[j++] += sum[m_num_planes]; }
    };
};
#endif

static const char *errors_table[] = {
    "ERROR: Unable to parse input metric specifications!",
    "ERROR: Empty metrics set!",
    "ERROR: Unable to open first sequence file!",
    "ERROR: Unable to open second sequence file!",
    "ERROR: Unspecified error during metric calculation!",
    "ERROR: Empty input file!",
    "WARNING: Incorrect selective frames parameters: not enough frames in YUVs!",
    "ERROR: Unsupported sequence type!",
    "ERROR: Unable to compare interlaced with progressive sequences!",
    "ERROR: Unable to compare sequences of different sizes on chromaticity channels!",
    "ERROR: Unable to compare RGB with YUV!\n",
    "ERROR: Unable to use parameters \"fs\" and \"numseekframe\" together!",
    "WARNING: Wrong seek ranges!",
    "ERROR: Failed to allocate memory!",
    "ERROR: Unsupported bit depth!"
};

int32_t usage(void)
{
    std::cout << "Usage:" << std::endl;
    std::cout << "metrics_calc_lite.exe <Options> <metric1> ... [<metricN>]... <plane1> ...[<planeN>] ..." << std::endl;
#if defined(NO_IPP) || defined(LEGACY_IPP)
    std::cout << "Possible metrics are: psnr, apsnr, ssim" << std::endl;
#else
    std::cout << "Possible metrics are: psnr, apsnr, ssim, mssim, artifacts, mwdvqm, uqi" << std::endl;
#endif
    std::cout << "Possible planes are: y, u, v, overall, all" << std::endl;
    std::cout << "Required options are:" << std::endl;
    std::cout << "    -i1 <filename> - name of first file to compare" << std::endl;
    std::cout << "    -i2 <filename> - name of second file to compare" << std::endl;
    std::cout << "    -w  <integer> - width of sequences pixels" << std::endl;
    std::cout << "    -h  <integer> - height of sequences pixels" << std::endl;
    std::cout << "Optional parameters are:" << std::endl;
    std::cout << "    -fs1 <i1> <i2> <i3> - calculate metric only for <i1> number of frames from 1st file starting with <i2>th sequence frame with step <i3>" << std::endl;
    std::cout << "    -fs2 <i1> <i2> <i3> - calculate metric only for <i1> number of frames from 2nd file starting with <i2>th sequence frame with step <i3>" << std::endl;
    std::cout << "    -fs <i1> <i2> <i3>  - calculate metric only for <i1> number of frames starting with <i2>th sequence frame with step <i3>" << std::endl;
    std::cout << "    -alpha              - calculate metrics for RGB alpha channel" << std::endl;
    std::cout << "    -numseekframe1 <from> <to> <num> - performs seeks to particular position in 1st file. FROM - position FROM, TO - seek position, NUM - number of iterations" << std::endl;
    std::cout << "    -numseekframe2 <from> <to> <num> - performs seeks to particular position in 2nd file. FROM - position FROM, TO - seek position, NUM - number of iterations" << std::endl;
    std::cout << "    -nopfm              - suppress per-frame metrics output" << std::endl;
    std::cout << "    -st type1 [type2]   - input sequences type (type1 for both sequences, type2 override type for second sequence)" << std::endl;
    std::cout << "                          4:2:0 types: i420p (default), i420i, yv12p, nv12p, yv12i, nv12i" << std::endl;
    std::cout << "                          4:2:2 types: yuy2p, yuy2i, nv16p, nv16i, i422p, i422i" << std::endl;
    std::cout << "                          4:4:4 types: ayuvp, ayuvi, y410p, y410i, y416p, y416i, i444p, i444i, i410p, i410i" << std::endl;
    std::cout << "                          RGB types  : rgb32p, rgb32i, a2rgb10p, a2rgb10i, argb16p" << std::endl;
    std::cout << "    -bd <integer>       - bit depth of sequences pixels" << std::endl;
    std::cout << "                          Possible values: 8, 10, 12, 16" << std::endl;
    std::cout << "    -rshift1 <integer>  - shift pixel values for <integer> bits to the right in first file" << std::endl;
    std::cout << "    -rshift2 <integer>  - shift pixel values for <integer> bits to the right in second file" << std::endl;
    std::cout << "    -btm_first          - bottom field first for interlaced sources" << std::endl;
    std::cout << "    -btm_first1         - bottom field first for the 1st source" << std::endl;
    std::cout << "    -btm_first2         - bottom field first for the 2nd source" << std::endl;
    std::cout << "NOTES:    1. Different chromaticity representations can be compared on Y channel only." << std::endl;
    std::cout << "          2. In case of 10 bits non-zero values must be located from bit #0 to bit #9." << std::endl;
    std::cout << "             If such bits are located from bit #6 to bit #15 use parameters \"-rshift1 6 -rshift2 6\"" << std::endl;
    std::cout << "Example: " << std::endl;
    std::cout << "    metrics_calc_lite.exe -i1 foreman.yuv -i2 x264_decoded.yuv -w 352 -h 288 psnr all ssim y" << std::endl;
    std::cout << "    metrics_calc_lite.exe -i1 foreman.yuv -i2 x264_decoded.yuv -w 352 -h 288 -nopfm -st i420p -fs 20 0 1 psnr y" << std::endl << std::endl;

    std::cout << "Wrong input parameters!!!" << std::endl;

    return -1;
}

int32_t parse_metrics(Component &cmps, int32_t argc, char** argv, int32_t curc)
{
    uint32_t cm;
    bool         not_metric, not_plane;

    while ( curc < argc ) {
        cm = 0; not_metric = true;
        while ( curc < argc ) {
            if      ( strcmp( argv[curc], "psnr" ) == 0      && curc + 1 < argc ) { cm |= MASK_PSNR; cm |= MASK_MSE; curc++; not_metric = false; }
            else if ( strcmp( argv[curc], "apsnr" ) == 0     && curc + 1 < argc ) { cm |= MASK_APSNR; cm |= MASK_MSE; curc++; not_metric = false; }
            else if ( strcmp( argv[curc], "ssim" ) == 0      && curc + 1 < argc ) { cm |= MASK_SSIM; curc++; not_metric = false; }
#if !defined(NO_IPP) && !defined(LEGACY_IPP)
            else if ( strcmp( argv[curc], "artifacts" ) == 0 && curc + 1 < argc ) { cm |= MASK_ARTIFACTS; curc++; not_metric = false; }
            else if ( strcmp( argv[curc], "mwdvqm" ) == 0    && curc + 1 < argc ) { cm |= MASK_MWDVQM; curc++; not_metric = false; }
            else if ( strcmp( argv[curc], "uqi" ) == 0       && curc + 1 < argc ) { cm |= MASK_UQI; curc++; not_metric = false; }
            else if ( strcmp( argv[curc], "mssim") == 0      && curc + 1 < argc ) { cm |= MASK_MSSIM; curc++; not_metric = false; }
#endif
            else break;
        }
        if (not_metric) return -1;

        not_plane = true;
        while ( curc < argc ) {
            if      ( *argv[curc] == tolower(cmps[0].first) ) { cmps[0].second |= cm; curc++; not_plane = false;}
            else if ( *argv[curc] == tolower(cmps[1].first) ) { cmps[1].second |= cm; curc++; not_plane = false;}
            else if ( *argv[curc] == tolower(cmps[2].first) ) { cmps[2].second |= cm; curc++; not_plane = false;}
            else if ( strcmp( argv[curc], "overall" ) == 0  ) { cmps[cmps.size()-1].second |= cm; curc++; not_plane = false;}
            else if ( strcmp( argv[curc], "all" ) == 0 ) { for(size_t i = 0; i < cmps.size(); i++) cmps[i].second |= cm; curc++; not_plane = false;}
            else break;
        }
        if (not_plane) return -1;
    }

    return 0;
}

void parse_fourcc(char* str, ESequenceType& sq_type, EBitDepth& bd)
{
    if     ( strcmp( str, "i420p" ) == 0 )    { sq_type = I420P; }
    else if( strcmp( str, "i420i" ) == 0 )    { sq_type = I420I; }
    else if( strcmp( str, "nv12p" ) == 0 )    { sq_type = NV12P; }
    else if( strcmp( str, "nv12i" ) == 0 )    { sq_type = NV12I; }
    else if( strcmp( str, "yv12p" ) == 0 )    { sq_type = YV12P; }
    else if( strcmp( str, "yv12i" ) == 0 )    { sq_type = YV12I; }
    else if( strcmp( str, "yuy2p" ) == 0 )    { sq_type = YUY2P; }
    else if( strcmp( str, "yuy2i" ) == 0 )    { sq_type = YUY2I; }
    else if( strcmp( str, "nv16p" ) == 0 )    { sq_type = NV16P; }
    else if( strcmp( str, "nv16i" ) == 0 )    { sq_type = NV16I; }
    else if( strcmp( str, "i422p" ) == 0 )    { sq_type = I422P; }
    else if( strcmp( str, "i422i" ) == 0 )    { sq_type = I422I; }
    else if( strcmp( str, "ayuvp" ) == 0 )    { sq_type = AYUVP; }
    else if( strcmp( str, "ayuvi" ) == 0 )    { sq_type = AYUVI; }
    else if( strcmp( str, "y410p" ) == 0 )    { sq_type = Y410P; bd = D010; }
    else if( strcmp( str, "y410i" ) == 0 )    { sq_type = Y410I; bd = D010; }
    else if( strcmp( str, "y416p" ) == 0)     { sq_type = Y416P; bd = D016; }
    else if( strcmp( str, "y416i" ) == 0)     { sq_type = Y416I; bd = D016; }
    else if( strcmp( str, "i444p" ) == 0 )    { sq_type = I444P; }
    else if( strcmp( str, "i444i" ) == 0 )    { sq_type = I444I; }
    else if( strcmp( str, "i410p" ) == 0 )    { sq_type = I410P; bd = D010; }
    else if( strcmp( str, "i410i" ) == 0 )    { sq_type = I410I; bd = D010; }
    else if( strcmp( str, "rgb32p" ) == 0 )   { sq_type = RGB32P; }
    else if( strcmp( str, "rgb32i" ) == 0 )   { sq_type = RGB32I; }
    else if( strcmp( str, "a2rgb10p" ) == 0 ) { sq_type = A2RGB10P; bd = D010; }
    else if( strcmp( str, "a2rgb10i" ) == 0 ) { sq_type = A2RGB10I; bd = D010; }
    else if (strcmp( str, "argb16p"  ) == 0)   { sq_type = ARGB16P; bd = D016; }
    else                                      { sq_type = UNKNOWN; }
}

int32_t main(int32_t argc, char** argv)
{
    Component     cmps; // Y,U,V,Overall or B,G,R,A,Overall
    int32_t       cur_param, w, h, i, j, order1, order2;
    int32_t       fm1_cntr, fm1_frst, fm1_step,
                  fm2_cntr, fm2_frst, fm2_step,
                  seek_from1, seek_to1, seek_num1,
                  seek_from2, seek_to2, seek_num2;
    std::string   input_name1, input_name2;
    bool          no_pfm, alpha_channel;
    ESequenceType sq1_type, sq2_type;
    EBitDepth     bd;
    uint32_t      rshift1, rshift2;

    bool is_fs_set = false;
    bool is_fs1_set = false;
    bool is_fs2_set = false;

    cur_param = 1; w = h = 0; sq1_type = sq2_type = I420P; bd = D008; no_pfm = false; alpha_channel = false; order1 = 0; order2 = 0; rshift1 = 0; rshift2 = 0;
    fm1_cntr = -1; fm1_frst = 0; fm1_step = 1;
    fm2_cntr = -1; fm2_frst = 0; fm2_step = 1;
    seek_num1 = 0; seek_from1 = -1; seek_to1 = -1;
    seek_num2 = 0; seek_from2 = -1; seek_to2 = -1;
    while ( cur_param < argc ) {
        if ( strcmp( argv[cur_param], "-i1" ) == 0 && cur_param + 1 < argc ) {
            input_name1 = argv[ cur_param + 1 ]; cur_param += 2;
        } else if ( strcmp( argv[cur_param], "-i2" ) == 0 && cur_param + 1 < argc ) {
            input_name2 = argv[ cur_param + 1 ]; cur_param += 2;
        } else if ( strcmp( argv[cur_param], "-w" ) == 0 && cur_param + 1 < argc ) {
            w = atoi(argv[ cur_param + 1 ]); cur_param += 2;
        } else if ( strcmp( argv[cur_param], "-h" ) == 0 && cur_param + 1 < argc ) {
            h = atoi(argv[ cur_param + 1 ]); cur_param += 2;
        } else if ( strcmp( argv[cur_param], "-rshift1" ) == 0 && cur_param + 1 < argc ) {
            rshift1 = atoi(argv[ cur_param + 1 ]); cur_param += 2;
        } else if ( strcmp( argv[cur_param], "-rshift2" ) == 0 && cur_param + 1 < argc ) {
            rshift2 = atoi(argv[ cur_param + 1 ]); cur_param += 2;
        } else if ( strcmp( argv[cur_param], "-fs" ) == 0 && cur_param + 3 < argc ) {
            is_fs_set = true;
            fm1_cntr = fm2_cntr = atoi(argv[ cur_param + 1 ]);
            fm1_frst = fm2_frst = atoi(argv[ cur_param + 2 ]);
            fm1_step = fm2_step = atoi(argv[ cur_param + 3 ]);
            cur_param += 4;
        } else if ( strcmp( argv[cur_param], "-fs1" ) == 0 && cur_param + 3 < argc ) {
            if (!is_fs_set) {
                is_fs1_set = true;
                fm1_cntr = atoi(argv[ cur_param + 1 ]);
                fm1_frst = atoi(argv[ cur_param + 2 ]);
                fm1_step = atoi(argv[ cur_param + 3 ]);
            }
            cur_param += 4;
        } else if ( strcmp( argv[cur_param], "-fs2" ) == 0 && cur_param + 3 < argc ) {
            if (!is_fs_set) {
                is_fs2_set = true;
                fm2_cntr = atoi(argv[ cur_param + 1 ]);
                fm2_frst = atoi(argv[ cur_param + 2 ]);
                fm2_step = atoi(argv[ cur_param + 3 ]);
            }
            cur_param += 4;
        } else if ( strcmp( argv[cur_param], "-nopfm" ) == 0 ) {
            no_pfm = true; cur_param += 1;
        } else if ( strcmp( argv[cur_param], "-alpha" ) == 0 ) {
            alpha_channel = true; cur_param += 1;
        } else if ( strcmp( argv[cur_param], "-btm_first" ) == 0 ) {
            order1 = 1; order2 = 1; cur_param += 1;
        } else if ( strcmp( argv[cur_param], "-btm_first1" ) == 0 ) {
            order1 = 1; cur_param += 1;
        } else if ( strcmp( argv[cur_param], "-btm_first2" ) == 0 ) {
            order2 = 1; cur_param += 1;
        } else if ( strcmp( argv[cur_param], "-numseekframe1" ) == 0 ) {
            if (!is_fs_set && !is_fs1_set && !is_fs2_set) {
                seek_from1 = atoi(argv[ cur_param + 1 ]);
                seek_to1   = atoi(argv[ cur_param + 2 ]);
                seek_num1  = atoi(argv[ cur_param + 3 ]);
                cur_param += 4;
            } else {
                std::cout << errors_table[11] << std::endl; return -11;
            }
        } else if ( strcmp( argv[cur_param], "-numseekframe2" ) == 0 ) {
            if (!is_fs_set && !is_fs1_set && !is_fs2_set) {
                seek_from2 = atoi(argv[ cur_param + 1 ]);
                seek_to2   = atoi(argv[ cur_param + 2 ]);
                seek_num2  = atoi(argv[ cur_param + 3 ]);
                cur_param += 4;
            } else {
                std::cout << errors_table[11] << std::endl; return -11; }
        } else if ( strcmp( argv[cur_param], "-bd" ) == 0 && cur_param + 1 < argc ) {
            if( strcmp( argv[cur_param + 1], "8" ) == 0 )       { bd = D008; cur_param += 2; }
            else if( strcmp( argv[cur_param + 1], "10" ) == 0 ) { bd = D010; cur_param += 2; }
            else if( strcmp( argv[cur_param + 1], "12" ) == 0 ) { bd = D012; cur_param += 2; }
            else if( strcmp( argv[cur_param + 1], "16" ) == 0 ) { bd = D016; cur_param += 2; }
            else { std::cout << errors_table[14] << std::endl; return -14;}
        } else if ( strcmp( argv[cur_param], "-st" ) == 0 && cur_param + 1 < argc ) {
            parse_fourcc(argv[cur_param + 1], sq1_type, bd);
            if (sq1_type == UNKNOWN) { std::cout << errors_table[7] << std::endl; return -7; }
            cur_param += 2;
            if(cur_param < argc) {
                parse_fourcc(argv[cur_param], sq2_type, bd);
                if (sq2_type == UNKNOWN) { sq2_type = sq1_type; }
                else                     { cur_param++; }
            }
            if(is_interlaced(sq1_type)!=is_interlaced(sq2_type)) {
                std::cout << errors_table[8] << std::endl; return -8;
            }
        } else break;
    }

    if( is_rgb(sq1_type) != is_rgb(sq2_type) ) {
        std::cout << errors_table[10] << std::endl; return -10;
    }

    CReader *reader1 = 0, *reader2 = 0;

    if( is_rgb(sq1_type) ) {
        reader1 = new CRGBReader();
        reader2 = new CRGBReader();
        INIT_RGB(cmps, alpha_channel);
    } else {
        reader1 = new CYUVReader();
        reader2 = new CYUVReader();
        INIT_YUV(cmps);
    }

    if ( input_name1.empty() || input_name2.empty() || w <= 0 || h <= 0 ) { return usage(); }

    int32_t err = parse_metrics(cmps, argc, argv, cur_param);
    if ( err == -1 ) { std::cout << errors_table[0] << std::endl; return -1; }

    char all_metrics_mask = 0;
    for (size_t i = 0; i < cmps.size(); i++) all_metrics_mask |= cmps[i].second;
    if ( !all_metrics_mask ) { std::cout << errors_table[1] << std::endl; return -2; };

    if (get_chromaclass(sq1_type) != get_chromaclass(sq2_type))
        if(cmps[1].second != 0 || cmps[2].second != 0) { std::cout << errors_table[9] << std::endl; return -9; };

    err = reader1->OpenReadFile( input_name1.c_str(), w, h, sq1_type, order1, bd, rshift1);
    if ( err == MCL_ERR_INVALID_PARAM ) { std::cout << errors_table[2] << std::endl; return -3; }
    if ( err == MCL_ERR_MEMORY_ALLOC )  { std::cout << errors_table[13] << std::endl; return -13; }

    err = reader2->OpenReadFile( input_name2.c_str(), w, h, sq2_type, order2, bd, rshift2);
    if ( err == MCL_ERR_INVALID_PARAM ) { std::cout << errors_table[3] << std::endl; return -4; }
    if ( err == MCL_ERR_MEMORY_ALLOC )  { std::cout << errors_table[13] << std::endl; return -13; }

    int32_t frames1 = reader1->GetFramesCount();
    int32_t frames2 = reader2->GetFramesCount();
    int32_t frames = (std::min)(frames1, frames2);

    if (frames == 0) { std::cout << errors_table[5] << std::endl; return -6; }
    if (fm1_frst >= frames1) { std::cout << errors_table[6] << std::endl; return 0; }
    if (fm2_frst >= frames2) { std::cout << errors_table[6] << std::endl; return 0; }

    if (fm1_cntr < 0) {
        fm1_cntr = frames;
    } else {
        int32_t last = (std::min)(fm1_frst + (fm1_cntr-1)*fm1_step + 1, frames1);
        fm1_cntr = (last - fm1_frst - 1)/fm1_step + 1;
    }

    if (fm2_cntr < 0) {
        fm2_cntr = frames;
    } else {
        int32_t last = (std::min)(fm2_frst + (fm2_cntr-1)*fm2_step + 1, frames2);
        fm2_cntr = (last - fm2_frst - 1)/fm2_step + 1;
    }

    if (seek_num1 > 0) {
        if((seek_from1 < 1 || seek_from1 > reader1->GetFramesCount()) || (seek_to1 < 0 || seek_to1 >= reader1->GetFramesCount())) {
            seek_num1 = 0;
            std::cout << errors_table[12] << std::endl;
        }
    }
    if (seek_num2 > 0) {
        if((seek_from2 < 1 || seek_from2 > reader2->GetFramesCount()) || (seek_to2 < 0 || seek_to2 >= reader2->GetFramesCount())) {
            seek_num2 = 0;
            std::cout << errors_table[12] << std::endl;
        }
    }

    if (seek_num1 > 0) {
        fm1_cntr = (seek_from1 - seek_to1) * (seek_num1 + 1);
        fm1_frst = seek_to1;
        if (seek_num2 == 0) {
            fm2_cntr = reader2->GetFramesCount();
        }
    }
    if (seek_num2 > 0) {
        fm2_cntr = (seek_from2 - seek_to2) * (seek_num2 + 1);
        fm2_frst = seek_to2;
        if (seek_num1 == 0) {
            fm1_cntr = reader1->GetFramesCount();
        }
    }

    const int32_t fm_count = std::min(fm1_cntr, fm2_cntr);

#if !defined(NO_IPP)
    ippInit();
#endif

    std::vector < std::string >            metric_names;            // metric names
    std::vector < bool >                   out_flags;
    std::vector < double >                 avg_values;              // average per-sequence metric data
    std::vector < std::vector < double > > all_values ( fm_count ); // perframe metric data
    uint32_t                               all_metrics;
    double                                 norm;

    std::vector< CMetricEvaluator* > mevs;

    all_metrics = cmps[0].second | cmps[1].second | cmps[2].second | cmps[3].second;

    if (all_metrics & (MASK_PSNR|MASK_APSNR))
        mevs.push_back( new CPSNREvaluator() );

#if defined(NO_IPP) || defined(LEGACY_IPP)
    if (all_metrics & (MASK_SSIM))
        mevs.push_back( new CSSIMEvaluator() );
#else
    if (all_metrics & (MASK_SSIM | MASK_MSSIM | MASK_ARTIFACTS))
        mevs.push_back(new CMSSIMEvaluator());

    if (all_metrics & MASK_MWDVQM)
        mevs.push_back( new CMWDVQMEvaluator() );

    if (all_metrics & MASK_UQI)
        mevs.push_back( new CUQIEvaluator() );
#endif

    for (i = 0; i < (int)mevs.size(); i++) {
        mevs[i]->InitFrameParams(reader1, reader2);
        mevs[i]->InitComputationParams(cmps, metric_names, out_flags, avg_values);
        err = mevs[i]->AllocateResourses();
        if ( err == -2 ) { std::cout << errors_table[13] << std::endl; return -13; }
    }

    for (i = 0; i < fm_count; i++, fm1_frst+=fm1_step, fm2_frst+=fm2_step) {
        if(fm1_frst == seek_from1) { fm1_frst = seek_to1; }
        if(fm2_frst == seek_from2) { fm2_frst = seek_to2; }
        reader1->ReadRawFrame(fm1_frst); reader2->ReadRawFrame(fm2_frst);
        for (j = 0; j < (int)mevs.size(); j++) mevs[j]->ComputeMetrics(all_values[i],avg_values);
    }

    for (i = 0; i < (int)mevs.size(); i++) delete mevs[i];
    delete reader1;
    delete reader2;

    for (i = 0; i < (int)metric_names.size(); i++) {
        if(metric_names[i].find("MSE")!=std::string::npos) {
            out_flags[i]=false;
        }
    }

    /* Output metrics to stdout */
    if(!no_pfm) {
        for (i = 0; i < (int)metric_names.size(); i++) {
            if(!out_flags[i] || metric_names[i].find("APSNR")!=std::string::npos) continue;
            std::cout << "<pfr_metric=" << metric_names[i] << ">" << std::flush;
            for (j = 0; j < fm_count; j++) {
                std::cout << " " << std::setw(8) << std::setprecision(5) << std::setiosflags(std::ios::fixed) << all_values[j][i];
            }
            std::cout << "</pfr_metric>"<< std::endl;
        }
    }

    /* Update average metric values and output metrics to stdout */
    norm = 1.0 / fm_count;
    for (i = 0; i < (int32_t)avg_values.size(); i++) {
        avg_values[i] *= norm;
        if(metric_names[i].find("PSNR")!=std::string::npos && metric_names[i].find("APSNR")==std::string::npos) avg_values[i] = MSEToPSNR(avg_values[i], MaxError(bd));
    }

    for (i = 0; i < (int32_t)avg_values.size(); i++) {
        if(!out_flags[i]) continue;
        std::cout << "<avg_metric=" << metric_names[i] << ">" << std::flush;
        std::cout << " " << std::setw(8) << std::setprecision(5) << std::setiosflags(std::ios::fixed) << avg_values[i];
        std::cout << "</avg_metric>"<< std::endl;
    }
    return 0;
}
