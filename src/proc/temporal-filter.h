// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once
#include "types.h"

#define _ALTEK_TF_ 1
#define _ALTEK_TF_VERSION_ V1.0

namespace librealsense
{
    const size_t PRESISTENCY_LUT_SIZE = 256;

    class temporal_filter : public depth_processing_block
    {
    public:
        temporal_filter();

    protected:

#if _ALTEK_TF_
 //----------ALTEK Temporal Filter Function define----------------------- 
     //define min kit distance estimation
    #define _ALTEK_TF_MIN_DIST_ 100     
    //define max Mean Difference Check Threshold (for disparity)
    #define _ALTEK_TF_DELTA_MAX_ 5   
     //sub function, initial Temporal Filter IIR Delta LUT
     void _altek_tf_delta_init(uint16_t* _temppral_delta_LUT);
//---------------------------------
#endif

        void    update_configuration(const rs2::frame& f);
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;

        rs2::frame prepare_target_frame(const rs2::frame& f, const rs2::frame_source& source);

        template<typename T>
        void temp_jw_smooth(void* frame_data, void * _last_frame_data, uint8_t *history)
        {
            static_assert((std::is_arithmetic<T>::value), "temporal filter assumes numeric types");

            const bool fp = (std::is_floating_point<T>::value);

#if !_ALTEK_TF_
            T delta_z = static_cast<T>(_delta_param);
#endif 

            auto frame          = reinterpret_cast<T*>(frame_data);
            auto _last_frame    = reinterpret_cast<T*>(_last_frame_data);

            unsigned char mask = 1 << _cur_frame_index;

            // pass one -- go through image and update all
            for (size_t i = 0; i < _current_frm_size_pixels; i++)
            {
                T cur_val = frame[i];
                T prev_val = _last_frame[i];

                if (cur_val)
                {
                    if (!prev_val)
                    {
                        _last_frame[i] = cur_val;
                        history[i] = mask;
                    }
                    else
                    {  // old and new val
                        T diff = static_cast<T>(fabs(cur_val - prev_val));

#if _ALTEK_TF_
                        T delta_z = static_cast<T>(_temppral_delta_LUT[uint16_t(cur_val)]);
#endif

                        if (diff < delta_z)
                        {  // old and new val agree
                            history[i] |= mask;
                            float filtered = _alpha_param * cur_val + _one_minus_alpha * prev_val;
                            T result = static_cast<T>(filtered);
                            frame[i] = result;
                            _last_frame[i] = result;
                        }
                        else
                        {
                            _last_frame[i] = cur_val;
                            history[i] = mask;
                        }
                    }
                }
                else
                {  // no cur_val
                    if (prev_val)
                    { // only case we can help
                        unsigned char hist = history[i];
                        unsigned char classification = _persistence_map[hist];
                        if (classification & mask)
                        { // we have had enough samples lately
                            frame[i] = prev_val;
                        }
                    }
                    history[i] &= ~mask;
                }
            }

            _cur_frame_index = (_cur_frame_index + 1) % 8;  // at end of cycle
        }

    private:
        void on_set_persistence_control(uint8_t val);
        void on_set_alpha(float val);
        void on_set_delta(float val);

        void recalc_persistence_map();
        uint8_t                 _persistence_param;

        float                   _alpha_param;               // The normalized weight of the current pixel
        float                   _one_minus_alpha;
#if _ALTEK_TF_
        float                 _delta_param;               // A threshold when a filter is invoked
#else
        uint8_t                 _delta_param;               // A threshold when a filter is invoked
#endif
        size_t                  _width, _height, _stride;
        size_t                  _bpp;
        rs2_extension           _extension_type;            // Strictly Depth/Disparity
        size_t                  _current_frm_size_pixels;
        rs2::stream_profile     _source_stream_profile;
        rs2::stream_profile     _target_stream_profile;
        std::vector<uint8_t>    _last_frame;                // Hold the last frame received for the current profile
        std::vector<uint8_t>    _history;                   // represents the history over the last 8 frames, 1 bit per frame
        uint8_t                 _cur_frame_index;
#if _ALTEK_TF_
         float                   _focal_lenght_mm;
        float                   _stereo_baseline_mm;
        int32_t      _temppral_delta_LUT_buffer_init_flag;        //if value=0, then malloc for Temporal FIlter IIR delta LUT buffer.  if value=1, buffer ready.
        int32_t      _temppral_delta_LUT_value_init_flag;         //if value=0, then set new value to Temporal FIlter IIR delta LUT.  if value=1, Temporal FIlter IIR delta LUT is ok.
        uint16_t* _temppral_delta_LUT;                                        //Temporal FIlter IIR delta LUT, need malloc memory (buffer size is 65536*sizeof(uint16)).
#endif        
        // encodes whether a particular 8 bit history is good enough for all 8 phases of storage
        std::array<uint8_t, PRESISTENCY_LUT_SIZE> _persistence_map;
    };
    MAP_EXTENSION(RS2_EXTENSION_TEMPORAL_FILTER, librealsense::temporal_filter);
}
