// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include "../include/librealsense2/hpp/rs_sensor.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"
#include "source.h"
#include "option.h"
#include "environment.h"
#include "context.h"
#include "proc/synthetic-stream.h"
#include "proc/temporal-filter.h"

namespace librealsense
{
#if _ALTEK_TF_
    const size_t PERSISTENCE_MAP_NUM = 9;

    // The persistence parameter/holes filling mode
    const uint8_t persistence_min = 0;
    const uint8_t persistence_max = PERSISTENCE_MAP_NUM - 1;
    const uint8_t persistence_default = 5;  // Credible if one of the last two frames are valid at this pixel
    const uint8_t persistence_step = 1;

    //alpha -  the weight with default value 0.4, between 1 and 0 -- 1 means 100% weight from the current pixel
    const float temp_alpha_min = 0.f;
    const float temp_alpha_max = 1.f;
    const float temp_alpha_default = 0.7f;
    const float temp_alpha_step = 0.01f;

    //delta -  the filter threshold for edge classification and preserving ,[0-5] disparity. 
    const float temp_delta_min = 0.f;
    const float temp_delta_max = 5.f;
    const float temp_delta_default = 0.50f;
    const float temp_delta_step = 0.01f;

    temporal_filter::temporal_filter() :
        depth_processing_block("Temporal Filter"),
        _persistence_param(persistence_default),
        _alpha_param(temp_alpha_default),
        _one_minus_alpha(1 - _alpha_param),
        _delta_param(temp_delta_default),
        _width(0), _height(0), _stride(0), _bpp(0),
        _extension_type(RS2_EXTENSION_DEPTH_FRAME),
        _current_frm_size_pixels(0),
         _temppral_delta_LUT_buffer_init_flag(0),
        _temppral_delta_LUT_value_init_flag(0)
    {
        _stream_filter.stream = RS2_STREAM_DEPTH;
        _stream_filter.format = RS2_FORMAT_Z16;

        auto temporal_persistence_control = std::make_shared<ptr_option<uint8_t>>(
            persistence_min,
            persistence_max,
            persistence_step,
            persistence_default,
            &_persistence_param, "Persistency mode");

        temporal_persistence_control->set_description(0, "Disabled");
        temporal_persistence_control->set_description(1, "Valid in 8/8");
        temporal_persistence_control->set_description(2, "Valid in 2/last 3");
        temporal_persistence_control->set_description(3, "Valid in 2/last 4");
        temporal_persistence_control->set_description(4, "Valid in 2/8");
        temporal_persistence_control->set_description(5, "Valid in 1/last 2");
        temporal_persistence_control->set_description(6, "Valid in 1/last 5");
        temporal_persistence_control->set_description(7, "Valid in 1/8");
        temporal_persistence_control->set_description(8, "Always on");

        temporal_persistence_control->on_set([this, temporal_persistence_control](float val)
            {
                if (!temporal_persistence_control->is_valid(val))
                    throw invalid_value_exception(to_string()
                        << "Unsupported temporal persistence param "
                        << (int)val << " is out of range.");

                on_set_persistence_control(static_cast<uint8_t>(val));
            });

        register_option(RS2_OPTION_HOLES_FILL, temporal_persistence_control);

        auto temporal_filter_alpha = std::make_shared<ptr_option<float>>(
            temp_alpha_min,
            temp_alpha_max,
            temp_alpha_step,
            temp_alpha_default,
            &_alpha_param, "Alpha factor of Exp. moving average, 1=no filter, 0=infinite filter");
        temporal_filter_alpha->on_set([this](float val)
            {
                on_set_alpha(val);
            });

        auto temporal_filter_delta = std::make_shared<ptr_option<float>>(
            temp_delta_min,
            temp_delta_max,
            temp_delta_step,
            temp_delta_default,
            &_delta_param, "Edge-preserving (gradient) threshold(sub-pixel disparity)");
        temporal_filter_delta->on_set([this, temporal_filter_delta](float val)
            {
                /*
                if (!temporal_filter_delta->is_valid(val))
                    throw invalid_value_exception(to_string()
                        << "Unsupported temporal delta: " << val << " is out of range.");
                */
                on_set_delta(val);
                _delta_param = static_cast<float>(val);
                _temppral_delta_LUT_value_init_flag = 0;
            });

        register_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, temporal_filter_alpha);
        register_option(RS2_OPTION_FILTER_SMOOTH_DELTA, temporal_filter_delta);

        on_set_persistence_control(_persistence_param);
        on_set_delta(_delta_param);
        on_set_alpha(_alpha_param);
#else
    const size_t PERSISTENCE_MAP_NUM = 9;

    // The persistence parameter/holes filling mode
    const uint8_t persistence_min = 0;
    const uint8_t persistence_max = PERSISTENCE_MAP_NUM - 1;
    const uint8_t persistence_default = 3;  // Credible if two of the last four frames are valid at this pixel
    const uint8_t persistence_step = 1;

    //alpha -  the weight with default value 0.4, between 1 and 0 -- 1 means 100% weight from the current pixel
    const float temp_alpha_min = 0.f;
    const float temp_alpha_max = 1.f;
    const float temp_alpha_default = 0.4f;
    const float temp_alpha_step = 0.01f;

    //delta -  the filter threshold for edge classification and preserving with default value of 20 depth increments
    const uint8_t temp_delta_min = 1;
    const uint8_t temp_delta_max = 100;
    const uint8_t temp_delta_default = 20;
    const uint8_t temp_delta_step = 1;

    temporal_filter::temporal_filter() :
        depth_processing_block("Temporal Filter"),
        _persistence_param(persistence_default),
        _alpha_param(temp_alpha_default),
        _one_minus_alpha(1 - _alpha_param),
        _delta_param(temp_delta_default),
        _width(0), _height(0), _stride(0), _bpp(0),
        _extension_type(RS2_EXTENSION_DEPTH_FRAME),
        _current_frm_size_pixels(0)
    {
        _stream_filter.stream = RS2_STREAM_DEPTH;
        _stream_filter.format = RS2_FORMAT_Z16;

        auto temporal_persistence_control = std::make_shared<ptr_option<uint8_t>>(
            persistence_min,
            persistence_max,
            persistence_step,
            persistence_default,
            &_persistence_param, "Persistency mode");

        temporal_persistence_control->set_description(0, "Disabled");
        temporal_persistence_control->set_description(1, "Valid in 8/8");
        temporal_persistence_control->set_description(2, "Valid in 2/last 3");
        temporal_persistence_control->set_description(3, "Valid in 2/last 4");
        temporal_persistence_control->set_description(4, "Valid in 2/8");
        temporal_persistence_control->set_description(5, "Valid in 1/last 2");
        temporal_persistence_control->set_description(6, "Valid in 1/last 5");
        temporal_persistence_control->set_description(7, "Valid in 1/8");
        temporal_persistence_control->set_description(8, "Always on");

        temporal_persistence_control->on_set([this, temporal_persistence_control](float val)
            {
                if (!temporal_persistence_control->is_valid(val))
                    throw invalid_value_exception(to_string()
                        << "Unsupported temporal persistence param "
                        << (int)val << " is out of range.");

                on_set_persistence_control(static_cast<uint8_t>(val));
            });

        register_option(RS2_OPTION_HOLES_FILL, temporal_persistence_control);

        auto temporal_filter_alpha = std::make_shared<ptr_option<float>>(
            temp_alpha_min,
            temp_alpha_max,
            temp_alpha_step,
            temp_alpha_default,
            &_alpha_param, "Alpha factor of Exp. moving average, 1=no filter, 0=infinite filter");
        temporal_filter_alpha->on_set([this](float val)
            {
                on_set_alpha(val);
            });

        auto temporal_filter_delta = std::make_shared<ptr_option<uint8_t>>(
            temp_delta_min,
            temp_delta_max,
            temp_delta_step,
            temp_delta_default,
            &_delta_param, "Edge-preserving (gradient) threshold");
        temporal_filter_delta->on_set([this, temporal_filter_delta](float val)
            {
                if (!temporal_filter_delta->is_valid(val))
                    throw invalid_value_exception(to_string()
                        << "Unsupported temporal delta: " << val << " is out of range.");

                on_set_delta(val);
            });

        register_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, temporal_filter_alpha);
        register_option(RS2_OPTION_FILTER_SMOOTH_DELTA, temporal_filter_delta);

        on_set_persistence_control(_persistence_param);
        on_set_delta(_delta_param);
        on_set_alpha(_alpha_param);
#endif    
    }

    rs2::frame temporal_filter::process_frame(const rs2::frame_source& source, const rs2::frame& f)
    {
        update_configuration(f);
        auto tgt = prepare_target_frame(f, source);

#if _ALTEK_TF_
        if (_extension_type == RS2_EXTENSION_DEPTH_FRAME)
        {
            temp_jw_smooth<uint16_t>(const_cast<void*>(tgt.get_data()), _last_frame.data(), _history.data());
        }
#else
        // Temporal filter execution
        if (_extension_type == RS2_EXTENSION_DISPARITY_FRAME)
            temp_jw_smooth<float>(const_cast<void*>(tgt.get_data()), _last_frame.data(), _history.data());
        else
            temp_jw_smooth<uint16_t>(const_cast<void*>(tgt.get_data()), _last_frame.data(), _history.data());
#endif
        return tgt;
    }


    void temporal_filter::on_set_persistence_control(uint8_t val)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _persistence_param = val;
        recalc_persistence_map();
        _last_frame.clear();
        _history.clear();
    }

    void temporal_filter::on_set_alpha(float val)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _alpha_param = val;
        _one_minus_alpha = 1.f - _alpha_param;
        _cur_frame_index = 0;
        _last_frame.clear();
        _history.clear();
    }

    void temporal_filter::on_set_delta(float val)
    {
        std::lock_guard<std::mutex> lock(_mutex);
#ifdef _ALTEK_TF_
        _delta_param = static_cast<float>(val);
        _temppral_delta_LUT_value_init_flag = 0;
#else
        _delta_param = static_cast<uint8_t>(val);
#endif
        _cur_frame_index = 0;
        _last_frame.clear();
        _history.clear();
    }

    void  temporal_filter::update_configuration(const rs2::frame& f)
    {
        if (f.get_profile().get() != _source_stream_profile.get())
        {
            _source_stream_profile = f.get_profile();
            _target_stream_profile = _source_stream_profile.clone(RS2_STREAM_DEPTH, 0, _source_stream_profile.format());

            //TODO - reject any frame other than depth/disparity
            _extension_type = f.is<rs2::disparity_frame>() ? RS2_EXTENSION_DISPARITY_FRAME : RS2_EXTENSION_DEPTH_FRAME;
            _bpp = (_extension_type == RS2_EXTENSION_DISPARITY_FRAME) ? sizeof(float) : sizeof(uint16_t);
            auto vp = _target_stream_profile.as<rs2::video_stream_profile>();
#ifdef _ALTEK_TF_
            _focal_lenght_mm = vp.get_intrinsics().fx;
#endif
            _width = vp.width();
            _height = vp.height();
            _stride = _width*_bpp;
            _current_frm_size_pixels = _width * _height;

            _last_frame.clear();
            _last_frame.resize(_current_frm_size_pixels*_bpp);

            _history.clear();
            _history.resize(_current_frm_size_pixels*_bpp);
        }

#ifdef _ALTEK_TF_
        // Check if the new frame originated from stereo-based depth sensor
        // retrieve the stereo baseline parameter
        // TODO refactor disparity parameters into the frame's metadata
        auto snr = ((frame_interface*)f.get())->get_sensor().get();
        librealsense::depth_stereo_sensor* dss;
        bool _stereoscopic_depth = Is<librealsense::depth_stereo_sensor>(snr);
        dss = As<librealsense::depth_stereo_sensor>(snr);
        if (_stereoscopic_depth)
            _stereo_baseline_mm = dss->get_stereo_baseline_mm();

        //------ LUT buffer malloc ---------------------------------------
        if (_temppral_delta_LUT_buffer_init_flag == 0)
        {
            _temppral_delta_LUT = (uint16_t*)malloc(sizeof(uint16_t) * 65536);
            _temppral_delta_LUT_buffer_init_flag = 1;
        }
        //------ LUT assign value ----------------------------------------
        if (_temppral_delta_LUT_value_init_flag == 0)
        {
            _altek_tf_delta_init(_temppral_delta_LUT);
            _temppral_delta_LUT_value_init_flag = 1;
        }
#endif
    }

#ifdef _ALTEK_TF_
    void  temporal_filter::_altek_tf_delta_init(uint16_t* temppral_delta_LUT)
    {
        float tfb = _focal_lenght_mm * _stereo_baseline_mm;

        float delta_hf = _delta_param / 2;
        int _max_dist = static_cast<int>(tfb / (_ALTEK_TF_DELTA_MAX_ * 1.0f) + 0.5);
        if (_max_dist < 65535)_max_dist = 65534;
        for (int i = _ALTEK_TF_MIN_DIST_; i <= _max_dist; i++)
        {
            float tdisp = tfb / (i * 1.0f);
            temppral_delta_LUT[i] = static_cast<uint16_t>(0.5 + tfb / (tdisp - delta_hf) - tfb / (tdisp + delta_hf));
        }
        for (int i = 0; i < _ALTEK_TF_MIN_DIST_; i++)
        {
            temppral_delta_LUT[i] = temppral_delta_LUT[_ALTEK_TF_MIN_DIST_];
        }
        for (int i = _max_dist + 1; i < 65536; i++)
        {
            temppral_delta_LUT[i] = temppral_delta_LUT[_max_dist];
        }
    }
#endif

    rs2::frame temporal_filter::prepare_target_frame(const rs2::frame& f, const rs2::frame_source& source)
    {
        // Allocate and copy the content of the original Depth data to the target
        rs2::frame tgt = source.allocate_video_frame(_target_stream_profile, f, (int)_bpp, (int)_width, (int)_height, (int)_stride, _extension_type);

        memmove(const_cast<void*>(tgt.get_data()), f.get_data(), _current_frm_size_pixels * _bpp);
        return tgt;
    }

    void temporal_filter::recalc_persistence_map()
    {
        _persistence_map.fill(0);

        for (size_t i = 0; i < _persistence_map.size(); i++)
        {
            unsigned char last_7 = !!(i & 1);  // old
            unsigned char last_6 = !!(i & 2);
            unsigned char last_5 = !!(i & 4);
            unsigned char last_4 = !!(i & 8);
            unsigned char last_3 = !!(i & 16);
            unsigned char last_2 = !!(i & 32);
            unsigned char last_1 = !!(i & 64);
            unsigned char lastFrame = !!(i & 128); // new

            if (_persistence_param == 1)
            {
                int sum = lastFrame + last_1 + last_2 + last_3 + last_4 + last_5 + last_6 + last_7;
                if (sum >= 8)  // valid in eight of the last eight frames
                    _persistence_map[i] = 1;
            }
            else if (_persistence_param == 2) // <--- default choice in current libRS implementation
            {
                int sum = lastFrame + last_1 + last_2;
                if (sum >= 2) // valid in two of the last three frames
                    _persistence_map[i] = 1;
            }
            else if (_persistence_param == 3) // <--- default choice recommended
            {
                int sum = lastFrame + last_1 + last_2 + last_3;
                if (sum >= 2)  // valid in two of the last four frames
                    _persistence_map[i] = 1;
            }
            else if (_persistence_param == 4)
            {
                int sum = lastFrame + last_1 + last_2 + last_3 + last_4 + last_5 + last_6 + last_7;
                if (sum >= 2) // valid in two of the last eight frames
                    _persistence_map[i] = 1;
            }
            else if (_persistence_param == 5)
            {
                int sum = lastFrame + last_1;
                if (sum >= 1) // valid in one of the last two frames
                    _persistence_map[i] = 1;
            }
            else if (_persistence_param == 6)
            {
                int sum = lastFrame + last_1 + last_2 + last_3 + last_4;
                if (sum >= 1)  // valid in one of the last five frames
                    _persistence_map[i] = 1;
            }
            else if (_persistence_param == 7) //  <--- most filling
            {
                int sum = lastFrame + last_1 + last_2 + last_3 + last_4 + last_5 + last_6 + last_7;
                if (sum >= 1) // valid in one of the last eight frames
                    _persistence_map[i] = 1;
            }
            else if (_persistence_param == 8) //  <--- all 1's
            {
                _persistence_map[i] = 1;
            }
            else // all others, including 0, no persistance
            {
            }
        }

        // Convert to credible enough
        std::array<uint8_t, PRESISTENCY_LUT_SIZE> credible_threshold;
        credible_threshold.fill(0);

        for (auto phase = 0; phase < 8; phase++)
        {
            // evaluating last phase
            //int ephase = (phase + 7) % 8;
            unsigned char mask = 1 << phase;
            int i;

            for (i = 0; i < 256; i++) {
                unsigned char pos = (unsigned char)((i << (8 - phase)) | (i >> phase));
                if (_persistence_map[pos])
                    credible_threshold[i] |= mask;
            }
        }
        // Store results
        _persistence_map = credible_threshold;
    }
}
