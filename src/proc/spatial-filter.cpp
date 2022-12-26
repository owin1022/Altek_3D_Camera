// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include "../include/librealsense2/hpp/rs_sensor.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"
#include "option.h"
#include "environment.h"
#include "context.h"
#include "software-device.h"
#include "proc/synthetic-stream.h"
#include "proc/hole-filling-filter.h"
#include "proc/spatial-filter.h"

namespace librealsense
{
    enum spatial_holes_filling_types : uint8_t
    {
        sp_hf_disabled,
        sp_hf_2_pixel_radius,
        sp_hf_4_pixel_radius,
        sp_hf_8_pixel_radius,
        sp_hf_16_pixel_radius,
        sp_hf_unlimited_radius,
        sp_hf_max_value
    };

#if _ALTEK_SF_
	//Density Check Threshold: value[0-1], means 0%~100%
	const float alpha_min_val = 0.00f;
	const float alpha_max_val = 1.00f;
	const float alpha_default_val = 0.25f;
	const float alpha_step = 0.01f;
	//Mean Difference Check Threshold: value[0-max value], disparity level
	const float delta_min_val = 0.0f;
	const float delta_max_val = _ALTEK_SF_DELTA_MAX_*1.0f;
	const float delta_default_val = 0.38f;
	const float delta_step = 0.01f; //0.01f;
	//Depth Smooth Ratio [0-1], [smooth low - smooth high]
	const float filter_iter_min = 0.0f;
	const float filter_iter_max = 1.0f;
	const float filter_iter_def = 0.85f;
	const float filter_iter_step = 0.01f;
	//Mask Size: value[0-5], mask size 5x5-7x7 + 7x7-13x13
	const uint8_t holes_fill_min = 0;
	const uint8_t holes_fill_max = 5;
	const uint8_t holes_fill_step = 1;
	const uint8_t holes_fill_def = 4;
#else
	// The weight of the current pixel for smoothing is bounded within [25..100]%
	const float alpha_min_val = 0.25f;
	const float alpha_max_val = 1.f;
	const float alpha_default_val = 0.5f;
	const float alpha_step = 0.01f;

	// The depth gradient below which the smoothing will occur as number of depth levels
	const uint8_t delta_min_val = 1;
	const uint8_t delta_max_val = 50;
	const uint8_t delta_default_val = 20;
	const uint8_t delta_step = 1;

	// the number of passes used in the iterative smoothing approach
	const uint8_t filter_iter_min = 1;
	const uint8_t filter_iter_max = 5;
	const uint8_t filter_iter_def = 2;
	const uint8_t filter_iter_step = 1;

	// The holes filling mode
	const uint8_t holes_fill_min = sp_hf_disabled;
	const uint8_t holes_fill_max = sp_hf_max_value - 1;
	const uint8_t holes_fill_step = 1;
	const uint8_t holes_fill_def = sp_hf_disabled;
#endif

#if _ALTEK_SF_
    spatial_filter::spatial_filter() :
        depth_processing_block("Altek Spatial Filter"),
        _spatial_alpha_param(alpha_default_val),
        _spatial_delta_param(delta_default_val),
        _spatial_iterations(filter_iter_def),
        _width(0), _height(0), _stride(0), _bpp(0),
        _extension_type(RS2_EXTENSION_DEPTH_FRAME),
        _current_frm_size_pixels(0),
        _stereoscopic_depth(false),
        _focal_lenght_mm(0.f),
        _stereo_baseline_mm(0.f),
        _holes_filling_mode(holes_fill_def),
        _holes_filling_radius(0),
		_spatial_delta_LUT_buffer_init_flag(0),
		_spatial_delta_LUT_value_init_flag(0),
		_spatial_integralimage_buffer_init_flag(0),
		_spatial_integralimage_size_pixels(0),
		_spatial_integralimage_mask_mode(-1)
    {
        _stream_filter.stream = RS2_STREAM_DEPTH;
        _stream_filter.format = RS2_FORMAT_Z16;
		
		auto spatial_filter_alpha = std::make_shared<ptr_option<float>>(
			alpha_min_val,
			alpha_max_val,
			alpha_step,
			alpha_default_val,
			&_spatial_alpha_param, "Alpha is Density threshold");

		auto spatial_filter_delta = std::make_shared<ptr_option<float>>(
			delta_min_val,
			delta_max_val,
			delta_step,
			delta_default_val,
			&_spatial_delta_param, "Delta is Distance threshold(sub-pixel)");
		spatial_filter_delta->on_set([this, spatial_filter_delta](float val)
		{
			std::lock_guard<std::mutex> lock(_mutex);

			/*
			if (!spatial_filter_delta->is_valid(val))
				throw invalid_value_exception(to_string()
					<< "Unsupported spatial distance threshold: " << val << " is out of range.");
			*/
			_spatial_delta_param = static_cast<float>(val);
			_spatial_edge_threshold = float(_spatial_delta_param);
			_spatial_delta_LUT_value_init_flag = 0;
		});

		auto holes_filling_mode = std::make_shared<ptr_option<uint8_t>>(
			holes_fill_min,
			holes_fill_max,
			holes_fill_step,
			holes_fill_def,
			&_holes_filling_mode, "Filter Size");
		holes_filling_mode->set_description(sp_hf_disabled, "less + normal ");
		holes_filling_mode->set_description(sp_hf_2_pixel_radius, "less + high ");
		holes_filling_mode->set_description(sp_hf_4_pixel_radius, "normal + lower ");
		holes_filling_mode->set_description(sp_hf_8_pixel_radius, "normal + low ");
		holes_filling_mode->set_description(sp_hf_16_pixel_radius, "normal + normal ");
		holes_filling_mode->set_description(sp_hf_unlimited_radius, "normal + high ");

		auto spatial_filter_iterations = std::make_shared<ptr_option<float>>(
			filter_iter_min,
			filter_iter_max,
			filter_iter_step,
			filter_iter_def,
			&_spatial_iterations, "Depth Smooth Ratio");

		register_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, spatial_filter_alpha);
		register_option(RS2_OPTION_FILTER_SMOOTH_DELTA, spatial_filter_delta);
		register_option(RS2_OPTION_FILTER_MAGNITUDE, spatial_filter_iterations);
		register_option(RS2_OPTION_HOLES_FILL, holes_filling_mode);

#else
	spatial_filter::spatial_filter() :
		depth_processing_block("Spatial Filter"),
		_spatial_alpha_param(alpha_default_val),
		_spatial_delta_param(delta_default_val),
		_spatial_iterations(filter_iter_def),
		_width(0), _height(0), _stride(0), _bpp(0),
		_extension_type(RS2_EXTENSION_DEPTH_FRAME),
		_current_frm_size_pixels(0),
		_stereoscopic_depth(false),
		_focal_lenght_mm(0.f),
		_stereo_baseline_mm(0.f),
		_holes_filling_mode(holes_fill_def),
		_holes_filling_radius(0)
	{
		_stream_filter.stream = RS2_STREAM_DEPTH;
		_stream_filter.format = RS2_FORMAT_Z16;
		auto spatial_filter_alpha = std::make_shared<ptr_option<float>>(
			alpha_min_val,
			alpha_max_val,
			alpha_step,
			alpha_default_val,
			&_spatial_alpha_param, "Alpha factor of Exp.moving average, 1 = no filter, 0 = infinite filter");

		auto spatial_filter_delta = std::make_shared<ptr_option<uint8_t>>(
			delta_min_val,
			delta_max_val,
			delta_step,
			delta_default_val,
			&_spatial_delta_param, "Edge-preserving Threshold");
		spatial_filter_delta->on_set([this, spatial_filter_delta](float val)
		{
			std::lock_guard<std::mutex> lock(_mutex);

			if (!spatial_filter_delta->is_valid(val))
				throw invalid_value_exception(to_string()
					<< "Unsupported spatial delta: " << val << " is out of range.");

			_spatial_delta_param = static_cast<uint8_t>(val);
			_spatial_edge_threshold = float(_spatial_delta_param);
		});

		auto spatial_filter_iterations = std::make_shared<ptr_option<uint8_t>>(
			filter_iter_min,
			filter_iter_max,
			filter_iter_step,
			filter_iter_def,
			&_spatial_iterations, "Filtering iterations");

		auto holes_filling_mode = std::make_shared<ptr_option<uint8_t>>(
			holes_fill_min,
			holes_fill_max,
			holes_fill_step,
			holes_fill_def,
			&_holes_filling_mode, "Holes filling mode");

		holes_filling_mode->set_description(sp_hf_disabled, "Disabled");
		holes_filling_mode->set_description(sp_hf_2_pixel_radius, "2-pixel radius");
		holes_filling_mode->set_description(sp_hf_4_pixel_radius, "4-pixel radius");
		holes_filling_mode->set_description(sp_hf_8_pixel_radius, "8-pixel radius");
		holes_filling_mode->set_description(sp_hf_16_pixel_radius, "16-pixel radius");
		holes_filling_mode->set_description(sp_hf_unlimited_radius, "Unlimited");

		holes_filling_mode->on_set([this, holes_filling_mode](float val)
		{
			std::lock_guard<std::mutex> lock(_mutex);

			if (!holes_filling_mode->is_valid(val))
				throw invalid_value_exception(to_string()
					<< "Unsupported mode for spatial holes filling selected: value " << val << " is out of range.");

			_holes_filling_mode = static_cast<uint8_t>(val);
			switch (_holes_filling_mode)
			{
			case sp_hf_disabled:
				_holes_filling_radius = 0;      // disabled
				break;
			case sp_hf_unlimited_radius:
				_holes_filling_radius = 0xff;   // Unrealistic smearing; not particulary useful
				break;
			case sp_hf_2_pixel_radius:
			case sp_hf_4_pixel_radius:
			case sp_hf_8_pixel_radius:
			case sp_hf_16_pixel_radius:
				_holes_filling_radius = 0x1 << _holes_filling_mode; // 2's exponential radius
				break;
			default:
				throw invalid_value_exception(to_string()
					<< "Unsupported spatial hole-filling requested: value " << _holes_filling_mode << " is out of range.");
				break;
			}
	});

		register_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, spatial_filter_alpha);
		register_option(RS2_OPTION_FILTER_SMOOTH_DELTA, spatial_filter_delta);
		register_option(RS2_OPTION_FILTER_MAGNITUDE, spatial_filter_iterations);
		register_option(RS2_OPTION_HOLES_FILL, holes_filling_mode);
#endif
	}

    rs2::frame spatial_filter::process_frame(const rs2::frame_source& source, const rs2::frame& f)
    {
        rs2::frame tgt;
		
        update_configuration(f);
        tgt = prepare_target_frame(f, source);

        // Spatial domain transform edge-preserving filter
#if _ALTEK_SF_
		if (_extension_type == RS2_EXTENSION_DEPTH_FRAME)
		{
			dxf_smooth<uint16_t>(const_cast<void*>(tgt.get_data()), _spatial_alpha_param, _spatial_edge_threshold, _spatial_iterations);
		}
#else
		if (_extension_type == RS2_EXTENSION_DISPARITY_FRAME)
			dxf_smooth<float>(const_cast<void*>(tgt.get_data()), _spatial_alpha_param, _spatial_edge_threshold, _spatial_iterations);
		else
			dxf_smooth<uint16_t>(const_cast<void*>(tgt.get_data()), _spatial_alpha_param, _spatial_edge_threshold, _spatial_iterations);
#endif
        return tgt;
    }

    void  spatial_filter::update_configuration(const rs2::frame& f)
    {
        if (f.get_profile().get() != _source_stream_profile.get())
        {
            _source_stream_profile = f.get_profile();
            _target_stream_profile = _source_stream_profile.clone(RS2_STREAM_DEPTH, 0, _source_stream_profile.format());

            _extension_type = f.is<rs2::disparity_frame>() ? RS2_EXTENSION_DISPARITY_FRAME : RS2_EXTENSION_DEPTH_FRAME;
            _bpp = (_extension_type == RS2_EXTENSION_DISPARITY_FRAME) ? sizeof(float) : sizeof(uint16_t);
            auto vp = _target_stream_profile.as<rs2::video_stream_profile>();
            _focal_lenght_mm = vp.get_intrinsics().fx;
            _width = vp.width();
            _height = vp.height();
            _stride = _width * _bpp;
            _current_frm_size_pixels = _width * _height;

            // Check if the new frame originated from stereo-based depth sensor
            // retrieve the stereo baseline parameter
            // TODO refactor disparity parameters into the frame's metadata
            auto snr = ((frame_interface*)f.get())->get_sensor().get();
            librealsense::depth_stereo_sensor* dss;

            // Playback sensor
            if (auto a = As<librealsense::extendable_interface>(snr))
            {
                librealsense::depth_stereo_sensor* ptr;
                if ((_stereoscopic_depth = a->extend_to(TypeToExtension<librealsense::depth_stereo_sensor>::value, (void**)&ptr)))
                {
                    dss = ptr;
                    _stereo_baseline_mm = dss->get_stereo_baseline_mm();
                }
            }
            else if (auto depth_emul = As<librealsense::software_sensor>(snr))
            {
                // Software device can obtain these options via Options interface
                if (depth_emul->supports_option(RS2_OPTION_STEREO_BASELINE))
                {
                    _stereo_baseline_mm = depth_emul->get_option(RS2_OPTION_STEREO_BASELINE).query()*1000.f;
                    _stereoscopic_depth = true;
                }
            }
            else // Live sensor
            {
                _stereoscopic_depth = Is<librealsense::depth_stereo_sensor>(snr);
                dss = As<librealsense::depth_stereo_sensor>(snr);
                if (_stereoscopic_depth)
                    _stereo_baseline_mm = dss->get_stereo_baseline_mm();
            }

            _spatial_edge_threshold = _spatial_delta_param;// (_extension_type == RS2_EXTENSION_DISPARITY_FRAME) ?
                                                           // (_focal_lenght_mm * _stereo_baseline_mm) / float(_spatial_delta_param) : _spatial_delta_param;
        }

#if _ALTEK_SF_
		//------ LUT buffer malloc ---------------------------------------
		if (_spatial_delta_LUT_buffer_init_flag == 0)
		{
			_spatial_delta_LUT = (uint16_t*)malloc(sizeof(uint16_t) * 65536);
			_spatial_delta_LUT_buffer_init_flag = 1;
		}
		//------ LUT assign value ----------------------------------------
		if (_spatial_delta_LUT_value_init_flag == 0)
		{
			_altek_sf_mdc_th_init(_spatial_delta_LUT);
			_spatial_delta_LUT_value_init_flag = 1;
		}
		//------ integral image remalloc --------------------------------
		if (_spatial_integralimage_size_pixels != _current_frm_size_pixels  || _spatial_integralimage_mask_mode!= _holes_filling_mode)
		{
			int mask_half = 5;
			switch (_holes_filling_mode)
			{
				case 0:			mask_half = 4;		break;
				case 1:			mask_half = 5;		break;
				case 2:			mask_half = 4;		break;
				case 3:			mask_half = 5;		break;
				case 4:			mask_half = 6;		break;
				case 5:			mask_half = 7;		break;
				default:			mask_half = 5;		break;
			}
			int width2 = static_cast<int>(_width) + mask_half * 2 + 1;
			int height2 = static_cast<int>(_height) + mask_half * 2 + 1;

			if (_spatial_integralimage_buffer_init_flag == 1)
			{
				free(_spatial_count_integralimage);
				free(_spatial_value_integralimage);
			}
			
			_spatial_count_integralimage = (int32_t*)malloc(width2*height2 * sizeof(int32_t));
			_spatial_value_integralimage = (int32_t*)malloc(width2*height2 * sizeof(int32_t));
			
			_spatial_integralimage_buffer_init_flag = 1;
			_spatial_integralimage_size_pixels = static_cast<int32_t>(_current_frm_size_pixels);
			_spatial_integralimage_mask_mode = _holes_filling_mode;
		}
#endif
    }

    rs2::frame spatial_filter::prepare_target_frame(const rs2::frame& f, const rs2::frame_source& source)
    {
        // Allocate and copy the content of the original Depth data to the target
        rs2::frame tgt = source.allocate_video_frame(_target_stream_profile, f, int(_bpp), int(_width), int(_height), int(_stride), _extension_type);

        memmove(const_cast<void*>(tgt.get_data()), f.get_data(), _current_frm_size_pixels * _bpp);
        return tgt;
    }

    void spatial_filter::recursive_filter_horizontal_fp(void * image_data, float alpha, float deltaZ)
    {
        float *image = reinterpret_cast<float*>(image_data);
        int v, u;

        for (v = 0; v < _height;) {
            // left to right
            float *im = image + v * _width;
            float state = *im;
            float previousInnovation = state;

            im++;
            float innovation = *im;
            u = int(_width) - 1;
            if (!(*(int*)&previousInnovation > 0))
                goto CurrentlyInvalidLR;
            // else fall through

        CurrentlyValidLR:
            for (;;) {
                if (*(int*)&innovation > 0) {
                    float delta = previousInnovation - innovation;
                    bool smallDifference = delta < deltaZ && delta > -deltaZ;

                    if (smallDifference) {
                        float filtered = innovation * alpha + state * (1.0f - alpha);
                        *im = state = filtered;
                    }
                    else {
                        state = innovation;
                    }
                    u--;
                    if (u <= 0)
                        goto DoneLR;
                    previousInnovation = innovation;
                    im += 1;
                    innovation = *im;
                }
                else {  // switch to CurrentlyInvalid state
                    u--;
                    if (u <= 0)
                        goto DoneLR;
                    previousInnovation = innovation;
                    im += 1;
                    innovation = *im;
                    goto CurrentlyInvalidLR;
                }
            }

        CurrentlyInvalidLR:
            for (;;) {
                u--;
                if (u <= 0)
                    goto DoneLR;
                if (*(int*)&innovation > 0) { // switch to CurrentlyValid state
                    previousInnovation = state = innovation;
                    im += 1;
                    innovation = *im;
                    goto CurrentlyValidLR;
                }
                else {
                    im += 1;
                    innovation = *im;
                }
            }
        DoneLR:

            // right to left
            im = image + (v + 1) * _width - 2;  // end of row - two pixels
            previousInnovation = state = im[1];
            u = int(_width) - 1;
            innovation = *im;
            if (!(*(int*)&previousInnovation > 0))
                goto CurrentlyInvalidRL;
            // else fall through
        CurrentlyValidRL:
            for (;;) {
                if (*(int*)&innovation > 0) {
                    float delta = previousInnovation - innovation;
                    bool smallDifference = delta < deltaZ && delta > -deltaZ;

                    if (smallDifference) {
                        float filtered = innovation * alpha + state * (1.0f - alpha);
                        *im = state = filtered;
                    }
                    else {
                        state = innovation;
                    }
                    u--;
                    if (u <= 0)
                        goto DoneRL;
                    previousInnovation = innovation;
                    im -= 1;
                    innovation = *im;
                }
                else {  // switch to CurrentlyInvalid state
                    u--;
                    if (u <= 0)
                        goto DoneRL;
                    previousInnovation = innovation;
                    im -= 1;
                    innovation = *im;
                    goto CurrentlyInvalidRL;
                }
            }

        CurrentlyInvalidRL:
            for (;;) {
                u--;
                if (u <= 0)
                    goto DoneRL;
                if (*(int*)&innovation > 0) { // switch to CurrentlyValid state
                    previousInnovation = state = innovation;
                    im -= 1;
                    innovation = *im;
                    goto CurrentlyValidRL;
                }
                else {
                    im -= 1;
                    innovation = *im;
                }
            }
        DoneRL:
            v++;
        }
    }

    void spatial_filter::recursive_filter_vertical_fp(void * image_data, float alpha, float deltaZ)
    {
        float *image = reinterpret_cast<float*>(image_data);

        int v, u;

        // we'll do one column at a time, top to bottom, bottom to top, left to right,

        for (u = 0; u < _width;) {

            float *im = image + u;
            float state = im[0];
            float previousInnovation = state;

            v = int(_height) - 1;
            im += _width;
            float innovation = *im;

            if (!(*(int*)&previousInnovation > 0))
                goto CurrentlyInvalidTB;
            // else fall through

        CurrentlyValidTB:
            for (;;) {
                if (*(int*)&innovation > 0) {
                    float delta = previousInnovation - innovation;
                    bool smallDifference = delta < deltaZ && delta > -deltaZ;

                    if (smallDifference) {
                        float filtered = innovation * alpha + state * (1.0f - alpha);
                        *im = state = filtered;
                    }
                    else {
                        state = innovation;
                    }
                    v--;
                    if (v <= 0)
                        goto DoneTB;
                    previousInnovation = innovation;
                    im += _width;
                    innovation = *im;
                }
                else {  // switch to CurrentlyInvalid state
                    v--;
                    if (v <= 0)
                        goto DoneTB;
                    previousInnovation = innovation;
                    im += _width;
                    innovation = *im;
                    goto CurrentlyInvalidTB;
                }
            }

        CurrentlyInvalidTB:
            for (;;) {
                v--;
                if (v <= 0)
                    goto DoneTB;
                if (*(int*)&innovation > 0) { // switch to CurrentlyValid state
                    previousInnovation = state = innovation;
                    im += _width;
                    innovation = *im;
                    goto CurrentlyValidTB;
                }
                else {
                    im += _width;
                    innovation = *im;
                }
            }
        DoneTB:

            im = image + u + (_height - 2) * _width;
            state = im[_width];
            previousInnovation = state;
            innovation = *im;
            v = int(_height) - 1;
            if (!(*(int*)&previousInnovation > 0))
                goto CurrentlyInvalidBT;
            // else fall through
        CurrentlyValidBT:
            for (;;) {
                if (*(int*)&innovation > 0) {
                    float delta = previousInnovation - innovation;
                    bool smallDifference = delta < deltaZ && delta > -deltaZ;

                    if (smallDifference) {
                        float filtered = innovation * alpha + state * (1.0f - alpha);
                        *im = state = filtered;
                    }
                    else {
                        state = innovation;
                    }
                    v--;
                    if (v <= 0)
                        goto DoneBT;
                    previousInnovation = innovation;
                    im -= _width;
                    innovation = *im;
                }
                else {  // switch to CurrentlyInvalid state
                    v--;
                    if (v <= 0)
                        goto DoneBT;
                    previousInnovation = innovation;
                    im -= _width;
                    innovation = *im;
                    goto CurrentlyInvalidBT;
                }
            }

        CurrentlyInvalidBT:
            for (;;) {
                v--;
                if (v <= 0)
                    goto DoneBT;
                if (*(int*)&innovation > 0) { // switch to CurrentlyValid state
                    previousInnovation = state = innovation;
                    im -= _width;
                    innovation = *im;
                    goto CurrentlyValidBT;
                }
                else {
                    im -= _width;
                    innovation = *im;
                }
            }
        DoneBT:
            u++;
        }
    }

#if _ALTEK_SF_
	void spatial_filter::altek_spatial_filter(void * image_data, float alpha, float deltaZ, float iterations)
	{
		int mask_half = 5, mask_s_half_h = 3;
		switch (_holes_filling_mode)
		{
			case 0:			mask_half = 4, mask_s_half_h = 2;		break;
			case 1:			mask_half = 5, mask_s_half_h = 2;		break;
			case 2:			mask_half = 4, mask_s_half_h = 3;		break;
			case 3:			mask_half = 5, mask_s_half_h = 3;		break;
			case 4:			mask_half = 6, mask_s_half_h = 3;		break;
			case 5:			mask_half = 7, mask_s_half_h = 3;		break;
			default:			mask_half = 5, mask_s_half_h = 3;		break;
		}
		
		//------ MDC & DC  buffer initialization ----------------
		uint16_t* image = reinterpret_cast<uint16_t*>(image_data);
		int width2 = static_cast<int>(_width) + mask_half * 2 + 1;
		int height2 = static_cast<int>(_height) + mask_half * 2 + 1;
		int iter = 1;

		//------ Apply Filter --------------------------------------
		_altek_sf_mdc(image, iterations,_spatial_delta_LUT, _spatial_value_integralimage, _spatial_count_integralimage, mask_half, mask_s_half_h, width2, height2);
		_altek_sf_dc(image, _spatial_count_integralimage, iter, mask_half, width2, height2);
	}

	void  spatial_filter::_altek_sf_mdc_th_init(uint16_t* spatial_delta_LUT)
	{
		float tfb = _focal_lenght_mm*_stereo_baseline_mm;
		float spatial_delta_hf = _spatial_delta_param / 2;
		int _max_dist = static_cast<int>(tfb / (_ALTEK_SF_DELTA_MAX_*1.0f) + 0.5);
		if (_max_dist < 65535)_max_dist = 65534;
		for (int i = _ALTEK_SF_MIN_DIST_; i <= _max_dist; i++)
		{
			float tdisp = tfb / (i*1.0f);
			spatial_delta_LUT[i] = static_cast<uint16_t>(0.5 + tfb / (tdisp - spatial_delta_hf) - tfb / (tdisp + spatial_delta_hf));
		}
		for (int i = 0; i < _ALTEK_SF_MIN_DIST_; i++)
		{
			spatial_delta_LUT[i] = spatial_delta_LUT[_ALTEK_SF_MIN_DIST_];
		}
		for (int i = _max_dist + 1; i < 65536; i++)
		{
			spatial_delta_LUT[i] = spatial_delta_LUT[_max_dist];
		}
	}

	void  spatial_filter::_altek_sf_mdc(uint16_t* image, float thr, uint16_t* spatial_delta_LUT, int* timage, int* cimage, int mask_half, int mask_s_half, int width2, int height2)
	{
		memset(timage, 0, width2*height2 * sizeof(int32_t));
		memset(cimage, 0, width2*height2 * sizeof(int32_t));
		for (int u = mask_half + 1; u < height2 - mask_half; u++)
		{
			int c_sum = 0;
			int t_sum = 0;
			int* cDst = cimage + u * width2;
			int* imDst = timage + u * width2;
			uint16_t *imOri = image + (u - mask_half - 1) * _width + (-mask_half - 1);
			for (int v = mask_half + 1; v < width2 - mask_half; v++)
			{
				if (imOri[v] > 0)c_sum++;
				cDst[v] = cDst[v - width2] + c_sum;
				t_sum += static_cast<int>(imOri[v]);
				imDst[v] = imDst[v - width2] + t_sum;
			}
			for (int v = width2 - mask_half; v < width2; v++)
			{
				cDst[v] = cDst[v - width2] + c_sum;
				imDst[v] = imDst[v - width2] + t_sum;
			}
		}
		for (int u = height2 - mask_half; u < height2; u++)
		{
			int* cDst = cimage + u * width2;
			int* imDst = timage + u * width2;
			for (int v = 0; v < width2; v++)
			{
				cDst[v] = cDst[v - width2];
				imDst[v] = imDst[v - width2];
			}
		}

		int diff_mask_w = mask_half - mask_s_half;
		int diff_mask_h = mask_half - mask_s_half;

		for (int u = 0; u < _height; u++)
		{
			int *cDst1 = cimage + (u)* width2;
			int *cDst2 = cimage + (u)* width2 + (mask_half + mask_half + 1);
			int *cDst3 = cimage + (u + mask_half + mask_half + 1) * width2;
			int *cDst4 = cimage + (u + mask_half + mask_half + 1) * width2 + (mask_half + mask_half + 1);
			int *imDst1 = timage + (u)* width2;
			int *imDst2 = timage + (u)* width2 + (mask_half + mask_half + 1);
			int *imDst3 = timage + (u + mask_half + mask_half + 1) * width2;
			int *imDst4 = timage + (u + mask_half + mask_half + 1) * width2 + (mask_half + mask_half + 1);

			int* cDst1_df = cimage + (u+ diff_mask_h)*width2+ diff_mask_w;
			int* cDst2_df = cimage + (u+ diff_mask_h)*width2 + (mask_half + mask_half + 1- diff_mask_w);
			int* cDst3_df = cimage + (u + mask_half + mask_half + 1- diff_mask_h) * width2+ diff_mask_w;
			int* cDst4_df = cimage + (u + mask_half + mask_half + 1- diff_mask_h) * width2 + (mask_half + mask_half + 1- diff_mask_w);
			int* imDst1_df = timage + (u + diff_mask_h)*width2+ diff_mask_w;
			int* imDst2_df = timage + (u + diff_mask_h)*width2 + (mask_half + mask_half + 1- diff_mask_w);
			int* imDst3_df = timage + (u + mask_half + mask_half + 1 - diff_mask_h) * width2+ diff_mask_w;
			int* imDst4_df = timage + (u + mask_half + mask_half + 1 - diff_mask_h) * width2 + (mask_half + mask_half + 1- diff_mask_w);

			uint16_t *imOri = image + u * _width;
			for (int v = 0; v < _width; v++)
			{
				int dCnt = cDst1[v] + cDst4[v] - cDst2[v] - cDst3[v];
				int dDistance_Sum = imDst1[v] + imDst4[v] - imDst2[v] - imDst3[v];

				int dCnt_df = cDst1_df[v] + cDst4_df[v] - cDst2_df[v] - cDst3_df[v];
				int dDistance_Sum_df = imDst1_df[v] + imDst4_df[v] - imDst2_df[v] - imDst3_df[v];

				int dDistance_Avg = 0;
				int dDistance_Avg_df = 0;

				if (imOri[v]> 0)
				{
					dDistance_Avg = (dDistance_Sum + (dCnt >>1)) / dCnt;
					dDistance_Avg_df = (dDistance_Sum_df + (dCnt_df >> 1)) / dCnt_df;

					int ori_val = static_cast<int>(imOri[v]);
					int m_th = spatial_delta_LUT[dDistance_Avg_df];
					int s_th = int(m_th * thr+1);
					int diff1 = abs(dDistance_Avg_df - ori_val);
					int diff2 = abs(dDistance_Avg - ori_val);

					if (diff1 > m_th)
					{
						imOri[v] = 0;
					}
					else if (diff2 < s_th)
					{
							int r = (diff2 <<7) / s_th;
							imOri[v] = uint16_t((dDistance_Avg * (128-r) + dDistance_Avg_df *r)>>7);
					}
				}
			}
		}
	}

	void  spatial_filter::_altek_sf_dc(uint16_t * image, int* cimage, int iterations, int mask_half, int width2, int height2)
	{
		int tmp_Den_Thd = (int)((mask_half * 2 + 1)*(mask_half * 2 + 1)*_spatial_alpha_param + 0.5f);
		for (int i = 0; i<(iterations); i++)
		{
			memset(cimage, 0, width2*height2 * sizeof(int32_t));
			for (int u = mask_half + 1; u < height2 - mask_half; u++)
			{
				int c_sum = 0;
				int* cDst = cimage + u * width2;
				uint16_t *imOri = image + (u - mask_half - 1) * _width + (-mask_half - 1);
				for (int v = mask_half + 1; v < width2 - mask_half; v++)
				{
					if (imOri[v] > 0)c_sum++;
					cDst[v] = cDst[v - width2] + c_sum;
				}
				for (int v = width2 - mask_half; v < width2; v++)
				{
					cDst[v] = cDst[v - width2] + c_sum;
				}
			}
			for (int u = height2 - mask_half; u < height2; u++)
			{
				int* cDst = cimage + u * width2;
				for (int v = 0; v < width2; v++)
				{
					cDst[v] = cDst[v - width2];
				}
			}
			for (int u = 0; u < _height; u++)
			{
				int *cDst1 = cimage + (u)* width2;
				int *cDst2 = cimage + (u)* width2 + (mask_half + mask_half + 1);
				int *cDst3 = cimage + (u + mask_half + mask_half + 1) * width2;
				int *cDst4 = cimage + (u + mask_half + mask_half + 1) * width2 + (mask_half + mask_half + 1);
				uint16_t *imOri = image + u * _width;
				for (int v = 0; v < _width; v++)
				{
					int dCnt = cDst1[v] + cDst4[v] - cDst2[v] - cDst3[v];
					if (imOri[v]> 0)
					{
						if (dCnt < tmp_Den_Thd)
							imOri[v] = 0;
					}
				}
			}
		}
	}
#endif
}
