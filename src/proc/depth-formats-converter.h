// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#pragma once

#include "synthetic-stream.h"
#include "option.h"
#include "image.h"

namespace librealsense
{
    class inzi_converter : public interleaved_functional_processing_block
    {
    public:
        inzi_converter(rs2_format target_ir_format) :
            inzi_converter("INZI to depth and IR Transform", target_ir_format) {};

    protected:
        inzi_converter(const char* name, rs2_format target_ir_format);
        void process_function(byte * const dest[], const byte * source, int width, int height, int actual_size, int input_size) override;
    };

    class invi_converter : public functional_processing_block
    {
    public:
        invi_converter(rs2_format target_format) :
            invi_converter("INVI to IR Transform", target_format) {};

    protected:
        invi_converter(const char* name, rs2_format target_format) :
            functional_processing_block(name, target_format, RS2_STREAM_INFRARED, RS2_EXTENSION_VIDEO_FRAME) {};
        void process_function(byte * const dest[], const byte * source, int width, int height, int actual_size, int input_size) override;
    };

    class w10_converter : public functional_processing_block
    {
    public:
        w10_converter(const rs2_format& target_format) :
            w10_converter("W10 Transform", target_format) {};

    protected:
        w10_converter(const char* name, const rs2_format& target_format);
        void process_function(byte * const dest[], const byte * source, int width, int height, int actual_size, int input_size) override;
    };

	// al_converter
    class al24_converter : public interleaved_functional_processing_block
    {
    public:
		al24_converter(int depth_idx = 0, int left_idx = 1) :
			al24_converter("AL24 Converter", depth_idx, left_idx) {}

    protected:
		al24_converter(const char * name, int depth_idx, int left_idx)
			: interleaved_functional_processing_block(name, RS2_FORMAT_AL24, 	RS2_FORMAT_Z16, RS2_STREAM_DEPTH, RS2_EXTENSION_DEPTH_FRAME, 0,
																				RS2_FORMAT_Y8, RS2_STREAM_INFRARED, RS2_EXTENSION_VIDEO_FRAME, 1) {}

        void process_function(byte * const dest[], const byte * source, int width, int height, int actual_size, int input_size) override;
    };

	// Ken++  Z32
    class al_converter : public interleaved_functional_processing_block
    {
    public:
		al_converter(int depth_idx = 0, int left_idx = 1, int right_idx = 2 ) :
			al_converter("AL Converter", depth_idx, left_idx, right_idx) {}

    protected:
		al_converter(const char * name, int depth_idx, int left_idx, int right_idx)
			: interleaved_functional_processing_block(name, RS2_FORMAT_Z32, 	RS2_FORMAT_Z16, RS2_STREAM_DEPTH, RS2_EXTENSION_VIDEO_FRAME, 0,
																				RS2_FORMAT_Y8, RS2_STREAM_INFRARED, RS2_EXTENSION_VIDEO_FRAME, 1,
																				RS2_FORMAT_Y8, RS2_STREAM_INFRARED, RS2_EXTENSION_VIDEO_FRAME, 2) {}

        void process_function(byte * const dest[], const byte * source, int width, int height, int actual_size, int input_size) override;
    };
}
