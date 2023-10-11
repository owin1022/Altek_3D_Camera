// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Capture Example");

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::rates_printer printer;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    auto profile = pipe.start();

    // Declare filters
    rs2::decimation_filter dec_filter;
    rs2::spatial_filter spat_filter;
    // Configure filter parameters
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3);

    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 1);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 0);

    rs2::threshold_filter thr_filter(0.0f, 0.5f);

    //depth_frame = thr_filter.process(depth_frame);

    //thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.0f);
    //thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, 0.5f);

    /*rs2::threshold_filter tf(0.0f, 0.5f); 
    rs2::rates_printer printer; 
    apply_filter(tf)pipe.wait_for_frames()*/

    while (app) // Application still alive?
    {
        rs2::frameset data = pipe.wait_for_frames().    // Wait for next set of frames from the camera
                             apply_filter(printer).     // Print each enabled stream frame rate
                             apply_filter(color_map).   // Find and colorize the depth data
                             apply_filter(thr_filter).
                             apply_filter(spat_filter);

        rs2::frame depth_frame = data.get_depth_frame();

        rs2::frame filtered = depth_frame;
        // Note the concatenation of output/input frame to build up a chain
        filtered = dec_filter.process(filtered);
        filtered = spat_filter.process(filtered);

        //depth_frame.apply_filter(color_map).apply_filter(dec_filter).apply_filter(spat_filter);
        //emit newImage(frameToQImage(depth_frame.apply_filter(color_map).apply_filter(dec_filter).apply_filter(spat_filter)));
        // The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
        // Each texture is displayed on different viewport according to it's stream unique id

        //data.apply_filter(spat_filter);
        //auto colorized_depth = np.asanyarray(colorizer.colorize(filtered_depth).get_data());
        //cv2.imshow("filtered1 depth", colorized_depth);
        
        app.show(data);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}