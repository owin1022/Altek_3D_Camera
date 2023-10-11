// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout
#include <map>

void print_separator()
{
    std::cout << "\n======================================================\n" << std::endl;
}

std::string get_device_name(const rs2::device& dev)
{
    // Each device provides some information on itself, such as name:
    std::string name = "Unknown Device";
    if (dev.supports(RS2_CAMERA_INFO_NAME))
        name = dev.get_info(RS2_CAMERA_INFO_NAME);

    // and the serial number of the device:
    std::string sn = "########";
    if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
        sn = std::string("#") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

    return name + " " + sn;
}

uint32_t get_user_selection(const std::string& prompt_message)
{
    std::cout << "\n" << prompt_message;
    uint32_t input;
    std::cin >> input;
    std::cout << std::endl;
    return input;
}

// Hello RealSense example demonstrates the basics of connecting to a RealSense device
// and taking advantage of depth data
int main(int argc, char * argv[]) try
{
    rs2::context                          ctx;        // Create librealsense context for managing devices

    std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)

    std::vector<rs2::pipeline>            pipelines;

    // Capture serial numbers before opening streaming
    std::vector<std::string>              serials;
    rs2::device_list devices = ctx.query_devices();
    for (size_t i = 0; i < devices.size(); i++)
    {
        serials.push_back(devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    }
    //for (auto&& dev : ctx.query_devices())
        //serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    // Start a streaming pipe per each connected device
    for (auto&& serial : serials)
    {
        rs2::pipeline pipe(ctx);

        pipelines.emplace_back(pipe);
        // Map from each device's serial number to a different colorizer
        colorizers[serial] = rs2::colorizer();
    }
    //select camera
    //bool choose_a_device = true;
    //while (choose_a_device)
    //{
        print_separator();
        //First thing, let's choose a device:
         // First, create a rs2::context.
    // The context represents the current platform with respect to connected devices
        //rs2::context ctx;

        // Using the context we can get all connected devices in a device list
        //rs2::device_list devices = ctx.query_devices();

        uint32_t selected_device_index;
        //rs2::pipeline pipeline;
        if (devices.size() == 0)
        {
            std::cerr << "No device connected, please connect a RealSense device" << std::endl;

        }
        else
        {
            std::cout << "Found the following devices:\n" << std::endl;

            // device_list is a "lazy" container of devices which allows
            //The device list provides 2 ways of iterating it
            //The first way is using an iterator (in this case hidden in the Range-based for loop)
            int index = 0;
            for (rs2::device device : devices)
            {
                std::cout << "  " << index++ << " : " << get_device_name(device) << std::endl;
            }

            selected_device_index = get_user_selection("Select a device by index: ");

            // The second way is using the subscript ("[]") operator:
            if (selected_device_index >= devices.size())
            {
                throw std::out_of_range("Selected device index is out of range");
            }

        //}
        rs2::pipeline p = pipelines[selected_device_index];

        // Create a Pipeline - this serves as a top-level API for streaming and processing frames
        //rs2::pipeline p;

        // Configure and start the pipeline
        rs2::config cfg;
        cfg.enable_device(serials[selected_device_index]);
        p.start(cfg);
        //p.start();

        while (true)
        {
            // Block program until frames arrive
            rs2::frameset frames = p.wait_for_frames();

            // Try to get a frame of a depth image
            rs2::depth_frame depth = frames.get_depth_frame();

            // Get the depth frame's dimensions
            auto width = depth.get_width();
            auto height = depth.get_height();

            // Query the distance from the camera to the object in the center of the image
            float dist_to_center = depth.get_distance(width / 2, height / 2);

            // Print the distance
            std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
        }
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

