// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include "example.hpp"              // Include short list of convenience functions for rendering

#include <map>
#include <vector>
#include <cstring>

#define WIDTH           1280               // Defines the number of columns for each frame                         //
#define HEIGHT          720               // Defines the number of lines for each frame                           //
#define FPS             15                // Defines the rate of frames per second                                //
#define STREAM_INDEX    0                 // Defines the stream index, used for multiple streams of the same type //



struct _ALTEK_AI_BOX_INFO_
{
    unsigned short m_u16Box_ID;
    unsigned short m_u16Box_Left;
    unsigned short m_u16Box_Top;
    unsigned short m_u16Box_Right;
    unsigned short m_u16Box_Bottom;
    unsigned short m_u16Box_Distance;
    float m_f32Box_Degree;
};


int main(int argc, char * argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 960, "CPP Multi-Camera Example");

    rs2::context                          ctx;        // Create librealsense context for managing devices

    std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)

    std::vector<rs2::pipeline>            pipelines;

    struct _ALTEK_AI_BOX_INFO_ box_info;
   

    // Capture serial numbers before opening streaming
    std::vector<std::string>              serials;
    for (auto&& dev : ctx.query_devices()) 
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    //find color senseor and enable ai
    for (auto dev : ctx.query_devices())
    {
        int index = 0;
        // The same device should support gyro and accel
        for (auto sensor : dev.query_sensors()) //find color senseor
        {
            std::cout << "  " << index++ << " : " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
           
            for (auto profile : sensor.get_stream_profiles())
            {
                if (profile.stream_type() == RS2_STREAM_COLOR)  //color sensor found
                {
                    if (sensor.supports(RS2_OPTION_AL3D_AI_Enable))
                    {
                        sensor.set_option(RS2_OPTION_AL3D_AI_Enable, 1); // enable ai
                    }
                    if (sensor.supports(RS2_OPTION_AL3D_AI_Mode))
                    {
                        sensor.set_option(RS2_OPTION_AL3D_AI_Mode, 10); //(TBD) set ai mode, currenly not use
                    }
                    break;
                }
            }
        }
      
    }

   
    // Start a streaming pipe per each connected device
    for (auto&& serial : serials)
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(serial);
#if 1 //ggkk

        cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);
        cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_RGB8, FPS);
        
#endif
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        // Map from each device's serial number to a different colorizer
        colorizers[serial] = rs2::colorizer();
    }

    // We'll keep track of the last frame of each stream available to make the presentation persistent
    std::map<int, rs2::frame> render_frames;
    
    
    // Main app loop
    while (app)
    {
        // Collect the new frames from all the connected devices
        std::vector<rs2::frame> new_frames;
        for (auto &&pipe : pipelines)
        {
            
            rs2::frameset fs;
            if (pipe.poll_for_frames(&fs))
            {
                for (const rs2::frame& f : fs)
                    new_frames.emplace_back(f);

				
                //get color frame and then parsing
                auto frameColor = fs.get_color_frame();
                if (frameColor) 
                {
					
                    //get sn 
                    rs2::device color_device = pipe.get_active_profile().get_device();
                    std::string sn = color_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

                    auto ai_results_ptr = frameColor.get_al3d_ai_results();
                    char ai_results_data[1016];
                    memcpy(&ai_results_data[0], (char*)ai_results_ptr, 1016);
                    unsigned long m_u16TotalBytes = (unsigned long)ai_results_data[0];
                    unsigned long m_u16AI_BOX_Number = (unsigned long)ai_results_data[4];
                    unsigned long long m_u64PTS = (unsigned long long)ai_results_data[8];
                    char* ai_box_info_start = &ai_results_data[16];
                    
                    if (m_u16TotalBytes > 0 && m_u16AI_BOX_Number > 0)
                    {
                        //print header
                        //std::string msg =  sn + "->" + " m_u64PTS: " + std::to_string(m_u64PTS) + " m_u16TotalBytes: " + std::to_string(m_u16TotalBytes) + " m_u16AI_BOX_Number: " + std::to_string(m_u16AI_BOX_Number);
                        //rs2::log(RS2_LOG_SEVERITY_INFO, msg.c_str());
                        //std::cout << msg.c_str() << std::endl;

                        //print data info _ALTEK_AI_BOX_INFO_
                        for (int i = 0; i < m_u16AI_BOX_Number; i++)
                        {

                           
                            memcpy((char*)&box_info.m_u16Box_ID, ai_box_info_start, sizeof(box_info));
                            //unsigned short m_u16Box_ID = box_info.m_u16Box_ID;
                            //unsigned short m_u16Box_Left = box_info.m_u16Box_Left;
                            //unsigned short m_u16Box_Top = box_info.m_u16Box_Top;
                            //unsigned short m_u16Box_Right = box_info.m_u16Box_Right;
                            //unsigned short m_u16Box_Bottom = box_info.m_u16Box_Bottom;
                            //unsigned short m_u16Box_Distance = box_info.m_u16Box_Distance;
                            //float  m_f32Box_Degree = box_info.m_f32Box_Degree;

                            std::string msg2 = sn + "->" +" box_" + std::to_string(i) + "("
                                + std::to_string(box_info.m_u16Box_ID) + ","
                                + std::to_string(box_info.m_u16Box_Left) + ","
                                + std::to_string(box_info.m_u16Box_Top) + ","
                                + std::to_string(box_info.m_u16Box_Right) + ","
                                + std::to_string(box_info.m_u16Box_Bottom) + ","
                                + std::to_string(box_info.m_u16Box_Distance) + ","
                                + std::to_string(box_info.m_f32Box_Degree) + ")";

                            ai_box_info_start = ai_box_info_start + 16;
                            //rs2::log(RS2_LOG_SEVERITY_INFO, msg2.c_str());
                            std::cout << msg2.c_str() << std::endl;

                        }

                       
                    }
                }
                

            }
        }

        // Convert the newly-arrived frames to render-friendly format
        for (const auto& frame : new_frames)
        {
            // Get the serial number of the current frame's device
            auto serial = rs2::sensor_from_frame(frame)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            // Apply the colorizer of the matching device and store the colorized frame
            render_frames[frame.get_profile().unique_id()] = colorizers[serial].process(frame);
        }

        // Present all the collected frames with openGl mosaic
        app.show(render_frames);
       
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
