# Altek_3D_Camera_SDK
<p align="center"><img src="doc/img/altek_SUNNY_M1 .jpg" width="70%" /><br><br></p>

-----------------
[![GitHub CI](../../actions/workflows/buildsCI.yaml/badge.svg?branch=development)](../../actions/workflows/buildsCI.yaml)

## Build Code Guide on Windows10.

Prerequisite :
1. Install Cmake 
https://cmake.org/download/ 
Windows x64 Installer: Installer tool has changed. Uninstall CMake 3.4 or lower first! 
------------------------------------------------------------------------------------------------------
2. Install git 
https://git-scm.com/download/win 

------------------------------------------------------------------------------------------------------
3. Install Visual Studio 2019 community 
https://visualstudio.microsoft.com/zh-hant/thank-you-downloading-visual-studio/?sku=Community&rel=16 

Compile code process:
Choose input source code/output path and click configure button from Cmake-gui . After configure done withoout errors , click Generate button to generate librealsense2.sln.

Open Visual Studio 2019 to load project with librealsense2.sln ,compile ,and execute it .



## Overview
**Altek_3D_Camera SDK 1.0** is a cross-platform library for Altek® AQ360™ depth cameras  and the [T265 tracking camera](./doc/t265.md).



The SDK allows depth and color streaming, and provides intrinsic and extrinsic calibration information.
The library also offers synthetic streams (pointcloud, depth aligned to color and vise-versa), and a built-in support for [record and playback](./doc/record-and-playback.md) of streaming sessions.

Developer kits containing the necessary hardware to use this library are available for purchase at [store.altek.com](https://store.altek.com.tw/qualcomm/).
Information about the Altek_3D_Camera technology at [https://www.altek.com.tw/](https://store.altek.com.tw/qualcomm/)

> :open_file_folder: Don't have access to a Altek_3D_Camera? Check-out [sample data](./doc/sample-data.md)

## Special notice from Altek™ regarding the recent press announcement

Altek has decided to focus on AI Camera business and is announcing IoT-related product, and Tracking product lines this year. Altek will continue to provide Stereo products to its current distribution customers.  Altek will focus our new development on advancing innovative technologies that better support our core businesses.
The products identified in this notification will be discontinued and unavailable for additional orders after Feb 28, 2022.

The following Stereo Product Lines will continue to be supported: AL6100 depth cameras. 

We will also continue the work to support and develop our Altek open source SDK.

In the coming future Altek and AL6100 team will focus our new development on advancing innovative technologies that better support our core businesses and 


## Download and Install
* **Download** - The latest releases including Altek_3D_Camera SDK, Viewer and Depth Quality tools are available at: [**latest releases**](https://github.com/IntelRealSense/librealsense/releases). Please check the [**release notes**](https://github.com/IntelRealSense/librealsense/wiki/Release-Notes) for the supported platforms, new features and capabilities, known issues, how to upgrade the Firmware and more.

* **Install** - You can also install or build from source the SDK (on [Linux](./doc/distribution_linux.md) \ [Windows](./doc/distribution_windows.md) ), connect your AL6100 depth camera and you are ready to start writing your first application.

> **Support & Issues**: If you need product support (e.g. ask a question about / are having problems with the device), please check the [FAQ & Troubleshooting](https://github.com/IntelRealSense/librealsense/wiki/Troubleshooting-Q%26A) section.
> If not covered there, please search our [Closed GitHub Issues](https://github.com/IntelRealSense/librealsense/issues?utf8=%E2%9C%93&q=is%3Aclosed) page,  [Community](https://communities.intel.com/community/tech/realsense) and [Support](https://www.intel.com/content/www/us/en/support/emerging-technologies/intel-realsense-technology.html) sites.
> If you still cannot find an answer to your question, please [open a new issue](https://github.com/IntelRealSense/librealsense/issues/new).



## Ready to Hack!

Our library offers a high level API for using Altek depth cameras (in addition to lower level ones).
The following snippet shows how to start streaming frames and extracting the depth value of a pixel:

```cpp
// Create a Pipeline - this serves as a top-level API for streaming and processing frames
rs2::pipeline p;

// Configure and start the pipeline
p.start();

while (true)
{
    // Block program until frames arrive
    rs2::frameset frames = p.wait_for_frames();

    // Try to get a frame of a depth image
    rs2::depth_frame depth = frames.get_depth_frame();

    // Get the depth frame's dimensions
    float width = depth.get_width();
    float height = depth.get_height();

    // Query the distance from the camera to the object in the center of the image
    float dist_to_center = depth.get_distance(width / 2, height / 2);

    // Print the distance
    std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
}
```
For more information on the library, please follow our [examples](./examples), and read the [documentation](./doc) to learn more.

## Contributing
In order to contribute to Altek SDK, please follow our [contribution guidelines](CONTRIBUTING.md).

## License
This project is licensed under the [Apache License, Version 2.0](LICENSE).
Copyright 2018 Intel Corporation
