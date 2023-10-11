#include <librealsense2/rs.hpp>
#include <iostream>            
#include <mutex>
#include <algorithm>
#include <thread>
#include <chrono>
#include <iomanip>
#include "example.hpp"          
#include "example-imgui.hpp"
#include <fstream>  
#include <librealsense2-gl/rs_processing_gl.hpp> 

int test_func()
{
	return EXIT_SUCCESS;
}

int main(int argc, char * argv[]) try
{
	test_func();
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "al-test error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
