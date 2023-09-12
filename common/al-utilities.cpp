
#include <thread>
#include <algorithm>
#include <regex>
#include <cmath>
#include <time.h>

#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rs.hpp>

#include "al-utilities.h"

typedef struct
{
	uint32_t ulFrameIndex;
	uint32_t ulExpTime;
	int16_t  wBV;
	uint16_t uwISO;
	uint16_t uwAD_Gain;
	uint16_t uwFrameRate;
}DATA_PIPE_IR;

typedef struct
{
	DATA_PIPE_IR tPipe0mtInfo;
	DATA_PIPE_IR tPipe1mtInfo;
	uint16_t uwDepthWidth;
	uint16_t uwDepthHeight;
	uint16_t uwDepthType;
}IR_DATA;

void show_ext_info(rs2::stream_profile profile, rs2::device dev)
{
	bool ip_supports = dev.supports(RS2_CAMERA_INFO_IP_ADDRESS);

	if (ip_supports)
	{
		return;
	}

	if (RS2_STREAM_INFRARED == profile.stream_type())
	{
		bool er = dev.set_al3d_param(501, 2, 3, 4);

		if (er == true)
		{
			IR_DATA data;
			int dataSize = 0;
			rs2_error* e = NULL;
			std::string msg;
			float uwViewerGain = 0;
			float gainUnit = 6.25;

			std::shared_ptr<const rs2_raw_data_buffer> response_bytes(dev.get_al3d_data(), rs2_delete_raw_data);
			memcpy(&dataSize, rs2_get_raw_data(response_bytes.get(), &e) + 4, 4);
			memcpy(&data, rs2_get_raw_data(response_bytes.get(), &e) + 8, dataSize);

			msg = rs2::to_string()
				<< "---===========================----";
			rs2::log(RS2_LOG_SEVERITY_INFO, msg.c_str());
			msg = rs2::to_string()
				<< "---=== Metadata:PIPE0 Info ===----";
			rs2::log(RS2_LOG_SEVERITY_INFO, msg.c_str());
			msg = rs2::to_string()
				<< "Frame Index: " << data.tPipe0mtInfo.ulFrameIndex << ", FrameRate: " << data.tPipe0mtInfo.uwFrameRate;
			rs2::log(RS2_LOG_SEVERITY_INFO, msg.c_str());

			uwViewerGain = data.tPipe0mtInfo.uwISO - 100;

			if (uwViewerGain < gainUnit)
				uwViewerGain = 1;
			else
				uwViewerGain = (std::round((uwViewerGain / gainUnit)) + 1);

			msg = rs2::to_string()
				<< "(ExpTime,wBV): (" << data.tPipe0mtInfo.ulExpTime << "," << data.tPipe0mtInfo.wBV << "), "
				<< "(ISO,ADGain): (" << data.tPipe0mtInfo.uwISO << "," << data.tPipe0mtInfo.uwAD_Gain << "), "
				<< "(ViewerGain[manual]): (" << uwViewerGain << ")";

			rs2::log(RS2_LOG_SEVERITY_INFO, msg.c_str());
			msg = rs2::to_string()
				<< "---=== Metadata:PIPE1 Info ===----";
			rs2::log(RS2_LOG_SEVERITY_INFO, msg.c_str());
			msg = rs2::to_string()
				<< "Frame Index: " << data.tPipe1mtInfo.ulFrameIndex << ", FrameRate: " << data.tPipe1mtInfo.uwFrameRate;
			rs2::log(RS2_LOG_SEVERITY_INFO, msg.c_str());
			msg = rs2::to_string()
				<< "(ExpTime,wBV): (" << data.tPipe1mtInfo.ulExpTime << "," << data.tPipe1mtInfo.wBV << "), "
				<< "(ISO,ADGain): (" << data.tPipe1mtInfo.uwISO << "," << data.tPipe1mtInfo.uwAD_Gain << ")";
			rs2::log(RS2_LOG_SEVERITY_INFO, msg.c_str());
			msg = rs2::to_string()
				<< "---=== Metadata:Depth Info ===----";
			rs2::log(RS2_LOG_SEVERITY_INFO, msg.c_str());
			msg = rs2::to_string()
				<< "Depth(w,h,type): (" << data.uwDepthWidth << "," << data.uwDepthHeight << "," << data.uwDepthType << ")";
			rs2::log(RS2_LOG_SEVERITY_INFO, msg.c_str());
		}
	}
}

