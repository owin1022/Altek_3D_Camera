
#pragma once

// 440bytes
typedef struct
{
    // Calibration at Object Distance
    double m_eObj_Dist;// (mm)

    //[Left(Main) Camera] ------------------
    // Camera Parameters
    double m_efx_Main;// (pixel)		//<<<<<<  the ORIGINAL fx WITHOUT LDC
    double m_efy_Main;// (pixel)
    double m_eux_Main;// (pixel)
    double m_euy_Main;// (pixel)

    // Distortion Parameters
    double m_ek1_Main;
    double m_ek2_Main;
    double m_ep1_Main;
    double m_ep2_Main;
    double m_ek3_Main;
    double m_ek4_Main;
    double m_ek5_Main;
    double m_ek6_Main;

    // Image Resolution of Calibration
    unsigned short int m_uwCalib_W_Main;// (pixel)
    unsigned short int m_uwCalib_H_Main;// (pixel)

    // Sensor Image Resolution
    unsigned short int m_uwSensor_W_Main;// (pixel)
    unsigned short int m_uwSensor_H_Main;// (pixel)

    // Cell Size
    float m_fCell_Size_Main;// (mm)

    // Effective Focal Length
    float m_fFocal_Length_Main;// (mm)

    // VCM Offset & Slope
    float m_fVCM_Slope_Main;// (step/mm)
    float m_fVCM_Offset_Main;// (step)

    //[Right(Sub) Camera] ------------------
    // Camera Parameters
    double m_efx_Sub;// (pixel)
    double m_efy_Sub;// (pixel)
    double m_eux_Sub;// (pixel)
    double m_euy_Sub;// (pixel)

    // Distortion Parameters
    double m_ek1_Sub;
    double m_ek2_Sub;
    double m_ep1_Sub;
    double m_ep2_Sub;
    double m_ek3_Sub;
    double m_ek4_Sub;
    double m_ek5_Sub;
    double m_ek6_Sub;

    // Image Resolution of Calibration
    unsigned short int m_uwCalib_W_Sub;// (pixel)
    unsigned short int m_uwCalib_H_Sub;// (pixel)

    // Sensor Image Resolution
    unsigned short int m_uwSensor_W_Sub;// (pixel)
    unsigned short int m_uwSensor_H_Sub;// (pixel)

    // Cell Size
    float m_fCell_Size_Sub;// (mm)

    // Effective Focal Length
    float m_fFocal_Length_Sub;// (mm)

    // VCM Offset & Slope
    float m_fVCM_Slope_Sub;// (step/mm)
    float m_fVCM_Offset_Sub;// (step)

    // [Extrinsic Parameters] ------------------
//Relative RT
    double m_aeRelativeR_Main[9];
    double m_aeRelativeT_Main[3]; // (mm)
    double m_aeRelativeR_Sub[9];
    double m_aeRelativeT_Sub[3]; // (mm)



}tAmazonCalibInfo;

// 328bytes
typedef struct
{
    // Homography, from rectification to src
    double m_aeH_Main[9];
    double m_aeH_Sub[9];
    double m_aeR_Main[9];
    double m_aeR_Sub[9];

    // Rectification parameters
    double m_efx_rec;// (pixel)
    double m_efy_rec;// (pixel)
    double m_ecx_rec;// (pixel)     	
    double m_ecy_rec;// (pixel)	
    double m_eBaseline;// (mm)
}tOpenCVRectificationInfo;

// 16bytes debug information
typedef struct
{
    unsigned char testdatetime[7];
    unsigned char testdatetime_reserved;
    unsigned char ver[2];
    unsigned char bIsIRRGB;
    unsigned char reserved;
    unsigned char CRC32[4];
}tOpenCVDebugInfo;

// 384bytes
typedef struct
{
    tOpenCVRectificationInfo ucOpenCV_rec_328;
    unsigned char rec_reserved[40];
    tOpenCVDebugInfo ucOpenCV_debug_16;

}tOpenCVRectificationInfo_384;

typedef struct OpenCVK_824
{
    tAmazonCalibInfo ucOpenCV_440;                  //Altek's PD size = 440
    tOpenCVRectificationInfo_384 ucOpenCV_rec_384;  //Altek's PD size = 384

}OpenCVK_824bytes;


////////////////////////////////////////////////////////////////
typedef struct{
    double eErr_main;
    double eErr_sub;
    double eErr_stereo;

    int dErrCode;
}OpenCVK_LogStruct;

typedef struct {
    tOpenCVRectificationInfo tRectifyInfo;
    double eFOV;//rectify FOV
    double aeEulerAngle_relative[3];//relative R
    double aeEulerAngle_main[3];//main rotation of rectification
    double aeEulerAngle_sub[3];//sub rotation of rectification
}OpenCVK_DebugStruct;


typedef struct OpenCVK
{
    unsigned char ucAltek_PD[512];//Altek's PD size = 512
    unsigned char ucOpenCV_PD[440];//Altek's PD size = 440
    unsigned char ucOpenCV_Rec_PD[384];//Altek's PD size = 384

    OpenCVK_LogStruct tOpenCVK_LogStruct;
    OpenCVK_DebugStruct tOpenCVK_DebugStruct;

}OpenCVK_Data_Struct;

