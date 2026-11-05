#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <fstream>
#include <chrono>

// Sony Camera Remote SDK Header
#if __has_include("CameraRemote_SDK.h")
    #include "CameraRemote_SDK.h"
#else
    // Mock / Fallback definitions for compilation without SDK installed
    namespace SCRSDK {
        typedef void* CrDeviceHandle;
        enum CrFocusPositionDrive {
            CrFocusPositionDrive_Near_SubSmall = 1,
            CrFocusPositionDrive_Far_SubSmall  = 2
        };
        enum CrCommandId {
            CrCommandId_Release = 1,
            CrCommandId_FocusPositionDrive = 2
        };
        enum CrCommandParam {
            CrCommandParam_Up = 0,
            CrCommandParam_Down = 1
        };
        class ICrCameraObjectInfo {
        public:
            virtual const char* GetModel() const { return "Sony α7C (Mock)"; }
        };
        class ICrEnumCameraObjectInfo {
        public:
            virtual int GetCount() const { return 1; }
            virtual ICrCameraObjectInfo* GetCameraObjectInfo(int) const { 
                static ICrCameraObjectInfo info; 
                return &info; 
            }
        };
        inline bool Init(int) { return true; }
        inline void Release() {}
        inline int EnumCameraObjects(ICrEnumCameraObjectInfo** p) { 
            static ICrEnumCameraObjectInfo info; 
            *p = &info; 
            return 0; 
        }
        inline int Connect(ICrCameraObjectInfo*, void*, CrDeviceHandle* h) { 
            *h = (CrDeviceHandle)0x1234; 
            return 0; 
        }
        inline void Disconnect(CrDeviceHandle) {}
        inline int SendCommand(CrDeviceHandle, int, int) { return 0; }
    }
#endif

struct CalibrationPoint {
    double ruler_cm;
    double depth_mm;
    double steps_per_mm;
};

class StackingEngine {
public:
    StackingEngine(const std::string& cal_csv_path, const std::string& log_file_path);
    ~StackingEngine();

    bool initializeSDK();
    bool connectCamera();
    void disconnectCamera();
    void executeSweep(double range_mm, int frame_count, double ruler_cm);

private:
    std::string csv_path_;
    std::string log_path_;
    std::vector<CalibrationPoint> cal_table_;
    std::mutex log_mutex_;
    std::ofstream log_stream_;

    SCRSDK::CrDeviceHandle device_handle_ = 0;
    bool connected_ = false;

    void ensureCalibrationCSV();
    void loadCalibrationData();
    double interpolateStepsPerMm(double ruler_cm);

    void log(const std::string& level, const std::string& message);
    void driveFocus(SCRSDK::CrFocusPositionDrive value, int pulses);
    void triggerShutter();
    void sleepMs(int milliseconds);
};
