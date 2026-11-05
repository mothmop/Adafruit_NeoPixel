#include "StackingEngine.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <thread>
#include <cmath>

static constexpr int BACKLASH_REVERSE_MARGIN_STEPS = 50;
static constexpr int MECHANICAL_SETTLE_MS          = 350;
static constexpr int SHUTTER_CYCLE_MS             = 1200;
static constexpr int USB_COMMAND_INTERVAL_MS      = 15;

StackingEngine::StackingEngine(const std::string& cal_csv_path, const std::string& log_file_path)
    : csv_path_(cal_csv_path), log_path_(log_file_path) {
    log_stream_.open(log_path_, std::ios::out | std::ios::app);
    ensureCalibrationCSV();
    loadCalibrationData();
}

StackingEngine::~StackingEngine() {
    disconnectCamera();
    SCRSDK::Release();
    if (log_stream_.is_open()) {
        log_stream_.close();
    }
}

void StackingEngine::log(const std::string& level, const std::string& message) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    auto time = std::chrono::system_clock::to_time_t(now);

    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &time);
#else
    localtime_r(&time, &tm_buf);
#endif

    std::ostringstream ss;
    ss << "[" << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S")
       << "." << std::setfill('0') << std::setw(3) << ms.count() << "] "
       << "[" << level << "] " << message;

    std::cout << ss.str() << std::endl;
    if (log_stream_.is_open()) {
        log_stream_ << ss.str() << std::endl;
    }
}

void StackingEngine::sleepMs(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void StackingEngine::ensureCalibrationCSV() {
    std::ifstream check(csv_path_);
    if (!check.good()) {
        std::ofstream out(csv_path_);
        out << "Ruler_Distance_cm,True_Depth_mm,Steps_Per_mm\n";
        out << "2.0,10.0,24.5\n";
        out << "3.0,15.0,18.2\n";
        out << "4.0,20.0,14.1\n";
        out << "6.0,30.0,9.4\n";
        out << "8.0,40.0,6.8\n";
        out << "10.0,50.0,5.1\n";
        out.close();
        log("INFO", "Generated default calibration table: " + csv_path_);
    }
}

void StackingEngine::loadCalibrationData() {
    cal_table_.clear();
    std::ifstream in(csv_path_);
    std::string line;
    std::getline(in, line);

    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string r_str, d_str, s_str;
        if (std::getline(ss, r_str, ',') &&
            std::getline(ss, d_str, ',') &&
            std::getline(ss, s_str, ',')) {
            cal_table_.push_back({
                std::stod(r_str),
                std::stod(d_str),
                std::stod(s_str)
            });
        }
    }
    std::sort(cal_table_.begin(), cal_table_.end(),
              [](const CalibrationPoint& a, const CalibrationPoint& b) {
                  return a.ruler_cm < b.ruler_cm;
              });
}

double StackingEngine::interpolateStepsPerMm(double ruler_cm) {
    if (cal_table_.empty()) return 10.0;
    if (ruler_cm <= cal_table_.front().ruler_cm) return cal_table_.front().steps_per_mm;
    if (ruler_cm >= cal_table_.back().ruler_cm) return cal_table_.back().steps_per_mm;

    for (size_t i = 0; i < cal_table_.size() - 1; ++i) {
        if (ruler_cm >= cal_table_[i].ruler_cm && ruler_cm <= cal_table_[i + 1].ruler_cm) {
            double t = (ruler_cm - cal_table_[i].ruler_cm) /
                       (cal_table_[i + 1].ruler_cm - cal_table_[i].ruler_cm);
            return cal_table_[i].steps_per_mm + t * (cal_table_[i + 1].steps_per_mm - cal_table_[i].steps_per_mm);
        }
    }
    return cal_table_.back().steps_per_mm;
}

bool StackingEngine::initializeSDK() {
    auto res = SCRSDK::Init(0);
    if (!res) {
        log("ERROR", "SCRSDK::Init failed.");
        return false;
    }
    log("INFO", "Sony CRSDK Initialized.");
    return true;
}

bool StackingEngine::connectCamera() {
    SCRSDK::ICrEnumCameraObjectInfo* camera_list = nullptr;
    auto status = SCRSDK::EnumCameraObjects(&camera_list);

    if (status != 0 || !camera_list || camera_list->GetCount() == 0) {
        log("ERROR", "No USB camera found on bus.");
        return false;
    }

    auto* camera_info = camera_list->GetCameraObjectInfo(0);
    log("INFO", std::string("Device found: ") + camera_info->GetModel());

    SCRSDK::CrDeviceHandle handle = 0;
    auto connect_status = SCRSDK::Connect(camera_info, nullptr, &handle);
    if (connect_status != 0) {
        log("ERROR", "CRSDK::Connect failed with code: " + std::to_string(connect_status));
        return false;
    }

    device_handle_ = handle;
    connected_ = true;
    log("INFO", "Camera handshake established successfully.");
    return true;
}

void StackingEngine::disconnectCamera() {
    if (connected_ && device_handle_) {
        SCRSDK::Disconnect(device_handle_);
        connected_ = false;
        device_handle_ = 0;
        log("INFO", "Disconnected camera.");
    }
}

void StackingEngine::driveFocus(SCRSDK::CrFocusPositionDrive value, int pulses) {
    for (int i = 0; i < pulses; ++i) {
        if (connected_ && device_handle_) {
            SCRSDK::SendCommand(device_handle_, SCRSDK::CrCommandId::CrCommandId_FocusPositionDrive, value);
        }
        sleepMs(USB_COMMAND_INTERVAL_MS);
    }
}

void StackingEngine::triggerShutter() {
    if (connected_ && device_handle_) {
        SCRSDK::SendCommand(device_handle_, SCRSDK::CrCommandId::CrCommandId_Release, SCRSDK::CrCommandParam::CrCommandParam_Down);
        sleepMs(100);
        SCRSDK::SendCommand(device_handle_, SCRSDK::CrCommandId::CrCommandId_Release, SCRSDK::CrCommandParam::CrCommandParam_Up);
    }
    sleepMs(SHUTTER_CYCLE_MS);
}

void StackingEngine::executeSweep(double range_mm, int frame_count, double ruler_cm) {
    if (frame_count < 2) frame_count = 2;

    double steps_per_mm = interpolateStepsPerMm(ruler_cm);
    double step_distance_mm = range_mm / static_cast<double>(frame_count - 1);
    int step_pulses_per_frame = std::max(1, static_cast<int>(std::round(step_distance_mm * steps_per_mm)));

    std::ostringstream ss;
    ss << "Configured Sweep: Depth Range=" << range_mm << "mm | Frames=" << frame_count
       << " | Ruler=" << ruler_cm << "cm | Scalar=" << steps_per_mm << " steps/mm";
    log("INFO", ss.str());

    // Backlash Elimination: Reverse margin steps Far, then advance margin steps Near to seat gear teeth at Z=0
    log("INFO", "Backlash compensation: reversing past start plane...");
    driveFocus(SCRSDK::CrFocusPositionDrive::CrFocusPositionDrive_Far_SubSmall, BACKLASH_REVERSE_MARGIN_STEPS);
    sleepMs(MECHANICAL_SETTLE_MS);

    log("INFO", "Advancing forward by margin to seat gear teeth at Z=0 origin...");
    driveFocus(SCRSDK::CrFocusPositionDrive::CrFocusPositionDrive_Near_SubSmall, BACKLASH_REVERSE_MARGIN_STEPS);
    sleepMs(MECHANICAL_SETTLE_MS);

    for (int frame = 0; frame < frame_count; ++frame) {
        double current_z = frame * step_distance_mm;
        log("INFO", "Frame " + std::to_string(frame + 1) + "/" + std::to_string(frame_count) +
                    " at Z=" + std::to_string(current_z) + "mm");

        sleepMs(MECHANICAL_SETTLE_MS);
        triggerShutter();

        if (frame < frame_count - 1) {
            driveFocus(SCRSDK::CrFocusPositionDrive::CrFocusPositionDrive_Near_SubSmall, step_pulses_per_frame);
        }
    }
    log("INFO", "Focus stacking sweep complete.");
}
