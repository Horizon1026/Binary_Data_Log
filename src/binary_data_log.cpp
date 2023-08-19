#include "binary_data_log.h"
#include "log_report.h"

namespace SLAM_DATA {

BinaryDataLog::~BinaryDataLog() {
    if (file_ptr_ != nullptr) {
        file_ptr_->close();
        file_ptr_.reset();
    }
}

bool BinaryDataLog::CreateLogFile(const std::string &log_file_name) {
    // If last log file is not closed, close it.
    if (file_ptr_ != nullptr) {
        file_ptr_->close();
        file_ptr_.reset();
    }

    // Try to create new log file.
    file_ptr_ = std::make_unique<std::fstream>(log_file_name, std::ios::out | std::ios::binary);
    if (!file_ptr_->is_open()) {
        ReportError("[DataLog] Cannot create log file : " << log_file_name);
        return false;
    }

    return true;
}

}
