#include "binary_data_log.h"
#include "slam_operations.h"
#include "log_report.h"

namespace SLAM_DATA_LOG {

bool BinaryDataLog::LoadLogFile(const std::string &log_file_name) {
    std::ifstream log_file;
    log_file.open(log_file_name.c_str(), std::ios::in | std::ios::binary);
    if (!log_file.is_open()) {
        ReportError("[DataLog] Cannot open log file : " << log_file_name);
        return false;
    }

    return true;
}

}
