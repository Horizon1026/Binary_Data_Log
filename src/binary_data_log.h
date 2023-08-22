#ifndef _BINARY_DATA_LOG_H_
#define _BINARY_DATA_LOG_H_

#include "binary_data_log_file_protocal.h"

#include "iostream"
#include "fstream"
#include "memory"
#include "string"
#include "vector"
#include "unordered_map"

namespace SLAM_DATA_LOG {

/* Class BinaryDataLog Declaration. */
class BinaryDataLog {

public:
    BinaryDataLog() = default;
    virtual ~BinaryDataLog();

    bool CreateLogFile(const std::string &log_file_name = "data.binlog");

    bool RegisterPackage(std::unique_ptr<Package> &new_package);

    bool PrepareForRecording();

    bool RecordPackage(const uint16_t package_id,
                       const char *data_ptr);

    void ReportAllRegisteredPackages();

private:
    void WriteLogFileHeader();
    bool WriteAllRegisteredPackages();
    uint8_t SummaryBytes(const uint8_t *byte_ptr, const uint32_t size, const uint8_t init_value);

private:
    std::unique_ptr<std::fstream> file_ptr_ = nullptr;
    std::vector<std::unique_ptr<Package>> packages_;
    std::unordered_map<uint16_t, uint32_t> packages_id_with_size_;
};

}

#endif // end of _BINARY_DATA_LOG_H_
