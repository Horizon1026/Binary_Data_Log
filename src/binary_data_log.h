#ifndef _BINARY_DATA_LOG_H_
#define _BINARY_DATA_LOG_H_

#include "binary_data_log_file_protocal.h"

#include "iostream"
#include "fstream"
#include "chrono"
#include "memory"

#include "string"
#include "vector"
#include "unordered_map"

namespace SLAM_DATA_LOG {

struct TimestampedData {
    uint32_t timestamp_ms = 0;
    std::vector<uint8_t> data;
};

/* Class BinaryDataLog Declaration. */
class BinaryDataLog {

public:
    BinaryDataLog() = default;
    virtual ~BinaryDataLog();

    // Support for recorder.
    bool CreateLogFile(const std::string &log_file_name = "data.binlog");
    bool RegisterPackage(std::unique_ptr<Package> &new_package);
    bool PrepareForRecording();
    bool RecordPackage(const uint16_t package_id,
                       const char *data_ptr);

    // Support for decoder.
    bool LoadLogFile(const std::string &log_file_name);

    // Support for information.
    void ReportAllRegisteredPackages();

private:
    // Support for decodec.
    uint8_t SummaryBytes(const uint8_t *byte_ptr,
                         const uint32_t size,
                         const uint8_t init_value);
    std::string LoadStringFromBinaryFile(std::ifstream &log_file,
                                         uint32_t size);

    // Support for recorder.
    void WriteLogFileHeader();
    bool RecordAllRegisteredPackages();

    // Support for decoder.
    bool LoadOnePackage(std::ifstream &log_file);

private:
    // Support for decodec.
    std::vector<std::unique_ptr<Package>> packages_;
    std::unordered_map<uint16_t, uint32_t> packages_id_with_size_;

    // Support for recorder.
    std::unique_ptr<std::fstream> file_ptr_ = nullptr;
    std::chrono::time_point<std::chrono::system_clock> start_system_time_ = std::chrono::system_clock::now();

    // Support for decoder.
    std::unordered_map<uint16_t, std::vector<TimestampedData>> packages_id_with_data_;
};

}

#endif // end of _BINARY_DATA_LOG_H_
