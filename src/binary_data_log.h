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

struct PackageDataPerTick {
    uint32_t timestamp_ms = 0;
    std::vector<uint8_t> data;
    uint32_t index_in_file = 0;
};

/* Class BinaryDataLog Declaration. */
class BinaryDataLog {

public:
    BinaryDataLog() = default;
    virtual ~BinaryDataLog();

    void CleanUp();

    template <typename T>
    static T ConvertBytes(const uint8_t *bytes, ItemType type);

    // Support for recorder.
    bool CreateLogFile(const std::string &log_file_name = "data.binlog");
    bool RegisterPackage(std::unique_ptr<PackageInfo> &new_package);
    bool PrepareForRecording();
    bool RecordPackage(const uint16_t package_id, const char *data_ptr);

    // Support for decoder.
    bool LoadLogFile(const std::string &log_file_name, bool set_load_data = true);

    // Support for information.
    void ReportAllRegisteredPackages();
    void ReportAllLoadedPackages();

    // Const Reference for member variables.
    // Support for decodec.
    const std::unordered_map<uint16_t, std::unique_ptr<PackageInfo>> &packages_id_with_objects() const { return packages_id_with_objects_; }
    // Support for recorder.
    const std::unique_ptr<std::fstream> &file_ptr() const { return file_ptr_; }
    const std::chrono::time_point<std::chrono::system_clock> &start_system_time() const { return start_system_time_; }
    // Support for decoder.
    const std::unordered_map<uint16_t, std::vector<PackageDataPerTick>> &packages_id_with_data() const { return packages_id_with_data_; }

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
    bool CheckLogFileHeader(std::ifstream &log_file);
    bool LoadRegisteredPackages(std::ifstream &log_file);
    bool LoadOnePackage(std::ifstream &log_file, bool set_load_data = true);
    bool PreloadOnePackage(std::ifstream &log_file);

private:
    // Support for decodec.
    std::unordered_map<uint16_t, std::unique_ptr<PackageInfo>> packages_id_with_objects_;

    // Support for recorder.
    std::unique_ptr<std::fstream> file_ptr_ = nullptr;
    std::chrono::time_point<std::chrono::system_clock> start_system_time_ = std::chrono::system_clock::now();

    // Support for decoder.
    std::unordered_map<uint16_t, std::vector<PackageDataPerTick>> packages_id_with_data_;
};

/* Class BinaryDataLog Definition. */
template <typename T>
T BinaryDataLog::ConvertBytes(const uint8_t *bytes, ItemType type) {
    switch (type) {
        case ItemType::kUint8: {
            return static_cast<T>(*bytes);
        }
        case ItemType::kInt8: {
            const SLAM_UTILITY::int8_t *data_ptr = reinterpret_cast<const SLAM_UTILITY::int8_t *>(bytes);
            return static_cast<T>(*data_ptr);
        }
        case ItemType::kUint16: {
            const SLAM_UTILITY::uint16_t *data_ptr = reinterpret_cast<const SLAM_UTILITY::uint16_t *>(bytes);
            return static_cast<T>(*data_ptr);
        }
        // case ItemType::kInt16:
        //     return static_cast<T>(*reinterpret_cast<const int16_t *>(bytes));
        // case ItemType::kUint32:
        //     return static_cast<T>(*reinterpret_cast<const uint32_t *>(bytes));
        // case ItemType::kInt32:
        //     return static_cast<T>(*reinterpret_cast<const int32_t *>(bytes));
        // case ItemType::kUint64:
        //     return static_cast<T>(*reinterpret_cast<const uint64_t *>(bytes));
        // case ItemType::kInt64:
        //     return static_cast<T>(*reinterpret_cast<const int64_t *>(bytes));
        // case ItemType::kFloat:
        //     return static_cast<T>(*reinterpret_cast<const float *>(bytes));
        // case ItemType::kDouble:
        //     return static_cast<T>(*reinterpret_cast<const double *>(bytes));
        default:
            return static_cast<T>(0);
    }
}

}

#endif // end of _BINARY_DATA_LOG_H_
