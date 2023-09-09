#ifndef _BINARY_DATA_LOG_H_
#define _BINARY_DATA_LOG_H_

#include "binary_data_log_file_protocal.h"
#include "datatype_image.h"

#include "iostream"
#include "fstream"
#include "chrono"
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

    void CleanUp();
    static bool IsDynamicType(uint8_t type_code);
    static bool IsDynamicType(ItemType type);

    // Support for recorder.
    bool CreateLogFile(const std::string &log_file_name = "data.binlog");
    bool RegisterPackage(std::unique_ptr<PackageInfo> &new_package);
    bool PrepareForRecording();
    bool RecordPackage(const uint16_t package_id, const char *data_ptr);
    bool RecordPackage(const uint16_t package_id, const GrayImage &image);
    bool RecordPackage(const uint16_t package_id, const RgbImage &image);

    // Support for decoder.
    bool LoadLogFile(const std::string &log_file_name, bool load_dynamic_data = true);
    template <typename T> static T ConvertBytes(const uint8_t *bytes, ItemType type);
    uint8_t *LoadBinaryDataFromLogFile(uint32_t index_in_file, uint32_t size);

    // Support for information.
    void ReportAllRegisteredPackages();
    void ReportAllLoadedPackages();

    // Const Reference for member variables.
    // Support for decodec.
    const std::unique_ptr<std::ifstream> &file_r_ptr() const { return file_r_ptr_; }
    const std::unordered_map<uint16_t, std::unique_ptr<PackageInfo>> &packages_id_with_objects() const { return packages_id_with_objects_; }
    // Support for recorder.
    const std::unique_ptr<std::fstream> &file_w_ptr() const { return file_w_ptr_; }
    const std::chrono::time_point<std::chrono::system_clock> &start_system_time() const { return start_system_time_; }
    // Support for decoder.
    const std::unordered_map<uint16_t, std::vector<PackageDataPerTick>> &packages_id_with_data() const { return packages_id_with_data_; }

private:
    // Support for decodec.
    uint8_t SummaryBytes(const uint8_t *byte_ptr,
                         const uint32_t size,
                         const uint8_t init_value);
    std::string LoadStringFromBinaryFile(uint32_t size);

    // Support for recorder.
    void WriteLogFileHeader();
    bool RecordAllRegisteredPackages();
    float GetSystemTimestamp();
    bool RecordImage(const uint16_t package_id, const int32_t channels, const int32_t image_rows, const int32_t image_cols, const uint8_t *data_ptr);

    // Support for decoder.
    bool CheckLogFileHeader();
    bool LoadRegisteredPackages();
    bool LoadOnePackage(bool load_dynamic_data = true);
    bool LoadOnePackageWithStaticSize(uint8_t &sum_check_byte,
                                      PackageDataPerTick &timestamped_data,
                                      uint16_t package_id,
                                      uint32_t data_size,
                                      bool load_dynamic_data);
    bool LoadOnePackageWithDynamicSize(PackageInfo &package_info,
                                       uint8_t &sum_check_byte,
                                       PackageDataPerTick &timestamped_data,
                                       uint16_t package_id,
                                       bool load_dynamic_data);

private:
    // Support for decodec.
    std::unique_ptr<std::ifstream> file_r_ptr_ = nullptr;
    std::unordered_map<uint16_t, std::unique_ptr<PackageInfo>> packages_id_with_objects_;

    // Support for recorder.
    std::unique_ptr<std::fstream> file_w_ptr_ = nullptr;
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
        case ItemType::kInt16: {
            const SLAM_UTILITY::int16_t *data_ptr = reinterpret_cast<const SLAM_UTILITY::int16_t *>(bytes);
            return static_cast<T>(*data_ptr);
        }
        case ItemType::kUint32: {
            const SLAM_UTILITY::uint32_t *data_ptr = reinterpret_cast<const SLAM_UTILITY::uint32_t *>(bytes);
            return static_cast<T>(*data_ptr);
        }
        case ItemType::kInt32: {
            const SLAM_UTILITY::int32_t *data_ptr = reinterpret_cast<const SLAM_UTILITY::int32_t *>(bytes);
            return static_cast<T>(*data_ptr);
        }
        case ItemType::kUint64: {
            const SLAM_UTILITY::uint64_t *data_ptr = reinterpret_cast<const SLAM_UTILITY::uint64_t *>(bytes);
            return static_cast<T>(*data_ptr);
        }
        case ItemType::kInt64: {
            const SLAM_UTILITY::int64_t *data_ptr = reinterpret_cast<const SLAM_UTILITY::int64_t *>(bytes);
            return static_cast<T>(*data_ptr);
        }
        case ItemType::kFloat: {
            const float *data_ptr = reinterpret_cast<const float *>(bytes);
            return static_cast<T>(*data_ptr);
        }
        case ItemType::kDouble: {
            const double *data_ptr = reinterpret_cast<const double *>(bytes);
            return static_cast<T>(*data_ptr);
        }
        default:
            return static_cast<T>(0);
    }
}

}

#endif // end of _BINARY_DATA_LOG_H_
