#ifndef _BINARY_DATA_LOG_H_
#define _BINARY_DATA_LOG_H_

#include "binary_data_log_basic_type.h"
#include "binary_data_log_file_protocal.h"
#include "datatype_image.h"

#include "chrono"
#include "fstream"
#include "iostream"
#include "memory"

#include "string"
#include "unordered_map"
#include "vector"

namespace slam_data_log {

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
    bool RecordPackage(const uint16_t package_id, const char *data_ptr, const float time_stamp_s);
    bool RecordPackage(const uint16_t package_id, const GrayImage &image);
    bool RecordPackage(const uint16_t package_id, const GrayImage &image, const float time_stamp_s);
    bool RecordPackage(const uint16_t package_id, const RgbImage &image);
    bool RecordPackage(const uint16_t package_id, const RgbImage &image, const float time_stamp_s);
    bool RecordPackage(const uint16_t package_id, const std::vector<uint8_t> &data_bytes, const ItemType type);
    bool RecordPackage(const uint16_t package_id, const std::vector<uint8_t> &data_bytes, const ItemType type, const float time_stamp_s);
    bool RecordPackage(const uint16_t package_id, const Mat &matrix);
    bool RecordPackage(const uint16_t package_id, const Mat &matrix, const float time_stamp_s);
    bool RecordPackage(const uint16_t package_id, const std::vector<Vec3> &points_cloud);
    bool RecordPackage(const uint16_t package_id, const std::vector<Vec3> &points_cloud, const float time_stamp_s);
    bool RecordPackage(const uint16_t package_id, const std::vector<Vec3> &points_cloud, const int32_t step);
    bool RecordPackage(const uint16_t package_id, const std::vector<Vec3> &points_cloud, const int32_t step, const float time_stamp_s);
    bool RecordPackage(const uint16_t package_id, const std::vector<std::pair<Vec3, Vec3>> &line_cloud);
    bool RecordPackage(const uint16_t package_id, const std::vector<std::pair<Vec3, Vec3>> &line_cloud, const float time_stamp_s);

    // Support for decoder.
    bool LoadLogFile(const std::string &log_file_name, bool load_dynamic_package_full_data = false);
    template <typename T>
    static T ConvertBytes(const uint8_t *bytes, ItemType type);
    template <typename T>
    static T ConvertBytes(const uint8_t *bytes, ItemType type, DecodeType decoder);
    uint8_t *LoadBinaryDataFromLogFile(uint64_t index_in_file, uint32_t size);

    // Support for csv loader.
    static bool CreateLogFileByCsvFile(const std::string &csv_file_name, const std::string &log_file_name = "data.binlog");

    // Support for information.
    void ReportAllRegisteredPackages();
    void ReportAllLoadedPackages();

    // Reference for member variables.
    float &current_recorded_time_stamp_s() { return current_recorded_time_stamp_s_; }

    // Const Reference for member variables.
    // Support for decodec.
    const std::unique_ptr<std::ifstream> &file_r_ptr() const { return file_r_ptr_; }
    const std::map<uint16_t, std::unique_ptr<PackageInfo>> &packages_id_with_objects() const { return packages_id_with_objects_; }
    const std::pair<float, float> &timestamp_s_range_of_loaded_log() const { return timestamp_s_range_of_loaded_log_; }
    // Support for recorder.
    const std::unique_ptr<std::fstream> &file_w_ptr() const { return file_w_ptr_; }
    const std::chrono::time_point<std::chrono::system_clock> &start_system_time() const { return start_system_time_; }
    const float &current_recorded_time_stamp_s() const { return current_recorded_time_stamp_s_; }
    // Support for decoder.
    const std::unordered_map<uint16_t, std::vector<PackageDataPerTick>> &packages_id_with_data() const { return packages_id_with_data_; }

private:
    // Support for decodec.
    uint8_t SummaryBytes(const uint8_t *byte_ptr, const uint32_t size, const uint8_t init_value);
    std::string LoadStringFromBinaryFile(uint32_t size);

    // Support for recorder.
    void WriteLogFileHeader();
    bool RecordAllRegisteredPackagesAsFileHead();
    float GetSystemTimestamp();
    bool RecordImage(const uint16_t package_id, const int32_t channels, const int32_t image_rows, const int32_t image_cols, const uint8_t *data_ptr,
                     const float time_stamp_s);

    // Support for decoder.
    bool CheckLogFileHeader();
    bool LoadRegisteredPackagesFromFileHead();
    bool LoadOnePackage(bool load_full_data = true);
    bool LoadOnePackageWithStaticSize(uint8_t &sum_check_byte, PackageDataPerTick &timestamped_data, uint16_t package_id, uint32_t data_size,
                                      bool load_full_data);
    bool LoadOnePackageWithDynamicSize(PackageInfo &package_info, uint8_t &sum_check_byte, PackageDataPerTick &timestamped_data, uint16_t package_id,
                                       bool load_full_data);

    // Support for csv loader.
    static bool ParseTimestampInCsvHeader(const std::string &csv_header_name, double &timestamp_scale);

private:
    // Support for decodec.
    std::unique_ptr<std::ifstream> file_r_ptr_ = nullptr;
    std::map<uint16_t, std::unique_ptr<PackageInfo>> packages_id_with_objects_;
    std::pair<float, float> timestamp_s_range_of_loaded_log_ = std::make_pair(0, 0);

    // Support for recorder.
    std::unique_ptr<std::fstream> file_w_ptr_ = nullptr;
    std::chrono::time_point<std::chrono::system_clock> start_system_time_ = std::chrono::system_clock::now();
    float current_recorded_time_stamp_s_ = 0.0f;

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
            const slam_utility::int8_t *data_ptr = reinterpret_cast<const slam_utility::int8_t *>(bytes);
            return static_cast<T>(*data_ptr);
        }
        case ItemType::kUint16: {
            const slam_utility::uint16_t *data_ptr = reinterpret_cast<const slam_utility::uint16_t *>(bytes);
            return static_cast<T>(*data_ptr);
        }
        case ItemType::kInt16: {
            const slam_utility::int16_t *data_ptr = reinterpret_cast<const slam_utility::int16_t *>(bytes);
            return static_cast<T>(*data_ptr);
        }
        case ItemType::kUint32: {
            const slam_utility::uint32_t *data_ptr = reinterpret_cast<const slam_utility::uint32_t *>(bytes);
            return static_cast<T>(*data_ptr);
        }
        case ItemType::kInt32: {
            const slam_utility::int32_t *data_ptr = reinterpret_cast<const slam_utility::int32_t *>(bytes);
            return static_cast<T>(*data_ptr);
        }
        case ItemType::kUint64: {
            const slam_utility::uint64_t *data_ptr = reinterpret_cast<const slam_utility::uint64_t *>(bytes);
            return static_cast<T>(*data_ptr);
        }
        case ItemType::kInt64: {
            const slam_utility::int64_t *data_ptr = reinterpret_cast<const slam_utility::int64_t *>(bytes);
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

template <typename T>
T BinaryDataLog::ConvertBytes(const uint8_t *bytes, ItemType type, DecodeType decoder) {
    if (type != ItemType::kFloat) {
        return static_cast<T>(0);
    }
    const float *p = reinterpret_cast<const float *>(bytes);
    constexpr float kRadToDeg = 57.295779579f;
    float value = 0.0f;
    switch (decoder) {
        case DecodeType::kQuaternionToRoll: {
            value = std::atan2(2.0f * (p[0] * p[1] + p[2] * p[3]), 1.0f - 2.0f * (p[1] * p[1] + p[2] * p[2])) * kRadToDeg;
            break;
        }
        case DecodeType::kQuaternionToPitch: {
            value = std::asin(2.0f * (p[0] * p[2] - p[3] * p[1])) * kRadToDeg;
            break;
        }
        case DecodeType::kQuaternionToYaw: {
            value = std::atan2(2.0f * (p[0] * p[3] + p[1] * p[2]), 1.0f - 2.0f * (p[2] * p[2] + p[3] * p[3])) * kRadToDeg;
            break;
        }
        case DecodeType::kVector2dToMod: {
            value = std::sqrt(p[0] * p[0] + p[1] * p[1]);
            break;
        }
        case DecodeType::kVector3dToMod: {
            value = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
            break;
        }

        default:
            break;
    }
    return static_cast<T>(value);
}

}  // namespace slam_data_log

#endif  // end of _BINARY_DATA_LOG_H_
