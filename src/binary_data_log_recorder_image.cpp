#include "binary_data_log.h"
#include "slam_operations.h"
#include "log_report.h"

#include "chrono"

namespace SLAM_DATA_LOG {

bool BinaryDataLog::RecordPackage(const uint16_t package_id, const GrayImage &image) {
    return RecordImage(package_id, 1, image.rows(), image.cols(), image.data());
}

bool BinaryDataLog::RecordPackage(const uint16_t package_id, const RgbImage &image) {
    return RecordImage(package_id, 3, image.rows(), image.cols(), image.data());
}

bool BinaryDataLog::RecordImage(const uint16_t package_id,
                                const int32_t channels,
                                const int32_t image_rows,
                                const int32_t image_cols,
                                const uint8_t *data_ptr) {
    RETURN_FALSE_IF(file_w_ptr_ == nullptr);
    RETURN_FALSE_IF(data_ptr == nullptr);

    const auto it = packages_id_with_objects_.find(package_id);
    if (it == packages_id_with_objects_.end()) {
        ReportError("[DataLog] Package id " << package_id << " is not registered.");
        return false;
    }

    // Check image size.
    const uint32_t pixel_value_size = channels * image_rows * image_cols;
    // pixel_value, channel, rows, cols.
    const uint32_t image_data_size = pixel_value_size + 1 + 2 + 2;

    // Write the offset to the next package data.
    // offset, package_id, timestamp, binary_data, check_byte.
    const uint32_t offset = 4 + 2 + 4 + image_data_size + 1;
    file_w_ptr_->write(reinterpret_cast<const char *>(&offset), 4);
    uint8_t sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(&offset), 4, 0);

    // Write the package id.
    file_w_ptr_->write(reinterpret_cast<const char *>(&it->first), 2);
    sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(&it->first), 2, sum_check_byte);

    // Write the system timestamp.
    const float timestamp = GetSystemTimestamp();
    file_w_ptr_->write(reinterpret_cast<const char *>(&timestamp), 4);
    sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(&timestamp), 4, sum_check_byte);

    // Write the binary data.
    // Write image channels.
    uint8_t temp_channels = static_cast<uint8_t>(channels);
    file_w_ptr_->write(reinterpret_cast<const char *>(&temp_channels), 1);
    sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(&temp_channels), 1, sum_check_byte);
    // Write image rows/height.
    uint16_t temp_rows = static_cast<uint16_t>(image_rows);
    file_w_ptr_->write(reinterpret_cast<const char *>(&temp_rows), 2);
    sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(&temp_rows), 2, sum_check_byte);
    // Write image cols/width.
    uint16_t temp_cols = static_cast<uint16_t>(image_cols);
    file_w_ptr_->write(reinterpret_cast<const char *>(&temp_cols), 2);
    sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(&temp_cols), 2, sum_check_byte);
    // Write image pixel value.
    file_w_ptr_->write(reinterpret_cast<const char *>(data_ptr), pixel_value_size);
    sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(data_ptr), pixel_value_size, sum_check_byte);

    // Write the summary check byte.
    file_w_ptr_->write(reinterpret_cast<const char *>(&sum_check_byte), 1);
    return true;
}

}
