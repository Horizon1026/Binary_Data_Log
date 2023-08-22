#include "binary_data_log.h"
#include "slam_operations.h"
#include "log_report.h"

#include "chrono"

namespace SLAM_DATA_LOG {

bool BinaryDataLog::RecordAllRegisteredPackages() {
    if (packages_.empty()) {
        ReportInfo("[DataLog] No registered package.");
        return false;
    }

    // Statis the whole size.
    std::vector<uint32_t> offsets(1, 4);
    for (const auto &package : packages_) {
        uint32_t offset_to_next_package = 4;    // Offset itself.
        offset_to_next_package += 2;    // Package id.
        offset_to_next_package += 1;    // Length of package name string.
        offset_to_next_package += package->name.size();

        for (const auto &item : package->items) {
            offset_to_next_package += 1;    // Type of item.
            offset_to_next_package += 1;    // Length of item name string.
            offset_to_next_package += item.name.size();
        }

        offset_to_next_package += 1;    // Sum check of package.

        // The size/offset of this package is confirmed.
        offsets.emplace_back(offset_to_next_package);
        offsets.front() += offset_to_next_package;
    }

    // Write the offset index to the beginning of Part 3.
    file_ptr_->write(reinterpret_cast<const char *>(&offsets.front()), 4);

    // Iterate each package.
    uint32_t index = 1;
    for (auto &package : packages_) {
        // Write the offset index to the next package name.
        file_ptr_->write(reinterpret_cast<const char *>(&offsets[index]), 4);
        uint8_t sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(&offsets[index]), 4, sum_check_byte);
        ++index;
        // Write the package id.
        file_ptr_->write(reinterpret_cast<const char *>(&package->id), 2);
        sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(&package->id), 2, sum_check_byte);
        // Write the size of package name.
        const uint8_t name_size = static_cast<uint8_t>(package->name.size());
        file_ptr_->write(reinterpret_cast<const char *>(&name_size), 1);
        sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(&name_size), 1, sum_check_byte);
        // Write the package name.
        file_ptr_->write(package->name.c_str(), package->name.size());
        sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(package->name.c_str()), package->name.size(), sum_check_byte);

        // Iterate each item of this package.
        for (auto &item : package->items) {
            // Write the type of item.
            file_ptr_->write(reinterpret_cast<const char *>(&item.type), 1);
            sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(&item.type), 1, sum_check_byte);
            // Write the size of item name.
            const uint8_t name_size = static_cast<uint8_t>(item.name.size());
            file_ptr_->write(reinterpret_cast<const char *>(&name_size), 1);
            sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(&name_size), 1, sum_check_byte);
            // Write the item name.
            file_ptr_->write(item.name.c_str(), item.name.size());
            sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(item.name.c_str()), item.name.size(), sum_check_byte);
        }

        // Write the summary check byte.
        file_ptr_->write(reinterpret_cast<const char *>(&sum_check_byte), 1);
    }

    return true;
}

bool BinaryDataLog::RecordPackage(const uint16_t package_id,
                                  const char *data_ptr) {
    RETURN_FALSE_IF(file_ptr_ == nullptr);
    RETURN_FALSE_IF(data_ptr == nullptr);

    const auto it = packages_id_with_size_.find(package_id);
    if (it == packages_id_with_size_.end()) {
        ReportError("[DataLog] Package id " << package_id << " is not registered.");
        return false;
    }

    // Write the offset to the next package data.
    const uint32_t offset = 4 + 2 + 4 + it->second + 1;
    file_ptr_->write(reinterpret_cast<const char *>(&offset), 4);
    uint8_t sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(&offset), 4, 0);

    // Write the package id.
    file_ptr_->write(reinterpret_cast<const char *>(&it->first), 2);
    sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(&it->first), 2, sum_check_byte);

    // Write the system timestamp.
    std::chrono::time_point<std::chrono::system_clock> new_time_point = std::chrono::system_clock::now();
    std::chrono::duration<float> elapsed_seconds = new_time_point - start_system_time_;
    const uint32_t timestamp = static_cast<uint32_t>(elapsed_seconds.count() * 1e3f);
    file_ptr_->write(reinterpret_cast<const char *>(&timestamp), 4);
    sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(&timestamp), 4, sum_check_byte);

    // Write the binary data.
    file_ptr_->write(data_ptr, it->second);
    sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(data_ptr), it->second, sum_check_byte);

    // Write the summary check byte.
    file_ptr_->write(reinterpret_cast<const char *>(&sum_check_byte), 1);

    return true;
}

}
