#include "binary_data_log.h"
#include "slam_operations.h"
#include "log_report.h"

#include "chrono"

namespace SLAM_DATA_LOG {

BinaryDataLog::~BinaryDataLog() {
    if (file_ptr_ != nullptr) {
        file_ptr_->close();
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

    // Write log header.
    WriteLogFileHeader();

    return true;
}

bool BinaryDataLog::RegisterPackage(std::unique_ptr<Package> &new_package) {
    if (new_package == nullptr) {
        ReportError("[DataLog] Package to be registered is empty.");
        return false;
    }

    if (packages_id_with_size_.find(new_package->id) != packages_id_with_size_.end()) {
        ReportError("[DataLog] Package to be registered is exist now.");
        return false;
    }

    const uint16_t package_id = new_package->id;
    packages_.emplace_back(std::move(new_package));

    // Statis the whole size of binary data in this package.
    uint32_t size = 0;
    for (const auto &item : packages_.back()->items) {
        size += item_type_sizes[static_cast<uint32_t>(item.type)];
    }
    packages_id_with_size_.insert(std::make_pair(package_id, size));
    ReportDebug("package size is " << size);

    return true;
}

bool BinaryDataLog::PrepareForRecording() {
    RETURN_FALSE_IF_FALSE(WriteAllRegisteredPackages());
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
    uint32_t timestamp = 0xFFFF; // TODO:
    file_ptr_->write(reinterpret_cast<const char *>(&timestamp), 4);
    sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(&timestamp), 4, sum_check_byte);

    // Write the binary data.
    file_ptr_->write(data_ptr, it->second);
    sum_check_byte = SummaryBytes(reinterpret_cast<const uint8_t *>(data_ptr), it->second, sum_check_byte);

    // Write the summary check byte.
    file_ptr_->write(reinterpret_cast<const char *>(&sum_check_byte), 1);

    return true;
}

void BinaryDataLog::ReportAllRegisteredPackages() {
    ReportInfo("[DataLog] All registered packages:");
    for (const auto &package : packages_) {
        ReportInfo(">> Package name : " << package->name);
        ReportInfo("   Package id : " << package->id);
        ReportInfo("   Package items :");
        for (const auto &item : package->items) {
            ReportInfo("      [" << item_type_strings[static_cast<uint32_t>(item.type)] << "] : " << item.name);
        }
    }
}

void BinaryDataLog::WriteLogFileHeader() {
    file_ptr_->write(binary_log_file_header.c_str(), binary_log_file_header.size());
}

bool BinaryDataLog::WriteAllRegisteredPackages() {
    if (packages_.empty()) {
        ReportInfo("[DataLog] No registered package.");
        return false;
    }

    // Statis the whole size.
    std::vector<uint32_t> offsets(1, 0);
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

uint8_t BinaryDataLog::SummaryBytes(const uint8_t *byte_ptr, const uint32_t size, const uint8_t init_value) {
    uint8_t value = init_value;
    for (uint32_t i = 0; i < size; ++i) {
        value += byte_ptr[i];
    }
    return value;
}

}
