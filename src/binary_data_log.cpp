#include "binary_data_log.h"
#include "slam_operations.h"
#include "log_report.h"

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

    if (packages_.find(new_package->id) != packages_.end()) {
        ReportError("[DataLog] Package to be registered is exist now.");
        return false;
    }

    const uint16_t package_id = new_package->id;
    packages_[package_id] = std::move(new_package);
    return true;
}

bool BinaryDataLog::PrepareForRecording() {
    RETURN_FALSE_IF_FALSE(WriteAllRegisteredPackages());
    return true;
}

void BinaryDataLog::ReportAllRegisteredPackages() {
    ReportInfo("[DataLog] All registered packages:");
    for (const auto &package : packages_) {
        ReportInfo(">> Package name : " << package.second->name);
        ReportInfo("   Package id : " << package.second->id);
        ReportInfo("   Package items :");
        for (const auto &item : package.second->items) {
            ReportInfo("      [" << item_type_strings[static_cast<uint32_t>(item.type)] << "] : " << item.name);
        }
    }
}

void BinaryDataLog::WriteLogFileHeader() {
    const uint32_t header_offset = binary_log_file_header.size() + 4;
    file_ptr_->write(reinterpret_cast<const char *>(&header_offset), 4);
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
        offset_to_next_package += package.second->name.size();

        for (const auto &item : package.second->items) {
            offset_to_next_package += 1;    // Type of item.
            offset_to_next_package += 1;    // Length of item name string.
            offset_to_next_package += item.name.size();
        }

        offset_to_next_package += 1;    // Sum check of package.

        // The size/offset of this package is confirmed.
        offsets.emplace_back(offset_to_next_package);
        offsets.front() += offset_to_next_package;
    }

    // for (auto &item : offsets) {
    //     ReportDebug(item);
    // }

    return true;
}

}
