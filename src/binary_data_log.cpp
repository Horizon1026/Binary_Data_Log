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

    return true;
}

bool BinaryDataLog::PrepareForRecording() {
    RETURN_FALSE_IF_FALSE(RecordAllRegisteredPackages());
    return true;
}

void BinaryDataLog::WriteLogFileHeader() {
    file_ptr_->write(binary_log_file_header.c_str(), binary_log_file_header.size());
}

uint8_t BinaryDataLog::SummaryBytes(const uint8_t *byte_ptr, const uint32_t size, const uint8_t init_value) {
    uint8_t value = init_value;
    for (uint32_t i = 0; i < size; ++i) {
        value += byte_ptr[i];
    }
    return value;
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

}
