#include "binary_data_log.h"
#include "log_report.h"

namespace SLAM_DATA {

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

}
