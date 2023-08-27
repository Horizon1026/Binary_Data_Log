#include "binary_data_log.h"
#include "slam_operations.h"
#include "log_report.h"

namespace SLAM_DATA_LOG {

bool BinaryDataLog::LoadLogFile(const std::string &log_file_name, bool set_load_data) {
    std::ifstream log_file;
    log_file.open(log_file_name.c_str(), std::ios::in | std::ios::binary);
    if (!log_file.is_open()) {
        ReportError("[DataLog] Cannot open log file : " << log_file_name);
        return false;
    }

    // Check header.
    RETURN_FALSE_IF_FALSE(CheckLogFileHeader(log_file));

    // Load all registered packages information.
    RETURN_FALSE_IF_FALSE(LoadRegisteredPackages(log_file));

    // Load all data.
    while (1) {
        BREAK_IF(!LoadOnePackage(log_file, set_load_data));
    }

    return true;
}

bool BinaryDataLog::CheckLogFileHeader(std::ifstream &log_file) {
    log_file.seekg(0);

    std::string temp_header = binary_log_file_header;
    log_file.read(temp_header.data(), binary_log_file_header.size());
    if (temp_header != binary_log_file_header) {
        ReportWarn("[DataLog] Log header error, it cannot be decoded.");
        return false;
    }
    return true;
}

bool BinaryDataLog::LoadRegisteredPackages(std::ifstream &log_file) {
    log_file.seekg(binary_log_file_header.size());

    packages_id_with_objects_.clear();

    // Load offset index to the beginning of 'packages_content'.
    uint32_t offset_to_data_part = 0;
    log_file.read(reinterpret_cast<char *>(&offset_to_data_part), 4);

    // Load information of all registered packages.
    uint32_t offset = 4 + 1;
    while (offset < offset_to_data_part) {
        // Load offset to the next package.
        uint32_t offset_to_next_package = 0;
        log_file.read(reinterpret_cast<char *>(&offset_to_next_package), 4);
        uint8_t sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(&offset_to_next_package), 4, 0);

        offset += offset_to_next_package;

        // Load package id.
        std::unique_ptr<PackageInfo> package_ptr = std::make_unique<PackageInfo>();
        log_file.read(reinterpret_cast<char *>(&package_ptr->id), 2);
        sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(&package_ptr->id), 2, sum_check_byte);

        // Load package name length.
        uint8_t package_name_length = 0;
        log_file.read(reinterpret_cast<char *>(&package_name_length), 1);
        sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(&package_name_length), 1, sum_check_byte);

        // Load package name.
        package_ptr->name = LoadStringFromBinaryFile(log_file, package_name_length);
        sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(package_ptr->name.data()), package_name_length, sum_check_byte);

        uint32_t offset_in_package = 4 + 2 + 1 + package_name_length + 1;
        uint32_t item_data_index_in_package_data = 0;
        while (offset_in_package < offset_to_next_package) {
            package_ptr->items.emplace_back(PackageItemInfo());
            auto &new_item = package_ptr->items.back();
            new_item.bindata_index_in_package = item_data_index_in_package_data;

            // Load item type.
            log_file.read(reinterpret_cast<char *>(&new_item.type), 1);
            sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(&new_item.type), 1, sum_check_byte);
            item_data_index_in_package_data += item_type_sizes[static_cast<uint32_t>(new_item.type)];

            // Load item name length.
            uint8_t item_name_length = 0;
            log_file.read(reinterpret_cast<char *>(&item_name_length), 1);
            sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(&item_name_length), 1, sum_check_byte);

            // Load item name.
            new_item.name = LoadStringFromBinaryFile(log_file, item_name_length);
            sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(new_item.name.data()), item_name_length, sum_check_byte);

            offset_in_package += item_name_length + 2;
        }

        // Summary byte check.
        uint8_t loaded_sum_check_byte = 0;
        log_file.read(reinterpret_cast<char *>(&loaded_sum_check_byte), 1);

        if (sum_check_byte != loaded_sum_check_byte) {
            ReportWarn("[DataLog] Package summary check byte failed. Compute " <<
                static_cast<int32_t>(sum_check_byte) << " != " <<
                static_cast<int32_t>(loaded_sum_check_byte));
            return false;
        }

        RegisterPackage(package_ptr);
    }

    return true;
}

bool BinaryDataLog::LoadOnePackage(std::ifstream &log_file, bool set_load_data) {
    // Record the index in log file.
    PackageDataPerTick timestamped_data;
    timestamped_data.index_in_file = log_file.tellg();

    // Load offset to the next content.
    uint32_t offset_to_next_content = 0;
    log_file.read(reinterpret_cast<char *>(&offset_to_next_content), 4);
    uint8_t sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(&offset_to_next_content), 4, 0);

    // Check if this is the end of log file.
    RETURN_FALSE_IF(log_file.eof());

    // Load package id.
    uint16_t package_id = 0;
    log_file.read(reinterpret_cast<char *>(&package_id), 2);
    sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(&package_id), 2, sum_check_byte);

    // Check if this data package id is registered.
    const auto it = packages_id_with_objects_.find(package_id);
    if (it == packages_id_with_objects_.end()) {
        ReportWarn("[DataLog] Load one package data failed. Package id " << package_id <<
            " is not registered.");
        return false;
    }

    // Load system timestamp.
    log_file.read(reinterpret_cast<char *>(&timestamped_data.timestamp_ms), 4);
    sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(&timestamped_data.timestamp_ms), 4, sum_check_byte);

    // Load data.
    const uint32_t data_size = it->second->size;
    char *buffer = new char[data_size];
    log_file.read(buffer, data_size);
    sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(buffer), data_size, sum_check_byte);

    // Check summary byte.
    uint8_t loaded_sum_check_byte = 0;
    log_file.read(reinterpret_cast<char *>(&loaded_sum_check_byte), 1);
    if (loaded_sum_check_byte != sum_check_byte) {
        ReportWarn("[DataLog] Load one package data failed. Summary check error.");
        delete[] buffer;
        return false;
    }

    // Store this data package.
    auto &packages = packages_id_with_data_[package_id];
    packages.emplace_back(timestamped_data);

    if (set_load_data) {
        packages.back().data.reserve(data_size);
        for (uint32_t i = 0; i < data_size; ++i) {
            packages.back().data.emplace_back(buffer[i]);
        }
    }
    delete[] buffer;

    return true;
}


bool BinaryDataLog::PreloadOnePackage(std::ifstream &log_file) {
    // TODO:
    return true;
}

}
