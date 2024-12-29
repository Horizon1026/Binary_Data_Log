#include "binary_data_log.h"
#include "slam_operations.h"
#include "slam_log_reporter.h"
#include "slam_memory.h"

namespace SLAM_DATA_LOG {

uint8_t *BinaryDataLog::LoadBinaryDataFromLogFile(uint64_t index_in_file, uint32_t size) {
    if (file_r_ptr_ == nullptr) {
        ReportError("[DataLog] file_r_ptr_ is nullptr.");
        return nullptr;
    }

    uint8_t *buff = new uint8_t[size];
    file_r_ptr_->seekg(index_in_file, std::ios::beg);
    file_r_ptr_->read(reinterpret_cast<char *>(buff), size);

    return buff;
}

bool BinaryDataLog::LoadLogFile(const std::string &log_file_name, bool load_dynamic_package_full_data) {
    // If last log file is not closed, close it.
    if (file_r_ptr_ != nullptr) {
        file_r_ptr_->close();
        file_r_ptr_.reset();
    }

    // Try to create new log file.
    file_r_ptr_ = std::make_unique<std::ifstream>(log_file_name, std::ios::in | std::ios::binary);
    if (!file_r_ptr_->is_open()) {
        ReportError("[DataLog] Cannot open log file : " << log_file_name);
        return false;
    }

    // Check header.
    RETURN_FALSE_IF_FALSE(CheckLogFileHeader());
    // Load all registered packages information.
    RETURN_FALSE_IF_FALSE(LoadRegisteredPackagesFromFileHead());
    // Load all data.
    timestamp_s_range_of_loaded_log_ = std::make_pair(INFINITY, -INFINITY);
    while (!file_r_ptr_->eof()) {
        // Break only when it is end or out of file.
        const uint64_t index_in_file_now = file_r_ptr_->tellg();
        file_r_ptr_->seekg(0, std::ios::end);
        const uint64_t index_in_file_end = file_r_ptr_->tellg();
        if (index_in_file_now >= index_in_file_end) {
            break;
        } else {
            file_r_ptr_->seekg(index_in_file_now, std::ios::beg);
        }

        // If one package is broken in this file, skip it and continue loading.
        LoadOnePackage(load_dynamic_package_full_data);
    }

    // Reopen this log file. If not do this, the belowing 'LoadBinaryDataFromLogFile' will make error.
    file_r_ptr_->close();
    file_r_ptr_ = std::make_unique<std::ifstream>(log_file_name, std::ios::in | std::ios::binary);
    return true;
}

bool BinaryDataLog::CheckLogFileHeader() {
    file_r_ptr_->seekg(0);

    std::string temp_header = binary_log_file_header;
    file_r_ptr_->read(temp_header.data(), binary_log_file_header.size());
    if (temp_header != binary_log_file_header) {
        ReportWarn("[DataLog] Log header error, it cannot be decoded.");
        return false;
    }
    return true;
}

bool BinaryDataLog::LoadRegisteredPackagesFromFileHead() {
    file_r_ptr_->seekg(binary_log_file_header.size());

    packages_id_with_objects_.clear();

    // Load offset index to the beginning of 'packages_content'.
    uint32_t offset_to_data_part = 0;
    file_r_ptr_->read(reinterpret_cast<char *>(&offset_to_data_part), 4);

    // Check if this is the end of log file.
    RETURN_FALSE_IF(file_r_ptr_->eof());

    // Load information of all registered packages.
    uint32_t offset = 4 + 1;
    while (offset < offset_to_data_part) {
        // Load offset to the next package.
        uint32_t offset_to_next_package = 0;
        file_r_ptr_->read(reinterpret_cast<char *>(&offset_to_next_package), 4);
        uint8_t sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(&offset_to_next_package), 4, 0);

        offset += offset_to_next_package;

        // Load information of a package.
        // Load package id.
        std::unique_ptr<PackageInfo> package_ptr = std::make_unique<PackageInfo>();
        file_r_ptr_->read(reinterpret_cast<char *>(&package_ptr->id), 2);
        sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(&package_ptr->id), 2, sum_check_byte);
        // Load package name length.
        uint8_t package_name_length = 0;
        file_r_ptr_->read(reinterpret_cast<char *>(&package_name_length), 1);
        sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(&package_name_length), 1, sum_check_byte);
        // Load package name.
        package_ptr->name = LoadStringFromBinaryFile(package_name_length);
        sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(package_ptr->name.data()), package_name_length, sum_check_byte);

        uint32_t offset_in_package = 4 + 2 + 1 + package_name_length + 1;
        uint32_t item_data_index_in_package_data = 0;
        while (offset_in_package < offset_to_next_package) {
            package_ptr->items.emplace_back(PackageItemInfo());
            auto &new_item = package_ptr->items.back();
            new_item.bindata_index_in_package = item_data_index_in_package_data;

            // Load information of an item.
            // Load item type.
            file_r_ptr_->read(reinterpret_cast<char *>(&new_item.type), 1);
            sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(&new_item.type), 1, sum_check_byte);
            item_data_index_in_package_data += item_type_sizes[static_cast<uint32_t>(new_item.type)];
            // Load item name length.
            uint8_t item_name_length = 0;
            file_r_ptr_->read(reinterpret_cast<char *>(&item_name_length), 1);
            sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(&item_name_length), 1, sum_check_byte);
            // Load item name.
            new_item.name = LoadStringFromBinaryFile(item_name_length);
            sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(new_item.name.data()), item_name_length, sum_check_byte);

            offset_in_package += item_name_length + 2;
        }

        // Summary byte check.
        uint8_t loaded_sum_check_byte = 0;
        file_r_ptr_->read(reinterpret_cast<char *>(&loaded_sum_check_byte), 1);
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

bool BinaryDataLog::LoadOnePackage(bool load_dynamic_package_full_data) {
    // Record the index in log file.
    PackageDataPerTick timestamped_data;
    timestamped_data.index_in_file = file_r_ptr_->tellg();

    // Load offset to the next content.
    uint32_t offset_to_next_content = 0;
    file_r_ptr_->read(reinterpret_cast<char *>(&offset_to_next_content), 4);
    uint8_t sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(&offset_to_next_content), 4, 0);
    timestamped_data.size_of_all_in_file = offset_to_next_content;

    // Check if this is the end of log file.
    RETURN_FALSE_IF(file_r_ptr_->eof());

    // Load package id.
    uint16_t package_id = 0;
    file_r_ptr_->read(reinterpret_cast<char *>(&package_id), 2);
    sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(&package_id), 2, sum_check_byte);

    // Check if this data package id is registered.
    const auto it = packages_id_with_objects_.find(package_id);
    if (it == packages_id_with_objects_.end()) {
        ReportWarn("[DataLog] Load one package data failed. Package id " << package_id << " is not registered.");
        // If this is not the end of file, locate to the position of next package.
        if (!file_r_ptr_->eof()) {
            file_r_ptr_->seekg(timestamped_data.index_in_file, std::ios::beg);
            file_r_ptr_->seekg(offset_to_next_content, std::ios::cur);
        }
        return false;
    }

    // Load system timestamp.
    file_r_ptr_->read(reinterpret_cast<char *>(&timestamped_data.timestamp_s), 4);
    sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(&timestamped_data.timestamp_s), 4, sum_check_byte);
    // Update timestamp range.
    timestamp_s_range_of_loaded_log_.first = std::min(timestamp_s_range_of_loaded_log_.first, timestamped_data.timestamp_s);
    timestamp_s_range_of_loaded_log_.second = std::max(timestamp_s_range_of_loaded_log_.second, timestamped_data.timestamp_s);

    // Load data.
    const uint32_t data_size = it->second->size;
    bool load_result = true;
    if (data_size == 0) {
        load_result = LoadOnePackageWithDynamicSize(*(it->second), sum_check_byte, timestamped_data, package_id, load_dynamic_package_full_data);
    } else {
        load_result = LoadOnePackageWithStaticSize(sum_check_byte, timestamped_data, package_id, data_size, true);
    }
    if (!load_result) {
        ReportWarn("[DataLog] Load one package data failed for checking byte. Index in file : " <<
            timestamped_data.index_in_file << ". Data size [" << data_size << "]. Skip to load next package.");
    }

    // Locate to the position of next package.
    file_r_ptr_->seekg(timestamped_data.index_in_file, std::ios::beg);
    file_r_ptr_->seekg(offset_to_next_content, std::ios::cur);

    return load_result;
}

bool BinaryDataLog::LoadOnePackageWithStaticSize(uint8_t &sum_check_byte,
                                                 PackageDataPerTick &timestamped_data,
                                                 uint16_t package_id,
                                                 uint32_t data_size,
                                                 bool load_full_data) {
    char *buffer = new char[data_size];
    file_r_ptr_->read(buffer, data_size);
    sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(buffer), data_size, sum_check_byte);

    // Check summary byte.
    uint8_t loaded_sum_check_byte = 0;
    file_r_ptr_->read(reinterpret_cast<char *>(&loaded_sum_check_byte), 1);
    if (loaded_sum_check_byte != sum_check_byte) {
        ReportWarn("[DataLog] Load one package data failed. Summary check error.");
        delete[] buffer;
        return false;
    }

    // Store this data package and check timestamp.
    auto &packages = packages_id_with_data_[package_id];
    if (!packages.empty() && timestamped_data.timestamp_s == packages.back().timestamp_s) {
        ReportWarn("[DataLog] Same timestamp " << timestamped_data.timestamp_s << "s of package is detected when decoding static size data package.");
    }
    packages.emplace_back(timestamped_data);

    if (load_full_data) {
        packages.back().data.reserve(data_size);
        for (uint32_t i = 0; i < data_size; ++i) {
            packages.back().data.emplace_back(buffer[i]);
        }
    }
    delete[] buffer;

    return true;
}

bool BinaryDataLog::LoadOnePackageWithDynamicSize(PackageInfo &package_info,
                                                  uint8_t &sum_check_byte,
                                                  PackageDataPerTick &timestamped_data,
                                                  uint16_t package_id,
                                                  bool load_full_data) {
    RETURN_FALSE_IF(package_info.items.empty());

    // Compute data size in different type.
    uint32_t data_size = 0;
    switch (package_info.items.front().type) {
        case ItemType::kImage: {
            uint8_t channels = 0;
            uint16_t image_rows = 0;
            uint16_t image_cols = 0;
            file_r_ptr_->read(reinterpret_cast<char *>(&channels), 1);
            file_r_ptr_->read(reinterpret_cast<char *>(&image_rows), 2);
            file_r_ptr_->read(reinterpret_cast<char *>(&image_cols), 2);

            data_size = 5 + channels * image_rows * image_cols;
            file_r_ptr_->seekg(-5, std::ios::cur);
            break;
        }

        case ItemType::kMatrix: {
            uint16_t matrix_rows = 0;
            uint16_t matrix_cols = 0;
            file_r_ptr_->read(reinterpret_cast<char *>(&matrix_rows), 2);
            file_r_ptr_->read(reinterpret_cast<char *>(&matrix_cols), 2);

            data_size = 4 + matrix_rows * matrix_cols * sizeof(float);
            file_r_ptr_->seekg(-4, std::ios::cur);
            break;
        }

        case ItemType::kPngImage: {
            uint32_t num_of_png_bytes = 0;
            file_r_ptr_->read(reinterpret_cast<char *>(&num_of_png_bytes), 4);
            data_size = 4 + num_of_png_bytes;
            file_r_ptr_->seekg(-4, std::ios::cur);
            break;
        }

        case ItemType::kPointCloud: {
            uint32_t num_of_points = 0;
            file_r_ptr_->read(reinterpret_cast<char *>(&num_of_points), 4);
            data_size = 4 + num_of_points * 3 * sizeof(float);
            file_r_ptr_->seekg(-4, std::ios::cur);
            break;
        }

        default:
            return false;
    }

    // Load total data of dynamic size package.
    RETURN_FALSE_IF(data_size < 1);
    char *buffer = new char[data_size];
    file_r_ptr_->read(buffer, data_size);
    sum_check_byte = SummaryBytes(reinterpret_cast<uint8_t *>(buffer), data_size, sum_check_byte);

    // Check summary byte.
    uint8_t loaded_sum_check_byte = 0;
    file_r_ptr_->read(reinterpret_cast<char *>(&loaded_sum_check_byte), 1);
    if (loaded_sum_check_byte != sum_check_byte) {
        ReportWarn("[DataLog] Load one package data failed. Summary check error.");
        delete[] buffer;
        return false;
    }

    // Store this data package and check timestamp.
    auto &packages = packages_id_with_data_[package_id];
    if (!packages.empty() && timestamped_data.timestamp_s == packages.back().timestamp_s) {
        ReportWarn("[DataLog] Same timestamp " << timestamped_data.timestamp_s << "s of package is detected when decoding dynamic size data package.");
    }
    packages.emplace_back(timestamped_data);

    // For package with dynamic size, only store the localtion of data will be better.
    if (load_full_data) {
        packages.back().data.reserve(data_size);
        for (uint32_t i = 0; i < data_size; ++i) {
            packages.back().data.emplace_back(buffer[i]);
        }
    }
    delete[] buffer;

    return true;
}

}
