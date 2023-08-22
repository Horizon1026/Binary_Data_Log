#include "binary_data_log.h"
#include "slam_operations.h"
#include "log_report.h"

namespace SLAM_DATA_LOG {

bool BinaryDataLog::LoadLogFile(const std::string &log_file_name) {
    std::ifstream log_file;
    log_file.open(log_file_name.c_str(), std::ios::in | std::ios::binary);
    if (!log_file.is_open()) {
        ReportError("[DataLog] Cannot open log file : " << log_file_name);
        return false;
    }

    std::string temp_header = binary_log_file_header;
    log_file.read(temp_header.data(), binary_log_file_header.size());
    if (temp_header != binary_log_file_header) {
        ReportWarn("[DataLog] Log header error, it cannot be decoded.");
        return false;
    }

    packages_.clear();

    // Load offset index to the beginning of 'packages_content'.
    uint32_t offset_to_data_part = 0;
    log_file.read(reinterpret_cast<char *>(&offset_to_data_part), 4);

    // Load information of all registered packages.
    uint32_t offset = 4 + 1;
    while (offset < offset_to_data_part) {
        uint32_t offset_to_next_package = 0;
        log_file.read(reinterpret_cast<char *>(&offset_to_next_package), 4);
        offset += offset_to_next_package;

        std::unique_ptr<Package> package_ptr = std::make_unique<Package>();
        log_file.read(reinterpret_cast<char *>(&package_ptr->id), 2);

        uint8_t package_name_length = 0;
        log_file.read(reinterpret_cast<char *>(&package_name_length), 1);
        package_ptr->name = LoadStringFromBinaryFile(log_file, package_name_length);

        uint32_t offset_in_package = 4 + 2 + 1 + package_name_length + 1;
        while (offset_in_package < offset_to_next_package) {

            package_ptr->items.emplace_back(PackageItem());
            auto &new_item = package_ptr->items.back();

            log_file.read(reinterpret_cast<char *>(&new_item.type), 1);

            uint8_t item_name_length = 0;
            log_file.read(reinterpret_cast<char *>(&item_name_length), 1);
            new_item.name = LoadStringFromBinaryFile(log_file, item_name_length);

            offset_in_package += item_name_length + 2;
        }

        uint8_t sum_check_byte = 0;
        log_file.read(reinterpret_cast<char *>(&sum_check_byte), 1);
        // TODO: summary check byte.

        RegisterPackage(package_ptr);
    }

    return true;
}

}
