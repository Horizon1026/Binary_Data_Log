#include "binary_data_log.h"
#include "visualizor_2d.h"
#include "slam_operations.h"
#include "slam_log_reporter.h"
#include "filesystem"
#include "unistd.h"
#include "iostream"
#include "dirent.h"
#include "vector"
#include "cstring"

using namespace SLAM_DATA_LOG;
using namespace SLAM_VISUALIZOR;

#include "enable_stack_backward.h"

int main(int argc, char **argv) {
    ReportColorWarn(">> Test csv file to binlog.");
    const std::string csv_file = argv[1];
    const std::string binlog_file = argc > 2 ? argv[2] : csv_file.substr(0, csv_file.find_last_of('.')) + ".binlog";
    ReportInfo("[input] CSV file: " << csv_file);
    ReportInfo("[output] Binlog file: " << binlog_file);

    // Print csv_header of csv file.
    std::ifstream csv_file_stream(csv_file.c_str());
    if (!csv_file_stream.is_open()) {
        ReportError("Failed to open csv file: " + csv_file);
        return 1;
    }
    // Divide csv csv_header line into csv_header_items.
    std::string csv_header;
    std::getline(csv_file_stream, csv_header);
    std::vector<std::string> csv_header_items;
    std::istringstream csv_header_stream(csv_header);
    std::string temp_str;
    while (std::getline(csv_header_stream, temp_str, ',')) {
        temp_str.erase(std::remove(temp_str.begin(), temp_str.end(), ' '), temp_str.end());
        csv_header_items.push_back(temp_str);
    }
    // Find the index of "time_stamp_s".
    int32_t time_stamp_index = -1;
    double time_stamp_scale = 1.0;
    for (uint32_t i = 0; i < csv_header_items.size(); i++) {
        if (csv_header_items[i] == "timestamp") {
            time_stamp_index = i;
            time_stamp_scale = 1e-6;
            break;
        }
        if (csv_header_items[i].find("time_stamp_s") != std::string::npos) {
            time_stamp_index = i;
            time_stamp_scale = 1.0;
            break;
        }
        if (csv_header_items[i].find("time_stamp_ns") != std::string::npos) {
            time_stamp_index = i;
            time_stamp_scale = 1e-9;
            break;
        }
        if (csv_header_items[i].find("time_stamp_us") != std::string::npos) {
            time_stamp_index = i;
            time_stamp_scale = 1e-6;
            break;
        }
        if (csv_header_items[i].find("time_stamp_ms") != std::string::npos) {
            time_stamp_index = i;
            time_stamp_scale = 1e-3;
            break;
        }
        if (csv_header_items[i].find("timestamp[s]") != std::string::npos) {
            time_stamp_index = i;
            time_stamp_scale = 1.0;
            break;
        }
        if (csv_header_items[i].find("timestamp[ns]") != std::string::npos) {
            time_stamp_index = i;
            time_stamp_scale = 1e-9;
            break;
        }
        if (csv_header_items[i].find("timestamp[us]") != std::string::npos) {
            time_stamp_index = i;
            time_stamp_scale = 1e-6;
            break;
        }
        if (csv_header_items[i].find("timestamp[ms]") != std::string::npos) {
            time_stamp_index = i;
            time_stamp_scale = 1e-3;
            break;
        }
        if (csv_header_items[i].find("timestamp") != std::string::npos && csv_header_items[i].find("_s") != std::string::npos) {
            time_stamp_index = i;
            time_stamp_scale = 1.0;
            break;
        }
        if (csv_header_items[i].find("timestamp") != std::string::npos && csv_header_items[i].find("_ns") != std::string::npos) {
            time_stamp_index = i;
            time_stamp_scale = 1e-9;
            break;
        }
        if (csv_header_items[i].find("timestamp") != std::string::npos && csv_header_items[i].find("_us") != std::string::npos) {
            time_stamp_index = i;
            time_stamp_scale = 1e-6;
            break;
        }
        if (csv_header_items[i].find("timestamp") != std::string::npos && csv_header_items[i].find("_ms") != std::string::npos) {
            time_stamp_index = i;
            time_stamp_scale = 1e-3;
            break;
        }
        if (csv_header_items[i].find("time_stamp") != std::string::npos && csv_header_items[i].find("_s") != std::string::npos) {
            time_stamp_index = i;
            time_stamp_scale = 1.0;
            break;
        }
        if (csv_header_items[i].find("time_stamp") != std::string::npos && csv_header_items[i].find("_ns") != std::string::npos) {
            time_stamp_index = i;
            time_stamp_scale = 1e-9;
            break;
        }
        if (csv_header_items[i].find("time_stamp") != std::string::npos && csv_header_items[i].find("_us") != std::string::npos) {
            time_stamp_index = i;
            time_stamp_scale = 1e-6;
            break;
        }
        if (csv_header_items[i].find("time_stamp") != std::string::npos && csv_header_items[i].find("_ms") != std::string::npos) {
            time_stamp_index = i;
            time_stamp_scale = 1e-3;
            break;
        }
    }
    if (time_stamp_index == -1) {
        ReportError("Timestamp index not found in csv file: " + csv_file);
        return 1;
    }
    ReportInfo("Time stamp index found in csv file: " << time_stamp_index << " [name][" << csv_header_items[time_stamp_index] << "] [scale][" << time_stamp_scale << "]");

    // Parse csv header items into csv_header_items_map.
    // csv_header_items_map[package_name][item_name] = item_index;
    std::unordered_map<std::string, std::vector<std::pair<std::string, int32_t>>> csv_header_items_map;
    for (uint32_t i = 0; i < csv_header_items.size(); i++) {
        const std::string temp_package_name = csv_header_items[i].substr(0, csv_header_items[i].find('/'));
        const std::string item_name = csv_header_items[i].substr(csv_header_items[i].find('/') + 1);
        const std::string package_name = temp_package_name.size() == csv_header_items[i].size() ? "default_package" : temp_package_name;

        if (static_cast<int32_t>(i) == time_stamp_index) {
            continue;
        }
        csv_header_items_map[package_name].emplace_back(std::make_pair(item_name, i));
    }
    ReportInfo("Succeed to parse csv header items into binlog header:");
    for (const auto &package: csv_header_items_map) {
        ReportInfo(">> Package [name][" << package.first << "] [id][" << package.second.begin()->second << "]");
        for (const auto &item: package.second) {
            ReportInfo("   - Item [name][" << item.first << "] [col index][" << item.second << "]");
        }
    }

    // Create log file.
    BinaryDataLog log_recorder;
    if (!log_recorder.CreateLogFile(binlog_file)) {
        ReportError("Failed to create log file: " + binlog_file);
        return 1;
    }

    // Register packages to log recorder.
    for (const auto &package: csv_header_items_map) {
        std::unique_ptr<PackageInfo> package_ptr = std::make_unique<PackageInfo>();
        package_ptr->id = package.second.begin()->second;
        package_ptr->name = package.first;
        for (const auto &item : package.second) {
            package_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kDouble, .name = item.first});
        }
        if (!log_recorder.RegisterPackage(package_ptr)) {
            ReportError("Failed to register package: " + package.first);
            return 1;
        }
    }
    if (!log_recorder.PrepareForRecording()) {
        ReportError("Failed to prepare for recording: " + binlog_file);
        return 1;
    }

    // Iterate each line of csv_file_stream, write data into log file.
    std::string csv_line;
    while (std::getline(csv_file_stream, csv_line) && !csv_line.empty()) {
        std::istringstream csv_line_stream(csv_line);
        std::string temp_str;
        std::vector<double> values;
        while (std::getline(csv_line_stream, temp_str, ',')) {
            temp_str.erase(std::remove(temp_str.begin(), temp_str.end(), ' '), temp_str.end());
            values.emplace_back(std::stod(temp_str));
        }
        CONTINUE_IF(values.size() != csv_header_items.size() - 1);
        static double time_stamp_offset_s = values[time_stamp_index];

        for (const auto &package: csv_header_items_map) {
            const uint16_t package_id = package.second.begin()->second;
            uint8_t *data_ptr = reinterpret_cast<uint8_t *>(&values[package_id]);
            const double time_stamp_s = (values[time_stamp_index] - time_stamp_offset_s) * time_stamp_scale;
            log_recorder.RecordPackage(package_id, reinterpret_cast<const char *>(data_ptr), time_stamp_s);
            log_recorder.current_recorded_time_stamp_s() = time_stamp_s;
        }
    }
    return 0;
}
