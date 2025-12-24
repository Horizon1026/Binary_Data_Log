#include "binary_data_log.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"

#include "cstring"
#include "dirent.h"
#include "filesystem"
#include "iostream"
#include "unistd.h"
#include "vector"

namespace slam_data_log {

namespace {
    std::vector<std::string> kTimeStampSuffixList = {"_s", "_ns", "_us", "_ms", "[s]", "[ns]", "[us]", "[ms]"};
    std::vector<double> kTimeStampScaleList = {1.0, 1e-9, 1e-6, 1e-3, 1.0, 1e-9, 1e-6, 1e-3};
}  // namespace

bool BinaryDataLog::ParseTimestampInCsvHeader(const std::string &csv_header_name, double &timestamp_scale) {
    if (csv_header_name == "timestamp" || csv_header_name == "time_stamp") {
        timestamp_scale = 1e-6;
        return true;
    }

    using namespace slam_utility;
    if (SlamOperation::IsContained(csv_header_name, "timestamp") || SlamOperation::IsContained(csv_header_name, "time_stamp")) {
        for (uint32_t i = 0; i < kTimeStampSuffixList.size(); ++i) {
            if (SlamOperation::IsContained(csv_header_name, kTimeStampSuffixList[i])) {
                timestamp_scale = kTimeStampScaleList[i];
                return true;
            }
        }
    }
    return false;
}

bool BinaryDataLog::CreateLogFileByCsvFile(const std::string &csv_file_name, const std::string &log_file_name) {
    RETURN_FALSE_IF(csv_file_name.empty());
    RETURN_FALSE_IF(log_file_name.empty());

    // Try to open csv file first.
    std::ifstream csv_file_stream(csv_file_name.c_str());
    if (!csv_file_stream.is_open()) {
        ReportError("[DataLog] Failed to open csv file: " + csv_file_name);
        return false;
    }

    // Prepare for loading csv file.
    // csv_header_items_map[package_name][item_name] = item_index;
    std::vector<std::string> csv_header_items;
    std::unordered_map<std::string, std::vector<std::pair<std::string, int32_t>>> csv_header_items_map;
    int32_t time_stamp_index = -1;
    double time_stamp_scale = 1.0;
    std::string temp_str;

    // Print csv_header of csv file.
    std::string csv_header;
    std::getline(csv_file_stream, csv_header);
    std::istringstream csv_header_stream(csv_header);
    while (std::getline(csv_header_stream, temp_str, ',')) {
        temp_str.erase(std::remove(temp_str.begin(), temp_str.end(), ' '), temp_str.end());
        temp_str.erase(std::remove(temp_str.begin(), temp_str.end(), '\r'), temp_str.end());
        temp_str.erase(std::remove(temp_str.begin(), temp_str.end(), '\n'), temp_str.end());
        CONTINUE_IF(temp_str.empty());
        csv_header_items.emplace_back(temp_str);
    }

    // Find the index of "time_stamp_s".
    for (uint32_t i = 0; i < csv_header_items.size(); i++) {
        if (ParseTimestampInCsvHeader(csv_header_items[i], time_stamp_scale)) {
            time_stamp_index = i;
            break;
        }
    }
    if (time_stamp_index == -1) {
        ReportError("[DataLog] Timestamp index not found in csv file: " + csv_file_name);
        return false;
    }
    ReportInfo("[DataLog] Time stamp index found in csv file: " << time_stamp_index << " [name][" << csv_header_items[time_stamp_index] << "] [scale]["
                                                                << time_stamp_scale << "]");

    // Parse csv header items into csv_header_items_map.
    for (uint32_t i = 0; i < csv_header_items.size(); i++) {
        const std::string temp_package_name = csv_header_items[i].substr(0, csv_header_items[i].find('/'));
        const std::string item_name = csv_header_items[i].substr(csv_header_items[i].find('/') + 1);
        const std::string package_name = temp_package_name.size() == csv_header_items[i].size() ? "default_package" : temp_package_name;
        CONTINUE_IF(item_name.empty());
        CONTINUE_IF(static_cast<int32_t>(i) == time_stamp_index);
        csv_header_items_map[package_name].emplace_back(std::make_pair(item_name, i));
    }
    ReportInfo("[DataLog] Succeed to parse csv header items into binlog header:");
    for (const auto &package: csv_header_items_map) {
        ReportInfo("[DataLog] >> Package [name][" << package.first << "] [id][" << package.second.begin()->second << "]");
        for (const auto &item: package.second) {
            ReportInfo("[DataLog]    - Item [name][" << item.first << "] [col index][" << item.second << "]");
        }
    }

    // Create log file.
    BinaryDataLog log_recorder;
    if (!log_recorder.CreateLogFile(log_file_name)) {
        ReportError("[DataLog] Failed to create log file: " + log_file_name);
        return false;
    }

    // Register packages to log recorder.
    std::vector<std::string> items_name_in_one_package;
    for (const auto &package: csv_header_items_map) {
        const auto &items = package.second;
        items_name_in_one_package.clear();
        for (const auto &item: items) {
            items_name_in_one_package.emplace_back(item.first);
        }

        std::unique_ptr<PackageInfo> package_ptr = std::make_unique<PackageInfo>();
        package_ptr->id = items.begin()->second;
        package_ptr->name = package.first;

        for (int32_t i = 0; i < static_cast<int32_t>(items.size()); i++) {
            using namespace slam_utility;
            if (i < static_cast<int32_t>(items.size()) - 6) {
                if (SlamOperation::IsEndWith(items_name_in_one_package, i, std::vector<std::string> {"x", "y", "z", "w", "x", "y", "z"})) {
                    CONTINUE_IF(!SlamOperation::IsContained(items_name_in_one_package, i, std::vector<std::string> {"p_"}));
                    CONTINUE_IF(!SlamOperation::IsContained(items_name_in_one_package, i + 3, std::vector<std::string> {"q_"}));
                    std::string quat_item_name = items[i].first;
                    if (quat_item_name.size() <= 2) {
                        quat_item_name = "Transform";
                    } else if (quat_item_name[quat_item_name.size() - 2] == '_') {
                        quat_item_name = quat_item_name.substr(0, quat_item_name.size() - 2);
                        quat_item_name = SlamOperation::ReplaceBy(quat_item_name, "p_", "T_");
                    } else {
                        quat_item_name = quat_item_name.substr(0, quat_item_name.size() - 1);
                        quat_item_name = SlamOperation::ReplaceBy(quat_item_name, "p_", "T_");
                    }
                    package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kPose6Dof, .name = quat_item_name});
                    i += 6;
                    continue;
                }
            }
            if (i < static_cast<int32_t>(items.size()) - 2) {
                if (SlamOperation::IsEndWith(items_name_in_one_package, i, std::vector<std::string> {"x", "y", "z"})) {
                    std::string vector3_item_name = items[i].first;
                    if (vector3_item_name.size() <= 2) {
                        vector3_item_name = "vector3";
                    } else if (vector3_item_name[vector3_item_name.size() - 2] == '_') {
                        vector3_item_name = vector3_item_name.substr(0, vector3_item_name.size() - 2);
                    } else {
                        vector3_item_name = vector3_item_name.substr(0, vector3_item_name.size() - 1);
                    }
                    package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kVector3, .name = vector3_item_name});
                    i += 2;
                    continue;
                }
            }
            package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kFloat, .name = items[i].first});
        }
        if (!log_recorder.RegisterPackage(package_ptr)) {
            ReportError("[DataLog] Failed to register package: " + package.first);
            return false;
        }
    }
    if (!log_recorder.PrepareForRecording()) {
        ReportError("[DataLog] Failed to prepare for recording: " + log_file_name);
        return false;
    }

    // Iterate each line of csv_file_stream, write data into log file.
    std::string csv_line;
    std::vector<double> double_values;
    std::vector<float> float_values;
    std::vector<float> package_float_values;
    double time_stamp_offset_s = 0.0;
    bool is_time_stamp_offset_valid = false;
    while (std::getline(csv_file_stream, csv_line) && !csv_line.empty()) {
        std::istringstream csv_line_stream(csv_line);
        std::string temp_str;
        double_values.clear();
        while (std::getline(csv_line_stream, temp_str, ',')) {
            temp_str.erase(std::remove(temp_str.begin(), temp_str.end(), ' '), temp_str.end());
            CONTINUE_IF(temp_str.empty());
            double_values.emplace_back(std::stod(temp_str));
        }
        CONTINUE_IF(double_values.size() != csv_header_items.size());
        if (!is_time_stamp_offset_valid) {
            time_stamp_offset_s = double_values[time_stamp_index];
            is_time_stamp_offset_valid = true;
        }

        float_values.clear();
        for (const auto &value: double_values) {
            float_values.emplace_back(static_cast<float>(value));
        }

        for (const auto &package: csv_header_items_map) {
            const uint16_t package_id = package.second.begin()->second;
            package_float_values.clear();
            for (const auto &item : package.second) {
                package_float_values.emplace_back(float_values[item.second]);
            }
            const float time_stamp_s = static_cast<float>((double_values[time_stamp_index] - time_stamp_offset_s) * time_stamp_scale);
            log_recorder.RecordPackage(package_id, reinterpret_cast<const char *>(package_float_values.data()), time_stamp_s);
            log_recorder.current_recorded_time_stamp_s() = time_stamp_s;
        }
    }

    return true;
}

}  // namespace slam_data_log
