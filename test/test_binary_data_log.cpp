#include "binary_data_log.h"
#include "log_report.h"

using namespace SLAM_DATA_LOG;

#pragma pack(1)

struct ImuData {
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;
};

struct BaroData {
    uint32_t press = 0;
    float height = 0.0f;
};

#pragma pack()

void TestCreateLog(const std::string &log_file_name) {
    ReportInfo(YELLOW ">> Test creating binary data log." RESET_COLOR);

    // Create a log file.
    BinaryDataLog logger;
    if (logger.CreateLogFile(log_file_name)) {
        ReportInfo("Create a new log file.");
    } else {
        ReportInfo("Test failed: create a new log file.");
    }

    // Register new packages.
    {
        std::unique_ptr<Package> package_ptr = std::make_unique<Package>();
        package_ptr->id = 1;
        package_ptr->name = "imu";
        package_ptr->items.emplace_back(PackageItem{.type = ItemType::kFloat, .name = "gyro_x"});
        package_ptr->items.emplace_back(PackageItem{.type = ItemType::kFloat, .name = "gyro_y"});
        package_ptr->items.emplace_back(PackageItem{.type = ItemType::kFloat, .name = "gyro_z"});
        package_ptr->items.emplace_back(PackageItem{.type = ItemType::kFloat, .name = "accel_x"});
        package_ptr->items.emplace_back(PackageItem{.type = ItemType::kFloat, .name = "accel_y"});
        package_ptr->items.emplace_back(PackageItem{.type = ItemType::kFloat, .name = "accel_z"});

        if (logger.RegisterPackage(package_ptr)) {
            ReportInfo("Register a new package.");
        } else {
            ReportInfo("Test failed: register a new package.");
        }
    }
    {
        std::unique_ptr<Package> package_ptr = std::make_unique<Package>();
        package_ptr->id = 2;
        package_ptr->name = "baro";
        package_ptr->items.emplace_back(PackageItem{.type = ItemType::kUint32, .name = "press"});
        package_ptr->items.emplace_back(PackageItem{.type = ItemType::kFloat, .name = "height"});

        if (logger.RegisterPackage(package_ptr)) {
            ReportInfo("Register a new package.");
        } else {
            ReportInfo("Test failed: register a new package.");
        }
    }

    // Prepare for recording.
    if (logger.PrepareForRecording()) {
        ReportInfo("Prepare for recording.");
    } else {
        ReportInfo("Test failed: prepare for recording.");
    }

    // Report all registered packages.
    logger.ReportAllRegisteredPackages();

    // Record data.
    {
        ImuData imu_data;
        logger.RecordPackage(1, reinterpret_cast<const char *>(&imu_data));

        BaroData baro_data;
        logger.RecordPackage(2, reinterpret_cast<const char *>(&baro_data));
    }
}

void TestLoadLog(const std::string &log_file_name) {
    ReportInfo(YELLOW ">> Test loading binary data log." RESET_COLOR);

    BinaryDataLog logger;
    if (logger.LoadLogFile(log_file_name)) {
        ReportInfo("Load a new log file.");
    } else {
        ReportInfo("Test failed: load a new log file.");
    }
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test binary data log decodec." RESET_COLOR);

    const std::string log_file_name = "data.binlog";
    TestCreateLog(log_file_name);
    TestLoadLog(log_file_name);

    return 0;
}
