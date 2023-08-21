#include "binary_data_log.h"
#include "log_report.h"

using namespace SLAM_DATA_LOG;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test binary data log decodec." RESET_COLOR);

    // Create a log file.
    BinaryDataLog logger;
    if (logger.CreateLogFile()) {
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

    return 0;
}
