#include "binary_data_log.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "visualizor_2d.h"

#include "cstring"
#include "dirent.h"
#include "iostream"
#include "unistd.h"
#include "vector"

using namespace slam_data_log;
using namespace slam_visualizor;

#include "enable_stack_backward.h"

#pragma pack(1)

struct ImuData {
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;
    uint8_t valid = 0;
};

struct BaroData {
    uint32_t press = 0;
    float height = 0.0f;
    uint8_t valid = 0;
};

struct StateData {
    uint8_t is_vel_valid = 0;
    float vel_x = 0.0f;
    float vel_y = 0.0f;
    float vel_z = 0.0f;
    uint8_t is_pose_valid = 0;
    float pos_x = 0.0f;
    float pos_y = 0.0f;
    float pos_z = 0.0f;
    float atti_w = 0.0f;
    float atti_x = 0.0f;
    float atti_y = 0.0f;
    float atti_z = 0.0f;
};

#pragma pack()

bool GetFilesInPath(std::string dir, std::vector<std::string> &filenames) {
    DIR *ptr_dir;
    struct dirent *ptr;
    if (!(ptr_dir = opendir(dir.c_str()))) {
        ReportError("Cannot open dir " << dir);
        return false;
    }

    filenames.reserve(1000);

    while ((ptr = readdir(ptr_dir)) != 0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
            filenames.emplace_back(dir + "/" + ptr->d_name);
        }
    }

    closedir(ptr_dir);

    return true;
}

void RegisterAllPackages(BinaryDataLog &logger) {
    // Register new packages.
    {
        std::unique_ptr<PackageInfo> package_ptr = std::make_unique<PackageInfo>();
        package_ptr->id = 1;
        package_ptr->name = "imu";
        package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kVector3, .name = "gyro"});
        package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kVector3, .name = "accel"});
        package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kUint8, .name = "valid"});

        if (logger.RegisterPackage(package_ptr)) {
            ReportInfo("Register a new package.");
        } else {
            ReportError("Test failed: register a new package.");
        }
    }
    {
        std::unique_ptr<PackageInfo> package_ptr = std::make_unique<PackageInfo>();
        package_ptr->id = 2;
        package_ptr->name = "baro";
        package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kUint32, .name = "press"});
        package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kFloat, .name = "height"});
        package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kUint8, .name = "valid"});

        if (logger.RegisterPackage(package_ptr)) {
            ReportInfo("Register a new package.");
        } else {
            ReportError("Test failed: register a new package.");
        }
    }
    {
        std::unique_ptr<PackageInfo> package_ptr = std::make_unique<PackageInfo>();
        package_ptr->id = 3;
        package_ptr->name = "gray image";
        package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kImage, .name = "left"});

        if (logger.RegisterPackage(package_ptr)) {
            ReportInfo("Register a new package.");
        } else {
            ReportError("Test failed: register a new package.");
        }
    }
    {
        std::unique_ptr<PackageInfo> package_ptr = std::make_unique<PackageInfo>();
        package_ptr->id = 4;
        package_ptr->name = "rgb image";
        package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kImage, .name = "left"});

        if (logger.RegisterPackage(package_ptr)) {
            ReportInfo("Register a new package.");
        } else {
            ReportError("Test failed: register a new package.");
        }
    }
    {
        std::unique_ptr<PackageInfo> package_ptr = std::make_unique<PackageInfo>();
        package_ptr->id = 5;
        package_ptr->name = "matrix";
        package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kMatrix, .name = "matrix"});

        if (logger.RegisterPackage(package_ptr)) {
            ReportInfo("Register a new package.");
        } else {
            ReportError("Test failed: register a new package.");
        }
    }
    {
        std::unique_ptr<PackageInfo> package_ptr = std::make_unique<PackageInfo>();
        package_ptr->id = 6;
        package_ptr->name = "png image";
        package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kPngImage, .name = "left(png)"});

        if (logger.RegisterPackage(package_ptr)) {
            ReportInfo("Register a new package.");
        } else {
            ReportError("Test failed: register a new package.");
        }
    }
    {
        std::unique_ptr<PackageInfo> package_ptr = std::make_unique<PackageInfo>();
        package_ptr->id = 7;
        package_ptr->name = "slam state";
        package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kUint8, .name = "is_vel_valid"});
        package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kVector3, .name = "velocity"});
        package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kUint8, .name = "is_pose_valid"});
        package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kPose6Dof, .name = "pose"});

        if (logger.RegisterPackage(package_ptr)) {
            ReportInfo("Register a new package.");
        } else {
            ReportError("Test failed: register a new package.");
        }
    }
    {
        std::unique_ptr<PackageInfo> package_ptr = std::make_unique<PackageInfo>();
        package_ptr->id = 8;
        package_ptr->name = "local map";
        package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kPointCloud, .name = "point cloud"});

        if (logger.RegisterPackage(package_ptr)) {
            ReportInfo("Register a new package.");
        } else {
            ReportError("Test failed: register a new package.");
        }
    }
    {
        std::unique_ptr<PackageInfo> package_ptr = std::make_unique<PackageInfo>();
        package_ptr->id = 9;
        package_ptr->name = "line cloud";
        package_ptr->items.emplace_back(PackageItemInfo {.type = ItemType::kLineCloud, .name = "line cloud"});

        if (logger.RegisterPackage(package_ptr)) {
            ReportInfo("Register a new package.");
        } else {
            ReportError("Test failed: register a new package.");
        }
    }
}

void TestCreateLog(const std::string &log_file_name) {
    ReportInfo(YELLOW ">> Test creating binary data log." RESET_COLOR);

    // Create a log file.
    BinaryDataLog logger;
    if (logger.CreateLogFile(log_file_name)) {
        ReportInfo("Create a new log file.");
    } else {
        ReportError("Test failed: create a new log file.");
    }

    RegisterAllPackages(logger);

    // Prepare for recording.
    if (logger.PrepareForRecording()) {
        ReportInfo("Prepare for recording.");
    } else {
        ReportError("Test failed: prepare for recording.");
    }

    // Report all registered packages.
    logger.ReportAllRegisteredPackages();

    // Prepare images for recording.
    std::vector<std::string> image_filenames;
    RETURN_IF(!GetFilesInPath("../example/", image_filenames));
    std::sort(image_filenames.begin(), image_filenames.end());
    const uint32_t max_idx_of_image_file = image_filenames.size();
    uint32_t idx_of_image_file = 0;

    // Record data.
    for (uint32_t i = 0; i < 200; ++i) {
        const float timestamp = static_cast<float>(i) * 0.2f;
        const float temp = static_cast<float>(i) / 15.0f;
        ImuData imu_data {
            .gyro_x = -std::sin(temp + 0.34f),
            .gyro_y = std::sin(1.05f * temp + 1.5f),
            .gyro_z = std::sin(1.1f * temp + 1.0f),
            .accel_x = std::sin(1.15f * temp),
            .accel_y = std::sin(1.2f * temp + 0.05f),
            .accel_z = std::sin(1.25f * temp + 0.6f),
            .valid = i > 50,
        };
        logger.RecordPackage(1, reinterpret_cast<const char *>(&imu_data), timestamp);

        BaroData baro_data {
            .press = i * 2,
            .height = static_cast<float>(i * i),
            .valid = i < 30,
        };
        logger.RecordPackage(2, reinterpret_cast<const char *>(&baro_data), timestamp);

        StateData state_data {
            .is_vel_valid = i > 50,
            .vel_x = std::sin(temp),
            .vel_y = std::sin(temp + 0.05f),
            .vel_z = std::sin(temp + 0.6f),
            .is_pose_valid = i < 100,
            .pos_x = i + 1.0f,
            .pos_y = 1.2f * i,
            .pos_z = std::sqrt(static_cast<float>(i)),
            .atti_w = 1.0f,
            .atti_x = 0.0f,
            .atti_y = 0.0f,
            .atti_z = 0.0f,
        };
        logger.RecordPackage(7, reinterpret_cast<const char *>(&state_data), timestamp);

        if (i % (200 / max_idx_of_image_file) == 0 && idx_of_image_file < max_idx_of_image_file &&
            image_filenames[idx_of_image_file].find(".png") != std::string::npos) {
            // Record image.
            GrayImage gray_image;
            Visualizor2D::LoadImage(image_filenames[idx_of_image_file], gray_image);
            logger.RecordPackage(3, gray_image, timestamp);
            RgbImage rgb_image;
            Visualizor2D::LoadImage(image_filenames[idx_of_image_file], rgb_image);
            logger.RecordPackage(4, rgb_image, timestamp);

            // Record matrix.
            Mat matrix = Mat::Random(80, 120);
            for (int32_t i = 0; i < 80; ++i) {
                matrix(i, i) = i;
            }
            logger.RecordPackage(5, matrix, timestamp);

            // Record png image.
            std::vector<uint8_t> png_image;
            Visualizor2D::SaveToPngImageData(rgb_image, png_image);
            logger.RecordPackage(6, png_image, ItemType::kPngImage, timestamp);

            ++idx_of_image_file;
        }

        if (i % 20 == 0) {
            std::vector<Vec3> points;
            for (uint32_t j = 0; j < 40; ++j) {
                const float temp_j = static_cast<float>(j) / 10.0f;
                points.emplace_back(Vec3(std::sin(temp_j + 0.15f * i), std::cos(temp_j + 0.1f * i + 0.2f), std::sin(temp_j + 0.3f * i)));
            }
            logger.RecordPackage(8, points, timestamp);
        }

        if (i % 25 == 0) {
            std::vector<std::pair<Vec3, Vec3>> lines;
            for (uint32_t j = 0; j < 10; ++j) {
                const float temp_j = static_cast<float>(j) / 5.0f;
                Vec3 p1(std::sin(temp_j + 0.2f * i), std::cos(temp_j + 0.15f * i + 0.3f), std::sin(temp_j + 0.4f * i));
                Vec3 p2 = p1 + Vec3(0.2f, 0.3f, 0.4f);
                lines.emplace_back(std::make_pair(p1, p2));
            }
            logger.RecordPackage(9, lines, timestamp);
        }

        usleep(10000);
    }
}

void TestLoadLog(const std::string &log_file_name) {
    ReportInfo(YELLOW ">> Test loading binary data log." RESET_COLOR);

    BinaryDataLog logger;
    if (logger.LoadLogFile(log_file_name)) {
        ReportInfo("Load a new log file.");
    } else {
        ReportError("Test failed: load a new log file.");
    }

    // Report all registered packages.
    logger.ReportAllRegisteredPackages();
    // Report all loaded packages.
    logger.ReportAllLoadedPackages();
}

void TestPreloadLog(const std::string &log_file_name) {
    ReportInfo(YELLOW ">> Test loading binary data log." RESET_COLOR);

    BinaryDataLog logger;
    if (logger.LoadLogFile(log_file_name, false)) {
        ReportInfo("Load a new log file.");
    } else {
        ReportError("Test failed: load a new log file.");
    }

    // Report all registered packages.
    logger.ReportAllRegisteredPackages();
    // Report all loaded packages.
    logger.ReportAllLoadedPackages();
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test binary data log decodec." RESET_COLOR);

    const std::string log_file_name = "../../Binary_Data_Viewer/examples/data.binlog";
    TestCreateLog(log_file_name);
    TestLoadLog(log_file_name);
    TestPreloadLog(log_file_name);

    return 0;
}
