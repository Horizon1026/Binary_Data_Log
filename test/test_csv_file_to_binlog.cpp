#include "binary_data_log.h"
#include "visualizor_2d.h"
#include "slam_operations.h"
#include "slam_log_reporter.h"

#include "unistd.h"
#include "iostream"
#include "dirent.h"
#include "vector"
#include "cstring"

using namespace SLAM_DATA_LOG;
using namespace SLAM_VISUALIZOR;

#include "enable_stack_backward.h"

int main(int argc, char **argv) {
    std::string csv_file = argv[1];
    std::string binlog_file = csv_file + ".binlog";

    BinaryDataLog log_recorder;
    if (!log_recorder.CreateLogFile(binlog_file)) {
        ReportError("Failed to create log file: " + binlog_file);
        return 1;
    }

    return 0;
}
