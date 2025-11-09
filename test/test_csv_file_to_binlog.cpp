#include "binary_data_log.h"
#include "cstring"
#include "dirent.h"
#include "filesystem"
#include "iostream"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "unistd.h"
#include "vector"
#include "visualizor_2d.h"

using namespace SLAM_DATA_LOG;
using namespace SLAM_VISUALIZOR;

#include "enable_stack_backward.h"

int main(int argc, char **argv) {
    ReportColorWarn(">> Test csv file to binlog.");
    const std::string csv_file = argv[1];
    const std::string binlog_file = argc > 2 ? argv[2] : csv_file.substr(0, csv_file.find_last_of('.')) + ".binlog";
    ReportInfo("[input] CSV file: " << csv_file);
    ReportInfo("[output] Binlog file: " << binlog_file);

    BinaryDataLog::CreateLogFileByCsvFile(csv_file, binlog_file);

    return 0;
}
