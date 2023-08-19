#include "binary_data_log.h"
#include "log_report.h"

using namespace SLAM_DATA;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test binary data log decodec." RESET_COLOR);

    BinaryDataLog logger;
    if (logger.CreateLogFile()) {
        ReportInfo("Create a new log file.");
    } else {
        ReportInfo("Test failed.");
    }

    return 0;
}
