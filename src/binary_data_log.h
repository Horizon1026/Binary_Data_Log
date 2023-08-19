#ifndef _BINARY_DATA_LOG_H_
#define _BINARY_DATA_LOG_H_

#include "binary_data_log_file_protocal.h"

#include "iostream"
#include "fstream"
#include "memory"
#include "string"

namespace SLAM_DATA {

/* Class BinaryDataLog Declaration. */
class BinaryDataLog {

public:
    BinaryDataLog() = default;
    virtual ~BinaryDataLog();

    bool CreateLogFile(const std::string &log_file_name = "data.binlog");

    bool RegisterPackage();

private:
    std::unique_ptr<std::fstream> file_ptr_ = nullptr;

};

}

#endif // end of _BINARY_DATA_LOG_H_
