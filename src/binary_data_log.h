#ifndef _BINARY_DATA_LOG_H_
#define _BINARY_DATA_LOG_H_

#include "binary_data_log_file_protocal.h"

#include "iostream"
#include "fstream"
#include "string"

/* Class BinaryDataLog Declaration. */
class BinaryDataLog {

public:
    BinaryDataLog() = default;
    virtual ~BinaryDataLog() = default;

private:
    std::string log_file_name_ = "data.binlog";

};

#endif // end of _BINARY_DATA_LOG_H_
