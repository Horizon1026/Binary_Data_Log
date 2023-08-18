#ifndef _HEX_DATA_LOG_H_
#define _HEX_DATA_LOG_H_

#include "hex_data_log_file_protocal.h"
#include "string.h"

/* Class HexDataLog Declaration. */
class HexDataLog {

public:
    HexDataLog() = default;
    virtual ~HexDataLog() = default;

private:
    std::string log_file_name_;

};

#endif // end of _HEX_DATA_LOG_H_
