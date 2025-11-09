#ifndef _BINARY_DATA_LOG_BASIC_TYPE_H_
#define _BINARY_DATA_LOG_BASIC_TYPE_H_

namespace slam_data_log {

#pragma pack(1)

struct BinaryLog3DofVector {
    float x = 0;
    float y = 0;
    float z = 0;
};

struct BinaryLog6DofPose {
    float p_x = 0;
    float p_y = 0;
    float p_z = 0;
    float q_w = 0;
    float q_x = 0;
    float q_y = 0;
    float q_z = 0;
};

#pragma pack()

}  // namespace slam_data_log

#endif  // end of _BINARY_DATA_LOG_BASIC_TYPE_H_
