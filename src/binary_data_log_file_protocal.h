#ifndef _BINARY_DATA_LOG_FILE_PROTOCAL_H_
#define _BINARY_DATA_LOG_FILE_PROTOCAL_H_

#include "basic_type.h"
#include "string"
#include "vector"

// The log file 'log.binlog' is combined with header, packages_name and packages_content.

/* Part 1: Header */
/*
[0] - [n]: 'BinaryDataLog', which is fixed texts.
*/

/* Part 2: Packages' name */
/*
[0] - [3]: Offset index to the beginning of 'packages_content',
    which means the whole length of all 'packages_name',
    and including the checking type.

for each package:
    [0] - [3]: Offset index to the next 'packages_name',
        which means the length of this 'packages_name',
        and including the checking byte.
    [4] - [5]: Package id.
    [6]: Length of text name of this package.
    [7] - [n]: Text name of this package.

    for each item in package:
        [0]: Type of item. Which is enmu of 'uint8_t', 'float', 'double' and so on.
        [1]: Length of text name of this item.
        [2] - [n]: Text name of this item.

    [n + m]: Sum check byte of this package.

*/

/* Part 3: Packages' content */
/*
for each package:
    [0] - [3]: Offset index to the next 'package_content',
        which means the length of this 'package_content',
        and including the checking byte.
    [4] - [5]: Package id.
    [6] - [9]: System timestamp of this package content. Unit is millisecond.

    [10] - [n]: Binary data.(This is the only different part of different packages)

    [n + 1]: Sum check byte of this package.

for each vecor3 package:
    [10] - [13]: Vector in X axis.
    [14] - [17]: Vector in Y axis.
    [18] - [21]: Vector in Z axis.

for each pose 6dof package:
    [10] - [13]: Position in X axis.
    [14] - [17]: Position in Y axis.
    [18] - [21]: Position in Z axis.
    [22] - [25]: Rotation in quaternion w.
    [26] - [29]: Rotation in quaternion x.
    [30] - [33]: Rotation in quaternion y.
    [34] - [37]: Rotation in quaternion z.

for each image package:
    [10]: Channels.
    [11] - [12]: Image rows(height).
    [13] - [14]: Image cols(width).
    [15] - [n]: Binary data.

for each PNG image package:
    [10] - [13]: Number of bytes in png file.
    [14] - [n]: Binary data of png file.

for each matrix package:
    [10] - [11]: Matrix rows.
    [12] - [13]: Matrix cols.
    [14] - [n]: Binary data (standard float encode, row major).

for each point cloud package:
    [10] - [13]: Number of points in cloud.
    [14] - [17]: Point i position x.
    [18] - [21]: Point i position y.
    [22] - [25]: Point i position z.
    [26] - [29]: Point i + 1 position x.
    ...

*/

using namespace slam_utility;

namespace slam_data_log {

enum class ItemType : uint8_t {
    kUint8 = 0,
    kInt8 = 1,
    kUint16 = 2,
    kInt16 = 3,
    kUint32 = 4,
    kInt32 = 5,
    kUint64 = 6,
    kInt64 = 7,
    kFloat = 8,
    kDouble = 9,
    kVector3 = 10,
    kPose6Dof = 11,
    kImage = 12,
    kPngImage = 13,
    kMatrix = 14,
    kPointCloud = 15,
};

static std::vector<uint32_t> item_type_sizes = {
    1,   // kUint8.
    1,   // kInt8.
    2,   // kUint16.
    2,   // kInt16.
    4,   // kUint32.
    4,   // kInt32.
    8,   // kUint64.
    8,   // kInt64.
    4,   // kFloat.
    8,   // kDouble.
    12,  // kVector3.
    28,  // kPose6Dof.
    0,   // kImage.
    0,   // kPngImage.
    0,   // kMatrix.
    0,   // kPointCloud.
};

static std::vector<std::string> item_type_strings = {
    "kUint8", "kInt8",   "kUint16",  "kInt16",    "kUint32", "kInt32",    "kUint64", "kInt64",
    "kFloat", "kDouble", "kVector3", "kPose6Dof", "kImage",  "kPngImage", "kMatrix", "kPointCloud",
};

enum class DecodeType : uint8_t {
    kGeneral = 0,
    kQuaternionToPitch = 1,
    kQuaternionToRoll = 2,
    kQuaternionToYaw = 3,
    kVector2dToMod = 4,
    kVector3dToMod = 5,
};

static std::string binary_log_file_header = "BINARY_DATA_LOG";

struct PackageItemInfo {
    ItemType type = ItemType::kUint32;
    uint32_t bindata_index_in_package = 0;
    std::string name;
};

struct PackageInfo {
    uint16_t id = 0;
    // The number of bytes except 10 bytes of package head and 1 byte of sum check.
    // If this is zero, it means unfixed size. Log decoder should better load only package info but not full bytes.
    uint32_t size = 0;
    std::string name;
    std::vector<PackageItemInfo> items;
};

struct PackageDataPerTick {
    float timestamp_s = 0.0f;
    std::vector<uint8_t> data;         // Binary data stored in bytes.
    uint64_t index_in_file = 0;        // Start at 'offset'.
    uint32_t size_of_all_in_file = 0;  // Including offset, id, timestamp, binary_data, check_byte.
};

}  // namespace slam_data_log

#endif  // end of _BINARY_DATA_LOG_FILE_PROTOCAL_H_
