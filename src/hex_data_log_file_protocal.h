#ifndef _HEX_DATA_LOG_FILE_PROTOCAL_H_
#define _HEX_DATA_LOG_FILE_PROTOCAL_H_

// The log file 'log.hexlog' is combined with header, packages_name and packages_content.

/* Header */
/*
[0] - [16]: 'HorizonHexDataLog', which is fixed texts.
*/

/* Packages' name */
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

/* Packages' content */
/*

for each package:
	[0] - [3]: Offset index to the next 'package_content',
    	which means the length of this 'package_content',
        and including the checking byte.
	[4] - [5]: Package id.
    [6] - [9]: System timestamp of this package content.
    [10] - [n]: Hex data.
    [n + 1]: Sum check byte of this package.

*/

class enum ItemType : unsigned char {
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
    kImageU8C1 = 10,
    kImageU8C3 = 11,
};

#endif // end of _HEX_DATA_LOG_FILE_PROTOCAL_H_
