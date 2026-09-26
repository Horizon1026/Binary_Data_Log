"""Binary log file protocol, a python port of 'src/binary_data_log_file_protocal.h'.

The log file is combined with header, packages_name and packages_content.
All multi-byte values are stored as little endian, and all texts are stored as
raw bytes without any terminator, so text names cannot be longer than 255 bytes.

Part 1: Header
    [0] - [14]: b'BINARY_DATA_LOG', which is fixed text.

Part 2: Packages' name
    [0] - [3]: Offset index to the beginning of 'packages_content',
        which means the whole length of all 'packages_name',
        and including the checking byte.

    for each package:
        [0] - [3]: Offset index to the next 'packages_name',
            which means the length of this 'packages_name',
            and including the checking byte.
        [4] - [5]: Package id.
        [6]: Length of text name of this package.
        [7] - [n]: Text name of this package.

        for each item in package:
            [0]: Type of item, which is enum of 'ItemType'.
            [1]: Length of text name of this item.
            [2] - [n]: Text name of this item.

        [n + m]: Sum check byte of this package.

Part 3: Packages' content
    for each package:
        [0] - [3]: Offset index to the next 'package_content',
            which means the length of this 'package_content',
            and including the checking byte.
        [4] - [5]: Package id.
        [6] - [13]: System timestamp of this package content. Unit is second, type is double.
        [14] - [n]: Binary data, which is the only different part of different packages.
        [n + 1]: Sum check byte of this package.

Binary data of each type of package:
    - kImage: [0]: channels. [1] - [2]: rows(height). [3] - [4]: cols(width). [5] - [n]: pixel value.
    - kPngImage: [0] - [3]: number of bytes in png file. [4] - [n]: binary data of png file.
    - kMatrix: [0] - [1]: rows. [2] - [3]: cols. [4] - [n]: float data, row major.
    - kPointCloud: [0] - [3]: number of points. Then x, y, z of each point, each is a float.
    - kLineCloud: [0] - [3]: number of lines. Then 2 points of each line, each point is x, y, z floats.
    - Other static size types: memory of items, ordered as the items registered in the package.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum

__all__ = [
    "BINARY_LOG_FILE_HEADER",
    "MAX_TEXT_LENGTH",
    "ItemType",
    "ITEM_TYPE_NAMES",
    "ITEM_TYPE_SIZES",
    "ITEM_TYPE_STRUCT_FORMATS",
    "RECORD_HEAD_SIZE",
    "summary_bytes",
    "is_dynamic_type",
    "PackageItemInfo",
    "PackageInfo",
]

BINARY_LOG_FILE_HEADER: bytes = b"BINARY_DATA_LOG"
"""Part 1 of the log file, which is fixed text."""

MAX_TEXT_LENGTH: int = 255
"""Length of a text name is stored in one byte, so a name cannot be longer than 255 bytes."""

RECORD_HEAD_SIZE: int = 4 + 2 + 8
"""Offset, package id and timestamp of a package data in Part 3."""


class ItemType(IntEnum):
    """Type of an item in a package, same order as the C++ enum."""

    kUint8 = 0
    kInt8 = 1
    kUint16 = 2
    kInt16 = 3
    kUint32 = 4
    kInt32 = 5
    kUint64 = 6
    kInt64 = 7
    kFloat = 8
    kDouble = 9
    kVector3 = 10
    kPose6Dof = 11
    kImage = 12
    kPngImage = 13
    kMatrix = 14
    kPointCloud = 15
    kLineCloud = 16


ITEM_TYPE_NAMES: tuple[str, ...] = (
    "kUint8",
    "kInt8",
    "kUint16",
    "kInt16",
    "kUint32",
    "kInt32",
    "kUint64",
    "kInt64",
    "kFloat",
    "kDouble",
    "kVector3",
    "kPose6Dof",
    "kImage",
    "kPngImage",
    "kMatrix",
    "kPointCloud",
    "kLineCloud",
)

ITEM_TYPE_SIZES: tuple[int, ...] = (
    1,   # kUint8.
    1,   # kInt8.
    2,   # kUint16.
    2,   # kInt16.
    4,   # kUint32.
    4,   # kInt32.
    8,   # kUint64.
    8,   # kInt64.
    4,   # kFloat.
    8,   # kDouble.
    12,  # kVector3, which is 3 floats.
    28,  # kPose6Dof, which is position x/y/z and quaternion w/x/y/z.
    0,   # kImage, which is unfixed size.
    0,   # kPngImage, which is unfixed size.
    0,   # kMatrix, which is unfixed size.
    0,   # kPointCloud, which is unfixed size.
    0,   # kLineCloud, which is unfixed size.
)

ITEM_TYPE_STRUCT_FORMATS: tuple[str | None, ...] = (
    "B",     # kUint8.
    "b",     # kInt8.
    "H",     # kUint16.
    "h",     # kInt16.
    "I",     # kUint32.
    "i",     # kInt32.
    "Q",     # kUint64.
    "q",     # kInt64.
    "f",     # kFloat.
    "d",     # kDouble.
    None,    # kVector3, which is packed as 3 floats.
    None,    # kPose6Dof, which is packed as 7 floats.
    None,    # kImage.
    None,    # kPngImage.
    None,    # kMatrix.
    None,    # kPointCloud.
    None,    # kLineCloud.
)

NUMBER_OF_VALUES_OF_VECTOR3: int = 3
NUMBER_OF_VALUES_OF_POSE6DOF: int = 7

LAST_STATIC_ITEM_TYPE: ItemType = ItemType.kPose6Dof
"""Types after this one have unfixed size, so their data size is stored in the binary data itself."""


# Compute the protocol checksum byte.
def summary_bytes(byte_data: bytes | bytearray | memoryview, init_value: int = 0) -> int:
    """Sum check byte, a python port of 'BinaryDataLog::SummaryBytes'.

    The sum is computed on uint8 with wraparound, starting from 'init_value'.
    """
    values = byte_data if isinstance(byte_data, memoryview) else memoryview(byte_data)
    return (init_value + sum(values.cast("B"))) & 0xFF


# Identify variable-length payload types.
def is_dynamic_type(item_type: ItemType | int) -> bool:
    """Check if this type of item has unfixed size, a python port of 'BinaryDataLog::IsDynamicType'."""
    return ItemType(item_type) > LAST_STATIC_ITEM_TYPE


@dataclass
class PackageItemInfo:
    """Information of one item in a package."""

    type: ItemType = ItemType.kUint32
    name: str | bytes = ""
    # The index of this item's binary data inside the binary data of the whole package, which is
    # computed when the package is registered.
    bindata_index_in_package: int = 0


@dataclass
class PackageInfo:
    """Information of one package, which should be registered to the recorder before recording."""

    id: int = 0
    name: str | bytes = ""
    items: list[PackageItemInfo] = field(default_factory=list)
    # The number of bytes of the binary data of this package, which is computed when the package is
    # registered. If it is zero, this package has unfixed size.
    size: int = 0
