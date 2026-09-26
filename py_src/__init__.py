"""Python implementation of the binary data log storage, which is a port of the C++ implementation
in 'src'. Only the recording side is ported, the decoding side and the csv loader are not.
"""

try:  # Support both 'import py_src' and 'python3 py_src/example_record.py'.
    from .binary_data_log import BinaryDataLog
    from .binary_data_log_protocol import (
        BINARY_LOG_FILE_HEADER,
        ITEM_TYPE_NAMES,
        ITEM_TYPE_SIZES,
        ItemType,
        PackageInfo,
        PackageItemInfo,
        is_dynamic_type,
        summary_bytes,
    )
except ImportError:  # pragma: no cover
    from binary_data_log import BinaryDataLog  # type: ignore[no-redef]
    from binary_data_log_protocol import (  # type: ignore[no-redef]
        BINARY_LOG_FILE_HEADER,
        ITEM_TYPE_NAMES,
        ITEM_TYPE_SIZES,
        ItemType,
        PackageInfo,
        PackageItemInfo,
        is_dynamic_type,
        summary_bytes,
    )

__all__ = [
    "BinaryDataLog",
    "BINARY_LOG_FILE_HEADER",
    "ITEM_TYPE_NAMES",
    "ITEM_TYPE_SIZES",
    "ItemType",
    "PackageInfo",
    "PackageItemInfo",
    "is_dynamic_type",
    "summary_bytes",
]
