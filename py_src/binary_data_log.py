"""Binary data log recorder, a python port of 'src/binary_data_log_recorder_static.cpp'
and 'src/binary_data_log_recorder_dynamic.cpp'.

Only the recording side is ported here, the decoding side is not. The order of
using the recorder is the same as the C++ implementation::

    logger = BinaryDataLog()
    logger.create_log_file("data.binlog")
    logger.register_package(PackageInfo(id=1, name="imu", items=[PackageItemInfo(ItemType.kVector3, "gyro")]))
    logger.prepare_for_recording()
    logger.record_values(1, [gyro_x, gyro_y, gyro_z], timestamp_s)
    logger.close()

Only the python standard library is used. Objects supporting the buffer protocol
(such as numpy arrays) are also accepted as binary data, but numpy is not needed.
"""

from __future__ import annotations

import struct
import time
from collections.abc import Iterable, Sequence
from typing import Any, BinaryIO

try:  # Support both 'import py_src.binary_data_log' and 'python3 py_src/binary_data_log.py'.
    from .binary_data_log_protocol import (
        BINARY_LOG_FILE_HEADER,
        ITEM_TYPE_STRUCT_FORMATS,
        ITEM_TYPE_SIZES,
        MAX_TEXT_LENGTH,
        NUMBER_OF_VALUES_OF_POSE6DOF,
        NUMBER_OF_VALUES_OF_VECTOR3,
        RECORD_HEAD_SIZE,
        ItemType,
        PackageInfo,
        is_dynamic_type,
        summary_bytes,
    )
except ImportError:  # pragma: no cover
    from binary_data_log_protocol import (  # type: ignore[no-redef]
        BINARY_LOG_FILE_HEADER,
        ITEM_TYPE_STRUCT_FORMATS,
        ITEM_TYPE_SIZES,
        MAX_TEXT_LENGTH,
        NUMBER_OF_VALUES_OF_POSE6DOF,
        NUMBER_OF_VALUES_OF_VECTOR3,
        RECORD_HEAD_SIZE,
        ItemType,
        PackageInfo,
        is_dynamic_type,
        summary_bytes,
    )

__all__ = ["BinaryDataLog"]

_UINT8 = struct.Struct("<B")
_UINT16 = struct.Struct("<H")
_UINT32 = struct.Struct("<I")
_RECORD_HEAD = struct.Struct("<IHd")  # Offset to the next record, package id and timestamp.
_VECTOR3 = struct.Struct("<3f")
_POSE6DOF = struct.Struct("<7f")


# Encode and validate a package or item name.
def _text_bytes(text: str | bytes) -> bytes:
    """Convert a text name into bytes, and check its length."""
    raw = text if isinstance(text, bytes) else text.encode("utf-8")
    if len(raw) > MAX_TEXT_LENGTH:
        raise ValueError(f"Text name is too long, {len(raw)} > {MAX_TEXT_LENGTH} bytes : {text!r}")
    return raw


# Convert binary input and check its byte length.
def _to_bytes(data: Any, expected_size: int | None = None) -> bytes:
    """Convert bytes, a buffer object or a sequence of uint8 into bytes, and check its size."""
    if isinstance(data, (bytes, bytearray)):
        raw = bytes(data)
    elif isinstance(data, memoryview):
        raw = data.tobytes()
    else:
        to_bytes = getattr(data, "tobytes", None)
        raw = bytes(to_bytes()) if callable(to_bytes) else bytes(data)

    if expected_size is not None and len(raw) != expected_size:
        raise ValueError(f"Size of binary data mismatched, {len(raw)} != {expected_size} bytes.")
    return raw


# Flatten nested numeric input for packing.
def _flatten_values(values: Iterable[Any]) -> list[Any]:
    """Flatten a nested sequence of values into a flat list, keeping every value as it is."""
    result: list[Any] = []
    for value in values:
        if isinstance(value, Iterable) and not isinstance(value, (str, bytes, bytearray)):
            result.extend(_flatten_values(value))
        else:
            result.append(value)
    return result


# Pack matrix values as float32 bytes.
def _to_float32_bytes(values: Any, expected_size: int) -> bytes:
    """Convert float data into bytes of little endian float32, and check its size in bytes."""
    if isinstance(values, (bytes, bytearray)):
        raw = bytes(values)
    elif isinstance(values, memoryview):
        raw = values.tobytes()
    else:
        to_bytes = getattr(values, "tobytes", None)
        dtype = getattr(values, "dtype", None)
        if callable(to_bytes) and getattr(dtype, "itemsize", 4) != 4 and hasattr(values, "astype"):
            # For example a numpy array of float64, convert it into float32 first.
            raw = bytes(values.astype("float32").tobytes())
        elif callable(to_bytes):
            raw = bytes(to_bytes())
        else:
            float_values = [float(value) for value in _flatten_values(values)]
            raw = struct.pack(f"<{len(float_values)}f", *float_values)

    if len(raw) != expected_size:
        raise ValueError(f"Size of float32 data mismatched, {len(raw)} != {expected_size} bytes. Data should be float32.")
    return raw


# Reject values outside an unsigned field.
def _check_uint_range(value: int, max_value: int, name: str) -> None:
    if not 0 <= value <= max_value:
        raise ValueError(f"{name} should be in [0, {max_value}], but it is {value}.")


# Count numeric fields in one item.
def _number_of_values_of_item(item_type: ItemType) -> int:
    """The number of values that one item of this type takes in 'record_values'."""
    if item_type == ItemType.kVector3:
        return NUMBER_OF_VALUES_OF_VECTOR3
    if item_type == ItemType.kPose6Dof:
        return NUMBER_OF_VALUES_OF_POSE6DOF
    return 1


class BinaryDataLog:
    """Recorder of binary data log file."""

    # Initialize recorder state and timing.
    def __init__(self) -> None:
        self._file: BinaryIO | None = None
        self._packages: dict[int, PackageInfo] = {}
        self._start_system_time: float = time.time()
        self._is_head_recorded: bool = False
        # The timestamp of the last recorded package, which is maintained by the caller.
        self.current_recorded_time_stamp_s: float = 0.0

    # Return the recorder for a context block.
    def __enter__(self) -> "BinaryDataLog":
        return self

    # Close the file when a context block ends.
    def __exit__(self, *args: Any) -> None:
        self.close()

    # Reference for member variables.

    # Expose the current output stream.
    @property
    def file_ptr(self) -> BinaryIO | None:
        """The writable file stream, which is None if no log file is created."""
        return self._file

    # Expose registered package definitions.
    @property
    def packages(self) -> dict[int, PackageInfo]:
        """All registered packages, indexed by package id."""
        return self._packages

    # Expose the timestamp origin.
    @property
    def start_system_time(self) -> float:
        """The system time when this recorder is created or cleaned up, which timestamps are based on."""
        return self._start_system_time

    # Support for recorder.

    # Create a file and write its signature.
    def create_log_file(self, log_file_name: str = "data.binlog") -> bool:
        """Create a new log file and write the file header, which is 'BinaryDataLog::CreateLogFile'."""
        self.close()

        try:
            self._file = open(log_file_name, "wb")
        except OSError:
            self._file = None
            return False

        self._write_log_file_header()
        return True

    # Close the buffered output stream.
    def close(self) -> None:
        """Close the log file if it is opened.

        The log file is buffered, so 'close' should be called before the file is read back.
        """
        if self._file is not None:
            self._file.close()
        self._file = None
        self._is_head_recorded = False

    # Flush buffered records to disk.
    def flush(self) -> None:
        """Flush buffered log data into the file, which keeps the file readable before 'close'.

        The recording order is not changed, so a flushed file is still byte by byte the same
        as a file written by the C++ implementation.
        """
        if self._file is not None:
            self._file.flush()

    # Reset registered packages and timing.
    def clean_up(self) -> None:
        """Drop all registered packages and close the log file, which is 'BinaryDataLog::CleanUp'."""
        self._packages.clear()
        self.close()
        self._start_system_time = time.time()
        self.current_recorded_time_stamp_s = 0.0

    # Measure time since recorder creation.
    def get_system_timestamp(self) -> float:
        """Seconds elapsed since this recorder is created, which is 'BinaryDataLog::GetSystemTimestamp'."""
        return time.time() - self._start_system_time

    # Validate and register one package.
    def register_package(self, package: PackageInfo) -> bool:
        """Register a package, which is 'BinaryDataLog::RegisterPackage'.

        Registering will be failed if the package has no items, if one of its items has
        unfixed size while the package has more than one item, or if its id is already
        registered.
        """
        if package is None:
            raise TypeError("Package to be registered is invalid.")

        # Registering will be failed if there is no items in this package.
        if not package.items:
            return False

        # If one of items in package has dynamic size, this package should only have one item.
        if len(package.items) > 1:
            for item in package.items:
                if is_dynamic_type(item.type):
                    return False

        # Do not repeat registering the same package.
        if package.id in self._packages:
            return False

        if not 0 <= package.id <= 0xFFFF:
            raise ValueError(f"Package id should be in [0, 65535], but it is {package.id}.")

        # Check the text names before they are written into the file.
        _text_bytes(package.name)
        for item in package.items:
            _text_bytes(item.name)
            item.type = ItemType(item.type)

        # Statis the whole size of binary data in this package.
        package.size = 0
        for item in package.items:
            item.bindata_index_in_package = package.size
            package.size += ITEM_TYPE_SIZES[int(item.type)]

        self._packages[package.id] = package
        return True

    # Write the registered package table.
    def prepare_for_recording(self) -> bool:
        """Write information of all registered packages as the head of the log file, which is 'BinaryDataLog::PrepareForRecording'."""
        if self._file is None:
            return False
        if self._is_head_recorded:
            raise RuntimeError("The head of this log file is already recorded, do not prepare for recording twice.")
        return self._record_all_registered_packages_as_file_head()

    # Record one fixed-size binary package.
    def record_bytes(self, package_id: int, data: Any, time_stamp_s: float | None = None) -> bool:
        """Record a package of fixed size from binary data, which is 'RecordPackage(id, const char *data_ptr)'.

        'data' should contain exactly the same number of bytes as the registered size of this
        package, and its layout should match the items registered in this package.
        """
        package = self._get_package_for_recording(package_id)
        if package is None:
            return False
        if package.size <= 0:
            raise ValueError(f"Package {package_id} has unfixed size, use the record method of its item type instead.")

        return self._write_record(package_id, _to_bytes(data, package.size), time_stamp_s)

    # Pack and record fixed-size numeric values.
    def record_values(self, package_id: int, values: Sequence[Any], time_stamp_s: float | None = None) -> bool:
        """Record a package of fixed size from python values, which is a convenience of 'record_bytes'.

        'values' are packed in the order of the items registered in this package : one value for
        each number type item, 3 values for each kVector3 item, and 7 values for each kPose6Dof
        item. Nested sequences of numbers are flattened, so a kVector3 item takes either
        (x, y, z) or x, y, z.
        """
        package = self._get_package_for_recording(package_id)
        if package is None:
            return False
        if package.size <= 0:
            raise ValueError(f"Package {package_id} has unfixed size, use the record method of its item type instead.")

        values = _flatten_values(values)
        number_of_values_of_package = sum(_number_of_values_of_item(item.type) for item in package.items)
        if len(values) != number_of_values_of_package:
            raise ValueError(f"Package {package_id} takes {number_of_values_of_package} values, but {len(values)} values are given.")

        chunks: list[bytes] = []
        index = 0
        for item in package.items:
            number_of_values = _number_of_values_of_item(item.type)
            item_values = values[index:index + number_of_values]
            index += number_of_values
            if item.type == ItemType.kVector3:
                chunks.append(_VECTOR3.pack(*(float(value) for value in item_values)))
            elif item.type == ItemType.kPose6Dof:
                chunks.append(_POSE6DOF.pack(*(float(value) for value in item_values)))
            else:
                struct_format = ITEM_TYPE_STRUCT_FORMATS[int(item.type)]
                value = item_values[0] if struct_format in ("f", "d") else int(item_values[0])
                chunks.append(struct.pack(f"<{struct_format}", value))

        return self._write_record(package_id, b"".join(chunks), time_stamp_s)

    # Record image dimensions and raw pixels.
    def record_image(self, package_id: int, channels: int, rows: int, cols: int, image_data: Any, time_stamp_s: float | None = None) -> bool:
        """Record a package with kImage item, which is 'BinaryDataLog::RecordImage'.

        'image_data' should contain 'channels * rows * cols' bytes of pixel value.
        """
        package = self._get_package_for_recording(package_id, expected_item_type=ItemType.kImage)
        if package is None:
            return False
        _check_uint_range(channels, 0xFF, "channels")
        _check_uint_range(rows, 0xFFFF, "rows")
        _check_uint_range(cols, 0xFFFF, "cols")

        data = _to_bytes(image_data, channels * rows * cols)
        payload = _UINT8.pack(channels) + _UINT16.pack(rows) + _UINT16.pack(cols) + data
        return self._write_record(package_id, payload, time_stamp_s)

    # Record one grayscale image.
    def record_gray_image(self, package_id: int, rows: int, cols: int, image_data: Any, time_stamp_s: float | None = None) -> bool:
        """Record a gray image with one channel, which is 'RecordPackage(id, const GrayImage &image)'."""
        return self.record_image(package_id, 1, rows, cols, image_data, time_stamp_s)

    # Record one RGB image.
    def record_rgb_image(self, package_id: int, rows: int, cols: int, image_data: Any, time_stamp_s: float | None = None) -> bool:
        """Record a rgb image with three channels, which is 'RecordPackage(id, const RgbImage &image)'."""
        return self.record_image(package_id, 3, rows, cols, image_data, time_stamp_s)

    # Record one PNG payload.
    def record_png_image(self, package_id: int, png_data: Any, time_stamp_s: float | None = None) -> bool:
        """Record a package with kPngImage item, which is 'RecordPackage(id, const std::vector<uint8_t> &data_bytes, ItemType::kPngImage)'."""
        package = self._get_package_for_recording(package_id, expected_item_type=ItemType.kPngImage)
        if package is None:
            return False

        data = _to_bytes(png_data)
        if not data:
            raise ValueError(f"Binary data of package {package_id} is empty.")

        return self._write_record(package_id, _UINT32.pack(len(data)) + data, time_stamp_s)

    # Record one float32 matrix.
    def record_matrix(self, package_id: int, rows: int, cols: int, matrix_data: Any, time_stamp_s: float | None = None) -> bool:
        """Record a package with kMatrix item, which is 'RecordPackage(id, const Mat &matrix)'.

        'matrix_data' should contain 'rows * cols' values of float32, and it can be a nested
        sequence of numbers, a flat sequence of numbers, or a buffer of float32 data.
        """
        package = self._get_package_for_recording(package_id, expected_item_type=ItemType.kMatrix)
        if package is None:
            return False
        _check_uint_range(rows, 0xFFFF, "rows")
        _check_uint_range(cols, 0xFFFF, "cols")
        if rows < 1 or cols < 1:
            raise ValueError(f"Size of matrix is invalid, {rows} x {cols}.")

        data = _to_float32_bytes(matrix_data, rows * cols * 4)
        return self._write_record(package_id, _UINT16.pack(rows) + _UINT16.pack(cols) + data, time_stamp_s)

    # Record selected three-dimensional points.
    def record_point_cloud(self, package_id: int, points: Sequence[Any], step: int = 1, time_stamp_s: float | None = None) -> bool:
        """Record a package with kPointCloud item, which is 'RecordPackage(id, const std::vector<Vec3> &points_cloud, step)'.

        'points' is a sequence of points of 3 numbers, and only one point will be recorded in
        every 'step' points.
        """
        package = self._get_package_for_recording(package_id, expected_item_type=ItemType.kPointCloud)
        if package is None:
            return False
        if not points:
            raise ValueError(f"Point cloud of package {package_id} is empty.")
        _check_uint_range(step, 0x7FFFFFFF, "step")
        if step < 1:
            raise ValueError(f"Step of point cloud should be positive, but it is {step}.")

        number_of_points = len(points) // step
        payload = bytearray(_UINT32.pack(number_of_points))
        for index in range(0, number_of_points * step, step):
            point = [float(value) for value in _flatten_values(points[index])]
            if len(point) != NUMBER_OF_VALUES_OF_VECTOR3:
                raise ValueError(f"Point {index} of package {package_id} has {len(point)} values, but 3 values are needed.")
            payload += _VECTOR3.pack(*point)

        return self._write_record(package_id, bytes(payload), time_stamp_s)

    # Record pairs of three-dimensional points.
    def record_line_cloud(self, package_id: int, lines: Sequence[Any], time_stamp_s: float | None = None) -> bool:
        """Record a package with kLineCloud item, which is 'RecordPackage(id, const std::vector<std::pair<Vec3, Vec3>> &line_cloud)'.

        'lines' is a sequence of lines, and each line contains 2 points of 3 numbers.
        """
        package = self._get_package_for_recording(package_id, expected_item_type=ItemType.kLineCloud)
        if package is None:
            return False
        if not lines:
            raise ValueError(f"Line cloud of package {package_id} is empty.")

        payload = bytearray(_UINT32.pack(len(lines)))
        for index, line in enumerate(lines):
            for point in (line[0], line[1]):
                point_values = [float(value) for value in _flatten_values(point)]
                if len(point_values) != NUMBER_OF_VALUES_OF_VECTOR3:
                    raise ValueError(f"Point of line {index} in package {package_id} has {len(point_values)} values, but 3 values are needed.")
                payload += _VECTOR3.pack(*point_values)

        return self._write_record(package_id, bytes(payload), time_stamp_s)

    # Print registered package metadata.
    def report_all_registered_packages(self) -> None:
        """Print information of all registered packages, which is 'BinaryDataLog::ReportAllRegisteredPackages'."""
        print("[DataLog] Report all registered packages information:")
        for package in self._sorted_packages():
            item_type_names = ", ".join(f"{item.name} [{ItemType(item.type).name} at {item.bindata_index_in_package}]" for item in package.items)
            print(f">> Package name : {package.name}")
            print(f"   Package id : {package.id}")
            print(f"   Package size : {package.size} bytes")
            print(f"   Package items : {item_type_names}")

    # Support for recorder.

    # Order packages by their numeric identifiers.
    def _sorted_packages(self) -> list[PackageInfo]:
        """Packages are stored in the order of id, the same as the std::map of the C++ implementation."""
        return [self._packages[key] for key in sorted(self._packages.keys())]

    # Resolve and validate the target package.
    def _get_package_for_recording(self, package_id: int, expected_item_type: ItemType | None = None) -> PackageInfo | None:
        """Find a registered package, and check the type of its item if it is needed."""
        package = self._packages.get(package_id)
        if package is None:
            return None

        if expected_item_type is not None and package.items[0].type != expected_item_type:
            raise ValueError(f"Package {package_id} is registered as {ItemType(package.items[0].type).name}, "
                             f"but it is recorded as {expected_item_type.name}.")
        return package

    # Write the fixed file signature.
    def _write_log_file_header(self) -> None:
        self._file.write(BINARY_LOG_FILE_HEADER)

    # Serialize the package table.
    def _record_all_registered_packages_as_file_head(self) -> bool:
        """Write information of all registered packages to the head of the log file, which is 'BinaryDataLog::RecordAllRegisteredPackagesAsFileHead'."""
        if not self._packages:
            return False

        blocks = bytearray()
        # The offset itself is the first field of the head, which counts the whole length of Part 2.
        offset_to_data_part = 4
        for package in self._sorted_packages():
            offset = 4  # Offset to the next package.
            offset += 2  # Package id.
            body = bytearray(_UINT16.pack(package.id))

            # Write the length and the text of the package name.
            name = _text_bytes(package.name)
            offset += 1 + len(name)
            body += _UINT8.pack(len(name)) + name

            # Write the type and the name of each item in this package.
            for item in package.items:
                item_name = _text_bytes(item.name)
                offset += 1 + 1 + len(item_name)
                body += _UINT8.pack(int(item.type)) + _UINT8.pack(len(item_name)) + item_name

            offset += 1  # Sum check byte of this package.

            offset_bytes = _UINT32.pack(offset)
            blocks += offset_bytes + body + _UINT8.pack(summary_bytes(offset_bytes + body))
            offset_to_data_part += offset

        self._file.write(_UINT32.pack(offset_to_data_part))
        self._file.write(blocks)
        self._is_head_recorded = True
        return True

    # Write one checked and timestamped record.
    def _write_record(self, package_id: int, data: bytes, time_stamp_s: float | None) -> bool:
        """Write one package data into the log file, which is 'BinaryDataLog::RecordPackage'."""
        if self._file is None:
            return False
        if not self._is_head_recorded:
            raise RuntimeError("Prepare for recording before recording any package.")

        if time_stamp_s is None:
            time_stamp_s = self.get_system_timestamp()

        # offset, package_id, timestamp, binary_data, check_byte.
        offset = RECORD_HEAD_SIZE + len(data) + 1
        record_head = _RECORD_HEAD.pack(offset, package_id, float(time_stamp_s))
        self._file.write(record_head + data + _UINT8.pack(summary_bytes(record_head + data)))
        return True
