"""Example of recording a binary data log file, which is the python version of 'test/test_binary_data_log.cpp'.

Usage::

    python3 py_src/example_record.py [output.binlog]

All data recorded here is generated in memory, so no external file or library is needed.
"""

from __future__ import annotations

import struct
import sys
import zlib

try:  # Support both 'python3 -m py_src.example_record' and 'python3 py_src/example_record.py'.
    from .binary_data_log import BinaryDataLog
    from .binary_data_log_protocol import ItemType, PackageInfo, PackageItemInfo
except ImportError:  # pragma: no cover
    from binary_data_log import BinaryDataLog
    from binary_data_log_protocol import ItemType, PackageInfo, PackageItemInfo

GRAY_IMAGE_ROWS, GRAY_IMAGE_COLS = 48, 64
RGB_IMAGE_ROWS, RGB_IMAGE_COLS = 32, 40
PNG_IMAGE_ROWS, PNG_IMAGE_COLS = 24, 32
MATRIX_ROWS, MATRIX_COLS = 30, 40

NUMBER_OF_RECORDED_TICKS = 200
TIME_STEP_S = 0.25


# Declare the example data streams.
def register_all_packages(logger: BinaryDataLog) -> None:
    """Register all packages used by this example."""
    packages = [
        PackageInfo(id=1, name="imu", items=[
            PackageItemInfo(ItemType.kVector3, "gyro"),
            PackageItemInfo(ItemType.kVector3, "accel"),
            PackageItemInfo(ItemType.kUint8, "valid"),
        ]),
        PackageInfo(id=2, name="baro", items=[
            PackageItemInfo(ItemType.kUint32, "press"),
            PackageItemInfo(ItemType.kFloat, "height"),
            PackageItemInfo(ItemType.kUint8, "valid"),
        ]),
        PackageInfo(id=3, name="gray image", items=[
            PackageItemInfo(ItemType.kImage, "left"),
        ]),
        PackageInfo(id=4, name="rgb image", items=[
            PackageItemInfo(ItemType.kImage, "left"),
        ]),
        PackageInfo(id=5, name="matrix", items=[
            PackageItemInfo(ItemType.kMatrix, "matrix"),
        ]),
        PackageInfo(id=6, name="png image", items=[
            PackageItemInfo(ItemType.kPngImage, "left(png)"),
        ]),
        PackageInfo(id=7, name="slam state", items=[
            PackageItemInfo(ItemType.kUint8, "is_vel_valid"),
            PackageItemInfo(ItemType.kVector3, "velocity"),
            PackageItemInfo(ItemType.kUint8, "is_pose_valid"),
            PackageItemInfo(ItemType.kPose6Dof, "pose"),
        ]),
        PackageInfo(id=8, name="local map", items=[
            PackageItemInfo(ItemType.kPointCloud, "point cloud"),
        ]),
        PackageInfo(id=9, name="line cloud", items=[
            PackageItemInfo(ItemType.kLineCloud, "line cloud"),
        ]),
    ]

    for package in packages:
        if not logger.register_package(package):
            raise RuntimeError(f"Test failed: register package {package.name}.")


# Generate example grayscale pixels.
def build_gray_image() -> bytes:
    return bytes((row * GRAY_IMAGE_COLS + col) % 256 for row in range(GRAY_IMAGE_ROWS) for col in range(GRAY_IMAGE_COLS))


# Generate example RGB pixels.
def build_rgb_image() -> bytes:
    return bytes(value % 256 for index in range(RGB_IMAGE_ROWS * RGB_IMAGE_COLS) for value in (index, index * 3, index * 7))


# Generate an example float matrix.
def build_matrix() -> list[list[float]]:
    return [[float(row * MATRIX_COLS + col) * 0.5 for col in range(MATRIX_COLS)] for row in range(MATRIX_ROWS)]


# Build a PNG image with standard-library codecs.
def build_png_image_data() -> bytes:
    """Build a valid rgb png image in memory, so that this example needs no image file."""
    raw = bytearray()
    for row in range(PNG_IMAGE_ROWS):
        raw.append(0)  # Filter type of this scanline.
        for col in range(PNG_IMAGE_COLS):
            raw += bytes((row * 3 % 256, col * 5 % 256, (row + col) * 7 % 256))

    # Encode one PNG chunk and its checksum.
    def make_chunk(chunk_type: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + chunk_type + data + struct.pack(">I", zlib.crc32(chunk_type + data))

    return (b"\x89PNG\r\n\x1a\n"
            + make_chunk(b"IHDR", struct.pack(">IIBBBBB", PNG_IMAGE_COLS, PNG_IMAGE_ROWS, 8, 2, 0, 0, 0))
            + make_chunk(b"IDAT", zlib.compress(bytes(raw)))
            + make_chunk(b"IEND", b""))


# Write example samples to each data stream.
def record_data(logger: BinaryDataLog) -> None:
    """Record all data of this example, with fixed values to keep the log file reproducible."""
    gray_image = build_gray_image()
    rgb_image = build_rgb_image()
    png_image = build_png_image_data()
    matrix = build_matrix()

    for i in range(NUMBER_OF_RECORDED_TICKS):
        time_stamp_s = i * TIME_STEP_S
        temp = i * 0.25

        # Record imu data, which is a package of fixed size.
        logger.record_values(1, (temp, -temp, temp * 0.5, 1.0, 2.0, 3.0, 1 if i > 50 else 0), time_stamp_s)

        # Record baro data, which is a package of fixed size.
        logger.record_values(2, (i * 2, i + 0.5, 1 if i < 30 else 0), time_stamp_s)

        # Record slam state, which is a package of fixed size.
        logger.record_values(7, (1 if i < 100 else 0, 1.0, 2.0, 3.0, 1 if i < 100 else 0,
                                 i + 1.0, i * 1.25, i * 0.5, 1.0, 0.0, 0.0, 0.0), time_stamp_s)

        # Record images, matrix and png image.
        if i % 20 == 0:
            logger.record_gray_image(3, GRAY_IMAGE_ROWS, GRAY_IMAGE_COLS, gray_image, time_stamp_s)
            logger.record_rgb_image(4, RGB_IMAGE_ROWS, RGB_IMAGE_COLS, rgb_image, time_stamp_s)
            logger.record_matrix(5, MATRIX_ROWS, MATRIX_COLS, matrix, time_stamp_s)
            logger.record_png_image(6, png_image, time_stamp_s)

        # Record point cloud, which is a package of unfixed size.
        if i % 20 == 0:
            points = [(j * 0.5, j * 0.25 + i, j * 0.125) for j in range(40)]
            logger.record_point_cloud(8, points, 2, time_stamp_s)

        # Record line cloud, which is a package of unfixed size.
        if i % 25 == 0:
            lines = [((j * 0.5, j * 0.25, i * 0.125), (j * 0.5 + 1.0, j * 0.25 + 2.0, i * 0.125 + 3.0)) for j in range(10)]
            logger.record_line_cloud(9, lines, time_stamp_s)


# Create the example file and report its location.
def main() -> int:
    log_file_name = sys.argv[1] if len(sys.argv) > 1 else "data.binlog"

    logger = BinaryDataLog()
    if not logger.create_log_file(log_file_name):
        print(f"[DataLog] Test failed: create log file {log_file_name}.")
        return 1

    register_all_packages(logger)
    if not logger.prepare_for_recording():
        print("[DataLog] Test failed: prepare for recording.")
        return 1

    logger.report_all_registered_packages()
    record_data(logger)
    logger.close()

    print(f"[DataLog] Binary data log file is written to {log_file_name}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
