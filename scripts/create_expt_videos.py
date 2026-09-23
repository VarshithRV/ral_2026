#!/usr/bin/env python3

import os

import cv2
import rosbag2_py

from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


# ============================================================
# Configuration
# ============================================================

PARENT_FOLDER = os.path.expanduser(
    "~/colcon_ws/src/ral_2026/scripts/lattice_characterization_scripts/"
)

BAGS_FOLDER = os.path.join(
    PARENT_FOLDER,
    "rigid_gripper_expts_2"
)

TOPIC_NAME = "/camera/left_camera/color/image_raw"

FIRST_EXPERIMENT = 1
LAST_EXPERIMENT = 11

OUTPUT_FPS = 30.0

# Timer text settings
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 1.5
FONT_THICKNESS = 3

TEXT_POSITION = (30, 60)

# OpenCV uses BGR
TEXT_COLOR = (255, 255, 255)       # White
TEXT_BORDER_COLOR = (0, 0, 0)      # Black


# ============================================================
# ROS / OpenCV
# ============================================================

bridge = CvBridge()


def format_time(seconds):
    """Convert seconds to MM:SS:XX, where XX represents 10 ms units."""

    total_seconds = max(0.0, seconds)

    minutes = int(total_seconds // 60)
    secs = int(total_seconds % 60)

    # Hundredths of a second: 00-99
    hundredths = int((total_seconds % 1.0) * 100)

    return f"{minutes:02d}:{secs:02d}:{hundredths:02d}"


def open_bag(bag_path):
    """
    Open a ROS 2 sqlite3 bag.
    """

    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id="sqlite3"
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader.open(
        storage_options,
        converter_options
    )

    return reader


def add_timer(frame, elapsed_time):
    """
    Add an MM:SS timer to the image.
    """

    text = format_time(elapsed_time)

    # Draw black outline first for readability
    cv2.putText(
        frame,
        text,
        TEXT_POSITION,
        FONT,
        FONT_SCALE,
        TEXT_BORDER_COLOR,
        FONT_THICKNESS + 4,
        cv2.LINE_AA
    )

    # Draw white text
    cv2.putText(
        frame,
        text,
        TEXT_POSITION,
        FONT,
        FONT_SCALE,
        TEXT_COLOR,
        FONT_THICKNESS,
        cv2.LINE_AA
    )

    return frame


def process_bag(bag_path, output_path):
    """
    Extract the image topic from one bag and create an MP4 video.
    """

    print("\n----------------------------------------")
    print(f"Processing: {bag_path}")
    print(f"Output:     {output_path}")

    reader = open_bag(bag_path)

    # --------------------------------------------------------
    # Check that the requested topic exists
    # --------------------------------------------------------

    topics = reader.get_all_topics_and_types()

    available_topics = [
        topic.name
        for topic in topics
    ]

    if TOPIC_NAME not in available_topics:
        print(f"ERROR: Topic not found: {TOPIC_NAME}")
        print("Available topics:")

        for topic in available_topics:
            print(f"  {topic}")

        return False

    # --------------------------------------------------------
    # Process images
    # --------------------------------------------------------

    writer = None

    first_timestamp = None
    frame_count = 0

    while reader.has_next():

        topic, data, timestamp = reader.read_next()

        if topic != TOPIC_NAME:
            continue

        # rosbag timestamps are in nanoseconds
        timestamp_sec = timestamp * 1e-9

        # First image defines t = 0
        if first_timestamp is None:
            first_timestamp = timestamp_sec

        elapsed_time = timestamp_sec - first_timestamp

        # Deserialize ROS Image message
        msg = deserialize_message(
            data,
            Image
        )

        # Convert sensor_msgs/Image -> OpenCV image
        frame = bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        # ----------------------------------------------------
        # Initialize video writer using first image
        # ----------------------------------------------------

        if writer is None:

            height, width = frame.shape[:2]

            fourcc = cv2.VideoWriter_fourcc(
                *"mp4v"
            )

            writer = cv2.VideoWriter(
                output_path,
                fourcc,
                OUTPUT_FPS,
                (width, height)
            )

            if not writer.isOpened():
                raise RuntimeError(
                    f"Could not open video writer: "
                    f"{output_path}"
                )

            print(
                f"Resolution: {width} x {height}"
            )

            print(
                f"Output FPS: {OUTPUT_FPS}"
            )

        # ----------------------------------------------------
        # Add timer
        # ----------------------------------------------------

        frame = add_timer(
            frame,
            elapsed_time
        )

        # ----------------------------------------------------
        # Write frame
        # ----------------------------------------------------

        writer.write(frame)

        frame_count += 1

        if frame_count % 100 == 0:

            print(
                f"\rFrames: {frame_count} | "
                f"Timer: {format_time(elapsed_time)}",
                end="",
                flush=True
            )

    # --------------------------------------------------------
    # Finish video
    # --------------------------------------------------------

    if writer is not None:

        writer.release()

        print()

        print(
            f"Written {frame_count} frames"
        )

        print(
            f"Saved: {output_path}"
        )

        if os.path.exists(output_path):

            size_mb = (
                os.path.getsize(output_path)
                / (1024 * 1024)
            )

            print(
                f"Video size: {size_mb:.2f} MB"
            )

            return True

        else:

            print(
                "ERROR: Video writer finished but "
                "output file does not exist."
            )

            return False

    else:

        print(
            f"ERROR: No image frames found on "
            f"{TOPIC_NAME}"
        )

        return False


def main():

    # ========================================================
    # Verify paths
    # ========================================================

    print("========================================")
    print("ROS 2 Bag -> Video")
    print("========================================")

    print(
        f"Parent folder:\n"
        f"  {PARENT_FOLDER}"
    )

    print(
        f"\nBags folder:\n"
        f"  {BAGS_FOLDER}"
    )

    print(
        f"\nBags folder exists: "
        f"{os.path.isdir(BAGS_FOLDER)}"
    )

    if not os.path.isdir(BAGS_FOLDER):

        print(
            "\nERROR: Bags folder does not exist."
        )

        return

    # ========================================================
    # Process experiments
    # ========================================================

    successful = 0
    failed = 0

    for i in range(
        FIRST_EXPERIMENT,
        LAST_EXPERIMENT + 1
    ):

        bag_path = os.path.join(
            BAGS_FOLDER,
            f"expt_{i}"
        )

        output_path = os.path.join(
            BAGS_FOLDER,
            f"expt_{i}.mp4"
        )

        print("\n========================================")
        print(f"Experiment {i}")
        print("========================================")

        print(
            f"Bag path: {bag_path}"
        )

        # ----------------------------------------------------
        # Check bag directory
        # ----------------------------------------------------

        if not os.path.isdir(bag_path):

            print(
                f"Skipping missing bag: "
                f"{bag_path}"
            )

            failed += 1

            continue

        # ----------------------------------------------------
        # Process bag
        # ----------------------------------------------------

        try:

            success = process_bag(
                bag_path,
                output_path
            )

            if success:
                successful += 1
            else:
                failed += 1

        except Exception as e:

            print(
                f"\nERROR processing expt_{i}:"
            )

            print(
                repr(e)
            )

            failed += 1

    # ========================================================
    # Summary
    # ========================================================

    print("\n========================================")
    print("Finished processing all experiments")
    print("========================================")

    print(
        f"Successful: {successful}"
    )

    print(
        f"Failed/skipped: {failed}"
    )

    print(
        f"\nVideos are saved in:\n"
        f"  {BAGS_FOLDER}"
    )


if __name__ == "__main__":
    main()