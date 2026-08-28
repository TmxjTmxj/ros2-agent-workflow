#!/usr/bin/env python3
"""Capture a real ROS camera frame as PNG evidence."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path


class CameraCaptureError(RuntimeError):
    """Raised when a usable camera frame cannot be captured or encoded."""


def image_message_to_png(message, output_path: str | Path) -> dict:
    """Validate and encode a sensor_msgs/Image message as a PNG."""
    if int(message.width) <= 0 or int(message.height) <= 0:
        raise CameraCaptureError("camera frame has invalid dimensions")
    if not message.data:
        raise CameraCaptureError("camera frame contains no pixel data")

    import cv2
    from cv_bridge import CvBridge, CvBridgeError

    output = Path(output_path).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp.png")
    try:
        frame = CvBridge().imgmsg_to_cv2(message, desired_encoding="bgr8")
        if frame is None or frame.size == 0:
            raise CameraCaptureError("camera conversion produced an empty image")
        if not cv2.imwrite(str(temporary), frame):
            raise CameraCaptureError(f"OpenCV could not write {temporary}")
        temporary.replace(output)
    except (CvBridgeError, ValueError) as exc:
        raise CameraCaptureError(f"camera conversion failed: {exc}") from exc
    finally:
        temporary.unlink(missing_ok=True)

    return {
        "path": str(output),
        "width": int(message.width),
        "height": int(message.height),
        "bytes": output.stat().st_size,
        "encoding": str(message.encoding),
    }


def capture_one_frame(
    output_path: str | Path,
    topic: str = "/camera/image_raw",
    timeout: float = 10.0,
) -> dict:
    """Wait for exactly one fresh frame and write it to disk."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    context = Context()
    rclpy.init(context=context)
    node = Node("codex_camera_capture", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    received = []
    subscription = node.create_subscription(
        Image, topic, lambda message: received.append(message), qos_profile_sensor_data
    )
    deadline = time.monotonic() + timeout
    try:
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(0.2, deadline - time.monotonic()))
        if not received:
            raise CameraCaptureError(f"no camera frame received from {topic} within {timeout:.1f}s")
        return image_message_to_png(received[-1], output_path)
    finally:
        node.destroy_subscription(subscription)
        executor.remove_node(node)
        node.destroy_node()
        context.shutdown()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--topic", default="/camera/image_raw")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args(argv)
    try:
        payload = capture_one_frame(args.output, topic=args.topic, timeout=args.timeout)
        print(json.dumps({"ok": True, **payload}, ensure_ascii=False))
        return 0
    except (CameraCaptureError, OSError, RuntimeError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
        return 1


if __name__ == "__main__":
    sys.exit(main())
