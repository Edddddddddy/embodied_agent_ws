#!/usr/bin/env python3
"""Extract a sparse RGB contact sheet for manual OpenLORIS interval annotation."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--topic", default="/d400/color/image_raw")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--max-frames", type=int, default=24)
    parser.add_argument("--thumbnail-width", type=int, default=320)
    args = parser.parse_args()
    if args.interval <= 0 or args.max_frames < 1 or args.thumbnail_width < 32:
        parser.error("interval/max-frames/thumbnail-width must be positive")

    try:
        import numpy as np
        from PIL import Image, ImageDraw
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as error:
        raise SystemExit(
            "Missing review runtime; install requirements-slam-eval.txt"
        ) from error

    args.output_dir.mkdir(parents=True, exist_ok=True)
    index: list[dict[str, object]] = []
    thumbnails = []
    with AnyReader(
        [args.bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)
    ) as reader:
        connections = [item for item in reader.connections if item.topic == args.topic]
        if not connections:
            raise ValueError(f"topic missing from bag: {args.topic}")
        first_stamp_ns = None
        next_elapsed_s = 0.0
        for connection, timestamp_ns, raw in reader.messages(connections=connections):
            first_stamp_ns = timestamp_ns if first_stamp_ns is None else first_stamp_ns
            elapsed_s = (timestamp_ns - first_stamp_ns) * 1e-9
            if elapsed_s + 1e-9 < next_elapsed_s:
                continue
            message = reader.deserialize(raw, connection.msgtype)
            if message.encoding not in {"rgb8", "bgr8"}:
                raise ValueError(f"unsupported image encoding: {message.encoding}")
            pixels = np.asarray(message.data, dtype=np.uint8).reshape(
                int(message.height), int(message.step)
            )[:, : int(message.width) * 3]
            pixels = pixels.reshape(int(message.height), int(message.width), 3)
            if message.encoding == "bgr8":
                pixels = pixels[:, :, ::-1]
            image = Image.fromarray(pixels, mode="RGB")
            height = round(image.height * args.thumbnail_width / image.width)
            thumbnail = image.resize((args.thumbnail_width, height))
            filename = f"frame_{len(index):03d}_{elapsed_s:06.2f}s.jpg"
            thumbnail.save(args.output_dir / filename, quality=88)
            index.append(
                {
                    "frame": len(index),
                    "elapsed_s": round(elapsed_s, 6),
                    "stamp_s": round(timestamp_ns * 1e-9, 6),
                    "file": filename,
                }
            )
            thumbnails.append(thumbnail)
            next_elapsed_s += args.interval
            if len(index) >= args.max_frames:
                break

    if not thumbnails:
        raise ValueError("no image frames extracted")
    columns = 4
    label_height = 24
    cell_width = args.thumbnail_width
    cell_height = max(image.height for image in thumbnails) + label_height
    sheet = Image.new(
        "RGB",
        (columns * cell_width, math.ceil(len(thumbnails) / columns) * cell_height),
        "white",
    )
    draw = ImageDraw.Draw(sheet)
    for frame, image in enumerate(thumbnails):
        x = (frame % columns) * cell_width
        y = (frame // columns) * cell_height
        sheet.paste(image, (x, y + label_height))
        draw.text((x + 6, y + 5), f"#{frame}  t={index[frame]['elapsed_s']:.2f}s", fill="black")
    sheet.save(args.output_dir / "contact_sheet.jpg", quality=90)
    (args.output_dir / "index.json").write_text(
        json.dumps(
            {
                "bag": str(args.bag.resolve()),
                "topic": args.topic,
                "interval_s": args.interval,
                "frames": index,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"PASS: extracted {len(index)} review frames -> {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
