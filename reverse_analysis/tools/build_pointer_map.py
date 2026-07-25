#!/usr/bin/env python3
"""Build a segment-aware 32-bit pointer map for the extracted ESP32 image."""

from __future__ import annotations

import argparse
import csv
import json
import re
import struct
from pathlib import Path


NOTABLE = re.compile(
    r"RoboFish|PARTNER|partner|servo|pitch|pump|mode_|forward|backward|left|right|"
    r"camera|stream|wifi|ssid|ota|control|gpio|script|timeout|failsafe|demo|capture|"
    r"manifest|save_wifi|get_ip|status|duration|power|strength|angle|repeat",
    re.IGNORECASE,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("analysis_root", type=Path)
    args = parser.parse_args()
    root = args.analysis_root.resolve()

    manifest = json.loads((root / "metadata" / "image_manifest.json").read_text(encoding="utf-8"))
    segments = manifest["segments"]
    string_by_address: dict[int, str] = {}
    with (root / "strings" / "all_strings.tsv").open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle, delimiter="\t"):
            if row["virtual_address"]:
                string_by_address[int(row["virtual_address"], 16)] = row["text"]

    def target_segment(address: int) -> dict | None:
        for segment in segments:
            start = segment["load_address"]
            if start <= address < start + segment["length"]:
                return segment
        return None

    rows: list[dict] = []
    for source in segments:
        payload = (root / "segments" / source["filename"]).read_bytes()
        for local_offset in range(0, len(payload) - 3, 4):
            value = struct.unpack_from("<I", payload, local_offset)[0]
            target = target_segment(value)
            if target is None:
                continue
            text = string_by_address.get(value, "")
            rows.append(
                {
                    "source_address": f"0x{source['load_address'] + local_offset:08x}",
                    "source_segment": source["index"],
                    "source_type": source["memory_type"],
                    "target_address": f"0x{value:08x}",
                    "target_segment": target["index"],
                    "target_type": target["memory_type"],
                    "target_string": text,
                }
            )

    output_dir = root / "references"
    output_dir.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "source_address",
        "source_segment",
        "source_type",
        "target_address",
        "target_segment",
        "target_type",
        "target_string",
    ]
    for filename, selected in (
        ("pointers.tsv", rows),
        ("notable_string_pointers.tsv", [row for row in rows if NOTABLE.search(row["target_string"])]),
    ):
        with (output_dir / filename).open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames, delimiter="\t")
            writer.writeheader()
            writer.writerows(selected)

    print(f"pointers={len(rows)}")
    print(f"notable_string_pointers={sum(1 for row in rows if NOTABLE.search(row['target_string']))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
