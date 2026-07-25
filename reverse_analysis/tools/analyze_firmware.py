#!/usr/bin/env python3
"""Parse an ESP32 application image and extract deterministic static evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import struct
from pathlib import Path


FLASH_MODES = {0: "QIO", 1: "QOUT", 2: "DIO", 3: "DOUT"}
FLASH_FREQUENCIES = {0x0: "40 MHz", 0x1: "26 MHz", 0x2: "20 MHz", 0xF: "80 MHz"}
FLASH_SIZES = {0x0: "1 MB", 0x1: "2 MB", 0x2: "4 MB", 0x3: "8 MB", 0x4: "16 MB"}

NOTABLE_PATTERN = re.compile(
    r"RoboFish|PARTNER|partner|servo|pitch|pump|mode_|forward|backward|left|right|"
    r"camera|stream|wifi|ssid|ota|control|gpio|script|timeout|watchdog|task|core|"
    r"password|version|demo|float|dive|CPG|hybrid|bionic|prop|NVS|SPIFFS|LittleFS",
    re.IGNORECASE,
)


def classify_address(address: int) -> str:
    if 0x3C000000 <= address < 0x3E000000:
        return "DROM"
    if 0x3FC80000 <= address < 0x3FD00000:
        return "DRAM"
    if 0x40370000 <= address < 0x403E0000:
        return "IRAM"
    if 0x42000000 <= address < 0x44000000:
        return "IROM"
    if 0x50000000 <= address < 0x50002000:
        return "RTC_DATA"
    return "UNKNOWN"


def decode_c_string(payload: bytes) -> str:
    """Decode one fixed-width, NUL-padded C string."""
    return payload.split(b"\x00", 1)[0].decode("utf-8", errors="replace")


def write_utf8(path: Path, text: str) -> None:
    """Write UTF-8 with stable LF newlines on every host platform."""
    path.write_bytes(text.encode("utf-8"))


def parse_app_descriptor(image: bytes, segments: list[dict]) -> dict | None:
    """Parse esp_app_desc_t when it is present at the start of DROM."""
    magic = 0xABCD5432
    for segment in segments:
        if segment["memory_type"] != "DROM" or segment["length"] < 176:
            continue
        start = segment["data_file_offset"]
        payload = image[start : start + 176]
        if struct.unpack_from("<I", payload)[0] != magic:
            continue
        return {
            "magic": f"0x{magic:08x}",
            "secure_version": struct.unpack_from("<I", payload, 4)[0],
            "app_version": decode_c_string(payload[16:48]),
            "project_name": decode_c_string(payload[48:80]),
            "compile_time": decode_c_string(payload[80:96]),
            "compile_date": decode_c_string(payload[96:112]),
            "esp_idf": decode_c_string(payload[112:144]),
            "elf_sha256": payload[144:176].hex(),
        }
    return None


def parse_segments(image: bytes, count: int) -> tuple[list[dict], int]:
    cursor = 24
    segments: list[dict] = []
    for index in range(count):
        if cursor + 8 > len(image):
            raise ValueError(f"segment {index} header exceeds image")
        load_address, length = struct.unpack_from("<II", image, cursor)
        data_offset = cursor + 8
        data_end = data_offset + length
        if data_end > len(image):
            raise ValueError(f"segment {index} data exceeds image")
        segments.append(
            {
                "index": index,
                "header_file_offset": cursor,
                "data_file_offset": data_offset,
                "data_file_end": data_end,
                "load_address": load_address,
                "length": length,
                "memory_type": classify_address(load_address),
            }
        )
        cursor = data_end
    return segments, cursor


def locate_segment(segments: list[dict], file_offset: int) -> tuple[int | None, int | None]:
    for segment in segments:
        start = segment["data_file_offset"]
        end = segment["data_file_end"]
        if start <= file_offset < end:
            return segment["index"], segment["load_address"] + file_offset - start
    return None, None


def extract_ascii_strings(image: bytes, minimum: int = 4) -> list[dict]:
    strings: list[dict] = []
    start: int | None = None
    for offset, value in enumerate(image + b"\x00"):
        printable = 0x20 <= value <= 0x7E or value == 0x09
        if printable and start is None:
            start = offset
        elif not printable and start is not None:
            if offset - start >= minimum:
                text = image[start:offset].decode("ascii", errors="replace")
                strings.append({"file_offset": start, "length": offset - start, "text": text})
            start = None
    return strings


def safe_text(text: str) -> str:
    return text.replace("\r", "\\r").replace("\n", "\\n").replace("\t", "\\t")


def extract_utf8_c_strings(image: bytes, segments: list[dict], minimum: int = 4) -> list[dict]:
    results: list[dict] = []
    for segment in segments:
        start = segment["data_file_offset"]
        payload = image[start : segment["data_file_end"]]
        local_start = 0
        while local_start < len(payload):
            local_end = payload.find(b"\x00", local_start)
            if local_end < 0:
                local_end = len(payload)
            raw = payload[local_start:local_end]
            if len(raw) >= minimum:
                try:
                    decoded = raw.decode("utf-8")
                except UnicodeDecodeError:
                    decoded = ""
                if decoded:
                    printable = sum(character.isprintable() or character in "\r\n\t" for character in decoded)
                    if printable / len(decoded) >= 0.95:
                        results.append(
                            {
                                "file_offset": start + local_start,
                                "virtual_address": segment["load_address"] + local_start,
                                "segment": segment["index"],
                                "length": len(raw),
                                "text": decoded,
                            }
                        )
            local_start = local_end + 1
    return results


def extract_web_assets(image: bytes, output_dir: Path) -> list[dict]:
    output_dir.mkdir(parents=True, exist_ok=True)
    markers = [
        (b"<!doctype html", ".html", "html"),
        (b"<!DOCTYPE html", ".html", "html"),
        (b"const CACHE_NAME", ".js", "service_worker"),
        (b'{\n    "name": "RoboFish', ".json", "manifest"),
        (b'{"name":"RoboFish', ".json", "manifest"),
    ]
    found: list[tuple[int, int, str, str, bytes]] = []
    seen_offsets: set[int] = set()
    for marker, extension, category in markers:
        search_from = 0
        while True:
            start = image.find(marker, search_from)
            if start < 0:
                break
            search_from = start + 1
            if start in seen_offsets:
                continue
            end = image.find(b"\x00", start)
            if end < 0:
                end = len(image)
            payload = image[start:end]
            if len(payload) < 80:
                continue
            seen_offsets.add(start)
            found.append((start, end, extension, category, payload))

    manifest: list[dict] = []
    category_counts: dict[str, int] = {}
    for start, end, extension, category, payload in sorted(found):
        category_counts[category] = category_counts.get(category, 0) + 1
        name = f"{category}_{category_counts[category]:02d}_0x{start:08x}{extension}"
        destination = output_dir / name
        destination.write_bytes(payload)
        manifest.append(
            {
                "name": name,
                "file_offset": start,
                "length": end - start,
                "sha256": hashlib.sha256(payload).hexdigest(),
            }
        )
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("firmware", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    firmware_argument = args.firmware
    firmware = firmware_argument.resolve()
    output = args.output.resolve()
    metadata_dir = output / "metadata"
    segment_dir = output / "segments"
    string_dir = output / "strings"
    web_dir = output / "web_assets"
    for directory in (metadata_dir, segment_dir, string_dir, web_dir):
        directory.mkdir(parents=True, exist_ok=True)

    image = firmware.read_bytes()
    if len(image) < 24 or image[0] != 0xE9:
        raise ValueError("not an ESP application image")

    segment_count = image[1]
    flash_mode = image[2]
    size_frequency = image[3]
    entry_point = struct.unpack_from("<I", image, 4)[0]
    chip_id = struct.unpack_from("<H", image, 12)[0]
    segments, segment_data_end = parse_segments(image, segment_count)

    for segment in segments:
        payload = image[segment["data_file_offset"] : segment["data_file_end"]]
        filename = (
            f"segment_{segment['index']:02d}_{segment['memory_type']}_"
            f"0x{segment['load_address']:08x}_0x{segment['length']:x}.bin"
        )
        (segment_dir / filename).write_bytes(payload)
        segment["filename"] = filename
        segment["sha256"] = hashlib.sha256(payload).hexdigest()

    ascii_strings = extract_ascii_strings(image)
    for item in ascii_strings:
        segment_index, virtual_address = locate_segment(segments, item["file_offset"])
        item["segment"] = segment_index
        item["virtual_address"] = virtual_address

    header = "file_offset\tvirtual_address\tsegment\tlength\ttext\n"
    all_lines = [header]
    notable_lines = [header]
    for item in ascii_strings:
        virtual = "" if item["virtual_address"] is None else f"0x{item['virtual_address']:08x}"
        segment = "" if item["segment"] is None else str(item["segment"])
        line = (
            f"0x{item['file_offset']:08x}\t{virtual}\t{segment}\t{item['length']}\t"
            f"{safe_text(item['text'])}\n"
        )
        all_lines.append(line)
        if NOTABLE_PATTERN.search(item["text"]):
            notable_lines.append(line)
    write_utf8(string_dir / "all_strings.tsv", "".join(all_lines))
    write_utf8(string_dir / "notable_strings.tsv", "".join(notable_lines))

    utf8_strings = extract_utf8_c_strings(image, segments)
    utf8_lines = [header]
    for item in utf8_strings:
        utf8_lines.append(
            f"0x{item['file_offset']:08x}\t0x{item['virtual_address']:08x}\t{item['segment']}\t"
            f"{item['length']}\t{safe_text(item['text'])}\n"
        )
    write_utf8(string_dir / "utf8_c_strings.tsv", "".join(utf8_lines))

    assets = extract_web_assets(image, web_dir)
    checksum_offset = ((segment_data_end + 15) & ~15) - 1
    validation_hash_offset = checksum_offset + 1
    stored_hash = image[validation_hash_offset : validation_hash_offset + 32]
    calculated_checksum = 0xEF
    for segment in segments:
        payload = image[segment["data_file_offset"] : segment["data_file_end"]]
        for value in payload:
            calculated_checksum ^= value
    calculated_validation_hash = hashlib.sha256(image[:validation_hash_offset]).digest()

    manifest = {
        "source": firmware_argument.as_posix(),
        "size": len(image),
        "sha256": hashlib.sha256(image).hexdigest(),
        "header": {
            "magic": f"0x{image[0]:02x}",
            "segment_count": segment_count,
            "flash_mode": FLASH_MODES.get(flash_mode, f"unknown ({flash_mode})"),
            "flash_size": FLASH_SIZES.get(size_frequency >> 4, f"unknown ({size_frequency >> 4})"),
            "flash_frequency": FLASH_FREQUENCIES.get(size_frequency & 0xF, f"unknown ({size_frequency & 0xF})"),
            "entry_point": f"0x{entry_point:08x}",
            "chip_id": chip_id,
        },
        "segments": segments,
        "footer": {
            "segment_data_end": segment_data_end,
            "checksum_offset": checksum_offset,
            "checksum": f"0x{image[checksum_offset]:02x}" if checksum_offset < len(image) else None,
            "calculated_checksum": f"0x{calculated_checksum:02x}",
            "checksum_valid": checksum_offset < len(image) and image[checksum_offset] == calculated_checksum,
            "validation_hash_offset": validation_hash_offset,
            "validation_hash": stored_hash.hex(),
            "calculated_validation_hash": calculated_validation_hash.hex(),
            "validation_hash_valid": stored_hash == calculated_validation_hash,
        },
        "application": parse_app_descriptor(image, segments),
        "string_count": len(ascii_strings),
        "utf8_c_string_count": len(utf8_strings),
        "notable_string_count": sum(1 for item in ascii_strings if NOTABLE_PATTERN.search(item["text"])),
        "web_assets": assets,
    }
    write_utf8(
        metadata_dir / "image_manifest.json",
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
    )
    print(json.dumps(manifest, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
